#include "coil.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "adc.h"
#include "cmsis_os.h"
#include "tim.h"
#include "usart.h"
#include "usbd_cdc_if.h"

#include "coil_config.h"

#if (COIL_OS < 1U) || (COIL_OS > 16U)
#error "COIL_OS: 1...16, точка хранится в 16 битах"
#endif

// Блок = половина кольца DMA: 64 точки = 12,8 мс при 5 кГц. На каждый блок — один расчёт, шаг задаёт кварц таймера
#define COIL_BLOCK          64U
#define COIL_DMA_WORDS      (2U * COIL_BLOCK * COIL_OS)
#define COIL_BLOCK_S        ((double)COIL_BLOCK / (double)COIL_FS_HZ)
// Кольцо точек на канал: 8192 = 1,64 с при 5 кГц
#define COIL_RING           8192U
#define COIL_RING_MASK      (COIL_RING - 1U)
// Пределы окна подгонки, точек
#define COIL_FIT_MIN        COIL_BLOCK
#define COIL_FIT_MAX        2048U
// Частота для окна — по последним переходам К1 через уровень снизу вверх
#define COIL_CROSS_KEEP     16U
// Полная шкала: 12 бит × накопление
#define COIL_FULL           (4095U * COIL_OS)
// Отсчёт ближе этого к краям шкалы считается перегрузом
#define COIL_CLIP_CODES     (2U * COIL_OS)
// Уровень и полуразмах К1 для гистерезиса: сглаживание по блокам
#define COIL_LEVEL_ALPHA    0.2f
#define COIL_PI             3.14159265358979323846

typedef struct
{
  double a;
  double b;
  double power;
} CoilFit;

// Снимок для печати: пишет задача расчёта, читает задача вывода
typedef struct
{
  uint32_t blocks;        // обработано блоков с запуска
  uint32_t missed;        // блоков, на которые расчёт опоздал
  uint32_t missed_at;     // номер блока последнего опоздания
  uint32_t calc_us_last;  // время расчёта последнего блока
  uint32_t calc_us_max;
  uint8_t  valid;         // 0 — нет колебаний на К1
  float    swing_mv;      // размах К1 по блокам, когда сигнала нет
  float    freq_hz;
  float    amp1_mv;
  float    amp2_mv;
  int32_t  snr1_db;
  int32_t  snr2_db;
  double   lock_hz;       // частота по набегу фазы, сглаженная
  double   dt_us;         // Δt последнего блока
  double   dt_damped_us;  // Δt после демпфирования
  double   dt_shown_us;   // после демпфирования минус ноль
  double   flow_kgmin;    // расход с отсечкой
  uint32_t fit_points;
  uint32_t clip1;
  uint32_t clip2;
} CoilSnapshot;

extern USBD_HandleTypeDef hUsbDeviceFS;

// Кольцо DMA: в каждом слове младшие 16 бит — ADC1 (К1), старшие — ADC2 (К2), один момент выборки
static uint32_t coil_dma[COIL_DMA_WORDS];
static uint16_t coil_k1[COIL_RING];
static uint16_t coil_k2[COIL_RING];
// Сколько точек записано с запуска. Индекс в кольце — младшие биты
static volatile uint32_t coil_head;
static TaskHandle_t coil_calc_handle;

// Состояние расчёта — только задача расчёта
static uint32_t coil_done;           // точек, уже прошедших через детектор переходов
static float coil_level;             // уровень К1
static float coil_half;              // полуразмах К1
static uint8_t coil_level_ready;
static float coil_prev;
static uint8_t coil_armed;
static double coil_cross[COIL_CROSS_KEEP];
static uint32_t coil_cross_count;
static double coil_damped_us;
static uint8_t coil_have_damped;
static double coil_alpha;            // 1 − e^(−T/τ) на один блок
// Фаза К1 на последней точке прошлого окна и номер этой точки — для частоты по набегу фазы
static double coil_theta;
static uint32_t coil_theta_at;
static uint8_t coil_theta_ok;
static double coil_lock_hz;
static double coil_freq_alpha;

static CoilSnapshot coil_snap;
// CDC отправляет из буфера без копии, поэтому строка живёт до следующей печати
static char coil_text[200];

static void Coil_CalcTask(void *argument);
static void Coil_PrintTask(void *argument);

static const osThreadAttr_t coil_calc_attributes = {
  .name = "coilCalc",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

static const osThreadAttr_t coil_print_attributes = {
  .name = "coilPrint",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

void Coil_Init(void)
{
  coil_calc_handle = (TaskHandle_t)osThreadNew(Coil_CalcTask, NULL, &coil_calc_attributes);
  (void)osThreadNew(Coil_PrintTask, NULL, &coil_print_attributes);
}

// 16 запусков подряд → одна точка на канал, затем будим расчёт. Вызывается из прерывания DMA
static void Coil_Accumulate(const uint32_t *words)
{
  BaseType_t woken = pdFALSE;
  uint32_t head = coil_head;
  uint32_t point;
  uint32_t index;
  uint32_t sum1;
  uint32_t sum2;

  for (point = 0U; point < COIL_BLOCK; point++)
  {
    sum1 = 0U;
    sum2 = 0U;
    for (index = 0U; index < COIL_OS; index++)
    {
      sum1 += *words & 0xFFFFU;
      sum2 += *words >> 16;
      words++;
    }
    coil_k1[head & COIL_RING_MASK] = (uint16_t)sum1;
    coil_k2[head & COIL_RING_MASK] = (uint16_t)sum2;
    head++;
  }
  coil_head = head;
  if (coil_calc_handle != NULL)
  {
    vTaskNotifyGiveFromISR(coil_calc_handle, &woken);
    portYIELD_FROM_ISR(woken);
  }
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    Coil_Accumulate(&coil_dma[0]);
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    Coil_Accumulate(&coil_dma[COIL_DMA_WORDS / 2U]);
  }
}

// Строка в USART2 (PA2) и в USB CDC, если порт открыт на ПК
static void Coil_Send(int length)
{
  if (length <= 0)
  {
    return;
  }
  if (length >= (int)sizeof(coil_text))
  {
    length = (int)sizeof(coil_text) - 1;
  }
  (void)HAL_UART_Transmit(&huart2, (uint8_t *)coil_text, (uint16_t)length, 50U);
  // без этой проверки CDC_Transmit_FS разыменует пустой pClassData
  if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED)
  {
    (void)CDC_Transmit_FS((uint8_t *)coil_text, (uint16_t)length);
  }
}

// Новые точки К1 → уровень, полуразмах и переходы через уровень снизу вверх с гистерезисом 20 % полуразмаха.
// Позиции переходов — в отсчётах с запуска, с линейной интерполяцией между точками
static void Coil_TrackCrossings(uint32_t from, uint32_t to)
{
  float low = 1e9f;
  float high = -1e9f;
  float mean = 0.0f;
  float cur;
  float hyst;
  uint32_t n;

  for (n = from; n < to; n++)
  {
    cur = (float)coil_k1[n & COIL_RING_MASK];
    mean += cur;
    if (cur < low) { low = cur; }
    if (cur > high) { high = cur; }
  }
  mean /= (float)(to - from);
  if (coil_level_ready == 0U)
  {
    coil_level = mean;
    coil_half = (high - low) * 0.5f;
    coil_prev = (float)coil_k1[from & COIL_RING_MASK];
    coil_level_ready = 1U;
  }
  else
  {
    coil_level += COIL_LEVEL_ALPHA * (mean - coil_level);
    coil_half += COIL_LEVEL_ALPHA * ((high - low) * 0.5f - coil_half);
  }
  hyst = COIL_HYST_FRACTION * coil_half;

  for (n = from; n < to; n++)
  {
    cur = (float)coil_k1[n & COIL_RING_MASK];
    if (coil_prev < coil_level - hyst)
    {
      coil_armed = 1U;
    }
    if ((coil_armed != 0U) && (coil_prev < coil_level) && (cur >= coil_level))
    {
      coil_cross[coil_cross_count % COIL_CROSS_KEEP] =
        (double)(n - 1U) + (double)((coil_level - coil_prev) / (cur - coil_prev));
      coil_cross_count++;
      coil_armed = 0U;
    }
    coil_prev = cur;
  }
}

// Точек на период по последним переходам. 0 — переходов мало или последний слишком давно
static uint8_t Coil_Period(uint32_t end, float *period)
{
  const uint32_t used = (coil_cross_count < COIL_CROSS_KEEP) ? coil_cross_count : COIL_CROSS_KEEP;
  double first;
  double last;

  if (used < 3U)
  {
    return 0U;
  }
  last = coil_cross[(coil_cross_count - 1U) % COIL_CROSS_KEEP];
  first = coil_cross[(coil_cross_count - used) % COIL_CROSS_KEEP];
  *period = (float)((last - first) / (double)(used - 1U));
  // последний переход старше трёх периодов — сигнал пропал
  if (((double)end - last) > 3.0 * (double)*period)
  {
    return 0U;
  }
  return (*period >= 4.0f) ? 1U : 0U;
}

static double Coil_Det3(const double m[3][3])
{
  return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
       - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
       + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
}

// МНК y ≈ a·cos(ωk) + b·sin(ωk) + c для обоих каналов на одном окне, в кодах АЦП.
// Суммы во float: окно короткое (≤ 2048 точек), значения центрированы
static uint8_t Coil_Fit(uint32_t start, uint32_t count, float omega, CoilFit *f1, CoilFit *f2,
                        uint32_t *clip1, uint32_t *clip2)
{
  float cc = 0.0f, ss = 0.0f, cs = 0.0f, c1 = 0.0f, s1 = 0.0f;
  float yc1 = 0.0f, ys1 = 0.0f, yc2 = 0.0f, ys2 = 0.0f, p1 = 0.0f, p2 = 0.0f;
  float dc1 = 0.0f, dc2 = 0.0f, c = 1.0f, s = 0.0f, t, y1, y2;
  const float cw = cosf(omega);
  const float sw = sinf(omega);
  double mx[3][3], ma[3][3], mb[3][3], det;
  uint32_t k, r, q, code1, code2;

  *clip1 = 0U;
  *clip2 = 0U;
  for (k = 0U; k < count; k++)
  {
    code1 = coil_k1[(start + k) & COIL_RING_MASK];
    code2 = coil_k2[(start + k) & COIL_RING_MASK];
    dc1 += (float)code1;
    dc2 += (float)code2;
    if ((code1 <= COIL_CLIP_CODES) || (code1 >= COIL_FULL - COIL_CLIP_CODES)) { (*clip1)++; }
    if ((code2 <= COIL_CLIP_CODES) || (code2 >= COIL_FULL - COIL_CLIP_CODES)) { (*clip2)++; }
  }
  dc1 /= (float)count;
  dc2 /= (float)count;

  for (k = 0U; k < count; k++)
  {
    // опорный генератор поворотом на ω, раз в 256 шагов — точный пересчёт
    if ((k & 255U) == 0U)
    {
      c = cosf(omega * (float)k);
      s = sinf(omega * (float)k);
    }
    y1 = (float)coil_k1[(start + k) & COIL_RING_MASK] - dc1;
    y2 = (float)coil_k2[(start + k) & COIL_RING_MASK] - dc2;
    cc += c * c;  ss += s * s;  cs += c * s;  c1 += c;  s1 += s;
    yc1 += y1 * c;  ys1 += y1 * s;  p1 += y1 * y1;
    yc2 += y2 * c;  ys2 += y2 * s;  p2 += y2 * y2;
    t = c * cw - s * sw;
    s = s * cw + c * sw;
    c = t;
  }

  mx[0][0] = cc; mx[0][1] = cs; mx[0][2] = c1;
  mx[1][0] = cs; mx[1][1] = ss; mx[1][2] = s1;
  mx[2][0] = c1; mx[2][1] = s1; mx[2][2] = (double)count;
  det = Coil_Det3(mx);
  if (fabs(det) < 1e-9)
  {
    return 0U;
  }
  // Крамер: правая часть (yc, ys, 0) вместо первого столбца — a, вместо второго — b
  for (k = 0U; k < 2U; k++)
  {
    const double yc = (k == 0U) ? yc1 : yc2;
    const double ys = (k == 0U) ? ys1 : ys2;
    CoilFit *fit = (k == 0U) ? f1 : f2;

    for (r = 0U; r < 3U; r++)
    {
      for (q = 0U; q < 3U; q++)
      {
        ma[r][q] = mx[r][q];
        mb[r][q] = mx[r][q];
      }
    }
    ma[0][0] = yc; ma[1][0] = ys; ma[2][0] = 0.0;
    mb[0][1] = yc; mb[1][1] = ys; mb[2][1] = 0.0;
    fit->a = Coil_Det3(ma) / det;
    fit->b = Coil_Det3(mb) / det;
  }
  f1->power = (double)p1 / (double)count;
  f2->power = (double)p2 / (double)count;
  return 1U;
}

// Сигнал/шум: мощность основной гармоники против остатка, дБ
static int32_t Coil_Snr(double amp, double power)
{
  const double signal = amp * amp * 0.5;
  double noise = power - signal;

  if (noise < signal * 1e-9 + 1e-12)
  {
    noise = signal * 1e-9 + 1e-12;
  }
  return (int32_t)lround(10.0 * log10(signal / noise));
}

// Сигнала нет: демпфирование и частота по фазе начнутся заново
static void Coil_Lost(void)
{
  coil_have_damped = 0U;
  coil_theta_ok = 0U;
}

// Один расчёт на конец данных end. steps — сколько блоков прошло с прошлого расчёта (обычно 1)
static void Coil_Calc(uint32_t end, uint32_t steps, CoilSnapshot *snap)
{
  const double mv_per_code = COIL_VDDA_MV / (double)COIL_FULL;
  float period;
  uint32_t count;
  float omega;
  double freq, dphi, dt_us, amp1, amp2, alpha, theta, lock, shown, flow;
  int32_t snr1, snr2;
  CoilFit f1, f2;

  snap->valid = 0U;
  snap->swing_mv = (float)(2.0 * (double)coil_half * mv_per_code);
  if ((end < COIL_FIT_MAX) || (Coil_Period(end, &period) == 0U))
  {
    Coil_Lost();
    return;
  }
  // окно — целое число периодов, кончается на последней точке
  count = (uint32_t)lroundf(COIL_FIT_PERIODS * period);
  if (count < COIL_FIT_MIN) { count = COIL_FIT_MIN; }
  if (count > COIL_FIT_MAX) { count = COIL_FIT_MAX; }
  omega = 2.0f * (float)COIL_PI / period;
  freq = (double)COIL_FS_HZ / (double)period;
  if (Coil_Fit(end - count, count, omega, &f1, &f2, &snap->clip1, &snap->clip2) == 0U)
  {
    Coil_Lost();
    return;
  }
  amp1 = sqrt(f1.a * f1.a + f1.b * f1.b);
  amp2 = sqrt(f2.a * f2.a + f2.b * f2.b);
  snr1 = Coil_Snr(amp1, f1.power);
  snr2 = Coil_Snr(amp2, f2.power);
  // мала амплитуда или шума больше синуса — колебаний трубки нет, шум висящих входов не считаем
  if ((amp1 * mv_per_code < COIL_WEAK_MV) || (snr1 < COIL_MIN_SNR_DB) || (snr2 < COIL_MIN_SNR_DB))
  {
    Coil_Lost();
    return;
  }
  // Частота по набегу фазы К1. a·cos(ωk) + b·sin(ωk) = A·cos(ωk − ψ), ψ = atan2(b, a), фаза на последней точке
  // окна θ = ω·(count − 1) − ψ. За Δn точек она набегает на 2π·f·Δn/fs; остаток после ожидаемого ω·Δn
  // уточняет частоту точнее, чем переходы через ноль
  theta = (double)omega * (double)(count - 1U) - atan2(f1.b, f1.a);
  if (coil_theta_ok != 0U)
  {
    const double dn = (double)(end - coil_theta_at);

    lock = ((double)omega * dn + remainder(theta - coil_theta - (double)omega * dn, 2.0 * COIL_PI))
           / (2.0 * COIL_PI * dn) * (double)COIL_FS_HZ;
    alpha = (steps == 1U) ? coil_freq_alpha : 1.0 - pow(1.0 - coil_freq_alpha, (double)steps);
    coil_lock_hz += alpha * (lock - coil_lock_hz);
  }
  else
  {
    coil_lock_hz = freq;
  }
  coil_theta = remainder(theta, 2.0 * COIL_PI);
  coil_theta_at = end;
  coil_theta_ok = 1U;

  // Δφ = φ2 − φ1, выборка одновременная — поправки на сдвиг нет
  dphi = atan2(-f2.b, f2.a) - atan2(-f1.b, f1.a);
  while (dphi > COIL_PI)   { dphi -= 2.0 * COIL_PI; }
  while (dphi <= -COIL_PI) { dphi += 2.0 * COIL_PI; }
  dt_us = dphi / (2.0 * COIL_PI * coil_lock_hz) * 1e6;

  // демпфирование с шагом в блоках: пропущенные блоки учитываются степенью
  if (coil_have_damped == 0U)
  {
    coil_damped_us = dt_us;
    coil_have_damped = 1U;
  }
  else
  {
    alpha = (steps == 1U) ? coil_alpha : 1.0 - pow(1.0 - coil_alpha, (double)steps);
    coil_damped_us += alpha * (dt_us - coil_damped_us);
  }
  // расход по демпфированному Δt минус ноль, малый — в ноль
  shown = coil_damped_us - COIL_ZERO_US;
  flow = (fabs(shown) < COIL_CUTOFF_US) ? 0.0 : COIL_K_KGMIN_PER_US * shown;
  snap->lock_hz = coil_lock_hz;
  snap->dt_shown_us = shown;
  snap->flow_kgmin = flow;

  snap->valid = 1U;
  snap->freq_hz = (float)freq;
  snap->amp1_mv = (float)(amp1 * mv_per_code);
  snap->amp2_mv = (float)(amp2 * mv_per_code);
  snap->snr1_db = snr1;
  snap->snr2_db = snr2;
  snap->dt_us = dt_us;
  snap->dt_damped_us = coil_damped_us;
  snap->fit_points = count;
}

// Расчёт на каждый блок DMA: будит прерывание, шаг 12,8 мс от кварца таймера
static void Coil_CalcTask(void *argument)
{
  CoilSnapshot snap;
  uint32_t pending;
  uint32_t end;
  uint32_t started;
  uint32_t calc_us;

  (void)argument;
  memset(&snap, 0, sizeof(snap));
  // τ = 0 — без сглаживания: коэффициент 1
  coil_alpha = (COIL_TAU_S > 0.0) ? 1.0 - exp(-COIL_BLOCK_S / COIL_TAU_S) : 1.0;
  coil_freq_alpha = (COIL_FREQ_TAU_S > 0.0) ? 1.0 - exp(-COIL_BLOCK_S / COIL_FREQ_TAU_S) : 1.0;
  // счётчик тактов ядра для замера времени расчёта
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  // ADC2 — ведомый, его запускает ADC1. Порядок: ADC2, затем ADC1 с DMA, затем таймер
  // TIM2 = COIL_FS_HZ × COIL_OS. Таймеры APB1 тактируются удвоенной PCLK1, когда делитель APB1 не 1
  __HAL_TIM_SET_AUTORELOAD(&htim2, (2U * HAL_RCC_GetPCLK1Freq()) / (COIL_FS_HZ * COIL_OS) - 1U);
  (void)HAL_ADC_Start(&hadc2);
  (void)HAL_ADCEx_MultiModeStart_DMA(&hadc1, coil_dma, COIL_DMA_WORDS);
  (void)HAL_TIM_Base_Start(&htim2);

  for (;;)
  {
    pending = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100U));
    if (pending == 0U)
    {
      // за 100 мс ни одного блока: DMA или таймер стоят
      continue;
    }
    started = DWT->CYCCNT;
    end = coil_head;
    if (pending > 1U)
    {
      snap.missed += pending - 1U;
      snap.missed_at = snap.blocks + pending;
    }
    if (end - coil_done > COIL_RING / 2U)
    {
      // отстали больше чем на полкольца — детектор начинает заново
      coil_done = end - COIL_BLOCK;
      coil_level_ready = 0U;
      coil_cross_count = 0U;
    }
    Coil_TrackCrossings(coil_done, end);
    coil_done = end;
    Coil_Calc(end, pending, &snap);
    calc_us = (DWT->CYCCNT - started) / (SystemCoreClock / 1000000U);
    snap.blocks += pending;
    snap.calc_us_last = calc_us;
    if (calc_us > snap.calc_us_max)
    {
      snap.calc_us_max = calc_us;
    }
    taskENTER_CRITICAL();
    coil_snap = snap;
    taskEXIT_CRITICAL();
  }
}

// Число с фиксированной точкой: printf из newlib-nano не печатает %f. sign — всегда ставить знак
static void Coil_Fixed(char *out, size_t size, double value, uint32_t decimals, uint8_t sign)
{
  uint32_t scale = 1U;
  uint32_t index;
  uint32_t units;

  for (index = 0U; index < decimals; index++)
  {
    scale *= 10U;
  }
  units = (uint32_t)(fabs(value) * (double)scale + 0.5);
  (void)snprintf(out, size, "%s%lu.%0*lu", (value < 0.0) ? "-" : ((sign != 0U) ? "+" : ""),
                 units / scale, (int)decimals, units % scale);
}

// Печать снимка раз в 500 мс — только для человека, на расчёт не влияет:
// Δt в мкс (демпф., ноль вычтен), расход в кг/мин, частота по набегу фазы
static void Coil_PrintTask(void *argument)
{
  CoilSnapshot snap;
  uint32_t wake;
  char dt[16];
  char flow[16];
  char freq[16];

  (void)argument;
  Coil_Send(snprintf(coil_text, sizeof(coil_text),
                     "PWM_Kor: K1=PC1 K2=PC2, dt us (damped %lu ms, zero %ld ns), Q %s (K %lu.%03lu kg/min per us), f Hz (phase lock)\r\n",
                     (uint32_t)(COIL_TAU_S * 1000.0), (int32_t)(COIL_ZERO_US * 1000.0), COIL_FLOW_UNIT_NAME,
                     (uint32_t)(COIL_K_KGMIN_PER_US * 1000.0 + 0.5) / 1000U, (uint32_t)(COIL_K_KGMIN_PER_US * 1000.0 + 0.5) % 1000U));
  wake = osKernelGetTickCount();
  for (;;)
  {
    wake += COIL_PRINT_MS;
    (void)osDelayUntil(wake);
    taskENTER_CRITICAL();
    snap = coil_snap;
    taskEXIT_CRITICAL();
    if (snap.valid == 0U)
    {
      Coil_Send(snprintf(coil_text, sizeof(coil_text),
                         "dt=--- us  Q=0.000 %s  f=--- Hz  | no signal, K1 swing %lu mV\r\n",
                         COIL_FLOW_UNIT_NAME, (uint32_t)lroundf(snap.swing_mv)));
      continue;
    }
    Coil_Fixed(dt, sizeof(dt), snap.dt_shown_us, 4U, 1U);
    Coil_Fixed(flow, sizeof(flow), snap.flow_kgmin * COIL_FLOW_UNIT_FACTOR, 3U, 1U);
    Coil_Fixed(freq, sizeof(freq), snap.lock_hz, 3U, 0U);
    Coil_Send(snprintf(coil_text, sizeof(coil_text),
                       "dt=%s us  Q=%s %s  f=%s Hz  | A1=%lu A2=%lu mV snr=%ld/%ld dB%s\r\n",
                       dt, flow, COIL_FLOW_UNIT_NAME, freq,
                       (uint32_t)lroundf(snap.amp1_mv), (uint32_t)lroundf(snap.amp2_mv),
                       snap.snr1_db, snap.snr2_db,
                       (snap.clip1 + snap.clip2 != 0U) ? " CLIP" : ""));
  }
}
