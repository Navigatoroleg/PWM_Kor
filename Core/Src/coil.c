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

#if (COIL_FILTER != COIL_FILTER_NONE) && (COIL_FILTER != COIL_FILTER_WINDOW) && (COIL_FILTER != COIL_FILTER_DAMPING)
#error "COIL_FILTER: COIL_FILTER_NONE, COIL_FILTER_WINDOW или COIL_FILTER_DAMPING"
#endif

// Длительности фильтров, которые реально работают: невыбранный фильтр — 0, то есть выключен
#define COIL_AVG_USED_S     ((COIL_FILTER == COIL_FILTER_WINDOW) ? (double)COIL_AVG_S : 0.0)
#define COIL_TAU_USED_S     ((COIL_FILTER == COIL_FILTER_DAMPING) ? (double)COIL_TAU_S : 0.0)

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
  float    rms1_mv;       // действующее значение без постоянной составляющей, как «Амплитуда адаптера» Shtray Link
  float    rms2_mv;
  int32_t  snr1_db;
  int32_t  snr2_db;
  double   lock_hz;       // частота по набегу фазы, сглаженная
  double   dt_us;         // Δt последнего блока
  double   dt_damped_us;  // Δt после демпфирования
  double   dt_shown_us;   // после демпфирования минус ноль
  double   flow_kgmin;    // расход с отсечкой
  uint8_t  zero_state;    // COIL_ZERO_OFF / WAIT / ACC / DONE
  uint32_t zero_left_s;   // ожидание: сколько секунд осталось; накопление: сколько секунд накоплено
  uint32_t zero_count;    // сколько раз ноль записан — печать видит новый ноль по смене счётчика
  double   zero_us;       // действующий ноль
  double   zero_sd_us;    // разброс сырых Δt при накоплении нуля
  uint32_t zero_n;        // сколько замеров вошло в ноль
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

// Автообнуление: ожидание устоявшегося сигнала, потом среднее сырых Δt за COIL_ZERO_AVG_S.
// Время считается в блоках по 12,8 мс — так же строго, как демпфирование
#define COIL_ZERO_OFF       0U   // автообнуление выключено, ноль = COIL_ZERO_US
#define COIL_ZERO_WAIT      1U   // ждём COIL_ZERO_DELAY_S нормального сигнала
#define COIL_ZERO_ACC       2U   // копим сырые Δt
#define COIL_ZERO_DONE      3U   // ноль записан
static uint8_t coil_zero_state = (COIL_ZERO_AVG_S > 0.0) ? COIL_ZERO_WAIT : COIL_ZERO_OFF;
static uint32_t coil_zero_blocks;   // блоков в текущем этапе (ожидание или накопление)
static double coil_zero_us = COIL_ZERO_US;
static double coil_zero_sd_us;
static uint32_t coil_zero_n;
static uint32_t coil_zero_count;
// Среднее и разброс накопления по Велфорду: без хранения самих замеров
static double coil_zero_mean;
static double coil_zero_m2;

// Скользящее среднее сырых Δt за COIL_AVG_S секунд: кольцо значений по блокам и их сумма.
// Буфер на 30 с (2400 блоков по 12,8 мс); COIL_AVG_S больше — обрезается до 30 с
#define COIL_AVG_MAX        2400U
static float coil_avg_buf[COIL_AVG_MAX];
static uint32_t coil_avg_len;       // размер окна в блоках, из COIL_AVG_S
static uint32_t coil_avg_head;      // куда писать следующее значение
static uint32_t coil_avg_count;     // сколько значений уже в окне (после запуска окно заполняется не сразу)
static double coil_avg_sum;         // сумма значений в окне

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

// Детектор переходов К1 через уровень — первая, грубая оценка частоты трубки.
// Вызывается на каждый блок DMA с новыми точками [from, to) — каждая точка К1 проходит здесь ровно один раз.
// Делает три вещи:
//   1) по новому блоку обновляет уровень (среднее) и полуразмах К1 — плавно, по блокам;
//   2) ищет переходы К1 через уровень снизу вверх с гистерезисом COIL_HYST_FRACTION от полуразмаха;
//   3) складывает позиции переходов в кольцо coil_cross из COIL_CROSS_KEEP последних.
// Позиция перехода — дробный номер отсчёта с запуска: точка пересечения найдена линейной интерполяцией
// между двумя соседними точками. По расстоянию между переходами Coil_Period считает точек на период,
// а из него — окно подгонки синуса. Точную частоту потом даёт набег фазы в Coil_Calc
static void Coil_TrackCrossings(uint32_t from, uint32_t to)
{
  float low = 1e9f;     // минимум К1 в новом блоке, коды АЦП
  float high = -1e9f;   // максимум К1 в новом блоке
  float mean = 0.0f;    // среднее К1 в новом блоке
  float cur;            // текущая точка К1
  float hyst;           // гистерезис: на сколько К1 должен уйти ниже уровня, чтобы следующий переход засчитался
  uint32_t n;           // номер точки с запуска; в кольце — младшие биты (n & COIL_RING_MASK)

  // --- 1. Уровень и размах К1 по новому блоку ---------------------------------------------------
  // Блок — 64 точки, около 1,5 периода трубки при 120 Гц: минимум и максимум в нём попадают на пики
  for (n = from; n < to; n++)                       // перебор новых точек блока
  {
    cur = (float)coil_k1[n & COIL_RING_MASK];       // точка К1 из кольца (накопленная сумма 16 отсчётов)
    mean += cur;                                    // сумма для среднего
    if (cur < low) { low = cur; }                   // минимум блока
    if (cur > high) { high = cur; }                 // максимум блока
  }
  mean /= (float)(to - from);                       // среднее К1 по блоку

  if (coil_level_ready == 0U)
  {
    // Первый блок после запуска или после потери сигнала: берём значения блока как есть
    coil_level = mean;                              // уровень — среднее первого блока
    coil_half = (high - low) * 0.5f;                // полуразмах — половина размаха первого блока
    // Предыдущая точка для поиска перехода — первая точка блока: на ней самой перехода не будет
    coil_prev = (float)coil_k1[from & COIL_RING_MASK];
    coil_level_ready = 1U;                          // дальше — только сглаживание
  }
  else
  {
    // Дальше — экспоненциальное сглаживание по блокам с коэффициентом COIL_LEVEL_ALPHA = 0,2:
    // постоянная времени около 5 блоков (≈ 60 мс). Уровень не прыгает от шума одного блока,
    // но успевает за медленным уходом смещения и изменением амплитуды
    coil_level += COIL_LEVEL_ALPHA * (mean - coil_level);                 // уровень ← 20 % нового среднего
    coil_half += COIL_LEVEL_ALPHA * ((high - low) * 0.5f - coil_half);    // полуразмах ← 20 % нового полуразмаха
  }
  // Гистерезис — доля полуразмаха: при сигнале 470 мВ и доле 0,2 это около 94 мВ.
  // Шум и иглы меньше этого не дают ложных переходов
  hyst = COIL_HYST_FRACTION * coil_half;            // гистерезис в кодах АЦП

  // --- 2. Переходы через уровень снизу вверх ------------------------------------------------------
  // coil_prev и coil_armed живут между вызовами: переход на стыке двух блоков тоже не теряется
  for (n = from; n < to; n++)                       // второй проход по тем же новым точкам
  {
    cur = (float)coil_k1[n & COIL_RING_MASK];       // точка К1

    // «Взвод»: сигнал ушёл ниже уровня на величину гистерезиса — следующий подъём через уровень
    // будет настоящим переходом, а не дребезгом шума около уровня
    if (coil_prev < coil_level - hyst)
    {
      coil_armed = 1U;                              // детектор взведён
    }

    // Переход засчитан: детектор взведён, предыдущая точка ниже уровня, текущая — на уровне или выше
    if ((coil_armed != 0U) && (coil_prev < coil_level) && (cur >= coil_level))
    {
      // Точное место пересечения между точками n−1 и n — линейной интерполяцией:
      //   доля = (уровень − prev) / (cur − prev), от 0 до 1;
      //   позиция = (n − 1) + доля, в отсчётах с запуска.
      // Так период меряется точнее одного отсчёта (200 мкс при 5 кГц).
      // double — потому что номер отсчёта растёт без ограничения, а float держит только 7 значащих цифр
      coil_cross[coil_cross_count % COIL_CROSS_KEEP] =
        (double)(n - 1U) + (double)((coil_level - coil_prev) / (cur - coil_prev));
      // Счётчик всех переходов с запуска; в кольце из COIL_CROSS_KEEP хранятся только последние
      coil_cross_count++;
      // Сброс взвода: следующий переход засчитается, только когда сигнал снова опустится ниже уровня − гистерезис
      coil_armed = 0U;
    }

    // Текущая точка становится предыдущей — и для следующей точки, и для первой точки следующего блока
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
  // окно среднего — с нуля: старые Δt относятся к сигналу до пропажи
  coil_avg_count = 0U;
  coil_avg_head = 0U;
  coil_avg_sum = 0.0;
  // обнуление не закончено — начнётся заново, когда сигнал вернётся
  if ((coil_zero_state == COIL_ZERO_WAIT) || (coil_zero_state == COIL_ZERO_ACC))
  {
    coil_zero_state = COIL_ZERO_WAIT;
    coil_zero_blocks = 0U;
  }
}

// Скользящее среднее: новое Δt в окно, самое старое — из окна, возвращает среднее окна.
// steps > 1 — пропущенные блоки: значение кладётся steps раз, чтобы окно оставалось в секундах.
// COIL_AVG_S = 0 — без усреднения, Δt возвращается как есть
static double Coil_Average(double dt_us, uint32_t steps)
{
  uint32_t index;

  if (coil_avg_len <= 1U)
  {
    return dt_us;
  }
  for (index = 0U; index < steps; index++)
  {
    if (coil_avg_count == coil_avg_len)
    {
      coil_avg_sum -= coil_avg_buf[coil_avg_head];   // окно полное — самое старое уходит
    }
    else
    {
      coil_avg_count++;                              // окно ещё заполняется
    }
    coil_avg_buf[coil_avg_head] = (float)dt_us;
    coil_avg_sum += (float)dt_us;                    // в сумму — ровно то, что лежит в буфере
    coil_avg_head = (coil_avg_head + 1U) % coil_avg_len;
  }
  // раз в полный круг сумма пересчитывается заново — без накопления ошибки округления
  if (coil_avg_head == 0U)
  {
    coil_avg_sum = 0.0;
    for (index = 0U; index < coil_avg_count; index++)
    {
      coil_avg_sum += coil_avg_buf[index];
    }
  }
  return coil_avg_sum / (double)coil_avg_count;
}

// Автообнуление на один расчёт: сначала ждём устоявшегося сигнала, потом копим сырые Δt.
// steps — сколько блоков прошло (обычно 1): время этапов идёт в блоках, как и демпфирование
static void Coil_Zero(double dt_us, uint32_t steps)
{
  const uint32_t wait_blocks = (uint32_t)(COIL_ZERO_DELAY_S / COIL_BLOCK_S + 0.5);
  const uint32_t avg_blocks = (uint32_t)(COIL_ZERO_AVG_S / COIL_BLOCK_S + 0.5);
  double delta;

  if (coil_zero_state == COIL_ZERO_WAIT)
  {
    coil_zero_blocks += steps;
    if (coil_zero_blocks >= wait_blocks)
    {
      // ожидание прошло — начинаем копить с чистого листа
      coil_zero_state = COIL_ZERO_ACC;
      coil_zero_blocks = 0U;
      coil_zero_n = 0U;
      coil_zero_mean = 0.0;
      coil_zero_m2 = 0.0;
    }
    return;
  }
  if (coil_zero_state != COIL_ZERO_ACC)
  {
    return;
  }
  // Велфорд: среднее и сумма квадратов отклонений обновляются на каждом замере
  coil_zero_n++;
  delta = dt_us - coil_zero_mean;
  coil_zero_mean += delta / (double)coil_zero_n;
  coil_zero_m2 += delta * (dt_us - coil_zero_mean);
  coil_zero_blocks += steps;
  if (coil_zero_blocks >= avg_blocks)
  {
    // накоплено — среднее становится нулём
    coil_zero_us = coil_zero_mean;
    coil_zero_sd_us = (coil_zero_n > 1U) ? sqrt(coil_zero_m2 / (double)(coil_zero_n - 1U)) : 0.0;
    coil_zero_count++;
    coil_zero_state = COIL_ZERO_DONE;
  }
}

// Один расчёт на конец данных end. steps — сколько блоков прошло с прошлого расчёта (обычно 1)
static void Coil_Calc(uint32_t end, uint32_t steps, CoilSnapshot *snap)
{
  const double mv_per_code = COIL_VDDA_MV / (double)COIL_FULL;
  float period;
  uint32_t count;
  float omega;
  double freq, dphi, dt_us, avg_us, amp1, amp2, alpha, theta, lock, shown, flow;
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

  // скользящее среднее за COIL_AVG_S, затем демпфирование с шагом в блоках:
  // пропущенные блоки учитываются степенью
  avg_us = Coil_Average(dt_us, steps);
  if (coil_have_damped == 0U)
  {
    coil_damped_us = avg_us;
    coil_have_damped = 1U;
  }
  else
  {
    alpha = (steps == 1U) ? coil_alpha : 1.0 - pow(1.0 - coil_alpha, (double)steps);
    coil_damped_us += alpha * (avg_us - coil_damped_us);
  }
  // автообнуление копит сырые Δt — как кнопка «Обнулить» в Qt
  Coil_Zero(dt_us, steps);
  // расход по демпфированному Δt минус ноль, малый — в ноль
  shown = coil_damped_us - coil_zero_us;
  flow = (fabs(shown) < COIL_CUTOFF_US) ? 0.0 : COIL_K_KGMIN_PER_US * shown;
  snap->lock_hz = coil_lock_hz;
  snap->dt_shown_us = shown;
  snap->flow_kgmin = flow;

  snap->valid = 1U;
  snap->freq_hz = (float)freq;
  snap->amp1_mv = (float)(amp1 * mv_per_code);
  snap->amp2_mv = (float)(amp2 * mv_per_code);
  snap->rms1_mv = (float)(sqrt(f1.power) * mv_per_code);
  snap->rms2_mv = (float)(sqrt(f2.power) * mv_per_code);
  snap->snr1_db = snr1;
  snap->snr2_db = snr2;
  snap->dt_us = dt_us;
  snap->dt_damped_us = coil_damped_us;
  snap->fit_points = count;
}

// Расчёт на каждый блок DMA: будит прерывание, шаг 12,8 мс от кварца таймера
static void Coil_CalcTask(void *argument)
{
  CoilSnapshot snap;    // свой снимок результатов; наружу копируется целиком в конце каждого прохода
  uint32_t pending;     // сколько блоков DMA пришло с прошлого прохода (обычно 1)
  uint32_t end;         // номер точки, до которой есть данные (не включая её)
  uint32_t started;     // такты ядра в начале расчёта — для замера времени
  uint32_t calc_us;     // время расчёта этого прохода, мкс

  (void)argument;                                   // параметр задачи не используется
  memset(&snap, 0, sizeof(snap));                   // счётчики и результаты — с нуля

  // --- Подготовка: один раз при старте --------------------------------------------------------
  // Коэффициенты экспоненциального сглаживания на ОДИН блок (12,8 мс): α = 1 − e^(−T/τ).
  // Считаются один раз — дальше шаг демпфирования строго одинаковый. τ = 0 — без сглаживания: α = 1
  coil_alpha = (COIL_TAU_USED_S > 0.0) ? 1.0 - exp(-COIL_BLOCK_S / COIL_TAU_USED_S) : 1.0;    // для Δt
  coil_freq_alpha = (COIL_FREQ_TAU_S > 0.0) ? 1.0 - exp(-COIL_BLOCK_S / COIL_FREQ_TAU_S) : 1.0;
  // окно скользящего среднего в блоках: COIL_AVG_S / 12,8 мс, не больше буфера на 30 с
  coil_avg_len = (uint32_t)(COIL_AVG_USED_S / COIL_BLOCK_S + 0.5);
  if (coil_avg_len > COIL_AVG_MAX)
  {
    coil_avg_len = COIL_AVG_MAX;
  } // для частоты

  // Счётчик тактов ядра DWT CYCCNT: 168 МГц, по нему меряется время расчёта
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;   // разрешить блок трассировки, в нём DWT
  DWT->CYCCNT = 0U;                                 // обнулить счётчик тактов
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;              // запустить счётчик тактов

  // Запуск оцифровки. Порядок важен: ADC2 — ведомый, его запускает ADC1; ADC1 ждёт триггер от TIM2
  // TIM2 = COIL_FS_HZ × COIL_OS (80 кГц). Таймеры APB1 тактируются удвоенной PCLK1 (42 → 84 МГц), когда делитель APB1 не 1
  __HAL_TIM_SET_AUTORELOAD(&htim2, (2U * HAL_RCC_GetPCLK1Freq()) / (COIL_FS_HZ * COIL_OS) - 1U); // 84 МГц / 80 кГц − 1 = 1049
  (void)HAL_ADC_Start(&hadc2);                                          // включить ADC2, он ждёт ADC1
  (void)HAL_ADCEx_MultiModeStart_DMA(&hadc1, coil_dma, COIL_DMA_WORDS); // ADC1 + DMA в кольцо coil_dma, ждёт TIM2
  (void)HAL_TIM_Base_Start(&htim2);                                     // пошли запуски — данные текут сами

  // --- Основной цикл: один проход на каждый блок DMA (64 точки = 12,8 мс) -----------------------
  for (;;)
  {
    // Ждать уведомления из прерывания DMA (Coil_Accumulate → vTaskNotifyGiveFromISR).
    // Пока ждём — процессор свободен. pdTRUE: забрать все накопившиеся уведомления сразу и обнулить счётчик.
    // Возвращает их число: 1 — норма, больше 1 — расчёт не успел к прошлому блоку, 0 — тайм-аут 100 мс
    pending = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100U));
    if (pending == 0U)
    {
      // за 100 мс ни одного блока: стоят DMA или таймер. Просто ждём дальше
      continue;
    }
    started = DWT->CYCCNT;                          // засечка тактов: начало расчёта
    end = coil_head;                                // до какой точки дописал DMA; читается один раз — дальше работаем с этим значением
    if (pending > 1U)
    {
      snap.missed += pending - 1U;                  // сколько блоков пропущено (обработаются вместе с этим)
      snap.missed_at = snap.blocks + pending;       // номер блока, на котором отстали — для диагностики
    }
    if (end - coil_done > COIL_RING / 2U)
    {
      // Отстали больше чем на полкольца (0,8 с): старые точки уже перезаписаны DMA.
      // Детектор переходов начинает заново с последнего блока, чтобы не читать испорченные данные
      coil_done = end - COIL_BLOCK;                 // взять только последний блок
      coil_level_ready = 0U;                        // уровень и размах — заново
      coil_cross_count = 0U;                        // старые переходы забыть
    }
    Coil_TrackCrossings(coil_done, end);            // новые точки К1 → уровень, размах, переходы через ноль
    coil_done = end;                                // эти точки обработаны
    Coil_Calc(end, pending, &snap);                 // частота, подгонка синуса К1/К2, Δt, демпфирование, расход → snap
    snap.zero_state = coil_zero_state;              // этап автообнуления — для печати
    snap.zero_left_s = (coil_zero_state == COIL_ZERO_WAIT)
                       ? (uint32_t)(COIL_ZERO_DELAY_S - (double)coil_zero_blocks * COIL_BLOCK_S + 0.999)  // осталось ждать
                       : (uint32_t)((double)coil_zero_blocks * COIL_BLOCK_S);                           // уже накоплено
    snap.zero_count = coil_zero_count;
    snap.zero_us = coil_zero_us;
    snap.zero_sd_us = coil_zero_sd_us;
    snap.zero_n = coil_zero_n;
    calc_us = (DWT->CYCCNT - started) / (SystemCoreClock / 1000000U); // такты → мкс (168 тактов = 1 мкс)
    snap.blocks += pending;                         // счётчик обработанных блоков с запуска
    snap.calc_us_last = calc_us;                    // время этого расчёта
    if (calc_us > snap.calc_us_max)
    {
      snap.calc_us_max = calc_us;                   // худшее время с запуска; бюджет — 12 800 мкс на блок
    }
    // Отдать снимок задаче печати. Критическая секция — чтобы печать не прочитала снимок
    // наполовину старым, наполовину новым: копирование ~120 байт занимает микросекунды
    taskENTER_CRITICAL();                           // запретить прерывания и переключение задач
    coil_snap = snap;                               // скопировать весь снимок
    taskEXIT_CRITICAL();                            // разрешить обратно
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

// Пометка об автообнулении в конец строки: идёт ожидание или накопление
static void Coil_ZeroNote(char *out, size_t size, const CoilSnapshot *snap)
{
  if (snap->zero_state == COIL_ZERO_WAIT)
  {
    (void)snprintf(out, size, "  | zero: wait %lu s", snap->zero_left_s);
  }
  else if (snap->zero_state == COIL_ZERO_ACC)
  {
    (void)snprintf(out, size, "  | zero: %lu/%lu s", snap->zero_left_s, (uint32_t)(COIL_ZERO_AVG_S + 0.5));
  }
  else
  {
    out[0] = '\0';
  }
}

// Печать снимка раз в 500 мс — только для человека, на расчёт не влияет:
// Δt в мкс (демпф., ноль вычтен), расход в кг/мин, частота по набегу фазы
static void Coil_PrintTask(void *argument)
{
  CoilSnapshot snap;
  uint32_t wake;
  uint32_t zero_printed = 0U;   // сколько нулей уже напечатано — новый печатается один раз
  char dt[16];
  char flow[16];
  char freq[16];
  char sd[16];
  char note[32];

  (void)argument;
  Coil_Fixed(dt, sizeof(dt), COIL_ZERO_US, 4U, 1U);
  Coil_Send(snprintf(coil_text, sizeof(coil_text),
                     "PWM_Kor: K1=PC1 K2=PC2, dt us (average %lu ms, damped %lu ms), Q %s (K %lu.%03lu kg/min per us), f Hz, A peak/rms mV\r\n",
                     (uint32_t)(COIL_AVG_USED_S * 1000.0), (uint32_t)(COIL_TAU_USED_S * 1000.0), COIL_FLOW_UNIT_NAME,
                     (uint32_t)(COIL_K_KGMIN_PER_US * 1000.0 + 0.5) / 1000U, (uint32_t)(COIL_K_KGMIN_PER_US * 1000.0 + 0.5) % 1000U));
  Coil_Send(snprintf(coil_text, sizeof(coil_text),
                     "zero %s us, auto zero: wait %lu s, average %lu s (0 - off)\r\n",
                     dt, (uint32_t)(COIL_ZERO_DELAY_S + 0.5), (uint32_t)(COIL_ZERO_AVG_S + 0.5)));
  wake = osKernelGetTickCount();
  for (;;)
  {
    wake += COIL_PRINT_MS;
    (void)osDelayUntil(wake);
    taskENTER_CRITICAL();
    snap = coil_snap;
    taskEXIT_CRITICAL();

    // ноль только что записан — одна отдельная строка с его значением и разбросом
    if (snap.zero_count != zero_printed)
    {
      zero_printed = snap.zero_count;
      Coil_Fixed(dt, sizeof(dt), snap.zero_us, 4U, 1U);
      Coil_Fixed(sd, sizeof(sd), snap.zero_sd_us, 4U, 0U);
      Coil_Send(snprintf(coil_text, sizeof(coil_text),
                         "ZERO = %s us (sd %s us, n=%lu)\r\n", dt, sd, snap.zero_n));
      continue;
    }

    Coil_ZeroNote(note, sizeof(note), &snap);
    if (snap.valid == 0U)
    {
      Coil_Send(snprintf(coil_text, sizeof(coil_text),
                         "dt=--- us  Q=0.000 %s  f=--- Hz  | no signal, K1 swing %lu mV%s\r\n",
                         COIL_FLOW_UNIT_NAME, (uint32_t)lroundf(snap.swing_mv), note));
      continue;
    }
    Coil_Fixed(dt, sizeof(dt), snap.dt_shown_us, 4U, 1U);
    Coil_Fixed(flow, sizeof(flow), snap.flow_kgmin * COIL_FLOW_UNIT_FACTOR, 3U, 1U);
    Coil_Fixed(freq, sizeof(freq), snap.lock_hz, 3U, 0U);
    Coil_Send(snprintf(coil_text, sizeof(coil_text),
                       "dt=%s us  Q=%s %s  f=%s Hz  | A1=%lu/%lu A2=%lu/%lu mV%s%s\r\n",
                       dt, flow, COIL_FLOW_UNIT_NAME, freq,
                       (uint32_t)lroundf(snap.amp1_mv), (uint32_t)lroundf(snap.rms1_mv),
                       (uint32_t)lroundf(snap.amp2_mv), (uint32_t)lroundf(snap.rms2_mv),
                       (snap.clip1 + snap.clip2 != 0U) ? " CLIP" : "", note));
  }
}
