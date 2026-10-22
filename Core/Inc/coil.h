#ifndef COIL_H
#define COIL_H

#include <stdint.h>

// Создаёт задачу измерения: запуск TIM2 + ADC1/ADC2 по DMA, расчёт Δt, вывод строк в COM
void Coil_Init(void);

#endif
