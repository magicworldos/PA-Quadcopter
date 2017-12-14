#ifndef __PWM_IN
#define __PWM_IN

#include <typedef.h>

void pwm_in_init(void);

void tim2_irq_init(void);

void tim3_irq_init(void);

void tim2_gpio_init(void);

void tim3_gpio_init(void);

void tim2_capture_init(void);

void tim3_capture_init(void);

#endif
