//
// Created by avr1 on 8/17/24.
//

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>

#define LED_PORT GPIOC
#define LED_PIN GPIO13


void setup_timer(void);
void setup_gpio(void);

void tim2_isr(void) {
    // Check if the update interrupt flag is set
    if (timer_get_flag(TIM2, TIM_SR_UIF)) {
        // Clear the update interrupt flag
        timer_clear_flag(TIM2, TIM_SR_UIF);

        // Toggle the LED
        gpio_toggle(LED_PORT, LED_PIN);
    }
}

void setup_timer(void) {
    // Enable TIM2 clock
    rcc_periph_clock_enable(RCC_TIM2);

    // Set up timer: Prescaler and period for 1 Hz blinking (assuming 100 MHz system clock)
    timer_set_prescaler(TIM2, 10000 - 1); // Prescaler: 10000
    timer_set_period(TIM2, 10000 - 1);    // Period: 10000

    // Enable update interrupt
    timer_enable_irq(TIM2, TIM_DIER_UIE);

    // Enable timer
    timer_enable_counter(TIM2);

    // Enable TIM2 interrupt in NVIC
    nvic_enable_irq(NVIC_TIM2_IRQ);
}

void setup_gpio(void) {
    // Enable GPIOC clock
    rcc_periph_clock_enable(RCC_GPIOC);

    // Set GPIOC Pin 13 as output
    gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN);
}

int main(void) {
    // Setup GPIO and Timer
    setup_gpio();
    setup_timer();

    // Main loop
    while (1) {
        // Do nothing, everything is handled in the ISR
    }


}
