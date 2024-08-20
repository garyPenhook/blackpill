//
// Created by garyPenhook on 8/17/24.
// Using westudio Black Pill STM32F411CEU6
//

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>

#define LED_PORT GPIOC
#define LED_PIN GPIO13

void setup_timer(void);
void setup_gpio(void);

/**
 * Timer 2 Interrupt Service Routine (ISR)
 * This ISR is triggered when the timer update interrupt flag is set.
 * It toggles the LED connected to GPIO13 on port GPIOC.
 */
void tim2_isr(void) {
    // Check if the update interrupt flag is set
    if (timer_get_flag(TIM2, TIM_SR_UIF)) {
        // Clear the update interrupt flag
        timer_clear_flag(TIM2, TIM_SR_UIF);

        // Toggle the LED
        gpio_toggle(LED_PORT, LED_PIN);
    }
}

/**
 * Main function
 * Initializes the GPIO and timer, then enters an infinite loop.
 */
int main(void) {
    // Set up the GPIO for the LED
    setup_gpio();

    // Set up the timer
    setup_timer();

    // Enable the timer interrupt
    nvic_enable_irq(NVIC_TIM2_IRQ);

    // Start the timer
    timer_enable_counter(TIM2);

    // Infinite loop
    while (1) {
        // Do nothing, wait for the timer interrupt to toggle the LED
    }

    return 0;
}

/**
 * Set up the GPIO for the LED
 * Configures GPIO13 on port GPIOC as an output.
 */
void setup_gpio(void) {
    // Enable the GPIOC clock
    rcc_periph_clock_enable(RCC_GPIOC);

    // Set GPIO13 as an output
    gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN);
}

/**
 * Set up the timer
 * Configures Timer 2 to generate an interrupt every second.
 */
void setup_timer(void) {
    // Enable the Timer 2 clock
    rcc_periph_clock_enable(RCC_TIM2);

    // Reset the timer
    timer_reset(TIM2);

    // Set the timer prescaler and period for 1 Hz blinking (assuming 100 MHz system clock)
    timer_set_prescaler(TIM2, 10000 - 1);
    timer_set_period(TIM2, 10000 - 1);

    // Enable the update interrupt
    timer_enable_irq(TIM2, TIM_DIER_UIE);
}