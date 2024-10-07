#ifndef STM32F407XX_GPIO_DRIVER_H
#define STM32F407XX_GPIO_DRIVER_H

#include "stm32f407xx.h"

struct gpio_pin_config_t {
    uint8_t gpio_pin_number;
    uint8_t gpio_pin_mode;
    uint8_t gpio_pin_speed;
    uint8_t gpio_pin_pu_pd_control;
    uint8_t gpio_pin_opt_type;
    uint8_t gpio_pin_alt_func_mode;
};

struct gpio_handle_t {
    struct gpio_reg_t           *gpiox;          /* for the base adress */
    struct gpio_pin_config_t    gpio_pin_config;   
};

void gpio_init(struct gpio_handle_t *gpio_handle);
void gpio_deinit(struct gpio_reg_t *gpiox);

/* peripheral clock control: */
void gpio_pclck_control(struct gpio_reg_t *gpiox, uint8_t state);

uint8_t gpio_read_input_pin(struct gpio_reg_t *gpiox, uint8_t pin_number);
uint16_t gpio_read_input_port(struct gpio_reg_t *gpiox);

void gpio_write_output_pin(struct gpio_reg_t *gpiox, uint8_t pin_number, uint8_t value);
void gpio_write_output_port(struct gpio_reg_t *gpiox, uint16_t value);

void gpio_toggle_output_pin(struct gpio_reg_t *gpiox, uint8_t pin_number);

void gpio_irq_config(uint8_t irq_number, uint8_t irq_priority, uint8_t state);
void gpio_irq_handling(uint8_t pin_number);

#endif
