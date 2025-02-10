#ifndef STM32F407XX_GPIO_DRIVER_H
#define STM32F407XX_GPIO_DRIVER_H

#include "stm32f407xx.h"

struct gpio_pin_config {
    uint8_t number;
    uint8_t mode;
    uint8_t speed;
    uint8_t pu_pd_control;
    uint8_t optup_type;
    uint8_t pin_alt_func_mode;
};

struct gpio_handle {
    struct reg                  *gpiox;          /* for the base adress */
    struct gpio_pin_config      pin_config;   
};

enum {
    gpio_mode_in                    = 0,
    gpio_mode_out                   = 1,
    gpio_mode_alt                   = 2,
    gpio_mode_analog                = 3,
    gpio_mode_input_fal_edge        = 4,
    gpio_mode_input_ris_edge        = 5,
    gpio_mode_input_edge_trig       = 6,
    gpio_otyper_pp                  = 0,
    gpio_otyper_od                  = 1,
    gpio_ospeedr_low                = 0,
    gpio_ospeedr_medium             = 1,
    gpio_ospeedr_high               = 2,
    gpio_ospeedr_very_high          = 3,
    gpio_pupdr_no_pupd              = 0,
    gpio_pupdr_pu                   = 1,
    gpio_pupdr_pd                   = 2
};

enum {
    gpio_pin_0                      = 0,
    gpio_pin_01                     = 1,
    gpio_pin_02                     = 2,
    gpio_pin_03                     = 3,
    gpio_pin_04                     = 4
};

void enable_gpio_periph_clock(const uint8_t port_id);
void disable_gpio_periph_clock(const uint8_t port_id);
void gpio_port_init(struct gpio_port *p);








/* peripheral clock control: */
/*

void gpio_init(struct gpio_handle *g);
void gpio_deinit(struct gpio_reg *gpiox);

void gpio_pclck_control(struct gpio_reg *gpiox, uint8_t state);

uint8_t gpio_read_input_pin(struct gpio_reg *gpiox, uint8_t pin_number);
uint16_t gpio_read_input_port(struct gpio_reg *gpiox);

void gpio_write_output_pin(struct gpio_reg *gpiox, uint8_t pin_number, uint8_t value);
void gpio_write_output_port(struct gpio_reg *gpiox, uint16_t value);

void gpio_toggle_output_pin(struct gpio_reg *gpiox, uint8_t pin_number);

void gpio_irq_config(uint8_t irq_number, uint8_t irq_priority, uint8_t state);
void gpio_irq_handling(uint8_t pin_number);
*/

#endif
