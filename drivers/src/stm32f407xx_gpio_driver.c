#include "stm32f407xx_gpio_driver.h"

/* peripheral clock control: */
    
void enable_gpio_periph_clock(const uint8_t port_id)
{
    uint32_t ahb1enr = ahb1_bus_periph_base_addr + 0x3800;

    const uint8_t ascii_offset = 97;
    uint8_t offset = (ascii_offset + port_id) - ascii_offset;

    ahb1enr |= (1 << offset);
}

void disable_gpio_periph_clock(const uint8_t port_id)
{
    uint32_t ahb1enr = ahb1_bus_periph_base_addr + 0x3800;

    const uint8_t ascii_offset = 97;
    uint8_t offset = (ascii_offset + port_id) - ascii_offset;

     ahb1enr &= ~(1 << offset);
}

/*
struct gpio_port {
    uint8_t id;
    uint8_t base_addr;
    struct  gpio_reg reg;
};
*/

static uint32_t get_gpio_port_offset(const uint8_t id)
{
    uint32_t result = 0;

    switch(id) {
        case 'a':
            result = 0x0000;
            break;
        case 'b':
            result = 0x4000;
            break;
        default:
    }

    return result;
}

void gpio_port_init(struct gpio_port *p)
{
    uint32_t offset;

    offset = get_gpio_port_offset(p->id);
    p->base_addr = (ahb1_bus_periph_base_addr + offset);
}

/*
void gpio_pclck_control(struct gpio_reg *gpiox, uint8_t state)
{
    if(state == enable) {
        if(gpiox == GPIOA) {
            GPIOA_PCLK_ENABLE;
        } else if(gpiox == GPIOB) {
            GPIOB_PCLK_ENABLE;
        } else if(gpiox == GPIOC) {
            GPIOC_PCLK_ENABLE;
        } else if(gpiox == GPIOD) {
            GPIOD_PCLK_ENABLE;
        } else if(gpiox == GPIOE) {
            GPIOE_PCLK_ENABLE;
        } else if(gpiox == GPIOF) {
            GPIOF_PCLK_ENABLE;
        } else if(gpiox == GPIOG) {
            GPIOG_PCLK_ENABLE;
        } else if(gpiox == GPIOH) {
            GPIOH_PCLK_ENABLE;
        } else if(gpiox == GPIOI) {
            GPIOI_PCLK_ENABLE;
        }
    } else {
        if(gpiox == GPIOA) {
            GPIOA_PCLK_DISABLE;
        } else if(gpiox == GPIOB) {
            GPIOB_PCLK_DISABLE;
        } else if(gpiox == GPIOC) {
            GPIOC_PCLK_DISABLE;
        } else if(gpiox == GPIOD) {
            GPIOD_PCLK_DISABLE;
        } else if(gpiox == GPIOE) {
            GPIOE_PCLK_DISABLE;
        } else if(gpiox == GPIOF) {
            GPIOF_PCLK_DISABLE;
        } else if(gpiox == GPIOG) {
            GPIOG_PCLK_DISABLE;
        } else if(gpiox == GPIOH) {
            GPIOH_PCLK_DISABLE;
        } else if(gpiox == GPIOI) {
            GPIOI_PCLK_DISABLE;
        }
    }
}

*/

