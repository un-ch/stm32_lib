#include <unistd.h>

#include "temp.h"


void delay(void)
{
    for(volatile uint32_t i = 0; i < 500000; i++);
}

int main(void)
{
    GPIO_Handle_t GpioLed;

    GpioLed.pGPIOx = GPIOD;
    GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PUPD;

    GPIO_PeriClockControl(GPIOA, ENABLE);

    GPIO_Init(&GpioLed);

    while(1)
    {
        GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_13);
        delay();
    }
    
    return  0;
}




    /*

static void delay(void)
{
    for(volatile uint32_t i = 0; i < 20000000; i++);
}

int main(void)
{
    struct gpio_reg     *gpiod;
    struct rcc_reg      *rcc;
    uint32_t            offset =  0x0C00;
    uint32_t            pin_num = 12;

    gpiod = (struct gpio_reg  * const) ahb1_bus_periph_base_addr + offset; 

    offset = 0x3800;
    rcc = (struct rcc_reg  * const) ahb1_bus_periph_base_addr + offset; 
    rcc->ahb1enr |= (1 << 0);

    
    gpiod->moder = gpio_mode_out;
    gpiod->otyper &= ~( 1 << pin_num);

    for(;;) {
        gpiod->odr ^= (1 << pin_num);
        delay();
    }
    return exit_success_code;
}
    */
