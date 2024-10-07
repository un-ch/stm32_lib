#ifndef STM32F407XX_H
#define STM32F407XX_H

#include <stdint.h>     /* for uint32_t */

enum {
    exit_success_code   = 0,
    exit_failure_code   = 1,

    disable     = 0,
    enable      = 1,

    set         = 0,
    reset       = 1
};

#define FLASH_BASE_ADDR             0x08000000U
#define SRAM1_BASE_ADDR             0x20000000U
#define SRAM2_BASE_ADDR             0x2001C000U
#define ROM_BASE_ADDR               0x1FFF0000U
#define SRAM                        SRAM1_BASE_ADDR

#define PERIPH_BASE_ADDR            0x40000000U
#define APB1_BUS_PERIPH_BASE_ADDR   0x40000000U
#define APB2_BUS_PERIPH_BASE_ADDR   0x40010000U
#define AHB1_BUS_PERIPH_BASE_ADDR   0x40020000U
#define AHB2_BUS_PERIPH_BASE_ADDR   0x50000000U

#define GPIOA_BASE_ADDR             (AHB1_BUS_PERIPH_BASE_ADDR + 0x0000)
#define GPIOB_BASE_ADDR             (AHB1_BUS_PERIPH_BASE_ADDR + 0x4000)
#define GPIOC_BASE_ADDR             (AHB1_BUS_PERIPH_BASE_ADDR + 0x8000)
#define GPIOD_BASE_ADDR             (AHB1_BUS_PERIPH_BASE_ADDR + 0xC000)
#define GPIOE_BASE_ADDR             (AHB1_BUS_PERIPH_BASE_ADDR + 0x1000)
#define GPIOF_BASE_ADDR             (AHB1_BUS_PERIPH_BASE_ADDR + 0x1400)
#define GPIOG_BASE_ADDR             (AHB1_BUS_PERIPH_BASE_ADDR + 0x1800)
#define GPIOH_BASE_ADDR             (AHB1_BUS_PERIPH_BASE_ADDR + 0x1C00)
#define GPIOI_BASE_ADDR             (AHB1_BUS_PERIPH_BASE_ADDR + 0x2000)
#define GPIOJ_BASE_ADDR             (AHB1_BUS_PERIPH_BASE_ADDR + 0x2400)
#define GPIOK_BASE_ADDR             (AHB1_BUS_PERIPH_BASE_ADDR + 0x2800)

#define RCC_BASE_ADDR               (AHB1_BUS_PERIPH_BASE_ADDR + 0x3800)

#define I2C1_BASE_ADDR              (APB1_BUS_PERIPH_BASE_ADDR + 0x5400)
#define I2C2_BASE_ADDR              (APB1_BUS_PERIPH_BASE_ADDR + 0x5800)
#define I2C3_BASE_ADDR              (APB1_BUS_PERIPH_BASE_ADDR + 0x5C00)

#define SPI1_BASE_ADDR              (APB2_BUS_PERIPH_BASE_ADDR + 0x3000)
#define SPI2_BASE_ADDR              (APB1_BUS_PERIPH_BASE_ADDR + 0x3800)
#define SPI3_BASE_ADDR              (APB1_BUS_PERIPH_BASE_ADDR + 0x3C00)

#define USART1_BASE_ADDR            (APB2_BUS_PERIPH_BASE_ADDR + 0x1000)
#define USART2_BASE_ADDR            (APB1_BUS_PERIPH_BASE_ADDR + 0x4400)
#define USART3_BASE_ADDR            (APB1_BUS_PERIPH_BASE_ADDR + 0x4800)
#define USART6_BASE_ADDR            (APB2_BUS_PERIPH_BASE_ADDR + 0x1400)

#define UART4_BASE_ADDR             (APB1_BUS_PERIPH_BASE_ADDR + 0x4C00)
#define UART5_BASE_ADDR             (APB1_BUS_PERIPH_BASE_ADDR + 0x5000)

#define SPI1_BASE_ADDR              (APB2_BUS_PERIPH_BASE_ADDR + 0x3000)
#define SYSCFG_BASE_ADDR            (APB2_BUS_PERIPH_BASE_ADDR + 0x3800)
#define EXTI_BASE_ADDR              (APB2_BUS_PERIPH_BASE_ADDR + 0x3C00)

#define SPI_CR1                     (SPI1_BASE_ADDR + 0x0000)
#define SPI_CR2                     (SPI1_BASE_ADDR + 0x0004)
#define SPI_SR                      (SPI1_BASE_ADDR + 0x0008)
#define SPI_DR                      (SPI1_BASE_ADDR + 0x000C)
#define SPI_CRCPR                   (SPI1_BASE_ADDR + 0x0010)
#define SPI_RXCRCR                  (SPI1_BASE_ADDR + 0x0014)
#define SPI_TXCRCR                  (SPI1_BASE_ADDR + 0x0018)
#define SPI_I2SCFGR                 (SPI1_BASE_ADDR + 0x001C)
#define SPI_I2SPR                   (SPI1_BASE_ADDR + 0x0020)

#define GPIOA                       ((GPIO_Reg_t *)GPIOA_BASE_ADDR)
#define GPIOB                       ((GPIO_Reg_t *)GPIOB_BASE_ADDR)
#define GPIOC                       ((GPIO_Reg_t *)GPIOC_BASE_ADDR)
#define GPIOD                       ((GPIO_Reg_t *)GPIOD_BASE_ADDR)
#define GPIOE                       ((GPIO_Reg_t *)GPIOE_BASE_ADDR)
#define GPIOF                       ((GPIO_Reg_t *)GPIOF_BASE_ADDR)
#define GPIOG                       ((GPIO_Reg_t *)GPIOG_BASE_ADDR)
#define GPIOH                       ((GPIO_Reg_t *)GPIOH_BASE_ADDR)
#define GPIOI                       ((GPIO_Reg_t *)GPIOI_BASE_ADDR)
#define GPIOJ                       ((GPIO_Reg_t *)GPIOJ_BASE_ADDR)
#define GPIOK                       ((GPIO_Reg_t *)GPIOK_BASE_ADDR)

#define RCC                         ((RCC_Reg_t *)RCC_BASE_ADDR)

#define GPIOA_PCLK_ENABLE           ( RCC->ahb1enr |= ( 1 << 0 ))
#define GPIOB_PCLK_ENABLE           ( RCC->ahb1enr |= ( 1 << 1 ))

#define GPIOA_PCLK_DISABLE          ( RCC->ahb1enr &= ~( 1 << 0 ))
#define GPIOB_PCLK_DISABLE          ( RCC->ahb1enr &= ~( 1 << 1 ))

#define SPI1_PCLK_ENABLE            ( RCC->apb2enr |= ( 1 << 12 ))
#define SPI2_PCLK_ENABLE            ( RCC->apb1enr |= ( 1 << 14 ))
#define SPI3_PCLK_ENABLE            ( RCC->apb1enr |= ( 1 << 15 ))

#define SPI1_PCLK_DISABLE           ( RCC->apb2enr &= ~( 1 << 12 ))
#define SPI2_PCLK_DISABLE           ( RCC->apb1enr &= ~( 1 << 14 ))
#define SPI3_PCLK_DISABLE           ( RCC->apb1enr &= ~( 1 << 15 ))

#define I2C1_PCLK_ENABLE            ( RCC->apb1enr |= ( 1 << 21 ))
#define I2C2_PCLK_ENABLE            ( RCC->apb1enr |= ( 1 << 22 ))
#define I2C3_PCLK_ENABLE            ( RCC->apb1enr |= ( 1 << 23 ))

#define I2C1_PCLK_DISABLE           ( RCC->apb1enr &= ~( 1 << 21 ))
#define I2C2_PCLK_DISABLE           ( RCC->apb1enr &= ~( 1 << 22 ))
#define I2C3_PCLK_DISABLE           ( RCC->apb1enr &= ~( 1 << 23 ))

#define USART1_PCLK_ENABLE          ( RCC->apb2enr |= ( 1 << 4 ))
#define USART2_PCLK_ENABLE          ( RCC->apb1enr |= ( 1 << 17 ))
#define USART3_PCLK_ENABLE          ( RCC->apb2enr |= ( 1 << 18 ))
#define USART6_PCLK_ENABLE          ( RCC->apb2enr |= ( 1 << 5 ))

#define USART1_PCLK_DISABLE         ( RCC->apb2enr &= ~( 1 << 4 ))
#define USART2_PCLK_DISABLE         ( RCC->apb1enr &= ~( 1 << 17 ))
#define USART3_PCLK_DISABLE         ( RCC->apb2enr &= ~( 1 << 18 ))
#define USART6_PCLK_DISABLE         ( RCC->apb2enr &= ~( 1 << 5 ))

#define SYSCFG_PCLK_ENABLE          ( RCC->apb2enr |= ( 1 << 14 ))
#define SYSCFG_PCLK_DISABLE         ( RCC->apb2enr &= ~( 1 << 14 ))

struct gpio_reg_t {
    volatile uint32_t moder;        /* mode register                    */
    volatile uint32_t otyper;       /* output type register             */
    volatile uint32_t speedr;       /* output speed register            */
    volatile uint32_t puprd;        /* pull-up/pull-down register       */
    volatile uint32_t idr;          /* input data register              */
    volatile uint32_t odr;          /* output data register             */
    volatile uint32_t bsrr;         /* bit set/reset register           */
    volatile uint32_t lckr;         /* configuration lock register      */
    volatile uint32_t afrl;         /* alternate function low register  */
    volatile uint32_t afrh;         /* alternate function high register */
};

struct rcc_reg_t {
    volatile uint32_t cr;          /* RCC clock control register            */
    volatile uint32_t pllcfgr;     /* PLL configuration register            */
    volatile uint32_t cfgr;        /* RCC clock configuration register      */
    volatile uint32_t cir;         /* RCC clock  interrupt register         */
    volatile uint32_t ahb1rstr;    /* RCC AHB1 peripheral reset register    */
    volatile uint32_t ahb2rstr;    /* RCC AHB2 peripheral reset register    */
    volatile uint32_t ahb3rstr;    /* RCC AHB3 peripheral reset register    */
    uint32_t reserved01;
    volatile uint32_t apb1rstr;    /* RCC APB1 peripheral reset register    */
    volatile uint32_t apb2rstr;    /* RCC APB2 peripheral reset register    */
    uint32_t reserved02;
    uint32_t reserved03;
    volatile uint32_t ahb1enr;    /* RCC AHB1 peripheral clock enable register */
    volatile uint32_t ahb2enr;    /* RCC AHB2 peripheral clock enable register */
    volatile uint32_t ahb3enr;    /* RCC AHB3 peripheral clock enable register */
    uint32_t reserved04;
    volatile uint32_t apb1enr;    /* RCC APB1 peripheral clock enable register */
    volatile uint32_t apb2enr;    /* RCC APB2 peripheral clock enable register */
    uint32_t reserved05;
    uint32_t reserved06;
    volatile uint32_t ahb1lpenr;    /* RCC AHB1 peripheral clock enable in low power mode register */
    volatile uint32_t ahb2lpenr;    /* RCC AHB2 peripheral clock enable in low power mode register */
    volatile uint32_t ahb3lpenr;    /* RCC AHB3 peripheral clock enable in low power mode register */
    uint32_t reserved07;
    volatile uint32_t apb1lpenr;    /* RCC APB1 peripheral clock enable in low power mode register */
    volatile uint32_t apb2lpenr;    /* RCC APB2 peripheral clock enable in low power mode register */
    uint32_t reserved08;
    uint32_t reserved09;
    volatile uint32_t bdcr;
    volatile uint32_t csr;
    uint32_t reserved10;
    uint32_t reserved11;
    volatile uint32_t sscgr;
    volatile uint32_t plli2scfgr;
}; 

#endif
