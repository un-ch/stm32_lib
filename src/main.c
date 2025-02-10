
#include <stdint.h>     /* for uint32_t */
#include <unistd.h>

enum {
	exit_success_code   = 0,
	exit_failure_code   = 1,
	set         = 0,
	reset       = 1,
	disable     = 0,
	enable      = 1
};
enum {
	gpio_mode_input         = 0,
	gpio_mode_output        = 1,
	gpio_mode_alt           = 2,
	gpio_mode_analog        = 3,
	gpio_otyper_push_pull   = 0,
	gpio_otyper_open_drain  = 1,
	speed_low          = 0,
	speed_medium       = 1,
	speed_high         = 2,
	speed_very_high    = 3,
};

enum {
	flash_base_addr             = 0x08000000U,
	sram1_base_addr             = 0x20000000U,
	sram2_base_addr             = 0x2001C000U,
	rom_base_addr               = 0x1FFF0000U,
	sram                        = sram2_base_addr,
	periph_base_addr            = 0x40000000U,
	apb1_bus_periph_base_addr   = 0x40000000U,
	apb2_bus_periph_base_addr   = 0x40010000U,
	ahb1_bus_periph_base_addr   = 0x40020000U,
	ahb2_bus_periph_base_addr   = 0x50000000U,

	gpioa_base_addr             = ahb1_bus_periph_base_addr + 0x0000,
	gpiod_base_addr             = ahb1_bus_periph_base_addr + 0x0C00,

	rcc_base_addr               = ahb1_bus_periph_base_addr + 0x3800
};

#define GPIOD_BASE_ADDR             (AHB1_BUS_PERIPH_BASE_ADDR + 0x0C00)
#define RCC_BASE_ADDR               (AHB1_BUS_PERIPH_BASE_ADDR + 0x3800)
#define SYSCFG_BASE_ADDR            (APB2_BUS_PERIPH_BASE_ADDR + 0x3800)
#define EXTI_BASE_ADDR              (APB2_BUS_PERIPH_BASE_ADDR + 0x3C00)
#define GPIOD                       ((struct gpio_reg *)GPIOD_BASE_ADDR)
#define RCC                         ((struct rcc_reg *)RCC_BASE_ADDR)

#define GPIOD_PCLK_ENABLE           ( RCC->ahb1enr |= ( 1 << 3 ))
#define GPIOD_PCLK_DISABLE          ( RCC->ahb1enr &= ~( 1 << 3 ))

struct gpio_reg {
    volatile uint32_t moder;        /* mode register                    */
    volatile uint32_t otyper;       /* output type register             */
    volatile uint32_t ospeedr;      /* output speed register            */
    volatile uint32_t puprd;        /* pull-up/pull-down register       */
    volatile uint32_t idr;          /* input data register              */
    volatile uint32_t odr;          /* output data register             */
    volatile uint32_t bsrr;         /* bit set/reset register           */
    volatile uint32_t lckr;         /* configuration lock register      */
    volatile uint32_t afrl;         /* alternate function low register  */
    volatile uint32_t afrh;         /* alternate function high register */
};

struct gpio_port {
    uint8_t id;
    uint8_t base_addr;

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

struct rcc_reg {
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
	gpio_mode_input_fal_edge        = 4,
	gpio_mode_input_ris_edge        = 5,
	gpio_mode_input_edge_trig       = 6,
	gpio_pupdr_no_pupd              = 0,
	gpio_pupdr_pu                   = 1,
	gpio_pupdr_pd                   = 2
};

/*
void
enable_gpio_periph_clock(const uint8_t port_id)
{
    uint32_t ahb1enr = ahb1_bus_periph_base_addr + 0x3800;

    const uint8_t ascii_offset = 97;
    uint8_t offset = (ascii_offset + port_id) - ascii_offset;

    ahb1enr |= (1 << offset);
}

void
disable_gpio_periph_clock(const uint8_t port_id)
{
    uint32_t ahb1enr = ahb1_bus_periph_base_addr + 0x3800;

    const uint8_t ascii_offset = 97;
    uint8_t offset = (ascii_offset + port_id) - ascii_offset;

     ahb1enr &= ~(1 << offset);
}
*/

static void
rcc_peripheral_clock_control(const uint32_t gpiox_base_addr,
                                        struct rcc_reg *rcc,
                                        const uint8_t state)
{
	uint8_t shift_position  = 0;

	switch(gpiox_base_addr) {
	case gpioa_base_addr:
		shift_position = 0;
		rcc->ahb1enr |= (state << shift_position);
		break;
	case gpiod_base_addr:
		shift_position = 3; 
		rcc->ahb1enr |= (state << shift_position);
		break;
	}
}

static void
gpio_mode_register_control(struct gpio_reg *gpiox,
                               const uint8_t mode,
                                const uint8_t pin)
{
    uint8_t shift_position = pin * 2;

    gpiox->moder |= (mode << shift_position);
}

static void
gpio_output_type_register_control(struct gpio_reg *gpiox,
                                      const uint8_t type,
                                       const uint8_t pin)
{
    if(type == gpio_otyper_push_pull) {
        gpiox->otyper &= ~(1 << pin);
    }
    else {
        gpiox->otyper |= (type << pin);
    }
}

static void
toggle_gpio_pin(struct gpio_reg *gpiox, const uint8_t pin)
{
    gpiox->odr ^= (1 << pin);
}

static uint8_t
gpio_read_input_data_register(struct gpio_reg *gpiox,
                                   const uint8_t pin)
{
    uint8_t result = 0;
    const uint8_t mask = 0b00000001;

    result = ((uint8_t)(gpiox->idr >> pin) & mask);

    return result;
}

static void
gpio_output_speed_register_control(struct gpio_reg *gpiox,
                                const uint8_t speed_level,
                                        const uint8_t pin)
{
    uint8_t shift_position = pin * 2;

    if(speed_level == speed_low) {
        gpiox->ospeedr &= ~(speed_level << shift_position);
    }
    else {
        gpiox->ospeedr |= (speed_level << shift_position);
    }
}

static void
wrong_delay(void)
{
    const char divisor = 1;
    for(volatile uint32_t i = 0; i < 500000 / divisor; i++);
}

int
main()
{
	struct gpio_reg *gpioa  = (struct gpio_reg *)gpioa_base_addr;
	struct gpio_reg *gpiod  = (struct gpio_reg *)gpiod_base_addr;

	struct rcc_reg  *rcc    = (struct rcc_reg *)rcc_base_addr;

	const uint8_t   pin_num = 12;
	const uint8_t   user_button_pin_num = 0;
	uint8_t         user_button_state = 0;

	rcc_peripheral_clock_control(gpiod_base_addr, rcc, enable);
	rcc_peripheral_clock_control(gpioa_base_addr, rcc, enable);

	gpio_mode_register_control(gpiod, gpio_mode_output, pin_num);
	gpio_mode_register_control(gpioa, gpio_mode_input, user_button_pin_num);

	gpio_output_type_register_control(gpiod, gpio_otyper_push_pull, pin_num);
	gpio_output_speed_register_control(gpiod, speed_low, pin_num);

	for(;;) {

		toggle_gpio_pin(gpiod, pin_num);
		wrong_delay();
		/*
		user_button_state = gpio_read_input_data_register(gpioa, user_button_pin_num);

		if(user_button_state == 1) {
			gpiod->odr |= (1 << pin_num);
		}
		else {
			gpiod->odr &= ~(1 << pin_num);
		}
		*/
	}

	return exit_success_code;
}
