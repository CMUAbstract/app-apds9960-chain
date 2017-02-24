#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h> 
#include <stdarg.h> 

#include <libwispbase/wisp-base.h>
#include <libmsp/mem.h>
#include <libchain/chain.h>
#include <libio/log.h>
#include "libmspware/driverlib.h"
#include "proximity.h"
#include "pins.h"

#ifdef CONFIG_LIBEDB_PRINTF
#include <libedb/edb.h>
#endif

#ifdef CONFIG_EDB
#include <libedb/edb.h>
#else
#define WATCHPOINT(...)
#endif


#define INIT_TASK_DURATION_ITERS  400000
#define TASK_START_DURATION_ITERS 1600000
#define BLINK_DURATION_ITERS      400000
#define WAIT_TICK_DURATION_ITERS  300000
#define NUM_BLINKS_PER_TASK       5
#define WAIT_TICKS                3

// If you link-in wisp-base, then you have to define some symbols.
uint8_t usrBank[USRBANK_SIZE];

struct msg_blinks {
    CHAN_FIELD(unsigned, blinks);
};

struct msg_tick {
    CHAN_FIELD(unsigned, tick);
};

struct msg_self_tick {
    SELF_CHAN_FIELD(unsigned, tick);
};
#define FIELD_INIT_msg_self_tick { \
    SELF_FIELD_INITIALIZER \
}

struct msg_duty_cycle {
    CHAN_FIELD(unsigned, duty_cycle);
};

TASK(1, task_init)
TASK(2, task_1)
TASK(3, task_2)
TASK(4, task_3)

CHANNEL(task_init, task_1, msg_blinks);
CHANNEL(task_init, task_3, msg_tick);
CHANNEL(task_1, task_2, msg_blinks);
CHANNEL(task_2, task_1, msg_blinks);
SELF_CHANNEL(task_3, msg_self_tick);
MULTICAST_CHANNEL(msg_duty_cycle, ch_duty_cycle, task_init, task_1, task_2);

volatile unsigned work_x;


void initializeHardware();

static void burn(uint32_t iters)
{
    uint32_t iter = iters;
    while (iter--)
        work_x++;
}

void init()
{
    WISP_init();

    GPIO(PORT_LED_1, DIR) |= BIT(PIN_LED_1);
    GPIO(PORT_LED_2, DIR) |= BIT(PIN_LED_2);
#if defined(PORT_LED_3)
    GPIO(PORT_LED_3, DIR) |= BIT(PIN_LED_3);
#endif

    INIT_CONSOLE();

    __enable_interrupt();

#if defined(PORT_LED_3) // when available, this LED indicates power-on
    GPIO(PORT_LED_3, OUT) |= BIT(PIN_LED_3);
#endif
		initializeHardware();
    LOG("gesture app booted\r\n");
}

static void blink_led1(unsigned blinks, unsigned duty_cycle) {
    unsigned i;

    for (i = 0; i < blinks; ++i) {
        GPIO(PORT_LED_1, OUT) |= BIT(PIN_LED_1);
        burn(BLINK_DURATION_ITERS * 2 * duty_cycle / 100);

        GPIO(PORT_LED_1, OUT) &= ~BIT(PIN_LED_1);
        burn(BLINK_DURATION_ITERS * 2 * (100 - duty_cycle) / 100);
    }
}

static void blink_led2(unsigned blinks, unsigned duty_cycle) {
    unsigned i;

    for (i = 0; i < blinks; ++i) {
        GPIO(PORT_LED_2, OUT) |= BIT(PIN_LED_2);
        burn(BLINK_DURATION_ITERS * 2 * duty_cycle / 100);

        GPIO(PORT_LED_2, OUT) &= ~BIT(PIN_LED_2);
        burn(BLINK_DURATION_ITERS * 2 * (100 - duty_cycle) / 100);
    }
}

void i2c_setup(void) {
  /*
  * Select Port 1
  * Set Pin 6, 7 to input Secondary Module Function:
  *   (UCB0SIMO/UCB0SDA, UCB0SOMI/UCB0SCL)
  */


  GPIO_setAsPeripheralModuleFunctionInputPin(
    GPIO_PORT_P1,
    GPIO_PIN6 + GPIO_PIN7,
    GPIO_SECONDARY_MODULE_FUNCTION
  );



  EUSCI_B_I2C_initMasterParam param = {0};
  param.selectClockSource = EUSCI_B_I2C_CLOCKSOURCE_SMCLK;
  param.i2cClk = CS_getSMCLK();
  param.dataRate = EUSCI_B_I2C_SET_DATA_RATE_400KBPS;
  param.byteCounterThreshold = 0;
  param.autoSTOPGeneration = EUSCI_B_I2C_NO_AUTO_STOP;

  EUSCI_B_I2C_initMaster(EUSCI_B0_BASE, &param);
  

}
/*
static void delay(uint32_t cycles)
{
    unsigned i;
    for (i = 0; i < cycles / (1U << 15); ++i)
        __delay_cycles(1U << 15);
}
*/
void initializeHardware()
{
    WDTCTL = WDTPW | WDTHOLD;  // Stop watchdog timer

#if defined(BOARD_EDB) || defined(BOARD_WISP) || defined(BOARD_SPRITE_APP_SOCKET_RHA) || defined(BOARD_SPRITE_APP)
    PM5CTL0 &= ~LOCKLPM5;	   // Enable GPIO pin settings
#endif

#if defined(BOARD_SPRITE_APP_SOCKET_RHA) || defined(BOARD_SPRITE_APP)
    P1DIR |= BIT0 | BIT1 | BIT2;
    P1OUT &= ~(BIT0 | BIT1 | BIT2);
    P2DIR |= BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7;
    P2OUT &= ~(BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7);
    P3DIR |= BIT6 | BIT7;
    P3OUT &= ~(BIT6 | BIT7);
    P4DIR |= BIT0 | BIT1 | BIT4;
    P4OUT &= ~(BIT0 | BIT1 | BIT4);
    PJDIR |= BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5;
    PJOUT |= BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5;
#endif

#if defined(BOARD_SPRITE_APP_SOCKET_RHA) || defined(BOARD_SPRITE_APP)
    CSCTL0_H = 0xA5;
    CSCTL1 = DCOFSEL_6; //8MHz
    CSCTL3 = DIVA_0 + DIVS_0 + DIVM_0;
#endif

#ifdef CONFIG_EDB
    debug_setup();
    edb_set_app_output_cb(write_app_output);
#endif

    INIT_CONSOLE();

    __enable_interrupt();

    WATCHPOINT(WATCHPOINT_BOOT);

    i2c_setup();
		LOG("i2c setup done \r\n"); 
		proximity_init(); 

    LOG("space app: curtsk %u\r\n", curctx->task->idx);
}

void task_init()
{
    task_prologue();
    LOG("init\r\n");
		getGesture(); 

    // Solid flash signifying beginning of task
    GPIO(PORT_LED_1, OUT) |= BIT(PIN_LED_1);
    GPIO(PORT_LED_2, OUT) |= BIT(PIN_LED_2);
    burn(INIT_TASK_DURATION_ITERS);
    GPIO(PORT_LED_1, OUT) &= ~BIT(PIN_LED_1);
    GPIO(PORT_LED_2, OUT) &= ~BIT(PIN_LED_2);
    burn(INIT_TASK_DURATION_ITERS);

    unsigned blinks = NUM_BLINKS_PER_TASK;
    CHAN_OUT1(unsigned, blinks, blinks, CH(task_init, task_1));
    unsigned tick = 0;
    CHAN_OUT1(unsigned, tick, tick, CH(task_init, task_3));
    unsigned duty_cycle = 75;
    CHAN_OUT1(unsigned, duty_cycle, duty_cycle,
             MC_OUT_CH(ch_duty_cycle, task_init, task_1, task_2));

    TRANSITION_TO(task_3);
}

void task_1()
{
    task_prologue();
    unsigned blinks;
    unsigned duty_cycle;
		getGesture(); 
    LOG("task 1\r\n");
 		PRINTF("CAN'T STOP TASK 1 \r\n"); 

    // Solid flash signifying beginning of task
    GPIO(PORT_LED_1, OUT) |= BIT(PIN_LED_1);
    burn(TASK_START_DURATION_ITERS);
    GPIO(PORT_LED_1, OUT) &= ~BIT(PIN_LED_1);
    burn(TASK_START_DURATION_ITERS);

    blinks = *CHAN_IN2(unsigned, blinks, CH(task_init, task_1), CH(task_2, task_1));
    duty_cycle = *CHAN_IN1(unsigned, duty_cycle,
                           MC_IN_CH(ch_duty_cycle, task_init, task_1));

    LOG("task 1: blinks %u dc %u\r\n", blinks, duty_cycle);

    blink_led1(blinks, duty_cycle);
    blinks++;

    CHAN_OUT1(unsigned, blinks, blinks, CH(task_1, task_2));

    TRANSITION_TO(task_2);
}

void task_2()
{
    task_prologue();

    unsigned blinks;
    unsigned duty_cycle;

    LOG("task 2\r\n");
		PRINTF("can't stop task 2!\r\n");
    // Solid flash signifying beginning of task
    GPIO(PORT_LED_2, OUT) |= BIT(PIN_LED_2);
    burn(TASK_START_DURATION_ITERS);
    GPIO(PORT_LED_2, OUT) &= ~BIT(PIN_LED_2);
    burn(TASK_START_DURATION_ITERS);

    blinks = *CHAN_IN1(unsigned, blinks, CH(task_1, task_2));
    duty_cycle = *CHAN_IN1(unsigned, duty_cycle,
                           MC_IN_CH(ch_duty_cycle, task_init, task_2));

    LOG("task 2: blinks %u dc %u\r\n", blinks, duty_cycle);

    blink_led2(blinks, duty_cycle);
    blinks++;

    CHAN_OUT1(unsigned, blinks, blinks, CH(task_2, task_1));

    TRANSITION_TO(task_3);
}

void task_3()
{
    task_prologue();

    unsigned wait_tick = *CHAN_IN2(unsigned, tick, CH(task_init, task_3),
                                                   SELF_IN_CH(task_3));

    LOG("task 3: wait tick %u\r\n", wait_tick);

    GPIO(PORT_LED_1, OUT) |= BIT(PIN_LED_1);
    GPIO(PORT_LED_2, OUT) |= BIT(PIN_LED_2);
    burn(WAIT_TICK_DURATION_ITERS);
    GPIO(PORT_LED_1, OUT) &= ~BIT(PIN_LED_1);
    GPIO(PORT_LED_2, OUT) &= ~BIT(PIN_LED_2);
    burn(WAIT_TICK_DURATION_ITERS);

    if (++wait_tick < WAIT_TICKS) {
        CHAN_OUT1(unsigned, tick, wait_tick, SELF_OUT_CH(task_3));
        TRANSITION_TO(task_3);
    } else {
        unsigned tick = 0;
        CHAN_OUT1(unsigned, tick, tick, SELF_OUT_CH(task_3));
        TRANSITION_TO(task_1);
    }
}

ENTRY_TASK(task_init)
INIT_FUNC(init)
