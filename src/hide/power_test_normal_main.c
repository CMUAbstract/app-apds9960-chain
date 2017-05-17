#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h> 
#include <stdarg.h> 

#include <libwispbase/wisp-base.h>
#include <libmsp/mem.h>
//#include <libchain/chain.h>
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


// If you link-in wisp-base, then you have to define some symbols.
uint8_t usrBank[USRBANK_SIZE];

void initializeHardware(void);

void init()
{
    WISP_init();

    GPIO(PORT_LED_1, DIR) |= BIT(PIN_LED_1);
    GPIO(PORT_LED_2, DIR) |= BIT(PIN_LED_2);
		GPIO(PORT_RESET, DIR) |= BIT(PIN_RESET); 
		GPIO(PORT_SET,   DIR) |= BIT(PIN_SET  ); 
#if defined(PORT_LED_3)
    GPIO(PORT_LED_3, DIR) |= BIT(PIN_LED_3);
#endif

    INIT_CONSOLE();

    __enable_interrupt();

#if defined(PORT_LED_3) // when available, this LED indicates power-on
    GPIO(PORT_LED_3, OUT) |= BIT(PIN_LED_3);
#endif
		LOG("Starting init\r\n"); 
		initializeHardware();
    LOG("gesture app booted\r\n");
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

static void delay(uint32_t cycles)
{
    unsigned i;
    for (i = 0; i < cycles / (1U << 15); ++i)
        __delay_cycles(1U << 15);
}

void initializeHardware()
{
    WDTCTL = WDTPW | WDTHOLD;  // Stop watchdog timer

#if defined(BOARD_EDB) || defined(BOARD_WISP) || defined(BOARD_SPRITE_APP_SOCKET_RHA) || defined(BOARD_SPRITE_APP)
    PM5CTL0 &= ~LOCKLPM5;	   // Enable GPIO pin settings
#endif

#if defined(BOARD_SPRITE_APP_SOCKET_RHA) || defined(BOARD_SPRITE_APP)
 /*   P1DIR |= BIT0 | BIT1 | BIT2;
    P1OUT &= ~(BIT0 | BIT1 | BIT2);
    P2DIR |= BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7;
    P2OUT &= ~(BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7);
    P3DIR |= BIT6 | BIT7;
    P3OUT &= ~(BIT6 | BIT7);
    P4DIR |= BIT0 | BIT1 | BIT4;
    P4OUT &= ~(BIT0 | BIT1 | BIT4);
    PJDIR |= BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5;
    PJOUT |= BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5;
*/
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

 //   INIT_CONSOLE();

 //   __enable_interrupt();
		P3DIR |= (BIT4 | BIT5 | BIT6 ); 				//Clear P3.5 latch, then set it  
  	P3OUT &= ~(BIT4 | BIT5 | BIT6); 
	

		P4DIR |= (BIT6); 
		P4OUT &= ~(BIT6); 
		P1DIR |= (BIT0); 
		P1OUT &= ~(BIT0); 

		WATCHPOINT(WATCHPOINT_BOOT);

    i2c_setup();
		LOG("i2c setup done \r\n"); 
//		proximity_init(); 

   // LOG("space app: curtsk %u\r\n", curctx->task->idx);
}

volatile unsigned work_x; 

static void burn(uint32_t iters)
{
    uint32_t iter = iters;
    while (iter--)
        work_x++;
}

static void one_blink_led1(uint32_t time){
	GPIO(PORT_LED_1, OUT) |= BIT(PIN_LED_1);
	GPIO(PORT_AUX3, OUT)  |= BIT(PIN_AUX_2); 
	delay(time); 
	GPIO(PORT_LED_1, OUT) &= ~BIT(PIN_LED_1); 
	GPIO(PORT_AUX3, OUT)  &= ~BIT(PIN_AUX_2);
}

static void one_blink_led2(uint32_t time){
	GPIO(PORT_LED_2, OUT) |= BIT(PIN_LED_2); 
	GPIO(PORT_AUX3, OUT)  |= BIT(PIN_AUX_1);
	delay(time); 
	
}

int main(void){
	init();
	uint8_t test; 
	while(1){
		LOG("In loop \r\n"); 
		one_blink_led1(400000); 
		LOG("Blink 2 \r\n"); 
		one_blink_led2(400000); 
		delay(400000);
		LOG("prox data = %x \r\n", test); 
	}
	return 0; 
}
