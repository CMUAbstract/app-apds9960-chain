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
#define NUM_BLINKS_PER_TASK       1
#define WAIT_TICKS                3

#define NUM_AVGS 2 
#define NUM_SAMPS  8
#define ALERT_THRESH 30
#define MIN_DATA_SETS 5
#define MAX_GESTS 128

#define CNTPWR 1


// If you link-in wisp-base, then you have to define some symbols.
uint8_t usrBank[USRBANK_SIZE];

struct msg_flag_vals{
	CHAN_FIELD(uint8_t, flag);
	CHAN_FIELD(uint8_t, stale); 
};

struct msg_self_stale_flag{
	SELF_CHAN_FIELD(uint8_t, stale); 
};
#define FIELD_INIT_msg_self_stale_flag { \
	SELF_FIELD_INITIALIZER \
} 

struct msg_samples{
	CHAN_FIELD_ARRAY(uint8_t, samples, NUM_SAMPS); 
};

struct msg_self_gest_data{
	CHAN_FIELD_ARRAY(gesture_t, gestures, MAX_GESTS); 
	CHAN_FIELD(uint16_t, num_gests); 
};
#define FIELD_INIT_msg_self_gest_data { \
	SELF_FIELD_ARRAY_INITIALIZER(MAX_GESTS), \
	SELF_FIELD_INITIALIZER \
}  

struct msg_gest_data{
	CHAN_FIELD_ARRAY(gesture_t, gestures, MAX_GESTS); 
	CHAN_FIELD(uint16_t, num_gests); 
};

TASK(1, task_init)
TASK(2, task_sample)
TASK(3, task_gestCapture)
TASK(4, task_gestCalc)


CHANNEL(task_init, task_gestCalc, msg_gest_data); 

CHANNEL(task_sample, task_gestCapture, msg_flag_vals); 

SELF_CHANNEL(task_gestCapture, msg_self_stale_flag); 

CHANNEL(task_gestCapture, task_gestCalc, msg_samples); 

SELF_CHANNEL(task_gestCalc, msg_self_gest_data); 


volatile unsigned work_x;



static void burn(uint32_t iters)
{
    uint32_t iter = iters;
    while (iter--)
        work_x++;
}

void initializeHardware(void);

void init()
{
    WISP_init();
		
/*   GPIO(PORT_LED_1, DIR) |= BIT(PIN_LED_1);
    GPIO(PORT_LED_2, DIR) |= BIT(PIN_LED_2);
#if defined(PORT_LED_3)
    GPIO(PORT_LED_3, DIR) |= BIT(PIN_LED_3);
#endif
*/
    INIT_CONSOLE();

    __enable_interrupt();
/*
#if defined(PORT_LED_3) // when available, this LED indicates power-on
    GPIO(PORT_LED_3, OUT) |= BIT(PIN_LED_3);
#endif
*/
		LOG("Starting init\r\n"); 
		initializeHardware();
		delay(4000); 
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

void delay(uint32_t cycles)
{
    unsigned i;
    for (i = 0; i < cycles / (1U << 15); ++i)
        __delay_cycles(1U << 15);
}

void initializeHardware()
{		WISP_init(); 
		INIT_CONSOLE(); 
		__enable_interrupt(); 
		LOG("Starting HW setup \r\n"); 

    WDTCTL = WDTPW | WDTHOLD;  // Stop watchdog timer

#if defined(BOARD_EDB) || defined(BOARD_WISP) || defined(BOARD_SPRITE_APP_SOCKET_RHA) || defined(BOARD_SPRITE_APP)
    PM5CTL0 &= ~LOCKLPM5;	   // Enable GPIO pin settings
#endif

#if defined(BOARD_SPRITE_APP_SOCKET_RHA) || defined(BOARD_SPRITE_APP)
    /*P1DIR |= BIT0 | BIT1 | BIT2;
    P1OUT &= ~(BIT0 | BIT1 | BIT2);
    P2DIR |= BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7;
    P2OUT &= ~(BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7);
    P3DIR |= BIT6 | BIT7;
    P3OUT &= ~(BIT6 | BIT7);
    P4DIR |= BIT0 | BIT1 | BIT4;
    P4OUT &= ~(BIT0 | BIT1 | BIT4);
    PJDIR |= BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5;
    PJOUT |= BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5;*/
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


    WATCHPOINT(WATCHPOINT_BOOT);

    i2c_setup();
		LOG("i2c setup done \r\n"); 
		/*Iinitialize apds*/
		proximity_init(); 
		/*Now enable the proximity sensor*/
		enableProximitySensor(); 
    LOG("APDS TEST v1:  curtsk %u\r\n", curctx->task->idx);
}

void task_init()
{
    task_prologue();
    LOG("init\r\n");
    // Solid flash signifying beginning of task
    #ifdef CNTPWR
		GPIO(PORT_LED_1, OUT) |= BIT(PIN_LED_1);
    GPIO(PORT_LED_2, OUT) |= BIT(PIN_LED_2);
    burn(INIT_TASK_DURATION_ITERS);
    GPIO(PORT_LED_1, OUT) &= ~BIT(PIN_LED_1);
    GPIO(PORT_LED_2, OUT) &= ~BIT(PIN_LED_2);
		delay(INIT_TASK_DURATION_ITERS);
   	#endif
		
		//Init task_gestCalc fields
		uint8_t i; 
		for(i = 0; i < MAX_GESTS; i++){
			gest_dir  dataInit = DIR_NONE; 
			CHAN_OUT1(gest_dist, gestures[i], dataInit, CH(task_init, task_gestCalc)); 
		}
		uint16_t gestInit = 0; 
		CHAN_OUT1(uint16_t, num_gests, gestInit, CH(task_init, task_gestCalc)); 
		/*Set initial power config here, don't forget a delay!*/ 
  	TRANSITION_TO(task_sample);
}

void task_sample()
{
  task_prologue();
//	LOG("running task_sample \r\n");
	delay(400000); 
	uint8_t proxVal = readProximity();
	LOG("ProxVal: %u \r\n", proxVal); 	
	uint8_t flag = 0; 
	if(proxVal > ALERT_THRESH){
		flag = 1; 
		uint8_t stale = 0; 
		/*Add power system reconfiguration code here!!  
			Switch to high power bank, let's assume that we precharged the banks in the past*/ 
		CHAN_OUT1(uint8_t, flag, flag, CH(task_sample, task_gestCapture)); 
		CHAN_OUT1(uint8_t, stale, stale, CH(task_sample, task_gestCapture)); 
		enableGesture(); 
		TRANSITION_TO(task_gestCapture);
	}
	else{
		disableGesture(); 
		TRANSITION_TO(task_sample);
	}

}

void task_gestCapture()
{
    task_prologue();
		uint8_t flag = *CHAN_IN1(uint8_t, flag, CH(task_sample, task_gestCapture));  
		LOG("Running gesture \r\n");
		uint8_t stale = *CHAN_IN2(uint8_t, stale, SELF_IN_CH(task_gestCapture),
															CH(task_sample, task_gestCapture));
		if(stale){
			/*Have to hope that these occur atomically... */ 
			TRANSITION_TO(task_sample); 
		}
		stale = 1; 
		/*Mark that we've started a gesture*/ 
		CHAN_OUT1(uint8_t, stale, stale, SELF_OUT_CH(task_gestCapture)); 
		uint8_t num_samps = 0; 
		if(flag > 0){
		//	LOG("Enabling gesture \r\n"); 
		//	enableGesture(); 
			/*break down get gesture into a loop, loop until we hit the minimum number of data
			points, o/w fail --> stale gesture data needs to be flushed. */
			int8_t gestVal = getGesture();
			num_samps++; 
		}
		//LOG("Disabling gesture \r\n"); 
		//disableGesture(); 	
		if(num_samps > MIN_DATA_SETS){
			/*put data into channel, and if failure happens before all of the data is recorded,
			 * then no transition to gestCalc
			 */
		/*	for(uint8_t i = 0; i < MIN_DATA_SETS; i++){
				CHAN_OUT1(uint8_t, samples[i], samples[i], CH(task_gestCapture, task_gestCalc)); 
			}*/
			TRANSITION_TO(task_gestCalc);
		}
		else{
			/*Didn't capture enough data to decide*/ 
			TRANSITION_TO(task_sample); 
		}
}

void task_gestCalc()
{
    task_prologue();
   	LOG("Computing gesture... \r\n"); 
		uint8_t samples[NUM_SAMPS]; 
		uint16_t i;
		for(i =0 ; i < NUM_SAMPS; i++)
			samples[i] =  *CHAN_IN1(uint8_t, samples[i], CH(task_gestCapture, task_gestCalc)); 
		
		/*calculate the gesture, store the resulting gesture type and inc the number of
		 * gestures seen by writing to the self channel*/ 	
		
		TRANSITION_TO(task_sample);
    
}

ENTRY_TASK(task_init)
INIT_FUNC(init)
