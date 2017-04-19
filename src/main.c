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
#include <libcapybara/reconfig.h> 
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

//#define MEAS_PROX 

#define CNTPWR 1
#define LOG_PROX 1
#define DEFAULT_CFG 							0b1111 

//Define the current cap configuration as well as precharge status
__nv capybara_bankmask_t curbankcfg = DEFAULT_CFG;	

__nv prechg_status_t curprchg;


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

struct msg_gestures{
	CHAN_FIELD(gesture_data_t, gesture_data_sets); 
};

struct msg_self_gest_data{
	CHAN_FIELD(uint16_t, num_gests); 
	CHAN_FIELD_ARRAY(gesture_t, gestures, MAX_GESTS); 
};
#define FIELD_INIT_msg_self_gest_data { \
	SELF_FIELD_INITIALIZER, \
	SELF_FIELD_ARRAY_INITIALIZER(MAX_GESTS), \
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

CHANNEL(task_gestCapture, task_gestCalc, msg_gestures); 

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
/*		
   GPIO(PORT_LED_1, DIR) |= BIT(PIN_LED_1);
    GPIO(PORT_LED_2, DIR) |= BIT(PIN_LED_2);
*/   
	//Configure capybara banks to desired setting 
//	capybara_config_banks(curbankcfg); 

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
	//	while(1){
			LOG("gesture app booted\r\n");
	//		}
}

/**
 * @brief represents the gesture captured
 * @details DIR_NONE =  111
 						DIR_LEFT =  001
						DIR_RIGHT = 010 
						DIR_UP =    011
						DIR_DOWN =  100
		These values will get held for a little while, and then revert to 000. 
*/


void encode_IO(gest_dir val){
	switch(val){
		case DIR_NONE: 
			GPIO(PORT_DEBUG, OUT) |= BIT(PIN_DEBUG_1); 
			GPIO(PORT_DEBUG, OUT) |= BIT(PIN_DEBUG_2); 
			GPIO(PORT_DEBUG, OUT) |= BIT(PIN_DEBUG_3); 
			break; 
		case DIR_LEFT: 
			GPIO(PORT_DEBUG, OUT) |= BIT(PIN_DEBUG_1); 
			GPIO(PORT_DEBUG, OUT) &= ~BIT(PIN_DEBUG_2); 
			GPIO(PORT_DEBUG, OUT) &= ~BIT(PIN_DEBUG_3); 
			break; 
		case DIR_RIGHT: 
			GPIO(PORT_DEBUG, OUT) &= ~BIT(PIN_DEBUG_1); 
			GPIO(PORT_DEBUG, OUT) |= BIT(PIN_DEBUG_2); 
			GPIO(PORT_DEBUG, OUT) &= ~BIT(PIN_DEBUG_3); 
			break; 
		case DIR_UP: 
			GPIO(PORT_DEBUG, OUT) |= BIT(PIN_DEBUG_1); 
			GPIO(PORT_DEBUG, OUT) |= BIT(PIN_DEBUG_2); 
			GPIO(PORT_DEBUG, OUT) &= ~BIT(PIN_DEBUG_3); 
			break; 
		case DIR_DOWN: 
			GPIO(PORT_DEBUG, OUT) &= ~BIT(PIN_DEBUG_1); 
			GPIO(PORT_DEBUG, OUT) &= ~BIT(PIN_DEBUG_2); 
			GPIO(PORT_DEBUG, OUT) |= BIT(PIN_DEBUG_3); 
			break; 
	}
	delay(GESTURE_HOLD_TIME); 
		GPIO(PORT_DEBUG, OUT) &= ~BIT(PIN_DEBUG_1); 
		GPIO(PORT_DEBUG, OUT) &= ~BIT(PIN_DEBUG_2); 
		GPIO(PORT_DEBUG, OUT) &= ~BIT(PIN_DEBUG_3); 

	return; 
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
   	enableGesture();  
		disableGesture(); 
		//while(1){
		LOG("APDS TEST v1:  curtsk %u\r\n", curctx->task->idx);
	//	}
}

void task_init()
{
    task_prologue();
    LOG("init\r\n");
		//Init task_gestCalc fields
		uint8_t i; 
		for(i = 0; i < MAX_GESTS; i++){
			gest_dir  dataInit = DIR_NONE; 
			CHAN_OUT1(gest_dist, gestures[i], dataInit, CH(task_init, task_gestCalc)); 
		}
		uint16_t gestInit = 0; 
		CHAN_OUT1(uint16_t, num_gests, gestInit, CH(task_init, task_gestCalc)); 
		/*Set initial power config here, don't forget a delay!*/ 
#ifdef MEAS_PROX
		GPIO(PORT_DEBUG, DIR) |= BIT(PIN_DEBUG_3);
		GPIO(PORT_DEBUG, OUT) |= BIT(PIN_DEBUG_3); 
#endif

		TRANSITION_TO(task_sample);
}

void task_sample()
{
  task_prologue();
	LOG("In sample! \r\n"); 
	uint8_t proxVal = readProximity();
#if LOG_PROX
	LOG("proxVal = %u \r\n",proxVal); 
#endif
	delay(240000); 
	uint8_t flag = 0; 
	if(proxVal > ALERT_THRESH){
    //GPIO(PORT_LED_1, OUT) |= BIT(PIN_LED_1);
		flag = 1; 
		uint8_t stale = 0; 
		/*Add power system reconfiguration code here!!  
			Switch to high power bank, let's assume that we precharged the banks in the past*/ 
		CHAN_OUT1(uint8_t, flag, flag, CH(task_sample, task_gestCapture)); 
		CHAN_OUT1(uint8_t, stale, stale, CH(task_sample, task_gestCapture)); 
#ifdef MEAS_PROX
		GPIO(PORT_DEBUG, DIR) &= ~BIT(PIN_DEBUG_3); 
		GPIO(PORT_DEBUG, OUT) &= ~BIT(PIN_DEBUG_3); 
#endif
#ifdef MEAS_GEST
		GPIO(PORT_DEBUG, DIR) |= BIT(PIN_DEBUG_2); 
		GPIO(PORT_DEBUG, OUT) |= BIT(PIN_DEBUG_2); 
#endif
		
		reenableGesture();  
		TRANSITION_TO(task_gestCapture);
	}
	else{
		//LOG("Disabling gesture!!\r\n"); 	
		disableGesture(); 
  	
		/*GPIO(PORT_LED_1, OUT) &= ~BIT(PIN_LED_1);*/
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
		//TODO: get rid of stale variable... 	
		/*if(stale){
			LOG("Stale! \r\n"); 
			//Have to hope that these occur atomically...  
			//disableGesture(); 
			TRANSITION_TO(task_sample); 
		}*/
		stale = 1; 
		/*Mark that we've started a gesture*/ 
		CHAN_OUT1(uint8_t, stale, stale, SELF_OUT_CH(task_gestCapture)); 
		uint8_t num_samps = 0; 
		gesture_data_t gesture_data_; 
		if(flag > 0){
		//	LOG("Enabling gesture \r\n"); 
		//	enableGesture(); 
			/*break down get gesture into a loop, loop until we hit the minimum number of data
			points, o/w fail --> stale gesture data needs to be flushed. */
			resetGestureFields(&gesture_data_); 
			int8_t gestVal = getGestureLoop(&gesture_data_, &num_samps);
			//disableGesture(); 
				//TODO chan_out the dir as we get it, let it be overwritten, nbd. Then grab it
				//later if we run out of power. 
			}
		LOG("OUT OF GESTURE LOOP, num samps = %u, min = %u \r\n", num_samps, MIN_DATA_SETS); 	
		//disableGesture(); 
		if(num_samps > MIN_DATA_SETS){
				CHAN_OUT1(gesture_data_t, gesture_data_sets, gesture_data_, 
																											CH(task_gestCapture,task_gestCalc)); 
			  LOG("transitioning to final calc!\r\n"); 
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
		gesture_data_t gest_vals = *CHAN_IN1(gesture_data_t, gesture_data_sets,
																								CH(task_gestCapture, task_gestCalc)); 
		uint8_t i,j, num_samps = 4;
		
		gest_dir output = decodeGesture();
#ifdef MEAS_GEST
		GPIO(PORT_DEBUG, DIR) &= ~BIT(PIN_DEBUG_2); 
		GPIO(PORT_DEBUG, OUT) &= ~BIT(PIN_DEBUG_2); 
#endif
		LOG("------------------Dir = %u ---------------", output); 	
		//encode_IO(output); 
		delay(5000000);
		delay(5000000);
	
		/*calculate the gesture, store the resulting gesture type and inc the number of
		 * gestures seen by writing to the self channel*/ 	
		
		TRANSITION_TO(task_sample);
    
}

ENTRY_TASK(task_init)
INIT_FUNC(init)
