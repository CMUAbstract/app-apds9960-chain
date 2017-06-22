//usual libs
#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h> 
#include <stdarg.h> 
//Before periph to get away with gpio macro 
#include "libmspware/driverlib.h"

//specific libmsp pieces
#include <libmsp/watchdog.h> 
#include <libmsp/clock.h> 
#include <libmsp/gpio.h>
#include <libmsp/periph.h>
#include <libmsp/sleep.h>
#include <libmsp/mem.h>

//Other stuff
#include <libio/console.h> 
#include <libchain/chain.h>
#include <libcapybara/capybara.h> 
#include <libcapybara/reconfig.h> 
#include "proximity.h"
#include "pins.h"
//Left here for now... 
#include <libwispbase/wisp-base.h>

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
//#define USE_PHOTORES

#define CNTPWR 0
#define LOG_PROX 1
#define DEFAULT_CFG 							0b1111 
#define PROX_ONLY 1
//Define the current cap configuration as well as precharge status
__nv capybara_bankmask_t curbankcfg = DEFAULT_CFG;	

__nv prechg_status_t curprchg;

__nv sensor_sw_fail_cnt = 0; 
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
 //Handle usual init stuff 
  msp_watchdog_disable(); 
  msp_gpio_unlock(); 
  __enable_interrupt(); 
  //Wait until we hit stable power level
  capybara_wait_for_supply();

  capybara_config_pins();

  GPIO(PORT_CAPYBARA_CFG, OUT) &= ~BIT(PIN_CAPYBARA_CFG);
  GPIO(PORT_CAPYBARA_CFG, DIR) &= BIT(PIN_CAPYBARA_CFG);

  GPIO(PORT_SENSE_SW, OUT) &= ~BIT(PIN_SENSE_SW);
  GPIO(PORT_SENSE_SW, DIR) |= BIT(PIN_SENSE_SW);

  capybara_config_banks(0x3); 
  capybara_wait_for_supply();
  
  GPIO(PORT_DEBUG,OUT) &= ~BIT(PIN_DEBUG_3);
  GPIO(PORT_DEBUG,DIR) |= BIT(PIN_DEBUG_3);  
  
  //Check if we're in a failing state and reset if we are... 
  /*
  sensor_sw_fail_cnt++; 
  if(sensor_sw_fail_cnt > 3){
    sensor_sw_fail_cnt = 0; 
    while(1); 
  }
  */
  // Turn on sensor power supply
  GPIO(PORT_SENSE_SW, OUT) |= BIT(PIN_SENSE_SW);

  // In ~1ms (but not right now), our supply voltage might drop, due to
  // charging of sensor caps that may overwhelm the booster briefly. We
  // need to wait for the drop until we can wait for supply to stabilize
  // using the VBOOST_OK supervisor, which we do below. This code works
  // fine if there is no drop in the supply voltage at all.
  msp_sleep(30 /* cycles @ ACLK=VLOCLK=~10kHz ==> ~3ms */);
  capybara_wait_for_supply();

  msp_watchdog_disable(); 
  msp_gpio_unlock(); 
  
  GPIO(PORT_DEBUG,OUT) |= BIT(PIN_DEBUG_3);
  GPIO(PORT_DEBUG,DIR) |= BIT(PIN_DEBUG_3);  
  
  msp_clock_setup(); 
  INIT_CONSOLE(); 
  __enable_interrupt(); 
  PRINTF("Starting init\r\n"); 
  //Now send init commands to the apds
  initializeHardware();
  delay(4000); 
  sensor_sw_fail_cnt = 0;   
  LOG("gesture app booted\r\n");

}

/**
 * @brief represents the gesture captured
 * @details DIR_NONE =  111
 						DIR_LEFT =  001
						DIR_RIGHT = 010 
						DIR_UP =    011
						DIR_DOWN =  100
		These values will get held for a little while, and then revert to 000. 
    Note: DON'T USE THIS FUNCTION ON CAPYBARA
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
{		LOG("Starting HW setup \r\n"); 
    
    //GPIO(PORT_DEBUG, OUT) &= ~BIT(PIN_DEBUG_2); 
    //GPIO(PORT_DEBUG, DIR) |= BIT(PIN_DEBUG_2); 

    #ifdef USE_PHOTORES
        P3SEL0 |= BIT1; 
        P3SEL1 |= BIT1; 
        PM5CTL0 &= ~LOCKLPM5; 
    #endif

    i2c_setup();
		LOG("i2c setup done \r\n"); 
		/*Iinitialize apds*/
		proximity_init(); 
		/*Now enable the proximity sensor*/
    #ifndef  USE_PHOTORES
    enableProximitySensor(); 
    #endif
    //Poke the gesture sensor if we're using it... 
    #ifndef PROX_ONLY
        enableGesture();  
        disableGesture(); 
    #endif
		LOG("APDS TEST v1:  curtsk %u\r\n", curctx->task->idx);
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
    LOG("SANITY CHECK \r\n"); 

		TRANSITION_TO(task_sample);
}

void task_sample()
{
  task_prologue();
#ifdef MEAS_PROX
		LOG("Going up!!\r\n"); 
    GPIO(PORT_DEBUG, DIR) |= BIT(PIN_DEBUG_3);
		GPIO(PORT_DEBUG, OUT) |= BIT(PIN_DEBUG_3); 
#endif

//TODO double check that it's not a problem to use the apds output as an int16_t... 
int16_t proxVal = 0; 
#ifdef USE_PHOTORES
  //enable_photoresistor(); 
    proxVal = read_photoresistor(); 
#else
	proxVal = readProximity();
  delay(240000); 
#endif

#if LOG_PROX
	LOG("proxVal = %u \r\n",proxVal); 
#endif

#ifdef MEAS_PROX
	//	LOG("Coming down!!\r\n"); 
		GPIO(PORT_DEBUG, OUT) &= ~BIT(PIN_DEBUG_3); 
		GPIO(PORT_DEBUG, DIR) &= ~BIT(PIN_DEBUG_3); 
#endif
    //Hijack the code here if we're only running proximity sensing.  
    #ifdef PROX_ONLY
        TRANSITION_TO(task_sample); 
    #endif

uint8_t flag = 0; 
	if(proxVal > ALERT_THRESH){
    //GPIO(PORT_LED_1, OUT) |= BIT(PIN_LED_1);
		flag = 1; 
		uint8_t stale = 0; 
		/*Add power system reconfiguration code here!!  
			Switch to high power bank, let's assume that we precharged the banks in the past*/ 
		CHAN_OUT1(uint8_t, flag, flag, CH(task_sample, task_gestCapture)); 
		CHAN_OUT1(uint8_t, stale, stale, CH(task_sample, task_gestCapture)); 

#ifdef MEAS_GEST
/*
//Loose start!
    LOG("Pulling Gesture high! \r\n"); 
		GPIO(PORT_DEBUG, DIR) |= BIT(PIN_DEBUG_2); 
		GPIO(PORT_DEBUG, OUT) |= BIT(PIN_DEBUG_2); 
*/
#endif
		
		reenableGesture();  
		TRANSITION_TO(task_gestCapture);
	}
	else{
		//LOG("Disabling gesture!!\r\n"); 	
		disableGesture(); 
    //GPIO(PORT_DEBUG, OUT) &= ~BIT(PIN_DEBUG_2); 
  	
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
#ifdef MEAS_GEST

//Med start (rising edge)
    LOG("Pulling Gesture high! \r\n"); 
		GPIO(PORT_DEBUG, DIR) |= BIT(PIN_DEBUG_2); 
		GPIO(PORT_DEBUG, OUT) |= BIT(PIN_DEBUG_2); 

#endif
			int8_t gestVal = getGestureLoop(&gesture_data_, &num_samps);
			//disableGesture(); 
				//TODO chan_out the dir as we get it, let it be overwritten, nbd. Then grab it
				//later if we run out of power. 
			}
		//disableGesture(); 
		LOG("OUT OF GESTURE LOOP, num samps = %u, min = %u \r\n", num_samps, MIN_DATA_SETS); 	
		
    if(num_samps > MIN_DATA_SETS){
#ifdef MEAS_GEST
 
    //Med end (falling edge)
    GPIO(PORT_DEBUG, OUT) &= ~BIT(PIN_DEBUG_2); 
		LOG("Pulling gesture low!! \r\n"); 
    GPIO(PORT_DEBUG, DIR) &= ~BIT(PIN_DEBUG_2); 

#endif
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
/*    //loose end
    GPIO(PORT_DEBUG, OUT) &= ~BIT(PIN_DEBUG_2); 
		LOG("Pulling gesture low!! \r\n"); 
    GPIO(PORT_DEBUG, DIR) &= ~BIT(PIN_DEBUG_2); 
*/
#endif
  LOG("------------------Dir = %u ---------------", output); 	
		//encode_IO(output); 
		delay(5000000);
		delay(5000000);
	
		/*calculate the gesture, store the resulting gesture type and inc the number of
		 * gestures seen by writing to the self channel*/ 	
		
		TRANSITION_TO(task_sample);
    
}

#define _THIS_PORT 2
__attribute__ ((interrupt(GPIO_VECTOR(_THIS_PORT))))
void  GPIO_ISR(_THIS_PORT) (void)
{
    switch (__even_in_range(INTVEC(_THIS_PORT), INTVEC_RANGE(_THIS_PORT))) {
#if LIBCAPYBARA_PORT_VBOOST_OK == _THIS_PORT
        case INTFLAG(LIBCAPYBARA_PORT_VBOOST_OK, LIBCAPYBARA_PIN_VBOOST_OK):
            capybara_vboost_ok_isr();
            break;
#else
#error Handler in wrong ISR: capybara_vboost_ok_isr
#endif // LIBCAPYBARA_PORT_VBOOST_OK
    }
}
#undef _THIS_PORT

ENTRY_TASK(task_init)
INIT_FUNC(init)
