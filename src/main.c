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
#include <libcapybara/power.h> 

#include "proximity.h"
#include "pins.h"
//Left here for now... 
//#include <libwispbase/wisp-base.h>

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

#define CNTPWR 0
#define LOG_PROX 1
#define DEFAULT_CFG 							0b1111 
//#define PROX_ONLY 0

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


TASK(1, task_init, DEFAULT)
TASK(2, task_sample, PREBURST, HIGHP,LOWP)
TASK(3, task_gestCapture, BURST)
// Really should rope this task into gestCapture...  gestCalc reports the
// gesture, and while it can be separate, there's no point in stopping the burst
// run it- i.e., if there isn't enough energy, configure up to 0x3 and spit out
// the answer, but I'd really rather it happened WITH the burst 
TASK(4, task_gestCalc, CONFIGD, HIGHP)


CHANNEL(task_init, task_gestCalc, msg_gest_data); 

CHANNEL(task_sample, task_gestCapture, msg_flag_vals); 

SELF_CHANNEL(task_gestCapture, msg_self_stale_flag); 

CHANNEL(task_gestCapture, task_gestCalc, msg_gestures); 

SELF_CHANNEL(task_gestCalc, msg_self_gest_data); 

volatile unsigned work_x;

void initializeHardware(void);

/** @brief Handler for capybara power-on sequence 
    TODO add this to libcapybara...
*/
void _capybara_handler(void) {
    msp_watchdog_disable();
    msp_gpio_unlock();
    __enable_interrupt();
    capybara_wait_for_supply();
    capybara_config_pins();
    
    GPIO(PORT_SENSE_SW, OUT) &= ~BIT(PIN_SENSE_SW);
    GPIO(PORT_SENSE_SW, DIR) |= BIT(PIN_SENSE_SW);
    
    GPIO(PORT_RADIO_SW, OUT) &= ~BIT(PIN_RADIO_SW);
    GPIO(PORT_RADIO_SW, DIR) |= BIT(PIN_RADIO_SW);
    
    capybara_shutdown_on_deep_discharge(); 
    msp_watchdog_disable();
    msp_gpio_unlock();
    capybara_config_pins();
    msp_clock_setup(); 
    INIT_CONSOLE(); 
    __enable_interrupt(); 
    PRINTF("Done handler\r\n"); 
    //delay(400); 
    //P3OUT |= BIT6;
    //P3DIR |= BIT6;
    /*
    GPIO(PORT_DEBUG, OUT) |= BIT(PIN_DEBUG); 
    GPIO(PORT_DEBUG, DIR) |= BIT(PIN_DEBUG); 
    
    msp_clock_setup(); 
    INIT_CONSOLE(); 
    __enable_interrupt(); 
    LIBCHAIN_PRINTF("Testing in main!\r\n"); 
    */
   /* if(burst_status == 2){
        prechg_status = 0; 
        burst_status = 0; 
    }*/
    // TODO good grief, set up a switch statement! 
    // First check precharge state & run precharge if need be
    capybara_config_banks(base_config.banks); 
    capybara_wait_for_supply();  
  /*
  task_cfg_spec_t curpwrcfg = curctx->task->spec_cfg; 

    if(curpwrcfg == PREBURST ){
        if(!prechg_status){
            prechg_config.banks = curctx->task->precfg->banks; 
            capybara_config_banks(prechg_config.banks);
            prechg_status = 1; 
            capybara_shutdown(); 
            capybara_wait_for_supply(); 
        }
        // But if we are precharged, config to the task's operating config
        else{
          burst_status = 0; 
          base_config.banks = curctx->task->opcfg->banks; 
          capybara_config_banks(base_config.banks); 
        }
    }
    // Next check if there is an ongoing burst or if we finished a burst, but
    // died before writing it down... TODO figure out if this is actually a
    // concern
    else if(curpwrcfg == BURST){
        //P3OUT |= BIT6;
        //P3DIR |= BIT6;
        if(burst_status == 2){
            prechg_status = 0; 
            burst_status = 0; 
        }
        else{
            //P3OUT &= ~BIT6;
            //P3DIR |= BIT6;
            // We kick into this loop if we failed to complete a burst function
            // Need to be careful here- a burst task MUST be able to complete in
            // its allotted energy level, o/w we'll just continue trying in vain
            capybara_config_banks(prechg_config.banks); 
            capybara_wait_for_supply(); 
        } 
    }
    else{ 
        burst_status = 0; 
        // Check if the task we're executing now has a special power requirement
        if(curpwrcfg == CONFIGD){
            base_config.banks = curctx->task->opcfg->banks; 
        }
        // Finally, just re-up the standard bank config
        capybara_config_banks(base_config.banks); 
    }
    */
    //P3OUT &= ~BIT6; 
   // P3DIR |= BIT6;
}

void capybara_transition()
{     
    // need to explore exactly how we want BURST tasks to be followed --> should
    // we ever shutdown to reconfigure? Or should we always ride the burst wave
    // until we're out of energy? 
    // TODO no really, a case statement isn't going to kill you! 

    // Check previous burst state and register a finished burst
    if(burst_status){
        burst_status = 2; 
    }
    task_cfg_spec_t curpwrcfg = curctx->task->spec_cfg;  
    switch(curpwrcfg){
        case BURST:
            prechg_status = 0; 
            capybara_config_banks(prechg_config.banks); 
            burst_status = 1; 
            break; 
        
        case PREBURST:
            if(!prechg_status){
                prechg_config.banks = curctx->task->precfg->banks; 
                capybara_config_banks(prechg_config.banks);
                // Mark that we finished the config_banks_command
                prechg_status = 1; 
                capybara_shutdown(); 
                capybara_wait_for_supply(); 
            }
            //intentional fall through
        
        case CONFIGD: 
            if(base_config.banks != curctx->task->opcfg->banks){
                base_config.banks = curctx->task->opcfg->banks; 
                capybara_config_banks(base_config.banks); 
                capybara_wait_for_supply();  
            }
            //Another intentional fall through 

        default: 
            //capybara_config_banks(base_config.banks); 
            //capybara_wait_for_supply();  
            break; 
    }
            
  /*  
    // HaNDLe a burst
    if(curpwrcfg == BURST){
        prechg_status = 0; 
        capybara_config_banks(prechg_config.banks); 
        burst_status = 1; 
        //capybara_wait_for_supply(); 
    }
    else{
        burst_status = 0; 
        // Set up a precharge in response to a preburst task if we haven't
        // precharged already.
        if( curpwrcfg == PREBURST && !prechg_status){
            prechg_config.banks = curctx->task->precfg->banks; 
            capybara_config_banks(prechg_config.banks);
            // Mark that we finished the config_banks_command
            prechg_status = 1; 
            capybara_shutdown(); 
            capybara_wait_for_supply(); 
        }
        // Handle a new config, either from a CONFIGD task a preburst task 
        if(curpwrcfg == CONFIGD || curpwrcfg == PREBURST){
            base_config.banks = curctx->task->opcfg->banks;  
            capybara_config_banks(base_config.banks); 
            capybara_wait_for_supply();  
        } 
    }
   */
   LOG("Running task %u \r\n",curctx->task->idx); 
    
}

void init()
{
  _capybara_handler(); 

 /* msp_clock_setup(); 
  INIT_CONSOLE(); 
  __enable_interrupt(); 
  */
  //PRINTF("Starting init\r\n"); 
  //Now send init commands to the apds
  //initializeHardware();
  //delay(4000); 
  //PRINTF("gesture app booted pchg %u burst %u \r\n", prechg_status, burst_status);
  //LOG("Running task %u \r\n",curctx->task->idx); 
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
{		//LOG("Starting HW setup \r\n"); 
    
    //GPIO(PORT_DEBUG, OUT) &= ~BIT(PIN_DEBUG_2); 
    //GPIO(PORT_DEBUG, DIR) |= BIT(PIN_DEBUG_2); 

    #ifdef USE_PHOTORES
    /*  
        P3SEL0 |= BIT1; 
        P3SEL1 |= BIT1; 
        PM5CTL0 &= ~LOCKLPM5; 
    */
    #endif
    
    #ifndef  USE_PHOTORES
    i2c_setup();
		//LOG("i2c setup done \r\n"); 
		/*Iinitialize apds*/
		proximity_init(); 
		/*Now enable the proximity sensor*/
    enableProximitySensor(); 
        #ifndef PROX_ONLY
        enableGesture();  
        disableGesture(); 
        #endif
    #endif
		//LOG("APDS TEST v1:  curtsk %u\r\n", curctx->task->idx);
}

void task_init()
{   //base_config.banks = 0x1;
    //capybara_transition(); 
    burst_status = 0; 
    prechg_status = 0; 
    prechg_config.banks = 0x0; 
    base_config.banks = 0x0; 

    task_prologue();
    //LOG("init\r\n");
		//Init task_gestCalc fields
		uint8_t i; 
		for(i = 0; i < MAX_GESTS; i++){
			gest_dir  dataInit = DIR_NONE; 
			CHAN_OUT1(gest_dist, gestures[i], dataInit, CH(task_init, task_gestCalc)); 
		}
		uint16_t gestInit = 0; 
		CHAN_OUT1(uint16_t, num_gests, gestInit, CH(task_init, task_gestCalc)); 
		/*Set initial power config here, don't forget a delay!*/ 
    //LOG("SANITY CHECK \r\n"); 
    //issue_precharge(0xF); 
		TRANSITION_TO(task_sample);
}

void task_sample()
{ 
  capybara_transition();
  task_prologue();
	//LOG("In task sample!!!\r\n"); 

//TODO double check that it's not a problem to use the apds output as an int16_t... 
int16_t proxVal = 0; 
#ifdef USE_PHOTORES
    enable_photoresistor(); 
//while(1){
    proxVal = read_photoresistor(); 
#else
    proxVal = readProximity();
    delay(240000); 
#endif

#if LOG_PROX
    PRINTF("proxVal = %i stats:%i %i\r\n",proxVal,burst_status, prechg_status); 
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

    P3OUT |= BIT6;
		
		TRANSITION_TO(task_gestCapture);
	}
//}
else{
  //P3OUT |= BIT6;
  TRANSITION_TO(task_sample);
	
	}

}

void task_gestCapture()
{   capybara_transition();
    task_prologue();
		uint8_t flag = *CHAN_IN1(uint8_t, flag, CH(task_sample, task_gestCapture));  
		PRINTF("Running gesture %i %i\r\n", burst_status, prechg_status);
		uint8_t stale = *CHAN_IN2(uint8_t, stale, SELF_IN_CH(task_gestCapture),
															CH(task_sample, task_gestCapture));
		stale = 1; 
		/*Mark that we've started a gesture*/ 
		CHAN_OUT1(uint8_t, stale, stale, SELF_OUT_CH(task_gestCapture)); 
		uint8_t num_samps = 0; 
		gesture_data_t gesture_data_; 
		// Grab gesture
    if(flag > 0){
      // Turn on sensor power supply
      GPIO(PORT_SENSE_SW, OUT) |= BIT(PIN_SENSE_SW);
      msp_sleep(30 /* cycles @ ACLK=VLOCLK=~10kHz ==> ~3ms */);
      //Make sure we come out of sleep...  
      
      //GPIO(PORT_DEBUG, OUT) |= BIT(PIN_DEBUG); 
      //GPIO(PORT_DEBUG, DIR) |= BIT(PIN_DEBUG);
      //Make sure VBOOST_OK is high
      capybara_wait_for_supply();
      // Do preliminary init stuff 
      i2c_setup();
      proximity_init(); 
      enableGesture(); 
      for(int num_attempts = 0; num_attempts < 10; num_attempts++){
          reenableGesture();  
          
          resetGestureFields(&gesture_data_); 
          //PRINTF("gestloop now!\r\n"); 
          int8_t gestVal = getGestureLoop(&gesture_data_, &num_samps);
          PRINTF("OUT OF GESTURE LOOP, num samps = %u, min = %u \r\n",
                    num_samps, MIN_DATA_SETS); 	
          if(num_samps > MIN_DATA_SETS)
              break; 
      }
		}
	  	
    if(num_samps > MIN_DATA_SETS){
				CHAN_OUT1(gesture_data_t, gesture_data_sets, gesture_data_, 
																											CH(task_gestCapture,task_gestCalc)); 
			  //LOG("transitioning to final calc!\r\n"); 
				TRANSITION_TO(task_gestCalc);
		}
		else{
			/*Didn't capture enough data to decide*/ 
			TRANSITION_TO(task_sample); 
		}
}

void task_gestCalc()
{   capybara_transition();
    task_prologue();
   	PRINTF("Computing gesture... \r\n"); 
		gesture_data_t gest_vals = *CHAN_IN1(gesture_data_t, gesture_data_sets,
																								CH(task_gestCapture, task_gestCalc)); 
		uint8_t i,j, num_samps = 4;
		
		gest_dir output = decodeGesture();
    PRINTF("------------------Dir = %u ---------------", output); 	
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

INIT_FUNC(init)
//TRANSITION_FUNC(capybara_transition)
ENTRY_TASK(task_init)

