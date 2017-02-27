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
#define NUM_SAMPS  32

// If you link-in wisp-base, then you have to define some symbols.
uint8_t usrBank[USRBANK_SIZE];

struct msg_self_index{
	SELF_CHAN_FIELD(uint16_t, index); 
};
#define FIELD_INIT_msg_self_index { \
	SELF_FIELD_INITIALIZER \
}

struct msg_index{
	CHAN_FIELD(uint16_t, index); 
};

struct msg_samp_info{
	CHAN_FIELD(uint16_t, index); 
	CHAN_FIELD(uint8_t, sample); 
};

struct msg_self_samples{
	SELF_CHAN_FIELD_ARRAY(uint8_t, samples, NUM_SAMPS); 
	SELF_CHAN_FIELD(uint8_t, anoms); 
};
#define FIELD_INIT_msg_self_samples { \
	SELF_FIELD_ARRAY_INITIALIZER(NUM_SAMPS), \
	SELF_FIELD_INITIALIZER \
}

struct msg_samples{
	CHAN_FIELD_ARRAY(uint8_t, samples, NUM_SAMPS); 
	CHAN_FIELD(uint8_t, anoms); 
};

struct msg_self_warningParams{
	SELF_CHAN_FIELD_ARRAY(uint8_t, averages, NUM_AVGS); 
	SELF_CHAN_FIELD(uint8_t, baseline); 
	SELF_CHAN_FIELD(uint8_t, dev); 
}; 
#define FIELD_INIT_msg_self_warningParams { \
	SELF_FIELD_ARRAY_INITIALIZER(NUM_AVGS), \
	SELF_FIELD_INITIALIZER, \
	SELF_FIELD_INITIALIZER \
}

struct msg_warningParams{
	CHAN_FIELD_ARRAY(uint8_t, averages, NUM_AVGS); 
	CHAN_FIELD_ARRAY(uint8_t, samples, NUM_SAMPS); 
	CHAN_FIELD(uint8_t, baseline); 
	CHAN_FIELD(uint8_t, dev); 
}; 


TASK(1, task_init)
TASK(2, task_sample)
TASK(3, task_detect)
TASK(4, task_average)

CHANNEL(task_init, task_sample, msg_index); 
CHANNEL(task_init, task_detect, msg_samples); 
CHANNEL(task_init, task_average, msg_warningParams); 

SELF_CHANNEL(task_sample, msg_self_index); 
CHANNEL(task_sample, task_detect, msg_samp_info); 

SELF_CHANNEL(task_detect, msg_self_samples); 
CHANNEL(task_detect, task_average, msg_samples); 

SELF_CHANNEL(task_average, msg_self_warningParams); 
CHANNEL(task_average, task_detect, msg_warningParams); 

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

static void delay(uint32_t cycles)
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

    //INIT_CONSOLE();

    //__enable_interrupt();

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
    // Solid flash signifying beginning of task
    GPIO(PORT_LED_1, OUT) |= BIT(PIN_LED_1);
    GPIO(PORT_LED_2, OUT) |= BIT(PIN_LED_2);
    burn(INIT_TASK_DURATION_ITERS);
    GPIO(PORT_LED_1, OUT) &= ~BIT(PIN_LED_1);
    GPIO(PORT_LED_2, OUT) &= ~BIT(PIN_LED_2);
    burn(INIT_TASK_DURATION_ITERS);
		//Init task_sample fields
		uint16_t indexInit = 0; 
		CHAN_OUT1(uint16_t, index, indexInit, CH(task_init, task_sample)); 
		//Init task_detect fields
		uint8_t i; 
		for(i = 0; i < NUM_SAMPS; i++){
			uint8_t dataInit = 0; 
			CHAN_OUT1(uint8_t, samples[i], dataInit, CH(task_init, task_detect)); 
		}
		uint16_t anomInit = 0; 
		CHAN_OUT1(uint16_t, anoms, anomInit, CH(task_init, task_detect)); 
		//Init task_average fields
		for(i = 0; i < NUM_AVGS; i++){
			uint8_t avgInit = 0; 
			CHAN_OUT1(uint8_t, averages[i], avgInit, CH(task_init, task_average));
		}
		for(i = 0; i < NUM_SAMPS; i++){
			uint8_t dataInit = 0; 
			CHAN_OUT1(uint8_t, samples[i], dataInit, CH(task_init, task_average)); 
		}
		uint8_t baseInit = 0; 
		CHAN_OUT1(uint8_t, baseline, baseInit, CH(task_init, task_average)); 
		uint8_t devInit = 0xFF; 
		CHAN_OUT1(uint8_t, dev, devInit, CH(task_init, task_average)); 

  	TRANSITION_TO(task_average);
}

void task_sample()
{
    task_prologue();
		LOG("running task_sample \r\n");
	//	delay(READ_PROX_DELAY_CYCLES); 
		burn(400000); 
	//	LOG("Delay done \r\n"); 
		uint16_t index = *CHAN_IN2(uint8_t, index, SELF_IN_CH(task_sample),
																				CH(task_init,task_sample));
	//	LOG("Index = %u \r\n", index); 
	//	LOG("Index = %u log still works\r\n", index); 
		uint8_t proxVal = readProximity(); 
		CHAN_OUT1(uint8_t,sample, proxVal, CH(task_sample, task_detect)); 
		index++; 
		CHAN_OUT1(uint16_t, index, index, SELF_OUT_CH(task_sample)); 
		LOG("proximity = %x \r\n", proxVal); 
		TRANSITION_TO(task_detect);
}

void task_detect()
{
    task_prologue();
		LOG("Running detect \r\n"); 
    uint8_t baseline = *CHAN_IN1(uint8_t, baseline,CH(task_average, task_detect)); 
		
    uint8_t dev = *CHAN_IN1(uint8_t, dev,CH(task_average, task_detect));
		uint8_t sample = *CHAN_IN1(uint8_t, sample, CH(task_sample, task_detect));
		uint16_t index = *CHAN_IN1(uint16_t, index, CH(task_sample, task_detect)); 
		int flag = anomalyCheck(sample, baseline, dev); 
		uint8_t anoms = *CHAN_IN2(uint8_t, anoms, SELF_IN_CH(task_detect), 
		 																				 CH(task_init, task_detect)); 
		if(flag < 0){
			LOG("ANOMALY DETECTED!\r\n"); 
			anoms++; 
		}
		
		CHAN_OUT1(uint8_t, samples[index], sample, SELF_OUT_CH(task_detect)); 
		CHAN_OUT1(uint8_t, samples[index], sample, CH(task_detect, task_average)); 
		CHAN_OUT1(uint8_t, anoms, anoms, SELF_OUT_CH(task_detect)); 
		if(index % NUM_SAMPS == 0){
			TRANSITION_TO(task_average);
		}
		else{
			TRANSITION_TO(task_sample); 
		}
}

void task_average()
{
    task_prologue();
   	LOG("Computing Average... \r\n"); 
		uint8_t dev = *CHAN_IN2(uint8_t, dev, SELF_IN_CH(task_average), 
																					CH(task_init, task_average)); 
		uint8_t baseline = *CHAN_IN2(uint8_t, baseline, SELF_IN_CH(task_average), 
																					CH(task_init, task_average)); 
		/*Run through and calculate an average + 1-3 quartile spread*/ 
		uint8_t samples[NUM_SAMPS]; 
		uint16_t i, newAvg = 0;
		for(i = 0; i < NUM_SAMPS; i++)
			samples[i] = 0; 
		for(i =0 ; i < NUM_SAMPS; i++){
			uint8_t curSamp =  *CHAN_IN2(uint8_t, samples[i], CH(task_detect, task_average),
																												CH(task_init, task_average)); 
			newAvg += curSamp; 
			uint16_t j; 
			for(j = 0; j < i; j++){
				if(curSamp >= samples[j] && curSamp < samples[j+1]){
					uint16_t k;
					for(k = i-1; k >= j; k--)
						samples[k+1] = samples[k];
				break; 
				}
			}
			samples[j] = curSamp; 
		}
		LOG("samples = "); 
		for(i = 0; i < NUM_SAMPS; i++)
			LOG("%x ", samples[i]); 
		LOG("\r\n"); 
		/*Resetting dev here, could use old dev in more complicated fitler*/ 
		dev = samples[NUM_SAMPS/4 + NUM_SAMPS/2] - samples[NUM_SAMPS/4];
		newAvg = newAvg / NUM_SAMPS;
		LOG("Avg = %x, Dev = %x \r\n", newAvg, dev); 
		
		CHAN_OUT1(uint8_t, dev, dev, CH(task_average, task_detect)); 
		CHAN_OUT1(uint8_t, baseline, baseline, CH(task_average, task_detect)); 
	  
		TRANSITION_TO(task_sample);
    
}

ENTRY_TASK(task_init)
INIT_FUNC(init)
