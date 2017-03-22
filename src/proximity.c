#include <libmsp/mem.h>
#include <libio/log.h>

#include "proximity.h"
#include "libmspware/driverlib.h"

volatile unsigned char proximityId = 0;

void proximity_init(void) {
	uint8_t proximityID = 0;  
	uint8_t sensorID = 0; 
	//Already init'd i2c, now init connection to APDS
  EUSCI_B_I2C_setSlaveAddress(EUSCI_B0_BASE, APDS9960_I2C_ADDR); 
  EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);

  EUSCI_B_I2C_enable(EUSCI_B0_BASE);
  //Might not need from here: 
  while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));

  //EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);

  EUSCI_B_I2C_masterSendSingleByte(EUSCI_B0_BASE, APDS9960_ID);
  
	while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));
  
	EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_RECEIVE_MODE);

  EUSCI_B_I2C_masterReceiveStart(EUSCI_B0_BASE);

  proximityID = EUSCI_B_I2C_masterReceiveSingle(EUSCI_B0_BASE);
  
	EUSCI_B_I2C_masterReceiveMultiByteStop(EUSCI_B0_BASE);
  
  while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));
	/*Transmit address and read back returned value*/	
	restartTransmit();
	writeSingleByte(APDS9960_ID); 
	sensorID = readDataByte(); 

	LOG("I2C ID = %x \r\n",proximityID); 
	LOG("I2C ID2 = %x \r\n",sensorID); 
/*	
	EUSCI_B_I2C_disable(EUSCI_B0_BASE); 
  
	EUSCI_B_I2C_setSlaveAddress(EUSCI_B0_BASE, APDS9960_I2C_ADDR); 
  EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);

  EUSCI_B_I2C_enable(EUSCI_B0_BASE);
*/
	restartTransmit(); 
  writeDataByte(APDS9960_ENABLE, 0); 
	/*Run through and set a bundle of defaults*/
	writeDataByte(APDS9960_ATIME, DEFAULT_ATIME);
	writeDataByte(APDS9960_WTIME, DEFAULT_WTIME);
	writeDataByte(APDS9960_PPULSE, DEFAULT_PROX_PPULSE);
	writeDataByte(APDS9960_POFFSET_UR, DEFAULT_POFFSET_UR);
	writeDataByte(APDS9960_POFFSET_DL, DEFAULT_POFFSET_DL);
	writeDataByte(APDS9960_CONFIG1, DEFAULT_CONFIG1);
	restartTransmit(); 
	writeSingleByte(APDS9960_CONTROL); 
	uint8_t val = readDataByte(); 
	/*Set led drive*/ 
	uint8_t drive = DEFAULT_LDRIVE;
	drive = drive << 6; 
	val &= 0x3F; 
	val |= drive; 
	/*Set proximity gain*/ 
	drive = DEFAULT_PGAIN;
	drive &= 0x3; 
	drive = drive << 2; 
	val &= 0xF3; 
	val |= drive;
	/*Set ambient light gain*/
	drive = DEFAULT_AGAIN; 
	drive &= 0x3; 
	val &= 0xFC; 
	val |= drive; 
	/*Write all changes*/
	restartTransmit(); 
	writeDataByte(APDS9960_CONTROL,val); 
	writeDataByte(APDS9960_PILT, DEFAULT_PILT);
	writeDataByte(APDS9960_PIHT, DEFAULT_PIHT); 
	/*Set low thresh for ambient light interrupts*/
	uint16_t thresh = DEFAULT_AILT; 
	uint8_t lowByte = thresh & 0x00FF;
	uint8_t highByte = (thresh & 0xFF00) >> 8; 
	writeDataByte(APDS9960_AILTL, lowByte); 
	writeDataByte(APDS9960_AILTH, highByte); 
	/*Set high threshold for ambient light interrupts*/ 
	thresh = DEFAULT_AIHT; 
	lowByte = thresh & 0x00FF;
	highByte = (thresh & 0xFF00 >> 8); 
	writeDataByte(APDS9960_AIHTL, lowByte); 
	writeDataByte(APDS9960_AIHTH, highByte); 
	writeDataByte(APDS9960_PERS, DEFAULT_PERS); 
	writeDataByte(APDS9960_CONFIG2, DEFAULT_CONFIG2); 
	writeDataByte(APDS9960_CONFIG3, DEFAULT_CONFIG3); 


	 /* switching to the gesture stuff*/
	writeDataByte(APDS9960_GPENTH, DEFAULT_GPENTH); 
	writeDataByte(APDS9960_GEXTH, DEFAULT_GEXTH); 
	writeDataByte(APDS9960_GCONF1, DEFAULT_GCONF1);
	/*Tell device which reg to return, read vals,  modify a couple bits, and write it back*/ 
	/*This sequence sets gain for gesture engine*/ 
	restartTransmit(); 
	writeSingleByte(APDS9960_GCONF2);  
	val = readDataByte(); 
	LOG("start val = %x \r\n",val); 
	uint8_t gain = DEFAULT_GGAIN; 
	gain &= 0x03; 
	gain = gain << 5; 
	val &= 0x9F; 
	val |= gain; 
	drive = DEFAULT_GLDRIVE; 
	drive &= 0x03; 
	drive = drive << 3; 
	val &= 0xD7; 
	val |= drive; 
	uint8_t time = DEFAULT_GWTIME; 
	time &= 0x07; 
	val &= 0xF8; 
	val |= time; 	
	LOG("final val = %x \r\n",val); 
	restartTransmit(); 
	writeDataByte(APDS9960_GCONF2, val);
	/*Now write a bunch of values in the usual way...*/ 
	writeDataByte(APDS9960_GCONF2, val); 
	writeDataByte(APDS9960_GOFFSET_U, DEFAULT_GOFFSET);
	writeDataByte(APDS9960_GOFFSET_D, DEFAULT_GOFFSET);
	writeDataByte(APDS9960_GOFFSET_L, DEFAULT_GOFFSET);
	writeDataByte(APDS9960_GOFFSET_R, DEFAULT_GOFFSET);
	writeDataByte(APDS9960_GPULSE, DEFAULT_GPULSE);
	writeDataByte(APDS9960_GCONF3, DEFAULT_GCONF3);
	/*Sanity check on GCONF2*/
	restartTransmit(); 
	writeSingleByte(APDS9960_GCONF2); 
	uint8_t test = readDataByte(); 
	LOG("Gconf = %x \r\n",test); 
	/*enable gesture interrupt with default val*/
	restartTransmit(); 
	writeSingleByte(APDS9960_GCONF4);
	val = readDataByte(); 
	uint8_t enable = DEFAULT_GIEN;
	enable &= 0x01; 
	enable = enable << 1; 
	val &= 0xFD; 
	val |= enable; 
	restartTransmit(); 
	writeDataByte(APDS9960_GCONF4, val); 


  LOG("Proximity sensor set up. ID:  %x\r\n", proximityID);
	//	enableProximitySensor(); 
	restartTransmit(); 
	writeSingleByte(APDS9960_ID); 
	val = readDataByte(); 
	LOG("Val after prox enable = %x \r\n", val); 
	return; 
}

void enableProximitySensor(void){
	/*Set proximity gain*/
	restartTransmit(); 
	writeSingleByte(APDS9960_CONTROL); 
	uint8_t val = readDataByte(); 
	uint8_t drive = DEFAULT_PGAIN;  
	drive &= 0x3;
	drive = drive << 2; 
	val &= 0xF3; 
	val |=drive; 
	restartTransmit(); 
	writeDataByte(APDS9960_CONTROL, val); 
	/*Set LED drive*/
	restartTransmit(); 
	writeSingleByte(APDS9960_CONTROL); 
	val = readDataByte(); 
	/*Set led drive*/ 
	drive = DEFAULT_LDRIVE;
	drive = drive << 6; 
	val &= 0x3F; 
	val |= drive; 
	restartTransmit(); 
	writeDataByte(APDS9960_CONTROL, val); 
	/*Disable interrupt*/
	restartTransmit(); 
	writeSingleByte(APDS9960_ENABLE); 
	val = readDataByte(); 
	uint8_t enable = 0; 
	val &= 0xDF; 
	restartTransmit(); 
	writeDataByte(APDS9960_CONTROL, val); 
	/*Enable power*/
	restartTransmit(); 
	writeSingleByte(APDS9960_ENABLE); 
	val = readDataByte(); 
	val |= (1 << POWER);
	restartTransmit(); 
	writeDataByte(APDS9960_ENABLE,val); 
	/*Set proximity mode*/
	restartTransmit(); 
	writeSingleByte(APDS9960_ENABLE); 
	val = readDataByte(); 
	val |= (1 << PROXIMITY);
	restartTransmit(); 
	writeDataByte(APDS9960_ENABLE,val); 

	return; 
}

uint8_t readProximity(){
	uint8_t val = 0; 
	restartTransmit(); 
	//LOG("Waiting on write2 \r\n"); 
	writeSingleByte(APDS9960_PDATA); 
	//LOG("Waiting on read2 \r\n"); 
	/*Begin added section*/ 
	EUSCI_B_I2C_disable(EUSCI_B0_BASE); 
  EUSCI_B_I2C_setSlaveAddress(EUSCI_B0_BASE, APDS9960_I2C_ADDR); 
  EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_RECEIVE_MODE);
  EUSCI_B_I2C_enable(EUSCI_B0_BASE);
  EUSCI_B_I2C_masterReceiveStart(EUSCI_B0_BASE);
//	LOG("Waiting on read3 \r\n"); 
  val = EUSCI_B_I2C_masterReceiveSingle(EUSCI_B0_BASE);
//	LOG("Waiting on read4 \r\n"); 
  EUSCI_B_I2C_masterReceiveMultiByteStop(EUSCI_B0_BASE);

	/*End added section*/ 	
	//val = readDataByte(); 
	return val ; 
}

int8_t  getGesture(void){
	/*Test if a gesture is available*/
//	restartTransmit(); 
  EUSCI_B_I2C_disable(EUSCI_B0_BASE); 
	EUSCI_B_I2C_setSlaveAddress(EUSCI_B0_BASE, APDS9960_I2C_ADDR); 
  while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE)); 
	EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);
  EUSCI_B_I2C_enable(EUSCI_B0_BASE);
	writeSingleByte(APDS9960_ID); 
	uint8_t check = readDataByte();
	LOG("ID Check = %x \r\n",check); 
	restartTransmit(); 
	writeSingleByte(APDS9960_GSTATUS); //Check gstatus!
	uint8_t test = readDataByte(); 
	test &= APDS9960_GVALID;
	LOG("Gvalid  val= %x \r\n",test); 
	restartTransmit(); 
	writeSingleByte(APDS9960_ENABLE); 
	uint8_t enable = readDataByte(); 
	LOG("enable val = %x \r\n",enable); 
	enable &= 0x41; 
	test &= enable; 
	if(!test){
		return DIR_NONE; 
	}
	
		restartTransmit(); 
		writeSingleByte(APDS9960_GSTATUS); 
		uint8_t gstatus = readDataByte(); 
		LOG("gstatus val 1 = %x \r\n",gstatus); 
		if((gstatus & APDS9960_GVALID) == APDS9960_GVALID){
			restartTransmit(); 
			writeSingleByte(APDS9960_GFLVL); 
			uint8_t fifo_level = readDataByte(); 
			LOG("Fifo level = %u \r\n",fifo_level); 
			if(fifo_level > 1 ){
				/*Read in all of the bytes from the fifo*/ 
				restartTransmit(); 
				writeSingleByte(APDS9960_GFIFO_U);
				EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_RECEIVE_MODE);
				EUSCI_B_I2C_masterReceiveStart(EUSCI_B0_BASE);
				
				/*Note: we multiply by 4 to get UP,DOWN,LEFT,RIGHT captured*/ 
				uint8_t fifoContents[128];
				uint8_t i; 
				for(i = 0; i < fifo_level * 4; i++){
					fifoContents[i] =  EUSCI_B_I2C_masterReceiveSingle(EUSCI_B0_BASE);
				}
  			EUSCI_B_I2C_masterReceiveMultiByteStop(EUSCI_B0_BASE);
  			while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));
				for(i = 0; i < fifo_level * 4; i++){	
					if(i == 0 || i == fifo_level || i == fifo_level*2 || i == fifo_level*3){
						LOG("Got: ");
					}
					LOG("%u ", fifoContents[i]); 
					if(i == fifo_level -1 || i == fifo_level*2 -1 || i == fifo_level*3 -1 || i ==
								fifo_level*4 -1){
						LOG("\r\n ");
					}
				}
			}
		
			LOG("ESCAPED!\r\n"); 
		}
		LOG("ESCAPED ALL!\r\n"); 
		restartTransmit(); 
		/*Add in processing stuff here*/ 	
	return DIR_UP;  
}

void resetGestureFields(gesture_data_type *gesture){
	gesture->index = 0; 
	gesture->total_gestures = 0; 
	gesture->ud_delta = 0; 
	gesture->lr_delta = 0; 
	gesture->ud_count = 0; 
	gesture->lr_count = 0; 
	gesture->near_count = 0; 
	gesture->far_count = 0; 
	gesture->state = 0; 
	gesture->motion = DIR_NONE; 
	return; 
}

void enableGesture(void){
	uint8_t val, boost, enable, mode,test; 
	//	resetGestureFields(); 
	restartTransmit(); 
	/*Write 0 to ENABLE*/
	//writeDataByte(APDS9960_ENABLE, 0); 	
	writeDataByte(APDS9960_WTIME,0xFF); 
	writeDataByte(APDS9960_PPULSE, DEFAULT_GESTURE_PPULSE); 
	/*Set LED boost*/
	boost = LED_BOOST_300; 
	restartTransmit(); 
	writeSingleByte(APDS9960_CONFIG2); 
	val = readDataByte(); 
	LOG("CONFIG2 = %x \r\n", val); 
	boost &=0x3; 
	boost = boost << 4; 
	val &= 0xCF;
	val |= boost; 
	LOG("Writing %x to CONFIG2 \r\n", val); 
	restartTransmit(); 
	writeDataByte(APDS9960_CONFIG2, val); 
	restartTransmit(); 
	writeSingleByte(APDS9960_CONFIG2); 
	test = readDataByte(); 
	LOG("Confirmed Config2 = %x \r\n",test); 
	
	/*Enable gesture interrupt... for now*/
	enable = 1;
	restartTransmit(); 
	writeSingleByte(APDS9960_GCONF4); 
	val = readDataByte(); 
	LOG("GCONF4 = %x \r\n", val); 
	enable &= 0x1; 
	enable = enable << 1; 
	val &= 0xFD;
	val |= enable; 
	/*set gesture mode*/ 
	mode = 1; 
	mode  &= 0x1; 
	val &= 0xFE; 
	val |= mode;
	LOG("Writing %x to GCONF4 \r\n", val); 
	writeDataByte(APDS9960_GCONF4, val); 
	/*Enable power*/
	restartTransmit(); 
	writeSingleByte(APDS9960_ENABLE); 
	val = readDataByte();
	LOG("ENABLE = %x \r\n", val); 
	val |= (1 << POWER);
	restartTransmit(); 
	LOG("Writing %x to ENABLE \r\n", val); 
	writeDataByte(APDS9960_ENABLE,val); 
	/*Enable wait mode*/
	restartTransmit(); 
	writeSingleByte(APDS9960_ENABLE); 
	val = readDataByte(); 
	LOG("ENABLE = %x \r\n", val); 
	val |= (1 << WAIT); 
	restartTransmit(); 
	LOG("Writing %x to ENABLE \r\n", val); 
	writeDataByte(APDS9960_ENABLE,val); 
	/*Enable proximity mode*/ 
	restartTransmit(); 
	writeSingleByte(APDS9960_ENABLE); 
	val = readDataByte(); 
	LOG("ENABLE = %x \r\n", val); 
	val |= (1 << PROXIMITY); 
	restartTransmit(); 
	LOG("Writing %x to ENABLE \r\n", val); 
	writeDataByte(APDS9960_ENABLE,val); 
	/*Enable gesture mode*/ 
	restartTransmit(); 
	writeSingleByte(APDS9960_ENABLE); 
	val = readDataByte(); 
	LOG("ENABLE = %x \r\n", val); 
	val |= (1 << GESTURE);
	LOG("Writing %x to ENABLE \r\n", val); 
	restartTransmit(); 
	writeDataByte(APDS9960_ENABLE, val); 
	restartTransmit(); 
	writeSingleByte(APDS9960_ENABLE); 
	test = readDataByte(); 	
	LOG("Confirmed write %x to ENABLE \r\n"); 
	//EUSCI_B_I2C_disable(EUSCI_B0_BASE); 
	return ;
}

void check_mode(void){
	uint8_t val ;
	restartTransmit(); 
	writeSingleByte(APDS9960_ENABLE); 
	val = readDataByte(); 
	LOG("enable reg = %x \r\n", val); 
	return; 
}

/**
 *@brief decides if a sample is outside of the allowed window. 
 *@details really just a stand in for something more complicated
 */
int8_t anomalyCheck(uint8_t sample, uint8_t baseline, uint8_t allowedDev){
	//Bounds limits set by max/min values of uint8_t (0x0 and 0xFF)
	if(baseline >= allowedDev){
		//check out of bounds low
		if(sample < baseline - allowedDev)
			return -1; 
	}
	if(baseline + allowedDev <= 0xFF){
		//check out of bounds high 
		if(sample > baseline + allowedDev)
			return -1; 
	}
	return 1; 
}


/*
 *@brief consecutively writes register address and value over i2c
 *@details analogous to wirewritedatabyte in wire.h
 */
void writeDataByte(uint8_t reg, uint8_t val){
  EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);
  EUSCI_B_I2C_masterSendStart(EUSCI_B0_BASE);
  EUSCI_B_I2C_masterSendMultiByteNext(EUSCI_B0_BASE, reg);
  EUSCI_B_I2C_masterSendMultiByteNext(EUSCI_B0_BASE, val);
  EUSCI_B_I2C_masterSendMultiByteStop(EUSCI_B0_BASE);
  while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));
	return; 
}
/*
 *@brief writes a single value over i2c with appropriate waiting etc
 */
void writeSingleByte(uint8_t val){
  EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);
  EUSCI_B_I2C_masterSendSingleByte(EUSCI_B0_BASE, val);
  while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));
	return; 
}

/*
 *@brief reads a byte over i2c
 */ 
uint8_t readDataByte(){
	uint8_t val;  
	EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_RECEIVE_MODE);
  EUSCI_B_I2C_masterReceiveStart(EUSCI_B0_BASE);
  val = EUSCI_B_I2C_masterReceiveSingle(EUSCI_B0_BASE);
  EUSCI_B_I2C_masterReceiveMultiByteStop(EUSCI_B0_BASE);
  while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));
	return val; 
}

/*
 *@brief handles flip from transmit to receive
 */ 
void restartTransmit(void){
	EUSCI_B_I2C_disable(EUSCI_B0_BASE); 
  EUSCI_B_I2C_setSlaveAddress(EUSCI_B0_BASE, APDS9960_I2C_ADDR); 
  EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);
  EUSCI_B_I2C_enable(EUSCI_B0_BASE);
//  EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);
 
	while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));
}
