#include "sam.h"
#include "board_init.h"
#include "Microfarm.h"
#include "MicrofarmCommunications.h"

MicrofarmStatusStruct 	daq_status, sensor_status;
MicrofarmDataStruct 	sensor_data[4];

/* Functions for running I2CM Functions */
uint8_t 	i2cm_begin_write_operation(uint8_t addr);
uint8_t 	i2cm_begin_read_operation(uint8_t addr, uint8_t* buffer);
uint8_t 	i2cm_continue_write_byte(uint8_t buffer);
uint8_t 	i2cm_continue_read_byte(uint8_t* buffer);
void 		i2cm_end_transmission();
void 		handleI2CMaster();
/* Functions for running I2CS Functions */
void handleI2CSlave();
void handleDAQMessage();

uint8_t 	testVar = 		0x00;

/*********************/
/* Utility Functions */
/*********************/

void mydelay()
{
	for(int i = 0; i < MYDELAY_CYCLES; i++) asm("nop");
}

void solenoidOn()
{
	GPIO.OUTSET.reg = SOLENOID_VALVE;
}

void solenoidOff()
{
	GPIO.OUTCLR.reg = SOLENOID_VALVE;
}

void LEDwrite(uint8_t bits)
{
	setRed0(bits & 0b10000000);
	setGreen0(bits & 0b01000000);
	setYellow0(bits & 0b00100000);
	setBlue0(bits & 0b00010000);
	setRed1(bits & 0b00001000);
	setGreen1(bits & 0b00000100);
	setYellow1(bits & 0b00000010);
	setBlue1(bits & 0b00000001);
}

void superResetBlink()
{
	for(int i = 0; i < 8; i++)
	{
		LEDwrite((1 << i) + (1 << (4+i)));
		mydelay();
	}
	LEDwrite(0);
}

void superBlink()
{
	for(int i = 0; i < 8; i++)
	{
		LEDwrite(1 << i);
		mydelay();
	}
	LEDwrite(0);
}

void setRed0(uint8_t setting)
{
	if(setting) GPIO.OUTSET.reg = R_LED0;
	else GPIO.OUTCLR.reg = R_LED0;
}

void setRed1(uint8_t setting)
{
	if(setting) GPIO.OUTSET.reg = R_LED1;
	else GPIO.OUTCLR.reg = R_LED1;
}

void setGreen0(uint8_t setting)
{
	if(setting) GPIO.OUTSET.reg = G_LED0;
	else GPIO.OUTCLR.reg = G_LED0;
}

void setGreen1(uint8_t setting)
{
	if(setting) GPIO.OUTSET.reg = G_LED1;
	else GPIO.OUTCLR.reg = G_LED1;
}

void setYellow0(uint8_t setting)
{
	if(setting) GPIO.OUTSET.reg = Y_LED0;
	else GPIO.OUTCLR.reg = Y_LED0;
}

void setYellow1(uint8_t setting)
{
	if(setting) GPIO.OUTSET.reg = Y_LED1;
	else GPIO.OUTCLR.reg = Y_LED1;
}

void setBlue0(uint8_t setting)
{
	if(setting) GPIO.OUTSET.reg = B_LED0;
	else GPIO.OUTCLR.reg = B_LED0;
}

void setBlue1(uint8_t setting)
{
	if(setting) GPIO.OUTSET.reg = B_LED1;
	else GPIO.OUTCLR.reg = B_LED1;
}

void LEDon()
{
	GPIO.OUTSET.reg = BOARD_LED;
}

void LEDoff()
{
	GPIO.OUTCLR.reg = BOARD_LED;
}

void disableWatchdog()
{
	WDT->CTRL.bit.ENABLE = 0;
}

void startWatchdog()
{
	// 0xB is max timeout
	WDT->CONFIG.bit.PER = 0xB;
	WDT->CTRL.bit.ENABLE = 1;
	while(WDT->STATUS.bit.SYNCBUSY); // Synchronize WDT
}

void resetWatchdog()
{
	WDT->CLEAR.reg = 0xA5; // 0xA5 required for clearing
	while(WDT->STATUS.bit.SYNCBUSY); // Clear WDT
}

/**************************/
/* EXTERNAL I2C FUNCTIONS */
/**************************/

/* DAQ - Externally used to listen for incoming messages */
void listenForDAQ()
{
	// Just Clear Interrupts Until Data is Ready
	if(SLAVE_BUS.INTFLAG.bit.AMATCH) 	SLAVE_BUS.INTFLAG.bit.AMATCH 	= 1;
	if(SLAVE_BUS.INTFLAG.bit.PREC) 		SLAVE_BUS.INTFLAG.bit.PREC 		= 1;
	if(SLAVE_BUS.INTFLAG.bit.ERROR)		SLAVE_BUS.INTFLAG.bit.ERROR 	= 1;
	if(SLAVE_BUS.INTFLAG.bit.DRDY)
	{
		if(SLAVE_BUS.STATUS.bit.DIR)
			daq_status.change = COMMS_TXREQ;
		else
		{
			daq_status.change = COMMS_RX;
			daq_status.receive = SLAVE_BUS.DATA.bit.DATA; // Save Data
		}
		handleDAQMessage(); // Figure out what to do
		SLAVE_BUS.INTFLAG.bit.DRDY = 1; // Free Bus
	}
}

/* SENSOR - SEND BUFFER DATA TO ADDRESS */
uint8_t sendToSensor(uint8_t address, uint8_t *buf, uint8_t length)
{
	// Attempt to Start the Communication
	if(!i2cm_begin_write_operation(address))
	{
		i2cm_end_transmission();
		return 0;
	}

	// Attempt to Send the Bytes
	for(uint8_t i = 0; i < length; i++)
	{

		if(!i2cm_continue_write_byte(*(buf + i)))
		{
			i2cm_end_transmission();
			return 0;
		}
	}

	i2cm_end_transmission();
	return 1;
}

/* SENSOR - READ TO BUFFER FROM ADDRESS */
uint8_t readFromSensor(uint8_t address, uint8_t *buf, uint8_t length)
{

	if(!i2cm_begin_read_operation(address, buf))
	{
		i2cm_end_transmission();
		return 0;
	}

	for(uint8_t i = 1; i < length; i++)
	{
		if(!i2cm_continue_read_byte(buf + i))
		{
			i2cm_end_transmission();
			return 0;
		}
	}

	i2cm_end_transmission();
	return 1;
}

/**********************/
/* MASTER/SENSOR CODE */
/**********************/

void i2cm_end_transmission()
{
	MASTER_BUS.CTRLB.bit.CMD = 0x3; // STOP
	while(MASTER_BUS.SYNCBUSY.bit.SYSOP); // Sync CMD
}

uint8_t i2cm_begin_write_operation(uint8_t addr)
{
	sensor_status.change = COMMS_IDLE;
	MASTER_BUS.ADDR.bit.ADDR = (addr << 1) | 0;
	while(MASTER_BUS.SYNCBUSY.bit.SYSOP); // Sync ADDR

	while(sensor_status.change == COMMS_IDLE)
		handleI2CMaster();

	return sensor_status.change == COMMS_TX;
}

uint8_t i2cm_begin_read_operation(uint8_t addr, uint8_t* buffer)
{
	sensor_status.change = COMMS_IDLE;
	MASTER_BUS.ADDR.bit.ADDR = (addr << 1) | 1;
	while(MASTER_BUS.SYNCBUSY.bit.SYSOP); // Sync ADDR

	while(sensor_status.change == COMMS_IDLE) handleI2CMaster();

	if(sensor_status.change == COMMS_RX)
		(*buffer) = sensor_status.receive; // Comes with Data!

	//testVar = sensor_status.change;

	return sensor_status.change == COMMS_RX;
}

uint8_t i2cm_continue_write_byte(uint8_t buffer)
{
	if(sensor_status.change != COMMS_TX) return 0; // Can't Send if We Aren't Sending!
	sensor_status.change = COMMS_IDLE;
	MASTER_BUS.DATA.bit.DATA = buffer;
	while(sensor_status.change == COMMS_IDLE) handleI2CMaster();
	return sensor_status.change == COMMS_TX;
}

uint8_t i2cm_continue_read_byte(uint8_t* buffer)
{
	if(sensor_status.change != COMMS_RX) return 0; // Can't receive if off..
	sensor_status.change = COMMS_IDLE;
	//MASTER_BUS.CTRLB.bit.ACKACT = 0; // ACK (instead of 1 for NACK), but its default 0 anyway
	MASTER_BUS.CTRLB.bit.CMD = 0x2; // Request Next Byte
	while(MASTER_BUS.SYNCBUSY.bit.SYSOP); // Sync CMD

	while(sensor_status.change == COMMS_IDLE) handleI2CMaster();

	if(sensor_status.change == COMMS_RX)
		(*buffer) = sensor_status.receive;

	return sensor_status.change == COMMS_RX;
}

/* INTERRUPT HANDLER */
void handleI2CMaster()
{
	if(MASTER_BUS.INTFLAG.bit.MB)
	{
		if(MASTER_BUS.STATUS.bit.RXNACK) sensor_status.change = COMMS_RXNACK;
		else if(MASTER_BUS.STATUS.bit.BUSSTATE == 0x3) sensor_status.change = COMMS_BUSY;
		else sensor_status.change = COMMS_TX;
		// MASTER_BUS.INTFLAG.bit.MB = 1; // Cleared on next command or close!
	}
	if(MASTER_BUS.INTFLAG.bit.SB)
	{
		sensor_status.receive = MASTER_BUS.DATA.bit.DATA;
		sensor_status.change = COMMS_RX;
		// MASTER_BUS.INTFLAG.bit.SB = 1; // Cleared on next command or close!
	}
	if(MASTER_BUS.INTFLAG.bit.ERROR)
	{
		sensor_status.change = COMMS_ERROR;
		MASTER_BUS.INTFLAG.bit.ERROR = 1; // Clear
	}
}

/******************/
/* SLAVE/DAQ CODE */
/******************/
void handleDAQMessage()
{
	if(daq_status.change == COMMS_RX)
	{
		switch(daq_status.receive)
		{
			case COMMAND_SOLENOIDON:
				GPIO.OUTSET.reg = SOLENOID_VALVE;
				break;
			case COMMAND_SOLENOIDOFF:
				GPIO.OUTCLR.reg = SOLENOID_VALVE;
				break;
			case COMMAND_TEST:
				// Possible Test Command
				superBlink();
				break;
		}
	}
	else if(daq_status.change == COMMS_TXREQ)
	{
		for(uint8_t i = 0; i < 4; i++)
		{
			// UID COMMANDS
			if(daq_status.receive == COMMAND_UID(i))
				SLAVE_BUS.DATA.bit.DATA = sensor_data[i].uid;
			// ONLINE COMMANDS
			else if(daq_status.receive == COMMAND_ONLINE(i))
				SLAVE_BUS.DATA.bit.DATA = sensor_data[i].online;
			// MOISTUREA (MSB)
			else if(daq_status.receive == COMMAND_MOISTUREA(i))
				SLAVE_BUS.DATA.bit.DATA = (sensor_data[i].moisture & 0xFF00) >> 8; // select MSB
			// MOISTUREB (LSB)
			else if(daq_status.receive == COMMAND_MOISTUREB(i))
				SLAVE_BUS.DATA.bit.DATA = sensor_data[i].moisture & 0x00FF; // select LSB
			// TEMPERATUREA (MSB)
			else if(daq_status.receive == COMMAND_TEMPERATUREA(i))
				SLAVE_BUS.DATA.bit.DATA = (sensor_data[i].temperature & 0xFF000000) >> 24;
			else if(daq_status.receive == COMMAND_TEMPERATUREB(i))
				SLAVE_BUS.DATA.bit.DATA = (sensor_data[i].temperature & 0x00FF0000) >> 16;
			else if(daq_status.receive == COMMAND_TEMPERATUREC(i))
				SLAVE_BUS.DATA.bit.DATA = (sensor_data[i].temperature & 0x0000FF00) >> 8;
			// TEMPERATURED (LSB)
			else if(daq_status.receive == COMMAND_TEMPERATURED(i))
				SLAVE_BUS.DATA.bit.DATA = sensor_data[i].temperature & 0x000000FF;
		}
	}
}

void handleI2CSlave()
{
	if(SLAVE_BUS.INTFLAG.bit.AMATCH)
	{
		SLAVE_BUS.INTFLAG.bit.AMATCH = 1; // Clear
	}
	if(SLAVE_BUS.INTFLAG.bit.DRDY)
	{
		if(SLAVE_BUS.STATUS.bit.DIR)
			daq_status.change = COMMS_TXREQ;
		else
		{
			daq_status.change = COMMS_RX;
			daq_status.receive = SLAVE_BUS.DATA.bit.DATA;
		}

		SLAVE_BUS.INTFLAG.bit.DRDY = 1; // Clear
	}
	if(SLAVE_BUS.INTFLAG.bit.PREC)
	{
		daq_status.change = COMMS_IDLE;
		SLAVE_BUS.INTFLAG.bit.PREC = 1; // Clear
	}
	if(SLAVE_BUS.INTFLAG.bit.ERROR)
	{
		daq_status.change = COMMS_ERROR;
		SLAVE_BUS.INTFLAG.bit.ERROR = 1; // Clear
	}
}

/******************/
/* EXTERNAL:      */
/* INITIALIZATION */
/******************/
void initializeSAMD09()
{
	// Initialize the SAM system
    SystemInit();
	// Adafruit's Initialization (rather than figuring out how to start from scratch)
	board_init();

	// Power up I2C Interfaces (running through SERCOM modules)
	PM->APBCMASK.reg |= (1 << 2) + (1 << 3); // Enable SERCOM0 and SERCOM1

	// SETUP WDT CLOCK
	// (CLK[3:0] | DIV[7:0] << 8)
	GCLK->GENDIV.reg = 0x4 | (0x03 << 8);
	while(GCLK->STATUS.bit.SYNCBUSY);
	// (CLK[3:0] | SRC[3:0] << 8 | GENEN << 16 | DIVSEL << 20)
	GCLK->GENCTRL.reg = 0x4 | (0x4 << 8) | (1 << 16) | (1 << 20);
	while(GCLK->STATUS.bit.SYNCBUSY);
	// (DEVICE[5:0] | CLK[3:0] << 8 | CLKEN << 14)
	GCLK->CLKCTRL.reg = 0x3 | (0x4 << 8) | (1 << 14);
	while(GCLK->STATUS.bit.SYNCBUSY);

	// Setup I2CS/I2SM clocks
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_SERCOM0_CORE) | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_CLKEN;
	while(GCLK->STATUS.bit.SYNCBUSY);
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_SERCOM1_CORE) | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_CLKEN;
	while(GCLK->STATUS.bit.SYNCBUSY);
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_SERCOMX_SLOW) | GCLK_CLKCTRL_GEN_GCLK1 | GCLK_CLKCTRL_CLKEN;
	while(GCLK->STATUS.bit.SYNCBUSY);



	// Setup ATSAMD09 pin connections
	GPIO.DIRSET.reg 				= BOARD_LED; 		// Board LED
	GPIO.DIRSET.reg 				= R_LED0 | R_LED1 | G_LED0 | G_LED1 | Y_LED0 | Y_LED1 | B_LED0 | B_LED1;
	GPIO.DIRSET.reg 				= GROUND_PIN;		// Pin for ground level
	GPIO.DIRSET.reg					= SOLENOID_VALVE; 	// SOLENOID_VALVE

	GPIO.OUTCLR.reg					= GROUND_PIN;		// Make ground pin ground

	GPIO.PINCFG[14].bit.PMUXEN 		= 1; 	// Allow I2CS to take over PA14 (SDA)
	GPIO.PINCFG[15].bit.PMUXEN 		= 1; 	// ^ but with PA15 (SCL)
	GPIO.PMUX[7].bit.PMUXE 			= 0x2; 	// I2CS takeover PA14
	GPIO.PMUX[7].bit.PMUXO 			= 0x2; 	// I2CS takeover PA15

	GPIO.PINCFG[22].bit.PMUXEN 		= 1; 	// Allow I2CM to take over PA22 (SDA)
	GPIO.PINCFG[23].bit.PMUXEN 		= 1; 	// ^ but with PA23 (SCL)
	GPIO.PMUX[11].bit.PMUXE 		= 0x2; 	// I2CM takeover PA22
	GPIO.PMUX[11].bit.PMUXO 		= 0x2; 	// I2CM takeover PA23



	// Configure I2CS
	SLAVE_BUS.CTRLA.bit.MODE 		= 0x4;	// Configure as slave bus
	//SLAVE_BUS.CTRLB.bit.SMEN 		= 1;	// TODO: Setup Smart Mode (Requires DMAC Configuration!)
											// Note: Don't really need smart mode, not really worth the hassle
	SLAVE_BUS.CTRLB.bit.AMODE 		= 0x0;	// Set address mode to masked
	SLAVE_BUS.ADDR.bit.ADDR 		= SEESAW_SLAVE_ADDRESS; // Set address to this value
	SLAVE_BUS.ADDR.bit.ADDRMASK 	= 0x00; // Set address mask to blank (exact match only)
	SLAVE_BUS.CTRLA.bit.ENABLE 		= 1;	// Enable Slave Bus
	while(SLAVE_BUS.SYNCBUSY.bit.ENABLE);

	// ENABLE INTERRUPTS
	SLAVE_BUS.INTENSET.bit.AMATCH 	= 1; // ADDRESS MATCH
	SLAVE_BUS.INTENSET.bit.DRDY 	= 1; // DATA READY
	SLAVE_BUS.INTENSET.bit.PREC 	= 1; // STOP
	SLAVE_BUS.INTENSET.bit.ERROR 	= 1; // VARIOUS ERRORS



	// Configure I2CM
	MASTER_BUS.CTRLA.bit.MODE 		= 0x5;	// I2C Master
	MASTER_BUS.BAUD.bit.BAUD 		= 0x0F; // Not sure what rate this is
	MASTER_BUS.CTRLA.bit.ENABLE 	= 1;	// Enable Master Bus
	while(MASTER_BUS.SYNCBUSY.bit.ENABLE);

	MASTER_BUS.STATUS.bit.BUSSTATE 	= 0x1;	// Set Bus to IDLE
	while(MASTER_BUS.SYNCBUSY.bit.SYSOP);	// Wait for Synchronization

	// ENABLE INTERRUPTS
	MASTER_BUS.INTENSET.bit.MB 		= 1; // MASTER ON BUS
	MASTER_BUS.INTENSET.bit.SB 		= 1; // SLAVE ON BUS
	MASTER_BUS.INTENSET.bit.ERROR 	= 1; // VARIOUS ERRORS
}
