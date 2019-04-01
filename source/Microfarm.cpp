#include "sam.h"
#include "Microfarm.h"
#include "MicrofarmCommunications.h"

extern MicrofarmDataStruct sensor_data[4];
extern uint8_t testVar; // Temporary variable for storing test data

static uint8_t TEMPERATURE_COMMAND[] 	= {0x00, 0x04};
static uint8_t MOISTURE_COMMAND[] 		= {0x0F, 0x10};
static uint8_t ADDR[]					= {0x36, 0x37, 0x38, 0x39};

// Function Declarations
void updateSensors();
void prepareSensors();

/* What to do before running the loop */
void configure()
{
	disableWatchdog(); // disable during setup
	initializeSAMD09();
	solenoidOff(); 	// Ensure solenoid defaults off and gets turned off
					//		in the event of unexpected reset
	startWatchdog(); // enable now that we are starting!
	// Blink to signal restart
	for(int i = 0; i < 3; i++) superResetBlink();
}

/* Main Execution Super-Loop */
uint32_t counter = 0;
void loop()
{
	resetWatchdog(); // Tell WDT we are still running correctly
	listenForDAQ(); // Listen and Reply to DAQ

	if(counter > CYCLES_PER_SENSOR_POLL) // Every once and a while, check on the soil moisture sensor_data
	{
		counter = 0;
		LEDon();
		updateSensors();
		LEDoff();
	}

	counter++;
}

/* What to do to update the sensors */
void updateSensors()
{
	// DATA BUFFER
	uint8_t buffer[4];
	uint8_t anyOn = 0;

	// SET DEFAULTS AND INCREASE UID
	for(int i = 0; i < 4; i++)
	{
		sensor_data[i].temperature 	= 0;
		sensor_data[i].moisture 	= 0;
		sensor_data[i].online 		= 1;
		sensor_data[i].uid++;
	}

	// READ MOISTURES
	for(int i = 0; i < 4; i++)
	{
		if(!sendToSensor(ADDR[i], MOISTURE_COMMAND, 2)) sensor_data[i].online = 0;
		else anyOn = 1;
	}

	if(anyOn)
	{
		mydelay();
		for(int i = 0; i < 4; i++)
		{
			if(!sensor_data[i].online) continue;
			if(!readFromSensor(ADDR[i], buffer, 2)) sensor_data[i].online = 0;
			else sensor_data[i].moisture = ((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1];
		}

		anyOn = 0;
		// READ TEMPERATURES
		for(int i = 0; i < 4; i ++)
		{
			if(!sensor_data[i].online) continue;
			if(!sendToSensor(ADDR[i], TEMPERATURE_COMMAND, 2)) sensor_data[i].online = 0;
			else anyOn = 1;
		}
		if(anyOn)
		{
			mydelay();
			for(int i = 0; i < 4; i++)
			{
				if(!sensor_data[i].online) continue;
				if(!readFromSensor(ADDR[i], buffer, 4)) sensor_data[i].online = 0;
				else sensor_data[i].temperature = ((uint32_t)buffer[0] << 24) |
					((uint32_t)buffer[1] << 16) | ((uint32_t)buffer[2] << 8) | (uint32_t)buffer[3];
			}
		}
	}
}
