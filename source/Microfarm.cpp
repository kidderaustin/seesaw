#include "sam.h"
#include "Microfarm.h"
#include "MicrofarmCommunications.h"

extern MicrofarmDataStruct sensor_data[4];
uint16_t prior_moisture_data[4][4];
uint32_t prior_temperature_data[4][4];

static uint8_t TEMPERATURE_COMMAND[] 	= {0x00, 0x04};
static uint8_t MOISTURE_COMMAND[] 		= {0x0F, 0x10};
static uint8_t ADDR[]					= {0x36, 0x37, 0x38, 0x39};

// Function Declarations
void updateSensors();
void prepareSensors();
uint16_t filter_moisture(int addr, uint16_t current);
uint32_t filter_temperature(int addr, uint32_t current);

/* What to do before running the loop */
void configure()
{
	disableWatchdog(); // disable during setup
	initializeSAMD09();
	solenoidOff(); 	// Ensure solenoid defaults off and gets turned off
					//		in the event of unexpected reset

	// Initialize prior data to zero!
	for(int i = 0; i < 4; i++)
	{
		for(int j = 0; j < 4; j++)
		{
			prior_moisture_data[i][j] = 0;
			prior_temperature_data[i][j] = 0;
		}
	}


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
			else sensor_data[i].moisture = filter_moisture(i, ((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1]);
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
				else sensor_data[i].temperature = filter_temperature(i, ((uint32_t)buffer[0] << 24) |
					((uint32_t)buffer[1] << 16) | ((uint32_t)buffer[2] << 8) | (uint32_t)buffer[3]);
			}
		}
	}
}

// Filters are a median/mean filter (mean of center 3 values)

uint16_t filter_moisture(int addr, uint16_t current)
{
	// Calculate filtered value
	uint16_t filtered = current;
	uint16_t max = current, min = current;
	uint16_t indexed;
	for(int i = 0; i < 4; i++)
	{
		indexed = prior_moisture_data[addr][i];
		// Set possible new min/max
		if(indexed > max) max = indexed;
		else if(indexed < min) min = indexed;
		// Sum
		filtered += prior_moisture_data[addr][i];
	}
	// Average center 3 of 5 values
	filtered = (filtered - min - max)/3;

	// Shift memory
	for(int i = 0; i < 3; i++)
	{
		prior_moisture_data[addr][i] = prior_moisture_data[addr][i + 1];
	}
	prior_moisture_data[addr][3] = current;

	return filtered;
}

uint32_t filter_temperature(int addr, uint32_t current)
{
	// Calculate filtered value
	uint32_t filtered = current;
	uint32_t max = current, min = current;
	uint32_t indexed;

	for(int i = 0; i < 4; i++)
	{
		indexed = prior_temperature_data[addr][i];
		// Set possible new min/max
		if(indexed > max) max = indexed;
		else if(indexed < min) min = indexed;
		// Sum
		filtered += prior_temperature_data[addr][i];
	}
	// Average center 3 of 5 values
	filtered = (filtered - min - max)/3;

	// Shift memory
	for(int i = 0; i < 3; i++)
	{
		prior_temperature_data[addr][i] = prior_temperature_data[addr][i + 1];
	}
	prior_temperature_data[addr][3] = current;

	return filtered;
}
