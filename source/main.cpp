#include "Microfarm.h"
int main(void)
{
	configure();
	while(1) loop();
}

/*******************************************************************************

Description of Code:
	main.cpp:
		Framework
	Microfarm.h/cpp:
		Functions to fill Framework
	MicrofarmCommunications.h/cpp
		Functions to abstract working with hardware
		Essentially the BSP




Intended Device Functionality
	I2C Slave Bus
		Handle ideally timely communications with upstream DAQ device
			Communicate:
				GPIO
				Sensor Data

	I2C Master Bus
		Obtain data from downstream sensors
			Communicate:
				Sensor Data

	GPIO
		LED status indicators
		Control solenoid valves and possibly other GPIO devices

*******************************************************************************/
