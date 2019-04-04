#ifndef MICROFARM_H
#define MICROFARM_H

// This value MUST be different for each seesaw board on the bus!
#define SEESAW_SLAVE_ADDRESS 	0x15

// Other configuration
#define MYDELAY_CYCLES 			500000
#define CYCLES_PER_SENSOR_POLL	1000


void configure();
void loop();

#endif
