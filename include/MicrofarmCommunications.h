#ifndef MICROFARM_COMMUNICATIONS_H
#define MICROFARM_COMMUNICATIONS_H

// USEFUL SHORTHAND
#define GPIO				PORT->Group[0]
#define SLAVE_BUS			SERCOM0->I2CS
#define MASTER_BUS			SERCOM1->I2CM

// PIN DEFINITIONS
#define R_LED1				(1 << 2)
#define G_LED1				(1 << 3)
#define Y_LED1				(1 << 4)
#define B_LED1				(1 << 5)
#define B_LED0				(1 << 6)
#define Y_LED0				(1 << 7)
#define G_LED0				(1 << 8)
#define R_LED0				(1 << 9)
#define GROUND_PIN			(1 << 10)
#define SOLENOID_VALVE		(1 << 25)
#define BOARD_LED			(1 << 27)

// COMMANDS
#define COMMAND_UID(i)			(0x10 + i*0x10)
#define COMMAND_MOISTUREA(i) 	(0x11 + i*0x10)
#define COMMAND_MOISTUREB(i) 	(0x12 + i*0x10)
#define COMMAND_TEMPERATUREA(i)	(0x13 + i*0x10)
#define COMMAND_TEMPERATUREB(i)	(0x14 + i*0x10)
#define COMMAND_TEMPERATUREC(i)	(0x15 + i*0x10)
#define COMMAND_TEMPERATURED(i)	(0x16 + i*0x10)
#define COMMAND_ONLINE(i) 		(0x17 + i*0x10)
#define COMMAND_TEST			0xFF
#define COMMAND_SOLENOIDON		0x90
#define COMMAND_SOLENOIDOFF		0x91

// COMMUNICATIONS STATES
#define COMMS_IDLE			0x00
#define COMMS_RX			0x01
#define COMMS_TX			0x02
#define COMMS_TXREQ			0x03
#define COMMS_RXNACK		0x04
#define COMMS_BUSY			0x05
#define COMMS_ERROR 		0xFF

// Debug displays
void superBlink();
void superResetBlink();

// Onboard LED Controls
void LEDon();
void LEDoff();

// Delay, duration set by MY_DELAY_CYCLES
void mydelay();

// Solenoid Interaction
void solenoidOff();
void solenoidOn();

// LED Test Display Interaction
void LEDwrite(uint8_t bits);
void setRed0(uint8_t setting);
void setRed1(uint8_t setting);
void setGreen0(uint8_t setting);
void setGreen1(uint8_t setting);
void setYellow0(uint8_t setting);
void setYellow1(uint8_t setting);
void setBlue0(uint8_t setting);
void setBlue1(uint8_t setting);

// Essential Functions
uint8_t sendToSensor(uint8_t address, uint8_t *buf, uint8_t length);
uint8_t readFromSensor(uint8_t address, uint8_t *buf, uint8_t length);
void	listenForDAQ();
void 	initializeSAMD09();
void 	disableWatchdog();
void 	startWatchdog();
void 	resetWatchdog();

typedef struct
{
	uint8_t receive 	= 0x00;
	uint8_t change 		= COMMS_IDLE;
} MicrofarmStatusStruct;

typedef struct
{
	uint16_t moisture;
	uint32_t temperature;
	uint8_t uid;
	uint8_t online;
} MicrofarmDataStruct;

#endif
