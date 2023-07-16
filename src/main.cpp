#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <CayenneLPP.h>
#include <EEPROM.h>
#define SEALEVELPRESSURE_HPA (1013.25)
/*OTAA params*/
uint8_t devEui[] = {<Your dev EUI>};
uint8_t appEui[] = {<Your app EUI>};
uint8_t appKey[] = {<Your app key>};
/* ABP para*/
uint8_t nwkSKey[] = {<Your nwk key>};
uint8_t appSKey[] = {<Your app key>};
uint32_t devAddr = (uint32_t)<Your dev address>;

/*LoraWan channelsmask, default channels 0-7*/
uint16_t userChannelsMask[6] = {0xFF00, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;
/*Time in-between join attemps.*/
uint32_t joinTimeMS = 300000;
/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t loraWanClass = LORAWAN_CLASS;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = 15000;

/*OTAA or ABP*/
bool overTheAirActivation = LORAWAN_NETMODE;

/*ADR enable*/
bool loraWanAdr = false; // LORAWAN_ADR;

/* set LORAWAN_Net_Reserve ON, the node could save the network info to flash, when node reset not need to join again */
bool keepNet = LORAWAN_NET_RESERVE;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = LORAWAN_UPLINKMODE;

/* Application port */
uint8_t appPort = 2;

/*Store each air pressure reading. If the pressure increase between readings is higher than the 5 min threshold ,
then decrease the cycle/sleep time to 5 minute (from the normal 10 mins). If the pressure increase b/w readings is higher than
the 1 min threshold, then decrease the cycle/sleep time to 1 minute. Rapid increase in pressure indicates
that balloon is dropping*/
float lastBarometricPressureHPA = 0;
float currentBarometricPressureHPA = 0;
float barometricPressureDeltaThreshold_5Min = 4;
float barometricPressureDeltaThreshold_1Min = 7;

enum ReceiveType
{
	NoRx = 0,
	ResetBoard = 1,
	ChangeFrequencyEU = 2,
	ChangeFrequencyNA = 3,
	ClearEEPROM = 4,
	OverrideFailsafe = 5,
	OverrideFailsafeCancel = 6,
	Unkwn = 7

};
const int FailoverTransmitJoinCount = 600;
const int MemoryLocation_Frequency = 0;
const int MemoryLocation_AlreadyJoined = 1;
const int MemoryLocation_AttemptedJoins = 2;
const int MemoryLocation_Transmits = 4;
const int MemoryLocation_OverrideFailsafe = 6;

// Store Receive Type, sends confirmation of receive next transmission, after which the desired command is fulfilled.
ReceiveType CurrentReceiveType = NoRx;
// Requires one last cycle before received request is fulfilled.
bool LastCycleBeforeReceiveFulfill = false;
// For the first 12 sends (first hour), send data every 5 mins regardless.
uint sendCount = 0;
uint16_t joinAttemptCount;
uint16_t transmitCount;
uint16_t joinSuccessfulSinceLastFreqChange;
uint16_t overrideFreqChangeFailsafe;
/*!
 * Number of trials to transmit the frame, if the LoRaMAC layer did not
 * receive an acknowledgment. The MAC performs a datarate adaptation,
 * according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
 * to the following table:
 *
 * Transmission nb | Data Rate
 * ----------------|-----------
 * 1 (first)       | DR
 * 2               | DR
 * 3               | max(DR-1,0)
 * 4               | max(DR-1,0)
 * 5               | max(DR-2,0)
 * 6               | max(DR-2,0)
 * 7               | max(DR-3,0)
 * 8               | max(DR-3,0)
 *
 * Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
 * the datarate, in case the LoRaMAC layer did not receive an acknowledgment
 */
uint8_t confirmedNbTrials = 4;

Adafruit_BMP3XX bmp;
// Receiving

// downlink data handle function example
void downLinkDataHandle(McpsIndication_t *mcpsIndication)
{
	Serial.printf("+REV DATA:%s,RXSIZE %d,PORT %d\r\n", mcpsIndication->RxSlot ? "RXWIN2" : "RXWIN1", mcpsIndication->BufferSize, mcpsIndication->Port);
	Serial.print("+REV DATA:");
	// Buffer size should ONLY BE 1.
	uint8_t rxValue = 0;
	for (uint8_t i = 0; i < mcpsIndication->BufferSize; i++)
	{
		Serial.printf("%02X", mcpsIndication->Buffer[i]);
		rxValue = mcpsIndication->Buffer[i];
	}
	Serial.println();
	if (rxValue == 0x52)
	{
		Serial.println("Rx RESET REQUEST");
		CurrentReceiveType = ResetBoard;
	}
	else if (rxValue == 0x4E)
	{
		Serial.println("Rx CHANGE NA FREQUENCY");
		CurrentReceiveType = ChangeFrequencyNA;
	}
	else if (rxValue == 0x45)
	{
		Serial.println("Rx CHANGE EU FREQUENCY");
		CurrentReceiveType = ChangeFrequencyEU;
	}
	else if (rxValue == 0x43)
	{
		Serial.println("Rx CLEAR EEPROM");
		CurrentReceiveType = ClearEEPROM;
	}else if(rxValue == 0x4F){
		Serial.println("Rx FAILSAFE OVERRIDE");
		CurrentReceiveType = OverrideFailsafe;
	}else if(rxValue == 0x46){
		Serial.println("Rx FAILSAFE OVERRIDE CANCEL");
		CurrentReceiveType = OverrideFailsafeCancel;
	}
	else
	{
		Serial.println("Rx UNKNOWN/ UNSUPPORTED");
		CurrentReceiveType = Unkwn;
	}

	//   uint32_t color=mcpsIndication->Buffer[0]<<16|mcpsIndication->Buffer[1]<<8|mcpsIndication->Buffer[2];
	// #if(LoraWan_RGB==1)
	//   turnOnRGB(color,5000);
	//   turnOffRGB();
	// #endif
}

/* Prepares the payload of the frame */
static void prepareTxFrame(uint8_t appPort, float baro, float temp, int rxType, uint16_t cumulativeJoinAttempts, uint16_t cumulativeTransmits, uint16_t failsafeOverrideStatus)
{
	/*appData size is LORAWAN_APP_DATA_MAX_SIZE which is defined in "commissioning.h".
	 *appDataSize max value is LORAWAN_APP_DATA_MAX_SIZE.
	 *if enabled AT, don't modify LORAWAN_APP_DATA_MAX_SIZE, it may cause system hanging or failure.
	 *if disabled AT, LORAWAN_APP_DATA_MAX_SIZE can be modified, the max value is reference to lorawan region and SF.
	 *for example, if use REGION_CN470,
	 *the max value for different DR can be found in MaxPayloadOfDatarateCN470 refer to DataratesCN470 and BandwidthsCN470 in "RegionCN470.h".
	 */
	CayenneLPP lpp(LORAWAN_APP_DATA_MAX_SIZE);
	// lpp.addTemperature(1, temp);
	//  Channel 1 Baro/temp: Real Air Pressure / Temp.
	lpp.addBarometricPressure(1, baro);
	lpp.addTemperature(1, temp);
	// Channel 2 Temperature: Last received command.
	lpp.addTemperature(2, rxType);
	// Channel 2 Analog: join attempts.
	lpp.addConcentration(2, cumulativeJoinAttempts);
	// Channel 2 Barometric: Total transmits.
	lpp.addBarometricPressure(2, cumulativeTransmits);
	// Channel 3 Temperature: Failsafe override status.
	lpp.addTemperature(3, failsafeOverrideStatus);
	lpp.getBuffer();
	appDataSize = lpp.getSize();
	memcpy(appData, lpp.getBuffer(), appDataSize);
}

void ReadEEPROM(uint16_t &networkVal, uint16_t &alreadyJoined, uint16_t &attemptedJoins, uint16_t &transmits, uint16_t &overrideFailsafe)
{
	Serial.println("Entering Main.ReadEEPROM.");
	// Address 1: Network Value.
	// Address 2: Already Joined.
	//   1 = already connected.
	//   2 = not already connected.
	// IF not already joined, then wait 4 hours in between join tries. (in case we're over an ocean).
	// try{
	EEPROM.begin(6);
	networkVal = EEPROM.read(0);
	alreadyJoined = EEPROM.read(1);
	// two-byte num
	attemptedJoins = EEPROM.read(2) | EEPROM.read(3) << 8;
	// two-byte num
	transmits = EEPROM.read(4) | EEPROM.read(5) << 8;
	// one byte num
	overrideFailsafe = EEPROM.read(6);
	EEPROM.end();
	// }catch(){
	// 	// Assume network value = 0 (915-US)
	// 	networkValue = 2;
	// 	previouslyConnected = 1;
	// }
}
void WriteEEPROM(int address, uint16_t value)
{
	Serial.println("Entering Main.WriteEEPROM.");
	// try{
	EEPROM.begin(address + 1);
	switch (address)
	{
	case (MemoryLocation_Frequency):
		EEPROM.write(address, value);
		break;
	case (MemoryLocation_AlreadyJoined):
		EEPROM.write(address, value);
		break;
	case (MemoryLocation_AttemptedJoins):
		EEPROM.write(address, value & 0xFF);
		EEPROM.write(address + 1, value >> 8);
		break;
	case (MemoryLocation_Transmits):
		EEPROM.write(address, value & 0xFF);
		EEPROM.write(address + 1, value >> 8);
		break;
	case (MemoryLocation_OverrideFailsafe):
		EEPROM.write(address,value);
		break;
	}
	EEPROM.write(address, value);
	EEPROM.end();
	
}
// Incrememnt join count, write to EEPROM if % 3 = 0.
void incrementJoinCount()
{
	Serial.println("Entering Main.incrementJoinCount.");
	joinAttemptCount++;
	if (joinAttemptCount % 3)
	{
		WriteEEPROM(MemoryLocation_AttemptedJoins, joinAttemptCount);
	}
	// FAILSAFE MODE: automatically changes frequency if counters are high enough.
	// If network still set to NA, and attempted transmits + attempted Joins > FailoverTransmitJoinCount, force network change to europe.
	if(loraWanRegion == LORAMAC_REGION_US915 && (transmitCount + joinAttemptCount > FailoverTransmitJoinCount) && overrideFreqChangeFailsafe == 0){
		Serial.println("Failsafe Mode - Network still set to NA and (transmits + attemptedJoins) > FailoverTransmitJoinCount, so force network change to LORAMAC_REGION_EU868.");
		
		Serial.println("Set the network flag, already joined flags in EEPROM.");
		WriteEEPROM(MemoryLocation_AlreadyJoined, 0);
		WriteEEPROM(MemoryLocation_Frequency, 1);
		Serial.println("Reset Board");
		CySoftwareReset();
	}
}
// If join was not already successful, then write to EEPROM that join was successful.
void joinSuccessful()
{
	Serial.println("Entering Main.joinSuccessful");
	if (joinSuccessfulSinceLastFreqChange == 0)
		WriteEEPROM(MemoryLocation_AlreadyJoined, 1);
	joinSuccessfulSinceLastFreqChange = 1;
}
void setup()
{
	boardInitMcu();
	Serial.begin(9600);
	CurrentReceiveType = NoRx;
	// Determine what network to connect to.
	// Return values:
	// 2 = NA (915US);
	// 3 = EU ()
	// Determine if we've previously been connected to this.
	// Return values:
	// 0 = in between join attempts, wait only 5 mins.
	// 1 = in between join attempts, wait for 5 hours. (preserve energy in case we're over an ocean).
	uint16_t network;
	uint16_t alreadyJoined;
	uint16_t attemptedJoins;
	uint16_t transmits;
	uint16_t overrideFailsafe;
	ReadEEPROM(network, alreadyJoined, attemptedJoins, transmits, overrideFailsafe);

	switch (network)
	{
	case 1:
		loraWanRegion = LORAMAC_REGION_EU868;
		Serial.println("Read from EEPROM: LORAMAC_REGION_EU868. ");
		break;
	case 0:
		loraWanRegion = LORAMAC_REGION_US915;
		Serial.println("Read from EEPROM: LORAMAC_REGION_US915. ");
	default:
		loraWanRegion = LORAMAC_REGION_US915;
		Serial.println("Nothing written to EEPROM, using default: LORAMAC_REGION_US915. ");
		break;
	}
	if (network != 0 && alreadyJoined == 0)
	{
		Serial.println("Not already connected And NETWORK != NA: join in-between set to 180 mins.");
		joinTimeMS = 180 * 60000;
	}
	else if (network == 0 && alreadyJoined == 1)
	{
		Serial.println("Already joined OR NETWORK == NA: join in-between set to 5 minutes.");
		joinTimeMS = 5 * 60000;
	}
	else
	{
		Serial.println("Already joined unknown: join in-between set to 5 minutes.");
		joinTimeMS = 5 * 60000;
	}
	joinSuccessfulSinceLastFreqChange = alreadyJoined;
	transmitCount = transmits;
	joinAttemptCount = attemptedJoins;
	overrideFreqChangeFailsafe = overrideFailsafe;
	Serial.print("Has LoRaWAN join occurred since last freq change?");

	Serial.println(joinSuccessfulSinceLastFreqChange);
	Serial.print("Number of total transmits: ");
	Serial.println(transmitCount);
	Serial.print("Number of cumulative join attempts: ");
	Serial.println(attemptedJoins);
	Serial.print("Override automatic freq change (failsafe):");
	Serial.println(overrideFreqChangeFailsafe);
	// FAILSAFE MODE: automatically changes frequency if counters are high enough.
	// If network still set to NA, and attempted transmits + attempted Joins > FailoverTransmitJoinCount, force network change to europe.
	if(network == 0 && (transmits + attemptedJoins > FailoverTransmitJoinCount) && overrideFreqChangeFailsafe == 0){
		Serial.println("Failsafe Mode - Network still set to NA and (transmits + attemptedJoins) > FailoverTransmitJoinCount, so force network change to LORAMAC_REGION_EU868.");
		loraWanRegion = LORAMAC_REGION_EU868;
		Serial.println("Set the network flag, already joined flags.");
		WriteEEPROM(MemoryLocation_AlreadyJoined, 0);
		WriteEEPROM(MemoryLocation_Frequency, 1);
		joinTimeMS = 10 * 60000;
	}
	

#if (AT_SUPPORT)
	enableAt();
#endif
	deviceState = DEVICE_STATE_INIT;
	LoRaWAN.ifskipjoin();

	// Setup barometer/thermometer
	if (!bmp.begin_I2C())
	{ // hardware I2C mode, can pass in address & alt Wire

		Serial.println("Could not find a valid BMP3 sensor, check wiring!");
	}
	else
	{
		Serial.println("Setting Altitude board settings.");
		bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
		bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
		bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
		bmp.setOutputDataRate(BMP3_ODR_50_HZ);
	}
}

void loop()
{

	switch (deviceState)
	{
	case DEVICE_STATE_INIT:
	{
#if (AT_SUPPORT)
		getDevParam();
#endif
		printDevParam();
		LoRaWAN.init(loraWanClass, loraWanRegion);
		deviceState = DEVICE_STATE_JOIN;
		break;
	}
	case DEVICE_STATE_JOIN:
	{
		LoRaWAN.join(&incrementJoinCount, &joinSuccessful, joinTimeMS);
		break;
	}
	case DEVICE_STATE_SEND:
	{
		Serial.write("Sending data!");
		if (!bmp.performReading())
		{
			Serial.println("Failed to perform reading :(");
			Serial.print("CurrentReceiveType: ");
			Serial.println(CurrentReceiveType);
			prepareTxFrame(appPort, (float)-1, (float)-1, CurrentReceiveType, joinAttemptCount, transmitCount, overrideFreqChangeFailsafe);
			LoRaWAN.send();
		}
		else
		{
			// Send it!
			Serial.print("Temperature = ");
			Serial.print(bmp.temperature);
			Serial.println(" *C");

			Serial.print("Pressure = ");
			Serial.print(bmp.pressure / 100.0);
			Serial.println(" hPa");
			Serial.print("Rx Type = ");
			Serial.println(CurrentReceiveType);
			lastBarometricPressureHPA = currentBarometricPressureHPA;
			currentBarometricPressureHPA = bmp.pressure / 100.0;

			prepareTxFrame(appPort, (float)currentBarometricPressureHPA, (float)bmp.temperature, CurrentReceiveType, joinAttemptCount, transmitCount, overrideFreqChangeFailsafe);
			LoRaWAN.send();
		}
		transmitCount++;
		// Only write the transmit count to memory if even (to save battery).
		if (transmitCount % 2 == 0)
		{
			WriteEEPROM(MemoryLocation_Transmits, transmitCount);
		}
		// FAILSAFE MODE: automatically changes frequency if counters are high enough.
		// If network still set to NA, and attempted transmits + attempted Joins > FailoverTransmitJoinCount, force network change to europe.
		if(loraWanRegion == LORAMAC_REGION_US915 && (transmitCount + joinAttemptCount > FailoverTransmitJoinCount) && overrideFreqChangeFailsafe == 0){
			Serial.println("Failsafe Mode - Network still set to NA and (transmits + attemptedJoins) > FailoverTransmitJoinCount, so force network change to LORAMAC_REGION_EU868.");
			
			Serial.println("Set the network flag, already joined flags in EEPROM.");
			WriteEEPROM(MemoryLocation_AlreadyJoined, 0);
			WriteEEPROM(MemoryLocation_Frequency, 1);
			Serial.println("Reset Board");
			CySoftwareReset();
		}
		deviceState = DEVICE_STATE_ACT;

		break;
	}
	case DEVICE_STATE_ACT:
	{
		// Act on Rx commands.
		deviceState = DEVICE_STATE_CYCLE;
		switch (CurrentReceiveType)
		{
		case NoRx:
		{
			break;
		}
		case ResetBoard:
		{
			if (!LastCycleBeforeReceiveFulfill)
			{
				LastCycleBeforeReceiveFulfill = true;
			}
			else
				CySoftwareReset();

			break;
		}

		case ChangeFrequencyEU:
		{
			if (!LastCycleBeforeReceiveFulfill)
			{
				LastCycleBeforeReceiveFulfill = true;
			}
			else
			{
				// Write the proper frequency to memory (1).
				WriteEEPROM(MemoryLocation_Frequency, 1);
				// Reset the "Prev Connected" flag.
				WriteEEPROM(MemoryLocation_AlreadyJoined, 0);
				// ... then, reset board.
				CySoftwareReset();
			}

			break;
		}
		case ChangeFrequencyNA:
		{
			// Write to EEPROM the desired frequency, and that this is a deliberate frequency change...
			// This ensures that the chip only attemps to join every hour (so as to save battery).
			if (!LastCycleBeforeReceiveFulfill)
			{
				LastCycleBeforeReceiveFulfill = true;
			}
			else
			{
				// Write the proper frequency to memory (1).
				WriteEEPROM(MemoryLocation_Frequency, 0);
				// Reset the "Prev Connected" flag.
				WriteEEPROM(MemoryLocation_AlreadyJoined, 0);
				// ... then, reset board.
				CySoftwareReset();
			}

			break;
		}
		case ClearEEPROM:
		{
			if (!LastCycleBeforeReceiveFulfill)
			{
				LastCycleBeforeReceiveFulfill = true;
			}
			else
			{
				// Write the proper frequency to memory (1).
				WriteEEPROM(MemoryLocation_Frequency, 0);
				// Reset the "Prev Connected" flag.
				WriteEEPROM(MemoryLocation_AlreadyJoined, 0);
				WriteEEPROM(MemoryLocation_AttemptedJoins, 0);
				WriteEEPROM(MemoryLocation_Transmits, 0);
				CySoftwareReset();
				CurrentReceiveType = NoRx;
			}
			break;
		}
		case OverrideFailsafe:
		{
			WriteEEPROM(MemoryLocation_OverrideFailsafe , 1);
			overrideFreqChangeFailsafe = 1;
			CurrentReceiveType = NoRx;
			break;
		}
		case OverrideFailsafeCancel:{
			WriteEEPROM(MemoryLocation_OverrideFailsafe, 0);
			overrideFreqChangeFailsafe = 0;
			CurrentReceiveType = NoRx;
			break;
		}
			
		}
		break;
	}
	case DEVICE_STATE_CYCLE:
	{
		// If first 12 sends, sleep for only 5 mins.
		if (sendCount < 12)
		{
			// Five minutes!
			 Serial.println("Going to sleep for 5 minutes");
			 txDutyCycleTime = appTxDutyCycle + 60000 * 5;
			// Serial.println("Going to sleep for 1 minutes");
			// txDutyCycleTime = appTxDutyCycle + 60000;
			sendCount++;
		}
		else
		{
			// Schedule next packet transmission
			// If the difference between current pressure and last pressure is above 7hPa,
			// schedule sleep to be one minute. If the difference is above 4 hPa, schedule 5 mins.
			// Else, schedule 15 mins.
			// This is to give more accurate data in case balloon is falling.
			if (currentBarometricPressureHPA - lastBarometricPressureHPA >= barometricPressureDeltaThreshold_1Min)
			{

				// One min!
				Serial.println("Going to sleep for 1 minute");
				txDutyCycleTime = appTxDutyCycle + 60000;
			}
			else if (currentBarometricPressureHPA - lastBarometricPressureHPA >= barometricPressureDeltaThreshold_5Min)
			{
				// Five minutes!
				Serial.println("Going to sleep for 5 minutes");
				txDutyCycleTime = appTxDutyCycle + 300000;
			}
			else
			{
				// Ten minutes.
				Serial.println("Going to sleep for 10 minutes");
				txDutyCycleTime = appTxDutyCycle + 600000;
			}
		}

		LoRaWAN.cycle(txDutyCycleTime);
		deviceState = DEVICE_STATE_SLEEP;
		break;
	}
	case DEVICE_STATE_SLEEP:
	{
		LoRaWAN.sleep();
		break;
	}
	default:
	{
		deviceState = DEVICE_STATE_INIT;
		break;
	}
	}
	
}