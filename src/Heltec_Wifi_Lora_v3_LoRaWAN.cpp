/* Heltec Automation LoRaWAN communication example
 *
 * Function:
 * 1. Upload node data to the server using the standard LoRaWAN protocol.
 * 2. The network access status of LoRaWAN is displayed on the screen.
 *
 * Description:
 * 1. Communicate using LoRaWAN protocol.
 *
 * HelTec AutoMation, Chengdu, China
 * 成都惠利特自动化科技有限公司
 * www.heltec.org
 *
 * this project also realess in GitHub:
 * https://github.com/Heltec-Aaron-Lee/WiFi_Kit_series
 * */

#include "LoRaWan_APP.h"
#define SENSOR 33
/* OTAA para*/
// uint8_t devEui[] = {0x51,0x21,0x00,0x00,0x01,0x0C,0x25,0x00};
// uint8_t appEui[] = {0x01,0x00,0x01,0x00,0x00,0x0C,0x25,0x00};
// uint8_t appKey[] = {0x13,0x3C,0xDF,0x4D,0xBD,0x24,0xEC,0xC4,0x93,0x4E,0xD7,0xED,0xDA,0x3B,0x5E,0x0E};

uint8_t devEui[] = {0x00, 0x25, 0x0C, 0x01, 0x00, 0x00, 0x21, 0x51};
uint8_t appEui[] = {0x00, 0x25, 0x0C, 0x00, 0x00, 0x01, 0x00, 0x01};
uint8_t appKey[] = {0x13, 0x3C, 0xDF, 0x4D, 0xBD, 0x24, 0xEC, 0xC4, 0x93, 0x4E, 0xD7, 0xED, 0xDA, 0x3B, 0x5E, 0x0E};

/* ABP para*/
uint8_t nwkSKey[] = {0x15, 0xb1, 0xd0, 0xef, 0xa4, 0x63, 0xdf, 0xbe, 0x3d, 0x11, 0x18, 0x1e, 0x1e, 0xc7, 0xda, 0x85};
uint8_t appSKey[] = {0xd7, 0x2c, 0x78, 0x75, 0x8c, 0xdc, 0xca, 0xbf, 0x55, 0xee, 0x4a, 0x77, 0x8d, 0x16, 0xef, 0x67};
uint32_t devAddr = (uint32_t)0x007e6ae1;

long currentMillis = 0;
long previousMillis = 0;
int interval = 1000;
float calibrationFactor = 11;
volatile byte pulseCount;
byte pulse1Sec = 0;
int flowRate;
unsigned long flowMilliLitres;
unsigned long totalMilliLitres;

/*LoraWan channelsmask*/
uint16_t userChannelsMask[6] = {0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t loraWanClass = CLASS_A;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = 15000;

/*OTAA or ABP*/
bool overTheAirActivation = true;

/*ADR enable*/
bool loraWanAdr = true;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = true;

/* Application port */
uint8_t appPort = 2;
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

void IRAM_ATTR pulseCounter()
{
	pulseCount++;
	Serial.print("Pulse count: ");
	Serial.println(pulseCount);
}

int calculate_flowRate()
{
	currentMillis = millis();
	if (currentMillis - previousMillis > interval)
	{
		pulse1Sec = pulseCount;
		pulseCount = 0;

		// Because this loop may not complete in exactly 1 second intervals we calculate
		// the number of milliseconds that have passed since the last execution and use
		// that to scale the output. We also apply the calibrationFactor to scale the output
		// based on the number of pulses per second per units of measure (litres/minute in
		// this case) coming from the sensor.
		flowRate = ((1000.0 / (millis() - previousMillis)) * pulse1Sec) / calibrationFactor;
		previousMillis = millis();

		// Divide the flow rate in litres/minute by 60 to determine how many litres have
		// passed through the sensor in this 1 second interval, then multiply by 1000 to
		// convert to millilitres.
		flowMilliLitres = (flowRate / 60) * 1000;

		// Add the millilitres passed in this second to the cumulative total
		totalMilliLitres += flowMilliLitres;

		// Print the flow rate for this second in litres / minute
		Serial.print("Flow rate: ");
		Serial.print(flowRate); // Print the integer part of the variable
		Serial.print("L/min\n");

		return flowRate;
	}
}

/* Prepares the payload of the frame */
static void prepareTxFrame(uint8_t port)
{
	/*appData size is LORAWAN_APP_DATA_MAX_SIZE which is defined in "commissioning.h".
	 *appDataSize max value is LORAWAN_APP_DATA_MAX_SIZE.
	 *if enabled AT, don't modify LORAWAN_APP_DATA_MAX_SIZE, it may cause system hanging or failure.
	 *if disabled AT, LORAWAN_APP_DATA_MAX_SIZE can be modified, the max value is reference to lorawan region and SF.
	 *for example, if use REGION_CN470,
	 *the max value for different DR can be found in MaxPayloadOfDatarateCN470 refer to DataratesCN470 and BandwidthsCN470 in "RegionCN470.h".
	 */
	appDataSize = 4;
	// appData[0] = 0x00;
	// appData[1] = 0x01;
	// appData[2] = 0x02;
	// appData[3] = 0x03;
	int a = calculate_flowRate();
	appData[0] = (a >> 8);
	appData[1] = (a & 0xFF);
}

RTC_DATA_ATTR bool firstrun = true;

void setup()
{
	Serial.begin(115200);
	Mcu.begin();
	if (firstrun)
	{
		LoRaWAN.displayMcuInit();
		firstrun = false;
	}
	deviceState = DEVICE_STATE_INIT;

	pinMode(SENSOR, INPUT_PULLUP);
	pulseCount = 0;
	flowRate = 0.0;
	flowMilliLitres = 0;
	totalMilliLitres = 0;
	previousMillis = 0;

	attachInterrupt(digitalPinToInterrupt(SENSOR), pulseCounter, FALLING);
}

void loop()
{
	switch (deviceState)
	{
	case DEVICE_STATE_INIT:
	{
#if (LORAWAN_DEVEUI_AUTO)
		LoRaWAN.generateDeveuiByChipID();
#endif
		LoRaWAN.init(loraWanClass, loraWanRegion);
		break;
	}
	case DEVICE_STATE_JOIN:
	{
		LoRaWAN.displayJoining();
		LoRaWAN.join();
		break;
	}
	case DEVICE_STATE_SEND:
	{
		LoRaWAN.displaySending();
		prepareTxFrame(appPort);
		LoRaWAN.send();
		deviceState = DEVICE_STATE_CYCLE;
		break;
	}
	case DEVICE_STATE_CYCLE:
	{
		// Schedule next packet transmission
		txDutyCycleTime = appTxDutyCycle + randr(0, APP_TX_DUTYCYCLE_RND);
		LoRaWAN.cycle(txDutyCycleTime);
		deviceState = DEVICE_STATE_SLEEP;
		break;
	}
	case DEVICE_STATE_SLEEP:
	{
		LoRaWAN.displayAck();
		LoRaWAN.sleep(loraWanClass);
		break;
	}
	default:
	{
		deviceState = DEVICE_STATE_INIT;
		break;
	}
	}
}