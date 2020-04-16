/*
 * Copyright (c) 2020, Luc Yriarte
 * License: BSD <http://www.opensource.org/licenses/bsd-license.php>
 */

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <Servo.h> 


/* **** **** **** **** **** ****
 * Constants
 * **** **** **** **** **** ****/

#define BPS_HOST 9600

#define serverPORT 80
#define WIFI_CLIENT_DELAY_MS 500
#define WIFI_CLIENT_CONNECTED_DELAY_MS 100
#define WIFI_SERVER_DELAY_MS 200
#define WIFI_CONNECT_DELAY_MS 3000
#define WIFI_CONNECT_RETRY_DELAY_MS 1000
#define WIFI_CONNECT_RETRY 5

#define REQ_BUFFER_SIZE 1024

#define MAIN_LOOP_POLL_MS 2

/* **** **** **** **** **** ****
 * Global variables
 * **** **** **** **** **** ****/

/*
 * WiFi
 */

WiFiServer wifiServer(serverPORT);
WiFiClient wifiClient;

typedef struct {
  char * SSID;
  char * passwd;
  IPAddress address;
  IPAddress gateway;
  IPAddress netmask;
} wifiNetInfo;

wifiNetInfo networks[] = {
  {
    "network1",
    "password",
    IPAddress(192,168,0,2),
    IPAddress(192,168,0,1),
    IPAddress(255,255,255,0)
  },
  {
    "network2",
    "password",
    IPAddress(0,0,0,0),
    IPAddress(0,0,0,0),
    IPAddress(0,0,0,0)
  }
};


int wifiStatus = WL_IDLE_STATUS;
char hostnameSSID[] = "ESP_XXXXXX";
char wifiMacStr[] = "00:00:00:00:00:00";
byte wifiMacBuf[6];

/* 
 * http request buffer
 */
char reqBuffer[REQ_BUFFER_SIZE];
int reqBufferIndex=0;


/* **** **** **** **** **** ****
 * Robot commands
 * **** **** **** **** **** ****/

/*
 * Time management
 */ 

typedef struct {
	int updated_ms;
	int poll_ms;
} POLLInfo;


/*
 * LED
 */
 
typedef struct {
	POLLInfo pollInfo;
	int gpio;
	int state;
	int blink;
} LEDInfo;

LEDInfo ledInfos[] = {
	{
		{0, -1},
		2,
		LOW,
		0
	}
};

#define N_LED (sizeof(ledInfos) / sizeof(LEDInfo))


/*
 * SERVO
 */
 
typedef struct {
	Servo * servoP;
	int gpio;
	int angle;
} SERVOInfo;

SERVOInfo servoInfos[] = {
	{
		new Servo(),
		10,
		90
	}
};

#define N_SERVO (sizeof(servoInfos) / sizeof(SERVOInfo))


/*
 * STEPPER
 */
 
typedef struct {
	POLLInfo pollInfo;
	int gpios[4];
	int steps;
	int step8Index;
} STEPPERInfo;

STEPPERInfo stepperInfos[] = {
	{// D0 D1 D2 D3
		{0, 1},
		{16,5,4,0},
		0,
		0
	},
	{// D5 D6 D7 D8
		{0, 1},
		{14,12,13,15},
		0,
		0
	}
};

#define N_STEPPER (sizeof(stepperInfos) / sizeof(STEPPERInfo))

byte steps8[] = {
  HIGH,  LOW,  LOW,  LOW,
  HIGH, HIGH,  LOW,  LOW,
   LOW, HIGH,  LOW,  LOW,
   LOW, HIGH, HIGH,  LOW,
   LOW,  LOW, HIGH,  LOW, 
   LOW,  LOW, HIGH, HIGH,
   LOW,  LOW,  LOW, HIGH,
  HIGH,  LOW,  LOW, HIGH,
};

void step8(int pin1, int pin2, int pin3, int pin4, int i) {
	digitalWrite(pin1, steps8[i++]);
	digitalWrite(pin2, steps8[i++]);
	digitalWrite(pin3, steps8[i++]);
	digitalWrite(pin4, steps8[i++]);
} 



/* **** **** **** **** **** ****
 * Functions
 * **** **** **** **** **** ****/

void setup() {
	int i,j;
	for (i=0; i < N_LED; i++)
		pinMode(ledInfos[i].gpio, OUTPUT);
	for (i=0; i < N_SERVO; i++) {
		pinMode(servoInfos[i].gpio, OUTPUT);
		servoInfos[i].servoP->attach(servoInfos[i].gpio);
		servoInfos[i].servoP->write(servoInfos[i].angle);
	}
	for (i=0; i < N_STEPPER; i++)
		for (j=0; j < 4; j++)
			pinMode(stepperInfos[i].gpios[j], OUTPUT);
	Serial.begin(BPS_HOST);
	wifiMacInit();
	Serial.print("WiFi.macAddress: ");
	Serial.println(wifiMacStr);
}


/*
 * Time management
 */

bool pollDelay(int poll_ms, int from_ms) {
	int elapsed_ms = millis() - from_ms;
	if (elapsed_ms < poll_ms) {
		delay(poll_ms - elapsed_ms);
		return true;
	}
	return false;
}

bool updatePollInfo(POLLInfo * pollInfoP) {
	int current_ms = millis();
	if (pollInfoP->poll_ms <= current_ms - pollInfoP->updated_ms) {
		pollInfoP->updated_ms = current_ms;
		return true;
	}
	return false;
}

bool handlePollInfoRequest(const char * req, POLLInfo * pollInfoP) {
	String strReq = req;
	pollInfoP->poll_ms = strReq.toInt();
	return true;
}

/*
 * WiFi
 */

void wifiMacInit() {
	WiFi.macAddress(wifiMacBuf);
	int i,j,k;
	j=0; k=4;
	for (i=0; i<6; i++) {
		wifiMacStr[j] = '0' + (wifiMacBuf[i] >> 4);
		if (wifiMacStr[j] > '9')
			wifiMacStr[j] += 7;
		if (i>2)
			hostnameSSID[k++] = wifiMacStr[j];
		++j;
		wifiMacStr[j] = '0' + (wifiMacBuf[i] & 0x0f);
		if (wifiMacStr[j] > '9')
			wifiMacStr[j] += 7;
		if (i>2)
			hostnameSSID[k++] = wifiMacStr[j];
		j+=2;
	}
}

bool wifiConnect(int retry) {
	int i,n;
	wifiStatus = WiFi.status();
	if (wifiStatus == WL_CONNECTED)
		return true;
	n = sizeof(networks) / sizeof(wifiNetInfo);
	for (i=0; i<n; i++) {
		if (wifiNetConnect(&networks[i], retry))
			return true;
	}
	WiFi.disconnect();
	wifiStatus = WiFi.status();
	return false;
}

bool wifiNetConnect(wifiNetInfo *net, int retry) {
	Serial.print("Connecting to: ");
	Serial.println(net->SSID);
	WiFi.config(net->address, net->gateway, net->netmask);  
	wifiStatus = WiFi.begin(net->SSID, net->passwd);
	Serial.print("trying..");
	while (wifiStatus != WL_CONNECTED && retry > 0) {
		retry--;
		Serial.print(".");
		delay(WIFI_CONNECT_DELAY_MS);
		wifiStatus = WiFi.status();
	}
	Serial.println();
	if (wifiStatus == WL_CONNECTED) {
		Serial.print("WiFi client IP Address: ");
		Serial.println(WiFi.localIP());
		if (MDNS.begin(hostnameSSID)) {
			Serial.print("Registered mDNS hostname: ");
			Serial.println(hostnameSSID);
  		}
	}
	return wifiStatus == WL_CONNECTED;
}


/*
 * Robot commands
 */

void updateLEDStatus(int index) {
	if (!updatePollInfo(&(ledInfos[index].pollInfo)))
		return;
	if (ledInfos[index].blink)
		ledInfos[index].state = ledInfos[index].state == LOW ? HIGH : LOW;
	if (ledInfos[index].blink > 0)
		ledInfos[index].blink--;
	digitalWrite(ledInfos[index].gpio, ledInfos[index].state);
}

bool handleLEDRequest(const char * req) {
	String strReq = req;
	int index = strReq.toInt();
	if (index < 0 || index >= N_LED)
		return false;
	strReq = strReq.substring(strReq.indexOf("/")+1);
	if (strReq.startsWith("POLL/"))
		return handlePollInfoRequest(strReq.substring(5).c_str(), &(ledInfos[index].pollInfo));
	if (strReq.startsWith("BLINK/")) {
		strReq = strReq.substring(strReq.indexOf("/")+1);
		ledInfos[index].blink = strReq.toInt();
		return true;
	}
	if (strReq.endsWith("ON"))
		ledInfos[index].state = HIGH;
	else if (strReq.endsWith("OFF"))
		ledInfos[index].state = LOW;
	else
		return false;
	return true;
}

void updateSERVOStatus(int index) {
	if (servoInfos[index].angle < 0 || servoInfos[index].angle > 180)
		return;
	servoInfos[index].servoP->write(servoInfos[index].angle);
	servoInfos[index].angle = -1;
}

bool handleSERVORequest(const char * req) {
	String strReq = req;
	int index = strReq.toInt();
	if (index < 0 || index >= N_SERVO)
		return false;
	strReq = strReq.substring(strReq.indexOf("/")+1);
	servoInfos[index].angle = strReq.toInt();
	if (servoInfos[index].angle < 0 || servoInfos[index].angle > 180)
		return false;
	return true;
}

void updateSTEPPERStatus(int index) {
	if (!updatePollInfo(&(stepperInfos[index].pollInfo)))
		return;
	if (stepperInfos[index].steps > 0) {
		step8(stepperInfos[index].gpios[0],stepperInfos[index].gpios[1],stepperInfos[index].gpios[2],stepperInfos[index].gpios[3],
			stepperInfos[index].step8Index);
		stepperInfos[index].steps--;
	}
	else if (stepperInfos[index].steps < 0) {
		step8(stepperInfos[index].gpios[3],stepperInfos[index].gpios[2],stepperInfos[index].gpios[1],stepperInfos[index].gpios[0],
			stepperInfos[index].step8Index);
		stepperInfos[index].steps++;
	}
	stepperInfos[index].step8Index = (stepperInfos[index].step8Index + 4) % 32;
}

bool handleSTEPPERRequest(const char * req) {
	String strReq = req;
	int index = strReq.toInt();
	if (index < 0 || index >= N_STEPPER)
		return false;
	strReq = strReq.substring(strReq.indexOf("/")+1);
	if (strReq.startsWith("POLL/"))
		return handlePollInfoRequest(strReq.substring(5).c_str(), &(stepperInfos[index].pollInfo));
	stepperInfos[index].steps = strReq.toInt();
	return true;
}


/*
 * HTTP request main dispatch
 */

bool handleHttpRequest(const char * req) {
	if (req == NULL)
		return false;
	String strReq = req;
	if (! strReq.startsWith("GET /"))
		return false;
	strReq = strReq.substring(5, strReq.indexOf(" HTTP"));
	bool result = false;
	if (strReq.startsWith("LED/"))
		result = handleLEDRequest(strReq.substring(4).c_str());
	if (strReq.startsWith("SERVO/"))
		result = handleSERVORequest(strReq.substring(6).c_str());
	else if (strReq.startsWith("STEPPER/"))
		result = handleSTEPPERRequest(strReq.substring(8).c_str());
	if (result)
		wifiClient.println(strReq);
	else
		wifiClient.println("ERROR");
	return result;
}


/* 
 * Main loop
 */

void loop() {
	int deviceIndex, start_loop_ms;
	while (!wifiConnect(WIFI_CONNECT_RETRY))
		delay(WIFI_CONNECT_RETRY_DELAY_MS);
	wifiServer.begin();
	delay(WIFI_SERVER_DELAY_MS);
	while (wifiStatus == WL_CONNECTED) {
		start_loop_ms = millis();
		wifiClient = wifiServer.available();
		if (wifiClient && wifiClient.connected()) {
			delay(WIFI_CLIENT_CONNECTED_DELAY_MS);
			reqBufferIndex = 0;
			while (wifiClient.available() && reqBufferIndex < REQ_BUFFER_SIZE-1) {
				reqBuffer[reqBufferIndex++] = wifiClient.read();
			}
			reqBuffer[reqBufferIndex] = 0;
			handleHttpRequest(reqBuffer);
			delay(WIFI_CLIENT_DELAY_MS);
			wifiClient.stop();
		}
		for (deviceIndex=0; deviceIndex<N_LED; deviceIndex++)
			updateLEDStatus(deviceIndex);
		for (deviceIndex=0; deviceIndex<N_SERVO; deviceIndex++)
			updateSERVOStatus(deviceIndex);
		for (deviceIndex=0; deviceIndex<N_STEPPER; deviceIndex++)
			updateSTEPPERStatus(deviceIndex);
		pollDelay(MAIN_LOOP_POLL_MS, start_loop_ms);
		wifiStatus = WiFi.status();
	}
}
