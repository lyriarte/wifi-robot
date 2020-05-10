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

#define MAIN_LOOP_POLL_MS 50
#define MINIMUM_UPDATE_MS 10
#define MOTOR_POLL_MAX_MS 1000

#define MAX_RANGE_CM 1000
#define ACT_RANGE_CM 30
#define ECHO_TIMEOUT_US 20000
#define ECHO_TO_CM(x) (x/60) 


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
 * TELEMETER
 */
 
typedef struct {
	int gpio_trig;
	int gpio_echo;
	int dist_cm;
} TELEMETERInfo;

TELEMETERInfo telemeterInfos[] = {
	{
		16,	// trig D0
		12,	// echo D6
		MAX_RANGE_CM
	},
	{
		0,	// trig D3
		13,	// echo D7
		MAX_RANGE_CM	
	}
};

#define N_TELEMETER (sizeof(telemeterInfos) / sizeof(TELEMETERInfo))


/*
 * LED
 */
 
typedef struct {
	POLLInfo pollInfo;
	int gpio;
	int state;
	int blink;
	int blink_on_ms;
	int blink_off_ms;
	int invert;
} LEDInfo;

LEDInfo ledInfos[] = {
	{
		{0, 200},
		2,	// D4 led
		LOW,
		-1,
		50,250,
		1
	},
	{
		{0, 5000},
		5,	// D1 motor left
		LOW,
		0,
		0,0,
		0
	},
	{
		{0, 5000},
		4,	// D2 motor right
		LOW,
		0,
		0,0,
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
		14,	// D5 steer
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


/*
 * WHEELBOT
 */

typedef struct {
	int leftTelemeter;
	int rightTelemeter;
	int leftWheel;
	int rightWheel;
	int rearServo;
	int poll_ms;
	int poll_max_ms;
	int steer;
} WHEELBOTInfo;

WHEELBOTInfo wheelbot = {
	0,1,1,2,0,-1,MOTOR_POLL_MAX_MS,90
};



/* **** **** **** **** **** ****
 * Functions
 * **** **** **** **** **** ****/

void setup() {
	int i,j;
	for (i=0; i < N_TELEMETER; i++) {
		pinMode(telemeterInfos[i].gpio_trig, OUTPUT);
		pinMode(telemeterInfos[i].gpio_echo, INPUT);
	}
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
	if (ledInfos[index].blink) {
		if (!updatePollInfo(&(ledInfos[index].pollInfo)))
			return;
		if (ledInfos[index].state == LOW) {
			ledInfos[index].state = HIGH;
			ledInfos[index].pollInfo.poll_ms = ledInfos[index].blink_on_ms;
		}
		else {
			ledInfos[index].state = LOW;
			ledInfos[index].pollInfo.poll_ms = ledInfos[index].blink_off_ms;
		}
	}
	if (ledInfos[index].blink > 0)
		ledInfos[index].blink--;
	if (ledInfos[index].invert)
		digitalWrite(ledInfos[index].gpio, ledInfos[index].state == HIGH ? LOW : HIGH);
	else
		digitalWrite(ledInfos[index].gpio, ledInfos[index].state);
}

bool handleLEDRequest(const char * req) {
	String strReq = req;
	int index = strReq.toInt();
	if (index < 0 || index >= N_LED)
		return false;
	strReq = strReq.substring(strReq.indexOf("/")+1);
	if (strReq.startsWith("POLL/")) {
		handlePollInfoRequest(strReq.substring(5).c_str(), &(ledInfos[index].pollInfo));
		ledInfos[index].blink_on_ms = ledInfos[index].blink_off_ms = ledInfos[index].pollInfo.poll_ms;
		return true;
	}
	if (strReq.startsWith("BLINK/")) {
		strReq = strReq.substring(strReq.indexOf("/")+1);
		ledInfos[index].blink = strReq.toInt();
		return true;
	}
	if (strReq.startsWith("BLINKON/")) {
		strReq = strReq.substring(strReq.indexOf("/")+1);
		ledInfos[index].blink_on_ms = strReq.toInt();
		return true;
	}
	if (strReq.startsWith("BLINKOFF/")) {
		strReq = strReq.substring(strReq.indexOf("/")+1);
		ledInfos[index].blink_off_ms = strReq.toInt();
		return true;
	}
	if (strReq.endsWith("ON")) {
		ledInfos[index].blink = 0;
		ledInfos[index].state = HIGH;
	}
	else if (strReq.endsWith("OFF")) {
		ledInfos[index].blink = 0;
		ledInfos[index].state = LOW;
	}
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
 * Wheelbot
 */ 

void updateTELEMETERStatus(int index) {
	unsigned long echoDuration;
	digitalWrite(telemeterInfos[index].gpio_trig, HIGH);
	delayMicroseconds(10);
	digitalWrite(telemeterInfos[index].gpio_trig, LOW);
	echoDuration = pulseIn(telemeterInfos[index].gpio_echo, HIGH, ECHO_TIMEOUT_US);
	telemeterInfos[index].dist_cm = echoDuration ? ECHO_TO_CM(echoDuration) : MAX_RANGE_CM;
}

void updateWheelbotStatus() {
	// Perception
	updateTELEMETERStatus(wheelbot.leftTelemeter);
	updateTELEMETERStatus(wheelbot.rightTelemeter);
	// Reflex override steer command
	int steer_action = wheelbot.steer;
	if (telemeterInfos[wheelbot.leftTelemeter].dist_cm < ACT_RANGE_CM && telemeterInfos[wheelbot.rightTelemeter].dist_cm < ACT_RANGE_CM)
		wheelbot.poll_ms = -1; // stop
	else if (telemeterInfos[wheelbot.leftTelemeter].dist_cm < ACT_RANGE_CM)
		steer_action = 135; // turn right
	else if (telemeterInfos[wheelbot.rightTelemeter].dist_cm < ACT_RANGE_CM)
		steer_action = 45; // turn left
	// Action
	if (wheelbot.poll_ms < 0) {
		// poll off, engine off
		ledInfos[wheelbot.leftWheel].blink = 0;
		ledInfos[wheelbot.rightWheel].blink = 0;		
		ledInfos[wheelbot.leftWheel].state = LOW;
		ledInfos[wheelbot.rightWheel].state = LOW;	
	}
	else {
		// poll on
		ledInfos[wheelbot.leftWheel].blink = -1;
		ledInfos[wheelbot.rightWheel].blink = -1;		
	}
	// straight ahead: left and right wheel on during max - poll timeslice
	ledInfos[wheelbot.leftWheel].blink_on_ms = ledInfos[wheelbot.rightWheel].blink_on_ms = wheelbot.poll_max_ms - wheelbot.poll_ms;
	// reduce inner wheel speed
	int angle_delta = steer_action - 90;
	if (angle_delta > 0)
		ledInfos[wheelbot.leftWheel].blink_on_ms = (ledInfos[wheelbot.leftWheel].blink_on_ms * (90 - angle_delta)) / 90;
	else 
		ledInfos[wheelbot.rightWheel].blink_on_ms = (ledInfos[wheelbot.rightWheel].blink_on_ms * (90 + angle_delta)) / 90;
	// off timeslice = max - on timeslice
	ledInfos[wheelbot.leftWheel].blink_off_ms = wheelbot.poll_max_ms - ledInfos[wheelbot.leftWheel].blink_on_ms;
	ledInfos[wheelbot.rightWheel].blink_off_ms = wheelbot.poll_max_ms - ledInfos[wheelbot.rightWheel].blink_on_ms;
	// servo steer
	servoInfos[wheelbot.rearServo].angle = steer_action;	
}

bool handleWHEELBOTRequest(const char * req) {
	String strReq = req;
	if (strReq.startsWith("POLL/"))
		wheelbot.poll_ms = min((int)strReq.substring(5).toInt(),wheelbot.poll_max_ms);
	else if (strReq.startsWith("POLLMAX/")) {
		wheelbot.poll_max_ms = strReq.substring(8).toInt();
		wheelbot.poll_ms = -1;
	}
	else if (strReq.startsWith("STEER/"))
		wheelbot.steer = min(180,max(0,(int)strReq.substring(6).toInt()));
	else
		return false;
	return true;
}


/*
 * HTTP request main dispatch
 */

void replyHttpSuccess(String strResult) {
	wifiClient.println("HTTP/1.1 200 OK");
	wifiClient.println("Content-Type: text/plain");
	wifiClient.println("Access-Control-Allow-Origin: *");
	wifiClient.println("Connection: close");
	wifiClient.println();
	wifiClient.print(strResult);
	wifiClient.println();
}

void replyHttpError(String strError) {
	wifiClient.println("HTTP/1.1 400 Bad Request");
	wifiClient.println("Access-Control-Allow-Origin: *");
	wifiClient.println();
	wifiClient.print(strError);
	wifiClient.println();
}

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
	else if (strReq.startsWith("WHEELBOT/"))
		result = handleWHEELBOTRequest(strReq.substring(9).c_str());
	if (result)
		replyHttpSuccess(strReq);
	else
		replyHttpError("ERROR");
	return result;
}


/* 
 * Main loop
 */


void updateStatus() {
	int deviceIndex;
	updateWheelbotStatus();
	for (deviceIndex=0; deviceIndex<N_LED; deviceIndex++)
		updateLEDStatus(deviceIndex);
	for (deviceIndex=0; deviceIndex<N_SERVO; deviceIndex++)
		updateSERVOStatus(deviceIndex);
	for (deviceIndex=0; deviceIndex<N_STEPPER; deviceIndex++)
		updateSTEPPERStatus(deviceIndex);
}

void delayedWifiClientStop(int start_ms) {
	while (wifiClient && wifiClient.connected() && millis() < start_ms + WIFI_CLIENT_DELAY_MS) {
		updateStatus();
		delay(MINIMUM_UPDATE_MS);
	}
	if (wifiClient)
		wifiClient.stop();
}

void loop() {
	int start_loop_ms;
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
			delayedWifiClientStop(millis());
		}
		updateStatus();
		pollDelay(MAIN_LOOP_POLL_MS, start_loop_ms);
		wifiStatus = WiFi.status();
	}
}
