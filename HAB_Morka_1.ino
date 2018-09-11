#include "Arduino.h"
#include <LibAPRS.h>
#include <TinyGPS++.h>


// kas kiek laiko (sekundemis) siusim pozicijos paketa
#define PACKET_INTERVAL 60

//The setup function is called once at startup of the sketch
// 3V3 arba 5V, mes naudojam 3.3V. Kadangi nenaudojam RX, tai sitas nera svarbu
#define ADC_REFERENCE REF_3V3
#define SERIAL_BAUDRATE 9600	// GPS modulis nusiresetina i sita greiti, tai taip ir paliekam

// LY3FF-1
#define APRS_CALLSIGN "LY3FF"
#define APRS_SSID     1

// milisekundemis
#define DELAY_AFTER_PTT_ON 500
#define APRS_PREAMBLE 550

#define OPEN_SQUELCH false
#define PTT_PIN 11

#define ON LOW
#define OFF HIGH

#define VERSION_NUMBER "0.1b"
#define VERSION VERSION_NUMBER " "  __DATE__

struct SUptime {
	uint32_t uptime;
	uint32_t days;
	uint8_t  hours;
	uint8_t  mins;
	uint8_t  secs;
};

struct SAPRSLocation{
	char latitude[9]; // 8 + null
	char longitude[10]; // 9 + null
};

SAPRSLocation aprsLocation;
SUptime SYSUptime;
TinyGPSPlus gps;

char strBuffer[40];		// buferis char operacijoms

uint32_t last_timestamp, timestamp = 0;
uint32_t cMillis, lMillis = 0;

// funkcija, kuri priima ieinanti paketa, kadangi mes tik transliuojam
// tai si funkcija tuscia.
void aprs_msg_callback(struct AX25Msg *msg) {
}

void updateSysUptime(const uint32_t& tstamp){
	SYSUptime.uptime =  tstamp;
    SYSUptime.days   =  tstamp / 86400;
    SYSUptime.hours  = (tstamp / 3600 ) % 24;
    SYSUptime.mins   = (tstamp / 60   ) % 60;
    SYSUptime.secs   =  tstamp % 60;
}

volatile uint32_t newlines = 0UL;

void setup() {
	updateSysUptime(0);
	gpsToAprs(0,0);
	// Greitis pagal GPS imtuva. RX pin - imtuvas, TX - debug output (USB)
	Serial.begin(SERIAL_BAUDRATE);

	pinMode(13, OUTPUT);	//LED
	digitalWrite(13, LOW);
	pinMode(PTT_PIN, OUTPUT);		//PTT
	digitalWrite(PTT_PIN, OFF);

	// Initialise APRS library - This starts the modem
	APRS_init(ADC_REFERENCE, OPEN_SQUELCH);
	// Callsign & SSID
	APRS_setCallsign(APRS_CALLSIGN, APRS_SSID);

	// You can define preamble and tail like this:
	 APRS_setPreamble(APRS_PREAMBLE);
	 APRS_setTail(70);

	// Balionas - "O", zmogus - "[", masina - ">"
	// APRS_setSymbol('O');   // balionas "O", jei pagrindine simboliu lentele
	APRS_setSymbol('[');	// zmogus - "["
	Serial.println(F("*** Startup ***"));
	Serial.println(F("Seklys MORKA-1"));
	Serial.println(F("Versija " VERSION));

}

void locationUpdate(){
	setPTT(ON);
	APRS_setPower(2);
	// http://www.earthpoint.us/Convert.aspx
	// GPS duoda Laipsniai.laipsnio dalys
//	Location: 54,705383,25.252481  Date/Time: 9/9/2018 15:31:43.00

	// APRS reikia: Laipsniai Minutes.minutes dalys (ne sekundes)
	//5442.30N lon 02515.19E
	APRS_setLat(aprsLocation.latitude);
	APRS_setLon(aprsLocation.longitude);
//	char *comment = "LibAPRS test";
//	APRS_sendLoc(comment, strlen(comment));
	sprintf(strBuffer,"Morka-1 UP: %d d. %02d:%02d", (int) SYSUptime.days,(int) SYSUptime.hours, (int) SYSUptime.mins);
	Serial.println(strBuffer);
	APRS_sendLoc(strBuffer, strlen(strBuffer));
	setPTT(OFF);
}

/*
 * PTT valdom rankiniu budu, kadangi
 * 1) automatinis valdymas mums netinka, nes PTT siam moduliui reikia paspausti ir palaikyti bent puse sekundes,
 *    kad issiustu visa paketa
 * 2) reikalingas active LOW PTT lygis.
 */
void setPTT(int state){
	digitalWrite(PTT_PIN, state);
	if (state == ON) {
		delay(DELAY_AFTER_PTT_ON);
	}
}

void blink(int d=25){
	// ciklo laikas milisekundemis = d * 2
		digitalWrite(13, HIGH);
		delay(d);
		digitalWrite(13, LOW);
		delay(d);
}

void gpsToAprs(double lat, double lon){
	// TODO galima patobulinti, kad rasytu i paduota, o ne globalia struktura
	char h_lat;
	char h_lon;

	// nusistatom pusrutulius, atitinkamai verciam is neigiamu laipsniu i teigiamus
	if (lat >=0){
		h_lat='N';
	} else {
		h_lat='S';
		lat=-lat;
	}

	if (lon >=0){
		h_lon='E';
	} else {
		h_lon='W';
		lon=-lon;
	}

	int deg_lat = (int) lat;// konversija i int panaikina skaiciu po kablelio.
	int deg_lon = (int) lon;

	double f = lat * 60;
	int lat_min = (uint32_t) f % 60;	 // minutes
	f *= 100;
	int lat_min_p = (uint32_t) f % 100; // minuciu skaicius po kablelio

	f = lon * 60;
	int lon_min = (uint32_t) f % 60;	 // minutes
	f *= 100;
	int lon_min_p = (uint32_t) f % 100; // minuciu skaicius po kablelio

	// rasom i tarpini buferi, kadangi kitaip nesikompiliuoja kodas
	snprintf_P(strBuffer, sizeof(strBuffer), PSTR("%02d%02d.%02d%c"), deg_lat,
			lat_min, lat_min_p, h_lat);
	strncpy(aprsLocation.latitude, strBuffer, sizeof(aprsLocation.latitude));

	snprintf_P(strBuffer, sizeof(strBuffer), PSTR("%03d%02d.%02d%c"), deg_lon,
			lon_min, lon_min_p, h_lon);
	strncpy(aprsLocation.longitude, strBuffer, sizeof(aprsLocation.longitude));

	// segfault :/
//	snprintf_P(aprsLocation.latitude, sizeof(aprsLocation.latitude),
//			PSTR("%02d%02d.%02d%01s"), deg_lat, lat_min, lat_min_p,h_lat
//			);
//	snprintf_P(aprsLocation.longitude, sizeof(aprsLocation.longitude),
//			PSTR("%03d%02d.%02d%01s"), deg_lon, lon_min, lon_min_p,h_lon
//			);
}

void displayInfo(){
	// debug output'as i serial porta
	Serial.print(F("Location: "));
	if (gps.location.isValid()) {
		Serial.print(gps.location.lat(), 6);
		Serial.print(F(","));
		Serial.print(gps.location.lng(), 6);
	} else {
		Serial.print(F("INVALID"));
	}

	Serial.print(F("  Date/Time: "));
	if (gps.date.isValid()) {
		Serial.print(gps.date.month());
		Serial.print(F("/"));
		Serial.print(gps.date.day());
		Serial.print(F("/"));
		Serial.print(gps.date.year());
	} else {
		Serial.print(F("INVALID"));
	}

	  Serial.print(F(" "));
	if (gps.time.isValid()) {
		if (gps.time.hour() < 10)
			Serial.print(F("0"));
		Serial.print(gps.time.hour());
		Serial.print(F(":"));
		if (gps.time.minute() < 10)
			Serial.print(F("0"));
		Serial.print(gps.time.minute());
		Serial.print(F(":"));
		if (gps.time.second() < 10)
			Serial.print(F("0"));
		Serial.print(gps.time.second());
		Serial.print(F("."));
		if (gps.time.centisecond() < 10)
			Serial.print(F("0"));
		Serial.print(gps.time.centisecond());
	} else {
		Serial.print(F("INVALID"));
	}
	Serial.println();
}

/*
 * Kvieciamas kas sekunde, nors negarantuotai, kad kas kiekviena sekunde. Kvieciama is main loop()
 */
void timerEverySecond(){
	updateSysUptime(timestamp);
	blink();
	if (gps.location.isValid()) {
		blink(50);
	}

	if ((timestamp % 3) == 0) {
		Serial.print(String(aprsLocation.latitude) + '/' + String(aprsLocation.longitude)+" ");
		displayInfo();
	}
	if ((timestamp % 10) == 0) {
		Serial.println("Up: " + String(SYSUptime.uptime));
	}
	if ((timestamp % PACKET_INTERVAL) == 0) {
		// sitas isValid() nevisai geras budas, nes kartais gali issiusti neteisinga arba sena informacija
		if (gps.location.isValid()) {
			Serial.println(F("Sending aprs location packet"));
			locationUpdate();
		}
	}

}

// The loop function is called in an endless loop
void loop() {
	timestamp = millis()/1000;

	while (Serial.available()){
	    if (gps.encode(Serial.read())){
	    	if (gps.location.isValid()) {
	    		gpsToAprs(gps.location.lat(), gps.location.lng());
	    	}
	    }
	}

	if (timestamp != last_timestamp) timerEverySecond();
	last_timestamp = timestamp;
	delay(100);
}
