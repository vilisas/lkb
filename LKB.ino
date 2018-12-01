/*
 * LKB Lietuviska Kelione Balionu
 * APRS High Altitude Balloon project
 *
 *
 * Dependencies:
 * APRSTelemetry
 * APRSBase91 (soon)
 *
 * Arduino: Arduino nano
 * GPS imtuvas
 * VHF trx modulis: SA828
 * Itampos reguliatorius: Pololu2561
 * GPS ir arduino maitinasi nuo Pololu2561 (Step up ir stabilizacija jei > 3.2v)
 * TRX maitinasi tiesiogiai nuo baterijos
 * APRS signala generuoja arduino naudojant libAPRS, RX nenaudojamas
 * Temperaturos sensoriai (2) 1-WIRE Dallas ds18b20
 *
 * Telemetrija:
 * Baterijos itampa
 * Isores temperatura
 * Vidaus temperatura
 *
 * TODO:
 * Telemetrija:
 * 	aukstis pedomis
 *
 */

// config must be included first
#include "lkb_config.h"
#include "Arduino.h"
#include <LibAPRS.h>
#include <TinyGPS++.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <APRSTelemetry.h>
#include <EEPROM.h>


#define VERSION_NUMBER "0.3b"
#define VERSION VERSION_NUMBER "-dev-"  __DATE__

	const PROGMEM char TELEMETRY_PARAM_NAMES[] = ":" APRS_TELEMETRY_CALL ":PARM.Battery,ITemp,OTemp";
	const PROGMEM char TELEMETRY_PARAM_UNITS[] = ":" APRS_TELEMETRY_CALL ":UNIT.v/100,deg.C,deg.C";
	// TODO: update equations
//	const PROGMEM char TELEMETRY_PARAM_EQUATIONS[] = ":" APRS_TELEMETRY_CALL ":EQNS.0,5.2,0,0,0,.53,-32,3,4.39,49,-32,3,18,1,2,3" ;
//	const PROGMEM char TELEMETRY_PARAM_EQUATIONS[] = ":" APRS_TELEMETRY_CALL ":EQNS.0,5.2,0,0,1,-55,0,1,-55" ;
	const PROGMEM char TELEMETRY_PARAM_EQUATIONS[] = ":" APRS_TELEMETRY_CALL ":EQNS.0,1,0,0,1,-80,0,1,-80" ;

    const double _mbMaxVoltage = (((double) MB_DIVIDER_R1 + (double) MB_DIVIDER_R2) * (double) MB_ADC_VOLTAGE) / (double) MB_DIVIDER_R2;
    // 	Vs	 = ((R1 + R2) * Vs) / R2



struct SUptime {
	uint16_t uptime;
	uint16_t days;
	uint16_t  hours;
	uint16_t  mins;
	uint16_t  secs;
};

struct SAPRSLocation{
	char latitude[9]; // 8 + null
	char longitude[10]; // 9 + null
	uint16_t checksum;
};

struct SAPRSLocation aprsLocation;
SUptime SYSUptime;
TinyGPSPlus gps;
OneWire *oneWire;
DallasTemperature *temperature;

APRSTelemetry telemetrija;


char strBuffer[50];		// buferis char operacijoms

uint32_t last_timestamp, timestamp = 0;
uint32_t cMillis, lMillis = 0;

unsigned int calculate_APRSLocation_checksum();

// funkcija, kuri priima ieinanti paketa, kadangi mes tik transliuojam
// tai si funkcija tuscia.
void aprs_msg_callback(struct AX25Msg *msg) {}

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

	pinMode(CS_PIN, OUTPUT);		//PTT
	digitalWrite(PTT_PIN, LOW);


	// Initialise APRS library - This starts the modem
	APRS_init(ADC_REFERENCE, OPEN_SQUELCH);

	// Callsign & SSID
	APRS_setCallsign(APRS_CALLSIGN, APRS_SSID);

	// You can define preamble and tail like this:
	 APRS_setPreamble(APRS_PREAMBLE);
	 APRS_setTail(70);

	// Balionas - "O", zmogus - "[", masina - ">"
	// APRS_setSymbol('O');   // balionas "O", jei pagrindine simboliu lentele
	APRS_setSymbol(APRS_SYMBOL);	// zmogus - "["
	Serial.println(F("*** Startup ***"));
	Serial.println(F("HAB LKB"));
	Serial.println(F("Versija " VERSION));

	telemetrija._tpe = TELEMETRY_PARAM_EQUATIONS;
	telemetrija._tpn = TELEMETRY_PARAM_NAMES;
	telemetrija._tpu = TELEMETRY_PARAM_UNITS;
	telemetrija.setSendPacketCallback(&sendTelemetryPacket);
	telemetrija.setup();

	// temperature
	oneWire = new OneWire(ONE_WIRE_BUS);
	temperature = new DallasTemperature(oneWire);
	temperature->begin();
	temperature->setResolution(9); // 0.5 degree resolution


	EEPROM.get(0, aprsLocation);


}

void locationUpdate(){
	/*
	 * Cia paruosiam lokacijos paketa, sukuriam komentara ir istransliuojam
	 */

	// http://www.earthpoint.us/Convert.aspx
	// GPS duoda Laipsniai.laipsnio dalys
    // Location: 54,705383,25.252481  Date/Time: 9/9/2018 15:31:43.00
	// APRS reikia: Laipsniai Minutes.minutes dalys (ne sekundes)
	// 5442.30N lon 02515.19E

	APRS_setLat(aprsLocation.latitude);
	APRS_setLon(aprsLocation.longitude);

//	int count = sprintf(strBuffer, "LKB ALT: %u m., up: %u d. %u h.", (unsigned int) gps.altitude.meters(), SYSUptime.days, SYSUptime.hours); // "LKB Up: 1 d. 15 h.";
	telemetrija.setBit(1, gps.location.isValid());
	telemetrija.updateTelemetrySequence();
	int count = sprintf(strBuffer, "LKB HAB |%s|", telemetrija._telemetry_comment);
	Serial.println(strBuffer);

	setPTT(ON);
	APRS_sendLoc(strBuffer, count);
//	APRS_sendLoc(strBuffer, strlen(strBuffer));
	setPTT(OFF);
}

/*
 * PTT valdom rankiniu budu, kadangi
 * 1) automatinis valdymas mums netinka, nes PTT siam moduliui reikia paspausti ir palaikyti bent puse sekundes,
 *    kad issiustu visa paketa
 * 2) reikalingas active LOW PTT lygis.
 * TODO: use enumeration
 */
void setPTT(int state){
	if (state == ON) {
		// set CS HIGH - turn on trx
		digitalWrite(CS_PIN, HIGH);
		digitalWrite(PTT_PIN, LOW);
		// set PTT LOW
		delay(DELAY_AFTER_PTT_ON);
	} else {
//		turn PTT OFF
		digitalWrite(PTT_PIN, HIGH);
//		set CS LOW - shut down trx
		digitalWrite(CS_PIN, LOW);
	}
}

void blink(int d=25){
	// ciklo laikas milisekundemis = d * 2
		digitalWrite(13, HIGH);
		delay(d);
		digitalWrite(13, LOW);
		delay(d);
}

unsigned int calculate_APRSLocation_checksum() {
	unsigned int sum = 0;
	for (uint8_t i = 0; i < sizeof(aprsLocation.latitude); i++) {
		sum += (uint8_t) aprsLocation.latitude[i];
	}

	for (uint8_t i = 0; i < sizeof(aprsLocation.longitude); i++) {
		sum += (uint8_t) aprsLocation.longitude[i];
	}
	Serial.print("cheksum=");
	Serial.println(sum);
	return sum;
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


//	aprsLocation.checksum =
	aprsLocation.checksum = calculate_APRSLocation_checksum();

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
		readBatteryVoltage();
		readTemperatureSensors();


		Serial.print(String(aprsLocation.latitude) + '/' + String(aprsLocation.longitude)+
				" sats=" + gps.satellites.value() +
				" hdop=" + gps.hdop.value() + " "
				);
		displayInfo();


	}
	if ((timestamp % 10) == 0) {
		Serial.println("Up: " + String(SYSUptime.uptime));
	}
	if ((timestamp % PACKET_INTERVAL) == 0) {
		// sitas isValid() nevisai geras budas, nes kartais gali issiusti neteisinga arba sena informacija
		// only if current GPS location is valid OR one read from eeprom has valid checksum
		if (gps.location.isValid() || calculate_APRSLocation_checksum() == aprsLocation.checksum)
		{
			Serial.println(F("Sending aprs location packet"));
			locationUpdate();
		}
	}

	if ((timestamp % STORE_GPS_POSITION_INTERVAL) == 0) {
		if (gps.location.isValid()){
			setPTT(OFF); 	// just in case
			EEPROM.put(0, aprsLocation);
			Serial.println("GPS location backed up to EEPROM");
		}

	}


}

/**
 * reads battery voltage from ADC pin
 */
void readBatteryVoltage() {
	/* Parodo baterijos itampa serial porte, BET, itampa skaiciuojama analog reference atzvilgiu, kas pas mus yra maitinimo itampa.
	 * Kai modulis maitinasi is baterijos per itampos stabilizatoriu, tai analog reference = 3.2V, bet jei prisijungiam per USB
	 * tai itampa gaunasi 5V. Rezultate _battery_voltage reiksme bus skirtinga. Darbiniam rezime telemetrijos pakete viskas turetu eiti.
	 * Norint serial porte matyti teisinga reiksme, reikia pakeisti MB_ADC_VOLTAGE is 3.2 i 5.0
	 */
	uint16_t adcValue = analogRead(VOLTAGE_ADC_PIN);
	Serial.print(F("adcValue="));
	Serial.println(adcValue);
	double voltage = (double) (adcValue * _mbMaxVoltage) / 1024;
	telemetrija.setBatteryVoltage(voltage);
	Serial.print("voltage=");
	Serial.println(voltage);
//	telemetrija.setBatteryVoltage((double) (adcValue * _mbMaxVoltage) / 1024);
}

void readTemperatureSensors() {
	temperature->requestTemperatures();
	telemetrija.updateTemperatures(temperature->getTempCByIndex(0), temperature->getTempCByIndex(1));

}

/**
 * callback for APRSTelemetry class to actually send our generated packet.
 */
void sendTelemetryPacket(){
		setPTT(ON);
		Serial.println("sending telemetry packet");
		Serial.println(String(telemetrija._packet_buffer));
		Serial.print(F("packet_length="));
		Serial.println((unsigned int) telemetrija._packet_length);
		APRS_sendPkt(telemetrija._packet_buffer, telemetrija._packet_length);
		setPTT(OFF);
}



// The loop function is called in an endless loop
void loop() {
	timestamp = millis()/1000;

	// read GPS info from serial port if available
	while (Serial.available()){
	    if (gps.encode(Serial.read())){
	    	if (gps.location.isValid()) {
	    		gpsToAprs(gps.location.lat(), gps.location.lng());
	    	}
	    }
	}

	telemetrija.loop();		// do telemetry stuff - generate and send packets when needed

	if (timestamp != last_timestamp) timerEverySecond();
	last_timestamp = timestamp;
	delay(100);
}
