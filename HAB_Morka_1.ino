#include "Arduino.h"
#include <LibAPRS.h>
#include <TinyGPS++.h>
#include <OneWire.h>
#include <DallasTemperature.h>


// kas kiek laiko (sekundemis) siusim pozicijos paketa
#define PACKET_INTERVAL 60

//The setup function is called once at startup of the sketch
// 3V3 arba 5V, mes naudojam 3.3V. Kadangi nenaudojam RX, tai sitas nera svarbu
#define ADC_REFERENCE REF_3V3
#define SERIAL_BAUDRATE 9600	// GPS modulis nusiresetina i sita greiti, tai taip ir paliekam

// LY3FF-1
#define APRS_CALLSIGN "LY3FF"
#define APRS_SSID     15

// saukinys 9 baitu ilgio formate telemetrijos paketu generavimui, bus irasyta i progmem
#define APRS_TELEMETRY_CALL "LY3FF-15 "
//                           123456789
// milisekundemis
#define DELAY_AFTER_PTT_ON 500
#define APRS_PREAMBLE 550

#define OPEN_SQUELCH false
#define PTT_PIN 11

// Temperaturos davikliai ant sios kojos
#define ONE_WIRE_BUS 12



#define ON LOW
#define OFF HIGH

#define VERSION_NUMBER "0.1b"
#define VERSION VERSION_NUMBER " "  __DATE__

#define TELEMETRY_TEST 1

class APRSTelemetry{
	// KISS -- Keep It Simple Stupid
	/*
	 * Analog channel 1 - Battery Voltage
	 * Analog channel 2 - Inside temperature
	 * Analog channel 3 - Outside temperature
	 *
	 * 8 digital channels
	 *
	 * every digital channel is set as literal 1 or 0 directly in bits[8] char array.
	 * Later we just add this array to the telemetry packet end
	 *
	 */

	/*
	 * Note: Again, the field widths are not all the same, and the byte counts include the comma separators
	 * where shown. The list can terminate after any field.
	*/


	private:
	char temperature[2] = { 0, 0 };
	char voltage = 0;

	unsigned int ts_read_sensors = 0; //
	unsigned int ts_telemetry_packet = 0;
	unsigned int ts_unit_packet = 0;
	unsigned int ts_parameter_packet = 0;
	unsigned int ts_eqns_packet = 0;

	OneWire *oneWire;
	DallasTemperature *sensors;
//	OneWire oneWire(12);
//	DallasTemperature sensors(&oneWire);
	//	OneWire oneWire(ONE_WIRE_BUS);
	//	DallasTemperature sensors(&oneWire);


	public:
	//char packet_buffer[34];	// siaip 34 baitai su null gale, bet paliekam truputi atsargos
	char packet_buffer[40];
	int packet_length = 0;
	char bits[9] = { '0', '0', '0', '0', '0', '0', '0', '0', 0 };

	const PROGMEM char TELEMETRY_PARAM_NAMES[] = ":" APRS_TELEMETRY_CALL ":PARM.Battery,ITemp,OTemp";
	const PROGMEM char TELEMETRY_PARAM_UNITS[] = ":" APRS_TELEMETRY_CALL ":UNIT.V/100,deg.C,deg.C";
	// TODO: update equations
	const PROGMEM char TELEMETRY_PARAM_EQUATIONS[] = ":" APRS_TELEMETRY_CALL ":EQNS.0,5.2,0,0,0,.53,-32,3,4.39,49,-32,3,18,1,2,3" ;

	/*
	 * five channels in sets of 3
	 * a * v**2 + b * v + c
	 *
	 */

	void updateBatteryVoltage(double voltage){
		//TODO: nustatyti itampos kanalo baita pagal lygti
		// (int) (4.2 * 100) / 5,2 = 80
		this->voltage =  (voltage * 100) / 5.2;

	}

	void updateTemperatures(int temp1, int temp2){
		// TODO: nustatyti temperaturu baitus pagal lygtis
		this->temperature[0] = temp1;
		this->temperature[1] = temp2;
	}

	void generateTelemetryPacket(){
		/*
		 * suraso telemetrijos paketa i packet_buffer. nurodo paketo ilgi i packet_length
		 */
		// T#MIC199,000,255,073,123,01101001   -- 33 bytes
//		packet_length = sprintf_P(packet_buffer, PSTR("aaa"),0);
		packet_length = sprintf_P(packet_buffer, PSTR("T#MIC%03u,%03u,%03u,%03u,%03u,"),
				this->voltage, this->temperature[0], this->temperature[1], 0, 0);
		strncat(&packet_buffer[packet_length],bits, 8);
		packet_length +=8;

	}

	void generateParametersPacket(){
		Serial.println(F("Generating parameters packet"));

	}

	void generateUnitsPacket(){
		// TODO generate unit description packet
		Serial.println(F("Generating units packet"));

	}

	void generateEQNSPacket(){
		// TODO generate equations packet
		Serial.println(F("Generating equations packet"));

	}

	void sendTelemetryPacket(){
		Serial.print(F("packet_buffer='"));
		Serial.print(packet_buffer);
		Serial.println(F("'"));
		Serial.print(F("packet_length="));
		Serial.println(this->packet_length);
		memset(packet_buffer,0,sizeof(packet_buffer));
		packet_length=0;
	}


	void setBit(uint8_t bit, bool state){
		if (bit - 1 < 8) {
			bits[bit] = (state ? '1' : '0');
		}
	}

	bool getBit(uint8_t bit){
		if (bit - 1 < 8) {
			return (bits[bit] == '1');
		}
		return false;
	}

	void readTemperatureSensors(){

		  Serial.print(F("Requesting temperatures..."));
		sensors->requestTemperatures();
//		  sensors.requestTemperatures(); // Send the command to get temperatures
		  Serial.println(F("DONE"));
//		  Serial.println(sensors->getTempCByIndex(0));
//		  Serial.println(sensors->getTempCByIndex(1));
//		  Serial.print(F("Temperature for the device 1 (index 0) is: "));
//		  Serial.println(sensors.getTempCByIndex(0)); // onboard
//		  Serial.println(sensors.getTempCByIndex(1)); // ant laido


	}


	void setup(){
		Serial.println(F("APRSTelemetry::setup()\nInitializing temp. sensors"));

		this->oneWire = new OneWire(ONE_WIRE_BUS);
		this->sensors = new DallasTemperature(this->oneWire);

		sensors->begin();
		sensors->setResolution(9); // zemesne rezoliucija - greiciau nuskaito (9 bitai - puses laipsnio tikslumu)


	}

	void loop(){
		unsigned int seconds = (millis() / 1000);

		if (seconds - this->ts_read_sensors > 3){
			// read temperature sensors
			this->readTemperatureSensors();

			this->updateTemperatures(sensors->getTempCByIndex(0), sensors->getTempCByIndex(1));
//			telemetrija.updateTemperatures(sensors.getTempCByIndex(0), sensors.getTempCByIndex(1));
			this->ts_read_sensors = seconds;
		}

		if (seconds - this->ts_telemetry_packet > 10){
			this->generateTelemetryPacket();
			this->sendTelemetryPacket();
			this->ts_telemetry_packet = seconds;
		}



		if (seconds - this->ts_unit_packet > 30){
			this->generateUnitsPacket();
			this->sendTelemetryPacket();
			this->ts_unit_packet = seconds;
		}

		if (seconds - this->ts_parameter_packet > 40){
			this->generateParametersPacket();
			this->sendTelemetryPacket();
			this->ts_parameter_packet = seconds;
		}

		if (seconds - this->ts_eqns_packet > 50){
			this->generateEQNSPacket();
			this->sendTelemetryPacket();
			this->ts_eqns_packet = seconds;
		}





	}





#ifdef TELEMETRY_TEST
	void test(){

	}
#endif

};





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
};

SAPRSLocation aprsLocation;
SUptime SYSUptime;
TinyGPSPlus gps;
APRSTelemetry telemetrija;


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
	Serial.println(F("HAB LKB"));
	Serial.println(F("Versija " VERSION));
	telemetrija.setup();


}

void locationUpdate(){
	/*
	 * Cia paruosiam lokacijos paketa, sukuriam komentara ir istransliuojam
	 */

	setPTT(ON);

	// http://www.earthpoint.us/Convert.aspx
	// GPS duoda Laipsniai.laipsnio dalys
    // Location: 54,705383,25.252481  Date/Time: 9/9/2018 15:31:43.00
	// APRS reikia: Laipsniai Minutes.minutes dalys (ne sekundes)
	// 5442.30N lon 02515.19E

	APRS_setLat(aprsLocation.latitude);
	APRS_setLon(aprsLocation.longitude);

	int count = sprintf(strBuffer,"LKB Up: %u d. %u h.", SYSUptime.days, SYSUptime.hours); // "LKB Up: 1 d. 15 h.";
//	Serial.println(strBuffer);

	APRS_sendLoc(strBuffer, count);
//	APRS_sendLoc(strBuffer, strlen(strBuffer));
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
//	readSensors();

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
