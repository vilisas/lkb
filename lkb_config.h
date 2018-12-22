// kas kiek laiko (sekundemis) siusim pozicijos paketa
#define PACKET_INTERVAL 60
#define TELEMETRY_PACKET_INTERVAL 45
//#define TELEMETRY_PACKET_INTERVAL 15

// store valid GPS position to EEPROM
#define STORE_GPS_POSITION_INTERVAL 120


#include <APRSTelemetry.h>



// 3V3 arba 5V, mes naudojam 3.3V. Kadangi nenaudojam RX, tai sitas nera svarbu
#define ADC_REFERENCE REF_3V3

// GPS input and debug output rate
#define SERIAL_BAUDRATE 115200

// LY3FF-1
#define APRS_CALLSIGN "LY1BWB"
#define APRS_SSID     15

// saukinys 9 baitu ilgio formate telemetrijos paketu generavimui, bus irasyta i progmem
#define APRS_TELEMETRY_CALL "LY1BWB-15"
//                           123456789
// time in milliseconds after CS set to HIGH and PTT activated
#define DELAY_AFTER_PTT_ON 2550
#define DELAY_AFTER_PTT_OFF 800
//#define DELAY_AFTER_PTT_ON 3550
#define APRS_PREAMBLE 550

// Ismatuotos ir rodomos itampos skirtumas, naudojama apskaiciuojant atmega328p maitinimo itampa pagal bandgap voltage
#define VCC_OFFSET -160

#define OPEN_SQUELCH false
#define PTT_PIN 11

// chip select pin - LOW - TRX shutdown
// A1
#define CS_PIN (char) A1


// 1-Wire temperature sensors pin
// D12
#define ONE_WIRE_BUS 12

#define LED_PIN 13


#define ON LOW
#define OFF HIGH

/*
 * Max reported voltage - 6.4 V, see scripts/battery.pl
 */
# define VOLTAGE_ADC_PIN (char) A2

// resistor values in Ohms
# define MB_DIVIDER_R1  10000
# define MB_DIVIDER_R2  10000

/*
 * ADC (Vcc/maitinimo) itampa, cia reikia nurodyti kuo tikslesne, kad baterijos itampos matavimai butu
 * kuo tikslesni. 5.2f
 */
# define MB_ADC_VOLTAGE 3.33f	// pololu upconverter VOUT
//# define MB_ADC_VOLTAGE 3.2f
//# define MB_ADC_VOLTAGE 4.7f	//arduino per usb
//#define MB_MAX_VOLTAGE ((MB_DIVIDER_R1 + MB_DIVIDER_R2) * MB_ADC_VOLTAGE) / MB_DIVIDER_R2

// Balionas - "O", zmogus - "[", masina - ">"
// APRS_setSymbol('O');   // balionas "O", jei pagrindine simboliu lentele
#define APRS_SYMBOL '['

