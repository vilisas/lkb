// kas kiek laiko (sekundemis) siusim pozicijos paketa
#define PACKET_INTERVAL 60
#define TELEMETRY_PACKET_INTERVAL 30

#include <APRSTelemetry.h>



// 3V3 arba 5V, mes naudojam 3.3V. Kadangi nenaudojam RX, tai sitas nera svarbu
#define ADC_REFERENCE REF_3V3
#define SERIAL_BAUDRATE 9600	// GPS modulis nusiresetina i sita greiti, tai taip ir paliekam

// LY3FF-1
#define APRS_CALLSIGN "LY1BWB"
#define APRS_SSID     15

// saukinys 9 baitu ilgio formate telemetrijos paketu generavimui, bus irasyta i progmem
#define APRS_TELEMETRY_CALL "LY1BWB-15"
//                           123456789
// time in milliseconds
#define DELAY_AFTER_PTT_ON 500
#define APRS_PREAMBLE 550

#define OPEN_SQUELCH false
#define PTT_PIN 11

// Temperaturos davikliai ant sios kojos
#define ONE_WIRE_BUS 12

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
# define MB_ADC_VOLTAGE 3.2f
//#define MB_MAX_VOLTAGE ((MB_DIVIDER_R1 + MB_DIVIDER_R2) * MB_ADC_VOLTAGE) / MB_DIVIDER_R2


