
Projektas LKB - Lietuviška kelionė balionu



CPU: ATMega328p (Arduino nano)
TRX: SA828 VHF module
GPS:
PSU: (CPU ir GPS maitinimui) Pololu 3.3V step up Voltage Regulator (U1V11F3)



PD0 RX  - 115200 BPS GPS imtuvas (Hardware Serial)
PD1 TX  - Debug output
PD2 D2*
PD3 D3* - libAPRS PTT

PD4 D4 \
PD5 D5 |_  4 bitu libAPRS DAC APRS signalo generavimui (pagal MicroModem manual)
PD6 D6 | 
PD7 D7 /


PB0 D8*   - 
PB1 D9*   - libAPRS RF TX LED
PB2 D10*  - libAPRS RF RX LED

PB3 D11 - PTT
PB4 D12 - 1Wire temperatūros sensoriai
PB5 D13 - Status LED.

PC0 A0* - Naudoja libAPRS audio in, mes jo nenaudojam.
PC1 A1  - Baterijos įtampos matavimas.
PC2 A2  - TRX CS - TRX modulio shutdown, kitaip jis veikia RX rezime. 
* - LKB šių kojų nenaudoja, bet jas naudoja libAPRS.

LED mirgsejimai:
 1x - programa veikia
 2x - programa veikia ir gauname gera gps pozicija is imtuvo.


Naudojamos bibliotekos:

TinyGPSPlus-1.0.2	https://github.com/mikalhart/TinyGPSPlus/
LibAPRS-master		https://github.com/markqvist/LibAPRS
APRSTelemetry		https://github.com/vilisas/APRSTelemetry




