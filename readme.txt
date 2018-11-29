
Projektas LKB - Lietuviška kelionė balionu


RX  - 9600 BPS GPS imtuvas (Hardware Serial)
TX  - Laisvas, galima naudot debug output'ui

D4 \
D5 |_  4 bitu DAC APRS signalo generavimui (pagal MicroModem manual)
D6 | 
D7 /

D11 - PTT, kitos porto kojos naudojamos LibAPRS bibliotekos, nežiūrint to, kad signalo generavimui naudojami tik 4 bitai - 4 porto kojos
D13 - Status LED.
A0  - Naudoja libAPRS, taigi, jos naudoti negalim.
A1  - Naudojam baterijos įtampos nuskaitymui.


LED mirgsejimai:
 1x - programa veikia
 2x - programa veikia ir gauname gera gps pozicija is imtuvo.


Naudojamos bibliotekos:

TinyGPSPlus-1.0.2
LibAPRS-master
APRSTelemetry



