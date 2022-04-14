#include "DPS.ino"
#include "BNO055.ino"
#include "GNSS.ino"
#include "OMRON_2SMPB02E.ino"
#include "XBeeModule.ino"

String xbee_port = 0; // to be set

void setup() {

    Serial.begin(115200);
    bno_init();
    dps_init();
    gnss_init();
    omron_init();

}

void loop() {
    ReadByte(xbee_port);
    bno_execute();
    dps_execute();
    gnss_execute();
    omron_execute();
}