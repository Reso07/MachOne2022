// This example gets individual Sensor objects for temperature/pressure!

#include <Adafruit_DPS310.h>
#include <iostream>
/*
using std::cerr;
using std::endl;
#include <fstream>
using std::ofstream;
*/

Adafruit_DPS310 dps;
Adafruit_Sensor *dps_temp = dps.getTemperatureSensor();
Adafruit_Sensor *dps_pressure = dps.getPressureSensor();

float peak_altitude;
// float iterator;

// ofstream outdata;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("DPS310");
  while (!dps.begin_I2C(0x76)) {
    Serial.println("Failed to find DPS");
    while (1) yield();
  }
  Serial.println("DPS OK!");

  // Setup highest precision
  dps.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
  dps.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);

  dps_temp->printSensorDetails();
  dps_pressure->printSensorDetails();

  peak_altitude = 0.0;

  /*
  iterator = 0;

  outdata.open("altitude.txt"); // opens the file
  if( !outdata ) { // file couldn't be opened
    Serial.println("Error: COULDN'T OPEN FILE");
    exit(1);
  }
  */

}

float KALMAN(float U) {
    static const float R = 40; // noise covariance (has to be calibrated)
    static const float H = 1.0; // measurement map scalar
    static float Q = 30; // initial estimated covariance
    static float P = 0; // inital error covariance (must be 0)
    static float U_hat = 0; // inital estimated state (assume we don't know)
    static float K = 0; // inital Kalman gain

    // begin
    K = P * H / (H * P * H + R);
    U_hat = U_hat + K * (U - H * U_hat);

    // update error covariance
    P = (1 - K * H) * P + Q;

    return U_hat;
}

void launchChute() {
  Serial.println("PARACHUTE IS LAUNCHED!");
  // code to launch the chute is to be added
}

void loop() {
  sensors_event_t temp_event, pressure_event;
  
  if (dps.temperatureAvailable()) {
    /*
    dps_temp->getEvent(&temp_event);
    Serial.print(F("Temperature = "));
    Serial.print(temp_event.temperature);
    Serial.println(" *C");
    Serial.println();
    */
  }

  // Reading pressure also reads temp so don't check pressure
  // before temp!
  if (dps.pressureAvailable()) {
    /*
    dps_pressure->getEvent(&pressure_event);
    Serial.print(F("Pressure = "));
    Serial.print(pressure_event.pressure);
    Serial.println(" hPa"); 
    Serial.println();
    */

    
    Serial.print(F("Altitude (Unfiltered) = "));
    float altitude = dps.readAltitude(1013.25);
    Serial.print(altitude);
    Serial.print(F(" Altitude (Filtered) = "));
    float filtered_altitude = KALMAN(altitude);
    Serial.print(filtered_altitude);
    Serial.println(" m");
    

    /*
    // For retrieving and testing the accuracy of the data
    float altitude = dps.readAltitude(1013.25);
    Serial.print(altitude);
    Serial.print("|");
    float filtered_altitude = KALMAN(altitude);
    Serial.print(filtered_altitude);
    Serial.println();
    */

    
    if (filtered_altitude > peak_altitude) {
      peak_altitude = filtered_altitude;
    } else if ((peak_altitude - altitude) >= 0.30) {
      // 1.0 is for testing; it is actually 20.0
      launchChute();
    }

    Serial.print(F("Peak: "));
    Serial.print(peak_altitude);
    Serial.println(" m");
    Serial.println();
    

    /*
    if (iterator < 100) {
      outdata << altitude << endl;
    } else {
      outdata.close();
    }
    iterator++;
    */
  }
}
