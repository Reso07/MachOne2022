// Sender Code
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#include <Adafruit_DPS310.h>

#define SCK  18
#define MISO  19
#define MOSI  23

RF24 radio(5, 17);

const byte address[6] = "00001";


Adafruit_DPS310 dps;
Adafruit_Sensor *dps_temp = dps.getTemperatureSensor();
Adafruit_Sensor *dps_pressure = dps.getPressureSensor();


void setup() {
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();

  
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

void loop() {
  
  sensors_event_t temp_event, pressure_event;
  float altitude;
  // float prs
  
  if (dps.temperatureAvailable() && dps.pressureAvailable()) {
    altitude = dps.readAltitude(1013.25); // Normal Case
    // altitude = KALMAN(dps.readAltitude(1013.25)); // Kalman Case

    // For pressure
    /*
    dps_pressure->getEvent(&pressure_event);
    prs = pressure_event.pressure;
    */
  }
  
  
  
  char text[10];
  dtostrf(altitude, 5, 2, text);
  radio.write(&text, sizeof(text));
  delay(200);
}