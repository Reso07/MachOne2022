#include <Adafruit_DPS310.h>

Adafruit_DPS310 dps;
Adafruit_Sensor *dps_temp = dps.getTemperatureSensor();
Adafruit_Sensor *dps_pressure = dps.getPressureSensor();

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("DPS310");
  if (! dps.begin_I2C()) {
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

double get_covariance(double data[]) {
    int length = len(data);
    double x_mean = length * (length + 1) / 2.0;
    double y_mean = 0.0;
    double co_mean = 0.0;
    for (int i = 0; i < length; ++i) {
        y_mean += data[i];
        co_mean += (data[i] * (i + 1));
    }
    y_mean /= length;
    co_mean /= length;

    return co_mean - y_mean * x_mean;

}

double KALMAN(double U) {
    static const double R = -2470728.387568969; // noise covariance
    static const double H = 1.0; // measurement map scalar
    static double Q = -2000000.0; // initial estimated covariance
    static double P = 0; // inital error covariance (must be 0)
    static double U_hat = 0; // inital estimated state (assume we don't know)
    static double K = 0; // inital Kalman gain

    // begin
    K = P * H / (H * P * H + R);
    U_hat = U_hat + K * (U - H * U_hat);

    // update error covariance
    P = (1 - K * H) * P + Q;

    return U_hat;
}

void loop() {
  sensors_event_t temp_event, pressure_event;
  
  if (dps.temperatureAvailable()) {
    dps_temp->getEvent(&temp_event);
    Serial.print(F("Temperature = "));
    Serial.print(temp_event.temperature);
    Serial.println(" *C");
    Serial.println();
  }

  // Reading pressure also reads temp so don't check pressure
  // before temp!
  if (dps.pressureAvailable()) {
    dps_pressure->getEvent(&pressure_event);
    Serial.print(F("Pressure = "));
    Serial.print(pressure_event.pressure);
    Serial.println(" hPa"); 
  
    Serial.println();
  }
}
