// Mach One code for OMRON_2SMPB02E Barometric sensor

#include "SMPB.h"

SMPB omron(SMPB_ADDR);

float sea_level_P; // in hPa
float peak_altitude; // in m


void setup() {
    Serial.begin(115200);
    Wire.begin();
    Serial.println("Omron Begin!");
    omron.init(T_STANBY_250ms);
    omron.set_average_power(TEMP_AVERAGE_1, PRESS_AVERAGE_4, NORMAL_MODE);

    sea_level_P = 1013.25;
    peak_altitude = 0.0;
    
    delay(300);
}

float get_altitude(float T, float P) {
  T += 273.15; // converting to Kelvins
  P /= 100;
  float prs_input = pow((sea_level_P / P), (1 / 5.257));

  return (prs_input - 1) * T / 0.0065;
  
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
    u8 a;

    float temp, prs;

    while (!omron.read_measure_stat()) {
        Serial.println("loading...........");
        Serial.println();
        delay(5);
    }
    
    omron.read_uncom_tempValue();
    omron.read_uncom_pressValue();

    temp = omron.get_tempValue() / 256.0;
    prs = omron.get_pressValue();
    
    Serial.print(F("Altitude (Unfiltered) = "));
    float altitude = get_altitude(temp, prs);
    Serial.print(altitude);


    // KALMAN
    /*
    Serial.print(F(" Altitude (Filtered) = "));
    float filtered_altitude = KALMAN(altitude);
    Serial.print(filtered_altitude);
    Serial.println(" m");
    

    if (filtered_altitude > peak_altitude) {
      peak_altitude = filtered_altitude;
    } else if ((peak_altitude - altitude) >= 0.30) {
      // given value is for testing; it is actually 20.0
      launchChute();
    }

    
    Serial.print(F("Peak: "));
    Serial.print(peak_altitude);
    Serial.println(" m");
    */
    
    Serial.println();
    delay(800);

}
