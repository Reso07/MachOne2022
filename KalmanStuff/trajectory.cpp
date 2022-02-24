#include <iostream>
#include <string>
#include <iomanip>
#include <sstream>
#include <vector>
#include <fstream>
using namespace std;
#pragma once

double KALMAN(double U) {
    static const double R = 40; // noise covariance
    static const double H = 1.00; // measurement map scalar
    static double Q = 10; // initial estimated covariance
    static double P = 0; // inital error covariance (nust be 0)
    static double U_hat = 0; // inital estimated state (assume we don't know)
    static double K = 0; // inital Kalman gain

    // begin
    K = P * H / (H * P * H + R);
    U_hat = U_hat + K * (U - H * U_hat);

    // update error covariance
    P = (1 - K * H) * P;

    return U_hat;
}