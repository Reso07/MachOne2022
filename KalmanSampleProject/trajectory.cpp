#include <iostream>
#include <string>
#include <iomanip>
#include <sstream>
#include <vector>
#include <fstream>
using namespace std;
#pragma once

/*
Important notes on Kalman:
A higher R (noise covariance) reduces the Kalman Gain (K) which filters more noise.
However, it slows down the speed of the filter.
* We have to find an optimum value for R.
*/

// function declerations
double KALMAN(double U);
// double get_covariance(double data[]);

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

/*
int len(double arr[]) {
    return sizeof(arr) / sizeof(arr[0]);
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
*/