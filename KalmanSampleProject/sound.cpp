#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include "trajectory.cpp"
using namespace std;

int main() {
    string A1, file;
    vector<double>U_N;
    file = "noisy.txt";

    int i = 0;
    ifstream coeff(file);
    if (coeff.is_open()) {
        while (!coeff.eof()) {
            getline(coeff, A1, '\n');
            cout << A1 << endl;
            U_N.push_back(stod(A1));
            ++i;
        }
        coeff.close();
        cout << "Number of entries: " << i << endl;
    }

    double NOISY[100];
    copy(U_N.begin(), U_N.end(), NOISY);

    double FILTERED[100];

    // cout << "Covariance of data: " << get_covariance(NOISY) << endl;

    // Kalman applying

    /*
    cout << "============================================" << endl;
    cout << "Unfiltered: " << NOISY[0] << endl;
    cout << "Filtered: " << KALMAN(NOISY[0]) << endl;
    */
    

    
    for (int c = 0; c < 100; c++) {
        FILTERED[c] = KALMAN(NOISY[c]);
        cout << FILTERED[c] << endl;
    }
    

     // system("pause");

}