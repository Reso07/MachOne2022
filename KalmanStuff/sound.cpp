# include <iostream>
# include <string>
# include <fstream>
# include <vector>
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

     double NOISY[i];
     copy(U_N.begin(), U_N.end(), NOISY);

     system("pause");

}