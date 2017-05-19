#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include <fstream>
using namespace std;

double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main(){
    Eigen::Vector3d a;
    Eigen::Vector3d b;
    a<<1,2,3;
    b<<1,4,9;
    Eigen::Vector3d coef;
    coef = polyfit(a,b,2);
    cout<<coef<<endl;
    double res;
    res = polyeval(coef,1.5);
    cout<<res<<endl;

    string name ="lake_track_waypoints.csv";
    fstream file;
    file.open(name,ifstream::in);
    if(! file.is_open()){
        return EXIT_FAILURE;
    }

    string line;
    getline(file,line);
    file.close();
    cout<<line<<endl;

}