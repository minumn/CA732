#define ARMA_DONT_USE_WRAPPER
#include <armadillo>

#include "MatrixAlgebra.h"
#include <iostream>

using namespace std;
using namespace arma;


  
void setup(){
    Matrix<3,3> matrix1;
    float data[9] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
    matrix1.fill(data);
/*
    mat A = randu<mat>(4,5);
    mat B = randu<mat>(4,5);
  
    cout << A*B.t() << endl;
    */
}

void loop(){
      
}