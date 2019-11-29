#include <statusfunctions.h>
#include "MatrixAlgebra.h"


Matrix<2,3> matrix;
Matrix<3,2> matrix2;

float data[9] = {10,2,3,4,5,6,7,8,9};
float data2;

void setup(){
    Serial. begin(250000);
}

void loop(){
    matrix.fill(12.0);
    float a[2][3] = {1,1,1, 1,1,1};
    float b[3][2] = {1,1,1, 1,1,1};
 
    Serial.println("");
    matrix.print();
    delay(1000);

    Serial.println("");
    matrix.fill(a);
    matrix2.fill(b);
    auto matrix3 = matrix2*matrix;
    matrix3.print();
    delay(1000);




    
}


