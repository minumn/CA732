#include "MatrixAlgebra.h"

int main(){
    Matrix<3,3> matrix1;
    float data[9] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
    matrix1.fill(data);

    return 0;
}