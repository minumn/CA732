/*
#include "MatrixAlgebra.h"

int main(){
    Matrix<3,3> matrix1;
    float data[9] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
    matrix1.fill(data);

    return 0;
}
*/


//#include <FlexCAN.h> // Pawelsky flex can library, used with Jens 
//#include <SMC66Registers.h> // is a dictionary with some of the registers of the motor 
//#include <Messenger.h>  // is a library for communication with arduino, not mine, but someone else 
//#include <CANsmc.h> // My library for communication with the motors 
//#include <math.h> // this is arduino standard math library
//#include <Chrono.h> // a timing library, useful for timing control loops 
#include <statusfunctions.h> // I believe an unuseful library, not finished. 
#include <MatrixMath.h>

#define N  (16)

mtx_type A[N][N];
mtx_type B[N][N];
mtx_type C[N][N];
mtx_type v[N];      // This is a row vector
mtx_type w[N];
mtx_type S[N][N];

mtx_type maxVal = 10;  // maxValimum random matrix entry range


void setup()
{
    Serial.begin(250000);
    pinMode(13,OUTPUT);

    // Initialize matrices
	for (int i = 0; i < N; i++)
	{
		v[i] = i + 1;                  // vector of sequential numbers
		for (int j = 0; j < N; j++)
		{
			A[i][j] = random(maxVal) - maxVal / 2.0f; // A is random
			if (i == j)
			{
				B[i][j] = 1.0f;                  // B is identity
			}
			else
			{
				B[i][j] = 0.0f;
			}
		}
	}

    S[0][0] = 0;
    S[0][1] = 0;
    S[1][0] = 0;
    S[1][1] = 1;

}


 void loop()
 { 

    delay(2000);
	Matrix.Invert(*A, N);
	Serial.println("\nInverted A:");
	Matrix.Print(*A, N, N, "A");
    //digitalWrite(13,1);
    

    while(1);
}

