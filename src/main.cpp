#include <statusfunctions.h> // I believe an unuseful library, not finished. 
#include <MatrixMath.h>

#define OFF 0
#define ON 1

#define N 3

#define Kvx 40.83
#define Kvy 52.5
#define Kvz 41.25

#define Kpx 1000
#define Kpy 500
#define Kpz 200

#define l 0.55
#define L 0.55
#define b 0.3
// #define alpha 0.2618
// #define beta 0.7854
#define sinAlpha 0.2588
#define cosAlpha 0.9659
#define cosBeta 0.7071
#define sinBeta 0.7071


mtx_type J[N][N] = {0,0,0, 0,0,0, 0,0,0};
mtx_type dJ[N][N] = {0,0,0, 0,0,0, 0,0,0};
mtx_type H[N][N] = {0,0,0, 0,0,0, 0,0,0};
mtx_type P[N][N] = {0,0,0, 0,0,0, 0,0,0};
mtx_type Md[N][N] = {0,0,0, 0,0,0, 0,0,0};
mtx_type Kv[N][N] = {Kvx,0,0, 0,Kvy,0, 0,0,Kvz};
mtx_type Kp[N][N] = {Kpx,0,0, 0,Kpy,0, 0,0,Kpz};
mtx_type q[N][1] = {-1,-1,-1};
mtx_type dq[N][1] = {-1,-1,-1};


mtx_type KvJ[N][N];


void dJupdate() // Have been verified
{
    float theta  = -q[0][0];
    float zeta   = -q[1][0];
    float eta    = -q[2][0];

    float dtheta = -dq[0][0];
    float dzeta  = -dq[1][0];
    float deta   = -dq[2][0];

    dJ[0][0] = b*(sin(theta)*sinAlpha*dtheta 
	- cos(theta)*cosAlpha*cosBeta*dtheta) 
	+ l*(cos(eta)*cos(zeta)*(sin(theta)*sinAlpha*dtheta 
	- cos(theta)*cosAlpha*cosBeta*dtheta) 
	+ cos(eta)*sin(zeta)*(cos(theta)*sinAlpha*dtheta 
	+ sin(theta)*cosAlpha*cosBeta*dtheta) 
	+ sin(eta)*cos(zeta)*(cos(theta)*sinAlpha 
	+ sin(theta)*cosAlpha*cosBeta)*deta 
	+ cos(eta)*cos(zeta)*(sin(theta)*sinAlpha 
	- cos(theta)*cosAlpha*cosBeta)*dzeta 
	- sin(eta)*sin(zeta)*(sin(theta)*sinAlpha 
	- cos(theta)*cosAlpha*cosBeta)*deta 
	+ cos(eta)*sin(zeta)*(cos(theta)*sinAlpha 
	+ sin(theta)*cosAlpha*cosBeta)*dzeta);

    dJ[0][1] = l*cos(eta)*sin(zeta)*(cos(theta)*sinAlpha*dtheta 
	+ sin(theta)*cosAlpha*cosBeta*dtheta) 
	+ l*cos(eta)*cos(zeta)*(sin(theta)*sinAlpha*dtheta 
	- cos(theta)*cosAlpha*cosBeta*dtheta) 
	+ l*sin(eta)*cos(zeta)*(cos(theta)*sinAlpha 
	+ sin(theta)*cosAlpha*cosBeta)*deta 
	+ l*cos(eta)*cos(zeta)*(sin(theta)*sinAlpha 
	- cos(theta)*cosAlpha*cosBeta)*dzeta 
	- l*sin(eta)*sin(zeta)*(sin(theta)*sinAlpha 
	- cos(theta)*cosAlpha*cosBeta)*deta 
	+ l*cos(eta)*sin(zeta)*(cos(theta)*sinAlpha 
	+ sin(theta)*cosAlpha*cosBeta)*dzeta;

    dJ[0][2] = l*sin(eta)*cos(zeta)*(cos(theta)*sinAlpha*dtheta 
	+ sin(theta)*cosAlpha*cosBeta*dtheta) 
	- l*sin(eta)*sin(zeta)*(sin(theta)*sinAlpha*dtheta 
	- cos(theta)*cosAlpha*cosBeta*dtheta) 
	+ l*cos(eta)*cos(zeta)*(sin(theta)*sinAlpha 
	- cos(theta)*cosAlpha*cosBeta)*deta 
	+ l*cos(eta)*sin(zeta)*(cos(theta)*sinAlpha 
	+ sin(theta)*cosAlpha*cosBeta)*deta 
	+ l*sin(eta)*cos(zeta)*(cos(theta)*sinAlpha 
	+ sin(theta)*cosAlpha*cosBeta)*dzeta 
	- l*sin(eta)*sin(zeta)*(sin(theta)*sinAlpha 
	- cos(theta)*cosAlpha*cosBeta)*dzeta 
	+ l*sin(eta)*cosAlpha*sinBeta*deta;

    dJ[1][0] = -b*(sin(theta)*cosAlpha*dtheta 
	+ cos(theta)*cosBeta*sinAlpha*dtheta) 
	- l*(cos(eta)*cos(zeta)*(sin(theta)*cosAlpha*dtheta 
	+ cos(theta)*cosBeta*sinAlpha*dtheta) 
	+ cos(eta)*sin(zeta)*(cos(theta)*cosAlpha*dtheta 
	- sin(theta)*cosBeta*sinAlpha*dtheta) 
	+ sin(eta)*cos(zeta)*(cos(theta)*cosAlpha 
	- sin(theta)*cosBeta*sinAlpha)*deta 
	+ cos(eta)*cos(zeta)*(sin(theta)*cosAlpha 
	+ cos(theta)*cosBeta*sinAlpha)*dzeta 
	- sin(eta)*sin(zeta)*(sin(theta)*cosAlpha 
	+ cos(theta)*cosBeta*sinAlpha)*deta 
	+ cos(eta)*sin(zeta)*(cos(theta)*cosAlpha 
	- sin(theta)*cosBeta*sinAlpha)*dzeta);

    dJ[1][1] = l*sin(eta)*sin(zeta)*(sin(theta)*cosAlpha 
	+ cos(theta)*cosBeta*sinAlpha)*deta 
	- l*cos(eta)*cos(zeta)*(sin(theta)*cosAlpha*dtheta 
	+ cos(theta)*cosBeta*sinAlpha*dtheta) 
	- l*sin(eta)*cos(zeta)*(cos(theta)*cosAlpha 
	- sin(theta)*cosBeta*sinAlpha)*deta 
	- l*cos(eta)*cos(zeta)*(sin(theta)*cosAlpha 
	+ cos(theta)*cosBeta*sinAlpha)*dzeta 
	- l*cos(eta)*sin(zeta)*(cos(theta)*cosAlpha*dtheta 
	- sin(theta)*cosBeta*sinAlpha*dtheta) 
	- l*cos(eta)*sin(zeta)*(cos(theta)*cosAlpha 
	- sin(theta)*cosBeta*sinAlpha)*dzeta;

    dJ[1][2] = l*sin(eta)*sin(zeta)*(sin(theta)*cosAlpha*dtheta 
	+ cos(theta)*cosBeta*sinAlpha*dtheta) 
	- l*sin(eta)*cos(zeta)*(cos(theta)*cosAlpha*dtheta 
	- sin(theta)*cosBeta*sinAlpha*dtheta) 
	- l*cos(eta)*cos(zeta)*(sin(theta)*cosAlpha 
	+ cos(theta)*cosBeta*sinAlpha)*deta 
	- l*cos(eta)*sin(zeta)*(cos(theta)*cosAlpha 
	- sin(theta)*cosBeta*sinAlpha)*deta 
	- l*sin(eta)*cos(zeta)*(cos(theta)*cosAlpha 
	- sin(theta)*cosBeta*sinAlpha)*dzeta 
	+ l*sin(eta)*sin(zeta)*(sin(theta)*cosAlpha 
	+ cos(theta)*cosBeta*sinAlpha)*dzeta 
	+ l*sin(eta)*sinAlpha*sinBeta*deta;

    dJ[2][0] = -sinBeta*(l*cos(theta)*sin(eta)*sin(zeta)*deta 
	- l*cos(eta)*cos(theta)*cos(zeta)*dtheta 
	- l*cos(eta)*cos(theta)*cos(zeta)*dzeta 
	- b*cos(theta)*dtheta + l*sin(eta)*cos(zeta)*sin(theta)*deta 
	+ l*cos(eta)*sin(theta)*sin(zeta)*dtheta + l*cos(eta)*sin(theta)*sin(zeta)*dzeta);

    dJ[2][1] = l*cos(eta)*cos(theta 
	+ zeta)*sinBeta*(dtheta + dzeta) 
	- l*sin(eta)*sin(theta + zeta)*sinBeta*deta;

    dJ[2][2] = l*sin(eta)*cosBeta*deta 
	+ l*cos(eta)*cos(theta)*cos(zeta)*sinBeta*deta 
	- l*cos(eta)*sin(theta)*sin(zeta)*sinBeta*deta 
	- l*cos(theta)*sin(eta)*sin(zeta)*sinBeta*dtheta 
	- l*sin(eta)*cos(zeta)*sin(theta)*sinBeta*dtheta 
	- l*cos(theta)*sin(eta)*sin(zeta)*sinBeta*dzeta 
	- l*sin(eta)*cos(zeta)*sin(theta)*sinBeta*dzeta;
}

void updateJacobian() // Have been verified
{
	double theta = -q[0][0];
	double zeta = -q[1][0];
	double eta = -q[2][0];

	J[0][0] = l*cos(eta)*sinAlpha*sin(theta)*sin(zeta) 
	- b*cosAlpha*cosBeta*sin(theta) 
	- b*sinAlpha*cos(theta)
	- l*cos(eta)*sinAlpha*cos(theta)*cos(zeta) 
	- l*cosAlpha*cosBeta*cos(eta)*cos(theta)*sin(zeta) 
	- l*cosAlpha*cosBeta*cos(eta)*cos(zeta)*sin(theta);

	J[1][0] = l*(cos((eta))*cos((zeta))*(cos((theta))*cosAlpha 
	- sin((theta))*cosBeta*sinAlpha) 
	- cos((eta))*sin((zeta))*(sin((theta))*cosAlpha 
	+ cos((theta))*cosBeta*sinAlpha)) 
	+ b*(cos((theta))*cosAlpha 
	- sin((theta))*cosBeta*sinAlpha);

	J[2][0] = sinBeta*(b*sin((theta)) 
	+ l*cos((eta))*cos((theta))*sin((zeta)) 
	+ l*cos((eta))*cos((zeta))*sin((theta)));

	J[0][1] = l*cos((eta))*sin((zeta))*(sin((theta))*sinAlpha 
	- cos((theta))*cosAlpha*cosBeta) 
	- l*cos((eta))*cos((zeta))*(cos((theta))*sinAlpha 
	+ sin((theta))*cosAlpha*cosBeta);

	J[1][1] = l*cos((eta))*cos((zeta))*(cos((theta))*cosAlpha 
	- sin((theta))*cosBeta*sinAlpha) 
	- l*cos((eta))*sin((zeta))*(sin((theta))*cosAlpha 
	+ cos((theta))*cosBeta*sinAlpha); 

 	J[2][1] = l*cos((eta))*sin((theta) 
	 + (zeta))*sinBeta;

	J[0][2] = l*sin((eta))*cos((zeta))*(sin((theta))*sinAlpha 
	- cos((theta))*cosAlpha*cosBeta) 
	+ l*sin((eta))*sin((zeta))*(cos((theta))*sinAlpha 
	+ sin((theta))*cosAlpha*cosBeta) 
	- l*cos((eta))*cosAlpha*sinBeta;
	
	J[1][2] = - l*sin((eta))*cos((zeta))*(sin((theta))*cosAlpha 
	+ cos((theta))*cosBeta*sinAlpha) 
	- l*sin((eta))*sin((zeta))*(cos((theta))*cosAlpha 
	- sin((theta))*cosBeta*sinAlpha) 
	- l*cos((eta))*sinAlpha*sinBeta;
	
	J[2][2] = l*cos((theta))*sin((eta))*cos((zeta))*sinBeta 
	- l*cos((eta))*cosBeta 
	- l*sin((eta))*sin((theta))*sin((zeta))*sinBeta;
}


void LED(bool on){
	digitalWrite(13,on);
}

void setup()
{
    Serial.begin(250000);
    pinMode(13,OUTPUT);
	LED(OFF);
}


void loop()
{ 
    
    Matrix.Multiply(*Kv, *J, N, N, N, *KvJ);
	
	dJupdate();
	Matrix.Print(*dJ, N, N, "J");
	//Matrix.Invert(*A, N);
    //Matrix.Multiply(*A, *B, N, N, N, *C);
    //Matrix.Add(*A, *B, N, N, *C);

	delay(2000);
}

