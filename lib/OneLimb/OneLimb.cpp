
#include "OneLimb.h"

OneLimb::OneLimb() :
    J{0,0,0, 0,0,0, 0,0,0},
    dJ{0,0,0, 0,0,0, 0,0,0},
    Jt{0,0,0, 0,0,0, 0,0,0},
    H{0,0,0},
    C{0,0,0},
    G{0,0,0},
    invP{0,0,0, 0,0,0, 0,0,0},
    Md{mp,0,0, 0,mp,0, 0,0,mp},
    Kv{Kvx,0,0, 0,Kvy,0, 0,0,Kvz},
    Kp{Kpx,0,0, 0,Kpy,0, 0,0,Kpz},
    q{-0.9528, 1.7262, 0.881}, // Realistic start values.
    qz1{-0.9528, 1.7262, 0.881},
    dq{0,0,0},
    x{0,0,-0.6},
    xr{0,0,-0.4},
    ex{0,0,0.2},
    res{0,0,0}
{
	
}
OneLimb::~OneLimb(){}

void OneLimb::writeToFile(const char* fileName){
	File dataFile = SD.open(fileName, FILE_WRITE);

	// if the file is available, write to it:
    if (dataFile) {
		// "theta, zeta, eta, dtheta, dzeta, deta, z, tau, ez"
		String dataString = "";
		dataString += theta; 
		dataString += ", ";
		dataString += zeta; 
		dataString += ", ";
		dataString += eta; 
		dataString += ", ";
		dataString += dtheta; 
		dataString += ", ";
		dataString += dzeta; 
		dataString += ", ";
		dataString += deta; 
		dataString += ", ";
		dataString += z;  
		dataString += ", ";
		dataString += xr[2][0]; 
		dataString += ", ";
		dataString += res[0][0]; 
		dataString += ", ";
		dataString += ex[2][0];

        dataFile.println(dataString);
        dataFile.close();
        
        // print to the serial port too:
        Serial.println(dataString);
    }
    // if the file isn't open, pop up an error:
    else {
        Serial.print("error opening file '");
		Serial.print(fileName);
		Serial.println("'.");
		delay(2000);
    }
}

void OneLimb::setZref(double Z){
	// Only values between -0.4 and -1.0 is allowed.
	if (Z > ZUPPERLIMIT) {
		Z = ZUPPERLIMIT;
	} else if (Z < ZLOWERLIMIT) {
		Z = ZLOWERLIMIT;
	}
	
	xr[2][0] = Z;

	// Update new error
	exUpdate();
	// Calculate new Torque.
	TorqueUpdate();
}

double OneLimb::motorPosToRad(double MotorPos){
	return MotorPos; //*0.0006 - 0.7114;
}

double OneLimb::motorPosToDeg(double MotorPos){
	return MotorPos*0.0357 - 40.76;
}

void OneLimb::setMotorPositionOffset(double MotorPosOffset){
	_MotorPosOffset = MotorPosOffset;
}

void OneLimb::newData(double AbsMotorPosition){
	qUpdate(AbsMotorPosition);

	// New jacobians as consequence of q, dq
    JUpdate();
	dJUpdate();
	JtUpdate();
	
	HUpdate();

	// New error x as consequence of new z
	exUpdate();
	
	invPUpdate();
	TorqueUpdate();
}

double OneLimb::getTorque(double AbsMotorPosition){
	newData(AbsMotorPosition);

	return res[0][0];
}

double OneLimb::getTorque(){
	return res[0][0];
}

void OneLimb::TorqueUpdate(){
	
	Matrix.Multiply(*Kv, *J, N, N, N, *KvJ);

	Matrix.Multiply(*Md, *dJ, N, N, N, *MddJ);

	Matrix.Add(*MddJ, *KvJ, N, N, *MddJ_p_KvJ);

	Matrix.Multiply(*MddJ_p_KvJ, *dq, N, N, 1, *MddJ_p_KvJ_t_dq);

	Matrix.Multiply(*Kp, *ex, N, N, 1, *Kpex);

	Matrix.Subtract(*MddJ_p_KvJ_t_dq, *Kpex, N, 1, *MddJ_p_KvJ_t_dq_m_Kpex);

	Matrix.Multiply(*Jt, *MddJ_p_KvJ_t_dq_m_Kpex, N, N, 1, *Jt_t_MddJ_p_KvJ_t_dq_m_Kpex);
	
	Matrix.Subtract(*H, *MddJ_p_KvJ_t_dq_m_Kpex, N, 1, *H_m_Jt_t_MddJ_p_KvJ_t_dq_m_Kpex);

	Matrix.Multiply(*invP, *H_m_Jt_t_MddJ_p_KvJ_t_dq_m_Kpex, N, N, 1, *res);

	// Matrix.Print(*res, 3, 1, "res");
	// Matrix.Print(*dq, 3, 1, "dq");
	// Matrix.Print(*G, 3, 1, "G");
	// Matrix.Print(*J, 3, 3, "J");

	// Matrix.Print(*invP, N, N, "invP");
	// Matrix.Print(*H_m_Jt_t_MddJ_p_KvJ_t_dq_m_Kpex, N, 1, "H_m_Jt_t_MddJ_p_KvJ_t_dq_m_Kpex");

	// Matrix.Print(*xr,3,1,"xr");
	// Matrix.Print(*x,3,1,"x");

	// Serial.println(N);

	// delay(2000);

	if(res[0][0] > 10){
		res[0][0] = 10;
	} else if (res[0][0] < -10) {
		res[0][0] = -10;
	}
}

void OneLimb::invPUpdate() /* Verified */ {
	invP[0][0] = 1;

	invP[0][1] = (l*cos(eta)*cos(eta)*sinBeta*sin(theta)*sin(zeta))/(l*cos(eta)*cos(eta)*sinBeta*cos(theta)*cos(zeta) 
		- l*cos(eta)*cos(eta)*sinBeta*sin(theta)*sin(zeta) 
		+ l*cosBeta*cos(eta)*cos(theta)*cos(theta)*sin(eta)*cos(zeta)*cos(zeta) 
		+ l*cosBeta*cos(eta)*sin(eta)*cos(zeta)*cos(zeta)*sin(theta)*sin(theta)
		+ l*cosBeta*cos(eta)*cos(theta)*cos(theta)*sin(eta)*sin(zeta)*sin(zeta)
		+ l*cosBeta*cos(eta)*sin(eta)*sin(theta)*sin(theta)*sin(zeta)*sin(zeta)) 
		- (l*cos(eta)*cos(eta)*sinBeta*cos(theta)*cos(zeta))/(l*cos(eta)*cos(eta)*sinBeta*cos(theta)*cos(zeta) 
		- l*cos(eta)*cos(eta)*sinBeta*sin(theta)*sin(zeta) 
		+ l*cosBeta*cos(eta)*cos(theta)*cos(theta)*sin(eta)*cos(zeta)*cos(zeta) 
		+ l*cosBeta*cos(eta)*sin(eta)*cos(zeta)*cos(zeta)*sin(theta)*sin(theta)
		+ l*cosBeta*cos(eta)*cos(theta)*cos(theta)*sin(eta)*sin(zeta)*sin(zeta)
		+ l*cosBeta*cos(eta)*sin(eta)*sin(theta)*sin(theta)*sin(zeta)*sin(zeta)) 
		- (b1*cosBeta*cos(theta)*cos(theta)*sin(eta)*cos(zeta))/(l*cos(eta)*cos(eta)*sinBeta*cos(theta)*cos(zeta) 
		- l*cos(eta)*cos(eta)*sinBeta*sin(theta)*sin(zeta) 
		+ l*cosBeta*cos(eta)*cos(theta)*cos(theta)*sin(eta)*cos(zeta)*cos(zeta) 
		+ l*cosBeta*cos(eta)*sin(eta)*cos(zeta)*cos(zeta)*sin(theta)*sin(theta)
		+ l*cosBeta*cos(eta)*cos(theta)*cos(theta)*sin(eta)*sin(zeta)*sin(zeta)
		+ l*cosBeta*cos(eta)*sin(eta)*sin(theta)*sin(theta)*sin(zeta)*sin(zeta)) 
		- (b1*cos(eta)*sinBeta*cos(theta))/(l*cos(eta)*cos(eta)*sinBeta*cos(theta)*cos(zeta) 
		- l*cos(eta)*cos(eta)*sinBeta*sin(theta)*sin(zeta) 
		+ l*cosBeta*cos(eta)*cos(theta)*cos(theta)*sin(eta)*cos(zeta)*cos(zeta) 
		+ l*cosBeta*cos(eta)*sin(eta)*cos(zeta)*cos(zeta)*sin(theta)*sin(theta)
		+ l*cosBeta*cos(eta)*cos(theta)*cos(theta)*sin(eta)*sin(zeta)*sin(zeta)
		+ l*cosBeta*cos(eta)*sin(eta)*sin(theta)*sin(theta)*sin(zeta)*sin(zeta)) 
		- (b1*cosBeta*sin(eta)*cos(zeta)*sin(theta)*sin(theta))/(l*cos(eta)*cos(eta)*sinBeta*cos(theta)*cos(zeta) 
		- l*cos(eta)*cos(eta)*sinBeta*sin(theta)*sin(zeta) 
		+ l*cosBeta*cos(eta)*cos(theta)*cos(theta)*sin(eta)*cos(zeta)*cos(zeta) 
		+ l*cosBeta*cos(eta)*sin(eta)*cos(zeta)*cos(zeta)*sin(theta)*sin(theta)
		+ l*cosBeta*cos(eta)*cos(theta)*cos(theta)*sin(eta)*sin(zeta)*sin(zeta)
		+ l*cosBeta*cos(eta)*sin(eta)*sin(theta)*sin(theta)*sin(zeta)*sin(zeta)) 
		- (l*cosBeta*cos(eta)*cos(theta)*cos(theta)*sin(eta)*cos(zeta)*cos(zeta))/(l*cos(eta)*cos(eta)*sinBeta*cos(theta)*cos(zeta) 
		- l*cos(eta)*cos(eta)*sinBeta*sin(theta)*sin(zeta) 
		+ l*cosBeta*cos(eta)*cos(theta)*cos(theta)*sin(eta)*cos(zeta)*cos(zeta) 
		+ l*cosBeta*cos(eta)*sin(eta)*cos(zeta)*cos(zeta)*sin(theta)*sin(theta)
		+ l*cosBeta*cos(eta)*cos(theta)*cos(theta)*sin(eta)*sin(zeta)*sin(zeta)
		+ l*cosBeta*cos(eta)*sin(eta)*sin(theta)*sin(theta)*sin(zeta)*sin(zeta)) 
		- (l*cosBeta*cos(eta)*sin(eta)*cos(zeta)*cos(zeta)*sin(theta)*sin(theta))/(l*cos(eta)*cos(eta)*sinBeta*cos(theta)*cos(zeta) 
		- l*cos(eta)*cos(eta)*sinBeta*sin(theta)*sin(zeta) 
		+ l*cosBeta*cos(eta)*cos(theta)*cos(theta)*sin(eta)*cos(zeta)*cos(zeta) 
		+ l*cosBeta*cos(eta)*sin(eta)*cos(zeta)*cos(zeta)*sin(theta)*sin(theta)
		+ l*cosBeta*cos(eta)*cos(theta)*cos(theta)*sin(eta)*sin(zeta)*sin(zeta)
		+ l*cosBeta*cos(eta)*sin(eta)*sin(theta)*sin(theta)*sin(zeta)*sin(zeta)) 
		- (l*cosBeta*cos(eta)*cos(theta)*cos(theta)*sin(eta)*sin(zeta)*sin(zeta))/(l*cos(eta)*cos(eta)*sinBeta*cos(theta)*cos(zeta) 
		- l*cos(eta)*cos(eta)*sinBeta*sin(theta)*sin(zeta) 
		+ l*cosBeta*cos(eta)*cos(theta)*cos(theta)*sin(eta)*cos(zeta)*cos(zeta) 
		+ l*cosBeta*cos(eta)*sin(eta)*cos(zeta)*cos(zeta)*sin(theta)*sin(theta)
		+ l*cosBeta*cos(eta)*cos(theta)*cos(theta)*sin(eta)*sin(zeta)*sin(zeta)
		+ l*cosBeta*cos(eta)*sin(eta)*sin(theta)*sin(theta)*sin(zeta)*sin(zeta)) 
		- (l*cosBeta*cos(eta)*sin(eta)*sin(theta)*sin(theta)*sin(zeta)*sin(zeta))/(l*cos(eta)*cos(eta)*sinBeta*cos(theta)*cos(zeta) 
		- l*cos(eta)*cos(eta)*sinBeta*sin(theta)*sin(zeta) 
		+ l*cosBeta*cos(eta)*cos(theta)*cos(theta)*sin(eta)*cos(zeta)*cos(zeta) 
		+ l*cosBeta*cos(eta)*sin(eta)*cos(zeta)*cos(zeta)*sin(theta)*sin(theta)
		+ l*cosBeta*cos(eta)*cos(theta)*cos(theta)*sin(eta)*sin(zeta)*sin(zeta)
		+ l*cosBeta*cos(eta)*sin(eta)*sin(theta)*sin(theta)*sin(zeta)*sin(zeta)),
		
	invP[0][2] = (b1*cosBeta*cos(theta)*cos(theta)*sin(zeta))/(l*cosBeta*cos(theta)*cos(theta)*sin(eta)*cos(zeta)*cos(zeta) 
		- l*cos(eta)*sinBeta*sin(theta)*sin(zeta) 
		+ l*cosBeta*sin(eta)*cos(zeta)*cos(zeta)*sin(theta)*sin(theta)
		+ l*cosBeta*cos(theta)*cos(theta)*sin(eta)*sin(zeta)*sin(zeta)
		+ l*cosBeta*sin(eta)*sin(theta)*sin(theta)*sin(zeta)*sin(zeta)
		+ l*cos(eta)*sinBeta*cos(theta)*cos(zeta)) 
		+ (b1*cosBeta*sin(theta)*sin(theta)*sin(zeta))/(l*cosBeta*cos(theta)*cos(theta)*sin(eta)*cos(zeta)*cos(zeta) 
		- l*cos(eta)*sinBeta*sin(theta)*sin(zeta) 
		+ l*cosBeta*sin(eta)*cos(zeta)*cos(zeta)*sin(theta)*sin(theta)
		+ l*cosBeta*cos(theta)*cos(theta)*sin(eta)*sin(zeta)*sin(zeta)
		+ l*cosBeta*sin(eta)*sin(theta)*sin(theta)*sin(zeta)*sin(zeta)
		+ l*cos(eta)*sinBeta*cos(theta)*cos(zeta));
			
	invP[1][0] = 0;

	invP[1][1] = (cos(eta)*sinAlpha*sinBeta)/(l*cosAlpha*cosAlpha*cos(eta)*cos(eta)*sinBeta*cos(theta)*cos(zeta) 
		+ l*cos(eta)*cos(eta)*sinAlpha*sinAlpha*sinBeta*cos(theta)*cos(zeta) 
		- l*cosAlpha*cosAlpha*cos(eta)*cos(eta)*sinBeta*sin(theta)*sin(zeta) 
		- l*cos(eta)*cos(eta)*sinAlpha*sinAlpha*sinBeta*sin(theta)*sin(zeta) 
		+ l*cosAlpha*cosAlpha*cosBeta*cos(eta)*cos(theta)*cos(theta)*sin(eta)*cos(zeta)*cos(zeta) 
		+ l*cosBeta*cos(eta)*sinAlpha*sinAlpha*cos(theta)*cos(theta)*sin(eta)*cos(zeta)*cos(zeta) 
		+ l*cosAlpha*cosAlpha*cosBeta*cos(eta)*sin(eta)*cos(zeta)*cos(zeta)*sin(theta)*sin(theta)
		+ l*cosAlpha*cosAlpha*cosBeta*cos(eta)*cos(theta)*cos(theta)*sin(eta)*sin(zeta)*sin(zeta)
		+ l*cosBeta*cos(eta)*sinAlpha*sinAlpha*sin(eta)*cos(zeta)*cos(zeta)*sin(theta)*sin(theta)
		+ l*cosBeta*cos(eta)*sinAlpha*sinAlpha*cos(theta)*cos(theta)*sin(eta)*sin(zeta)*sin(zeta)
		+ l*cosAlpha*cosAlpha*cosBeta*cos(eta)*sin(eta)*sin(theta)*sin(theta)*sin(zeta)*sin(zeta)
		+ l*cosBeta*cos(eta)*sinAlpha*sinAlpha*sin(eta)*sin(theta)*sin(theta)*sin(zeta)*sin(zeta)) 
		+ (cosAlpha*cos(theta)*sin(eta)*sin(zeta))/(l*cosAlpha*cosAlpha*cos(eta)*cos(eta)*sinBeta*cos(theta)*cos(zeta) 
		+ l*cos(eta)*cos(eta)*sinAlpha*sinAlpha*sinBeta*cos(theta)*cos(zeta) 
		- l*cosAlpha*cosAlpha*cos(eta)*cos(eta)*sinBeta*sin(theta)*sin(zeta) 
		- l*cos(eta)*cos(eta)*sinAlpha*sinAlpha*sinBeta*sin(theta)*sin(zeta) 
		+ l*cosAlpha*cosAlpha*cosBeta*cos(eta)*cos(theta)*cos(theta)*sin(eta)*cos(zeta)*cos(zeta) 
		+ l*cosBeta*cos(eta)*sinAlpha*sinAlpha*cos(theta)*cos(theta)*sin(eta)*cos(zeta)*cos(zeta) 
		+ l*cosAlpha*cosAlpha*cosBeta*cos(eta)*sin(eta)*cos(zeta)*cos(zeta)*sin(theta)*sin(theta)
		+ l*cosAlpha*cosAlpha*cosBeta*cos(eta)*cos(theta)*cos(theta)*sin(eta)*sin(zeta)*sin(zeta)
		+ l*cosBeta*cos(eta)*sinAlpha*sinAlpha*sin(eta)*cos(zeta)*cos(zeta)*sin(theta)*sin(theta)
		+ l*cosBeta*cos(eta)*sinAlpha*sinAlpha*cos(theta)*cos(theta)*sin(eta)*sin(zeta)*sin(zeta)
		+ l*cosAlpha*cosAlpha*cosBeta*cos(eta)*sin(eta)*sin(theta)*sin(theta)*sin(zeta)*sin(zeta)
		+ l*cosBeta*cos(eta)*sinAlpha*sinAlpha*sin(eta)*sin(theta)*sin(theta)*sin(zeta)*sin(zeta)) 
		+ (cosAlpha*sin(eta)*cos(zeta)*sin(theta))/(l*cosAlpha*cosAlpha*cos(eta)*cos(eta)*sinBeta*cos(theta)*cos(zeta) 
		+ l*cos(eta)*cos(eta)*sinAlpha*sinAlpha*sinBeta*cos(theta)*cos(zeta) 
		- l*cosAlpha*cosAlpha*cos(eta)*cos(eta)*sinBeta*sin(theta)*sin(zeta) 
		- l*cos(eta)*cos(eta)*sinAlpha*sinAlpha*sinBeta*sin(theta)*sin(zeta) 
		+ l*cosAlpha*cosAlpha*cosBeta*cos(eta)*cos(theta)*cos(theta)*sin(eta)*cos(zeta)*cos(zeta) 
		+ l*cosBeta*cos(eta)*sinAlpha*sinAlpha*cos(theta)*cos(theta)*sin(eta)*cos(zeta)*cos(zeta) 
		+ l*cosAlpha*cosAlpha*cosBeta*cos(eta)*sin(eta)*cos(zeta)*cos(zeta)*sin(theta)*sin(theta)
		+ l*cosAlpha*cosAlpha*cosBeta*cos(eta)*cos(theta)*cos(theta)*sin(eta)*sin(zeta)*sin(zeta)
		+ l*cosBeta*cos(eta)*sinAlpha*sinAlpha*sin(eta)*cos(zeta)*cos(zeta)*sin(theta)*sin(theta)
		+ l*cosBeta*cos(eta)*sinAlpha*sinAlpha*cos(theta)*cos(theta)*sin(eta)*sin(zeta)*sin(zeta)
		+ l*cosAlpha*cosAlpha*cosBeta*cos(eta)*sin(eta)*sin(theta)*sin(theta)*sin(zeta)*sin(zeta)
		+ l*cosBeta*cos(eta)*sinAlpha*sinAlpha*sin(eta)*sin(theta)*sin(theta)*sin(zeta)*sin(zeta)) 
		+ (cosBeta*sinAlpha*cos(theta)*sin(eta)*cos(zeta))/(l*cosAlpha*cosAlpha*cos(eta)*cos(eta)*sinBeta*cos(theta)*cos(zeta) 
		+ l*cos(eta)*cos(eta)*sinAlpha*sinAlpha*sinBeta*cos(theta)*cos(zeta) 
		- l*cosAlpha*cosAlpha*cos(eta)*cos(eta)*sinBeta*sin(theta)*sin(zeta) 
		- l*cos(eta)*cos(eta)*sinAlpha*sinAlpha*sinBeta*sin(theta)*sin(zeta) 
		+ l*cosAlpha*cosAlpha*cosBeta*cos(eta)*cos(theta)*cos(theta)*sin(eta)*cos(zeta)*cos(zeta) 
		+ l*cosBeta*cos(eta)*sinAlpha*sinAlpha*cos(theta)*cos(theta)*sin(eta)*cos(zeta)*cos(zeta) 
		+ l*cosAlpha*cosAlpha*cosBeta*cos(eta)*sin(eta)*cos(zeta)*cos(zeta)*sin(theta)*sin(theta)
		+ l*cosAlpha*cosAlpha*cosBeta*cos(eta)*cos(theta)*cos(theta)*sin(eta)*sin(zeta)*sin(zeta)
		+ l*cosBeta*cos(eta)*sinAlpha*sinAlpha*sin(eta)*cos(zeta)*cos(zeta)*sin(theta)*sin(theta)
		+ l*cosBeta*cos(eta)*sinAlpha*sinAlpha*cos(theta)*cos(theta)*sin(eta)*sin(zeta)*sin(zeta)
		+ l*cosAlpha*cosAlpha*cosBeta*cos(eta)*sin(eta)*sin(theta)*sin(theta)*sin(zeta)*sin(zeta)
		+ l*cosBeta*cos(eta)*sinAlpha*sinAlpha*sin(eta)*sin(theta)*sin(theta)*sin(zeta)*sin(zeta)) 
		- (cosBeta*sinAlpha*sin(eta)*sin(theta)*sin(zeta))/(l*cosAlpha*cosAlpha*cos(eta)*cos(eta)*sinBeta*cos(theta)*cos(zeta) 
		+ l*cos(eta)*cos(eta)*sinAlpha*sinAlpha*sinBeta*cos(theta)*cos(zeta) 
		- l*cosAlpha*cosAlpha*cos(eta)*cos(eta)*sinBeta*sin(theta)*sin(zeta) 
		- l*cos(eta)*cos(eta)*sinAlpha*sinAlpha*sinBeta*sin(theta)*sin(zeta) 
		+ l*cosAlpha*cosAlpha*cosBeta*cos(eta)*cos(theta)*cos(theta)*sin(eta)*cos(zeta)*cos(zeta) 
		+ l*cosBeta*cos(eta)*sinAlpha*sinAlpha*cos(theta)*cos(theta)*sin(eta)*cos(zeta)*cos(zeta) 
		+ l*cosAlpha*cosAlpha*cosBeta*cos(eta)*sin(eta)*cos(zeta)*cos(zeta)*sin(theta)*sin(theta)
		+ l*cosAlpha*cosAlpha*cosBeta*cos(eta)*cos(theta)*cos(theta)*sin(eta)*sin(zeta)*sin(zeta)
		+ l*cosBeta*cos(eta)*sinAlpha*sinAlpha*sin(eta)*cos(zeta)*cos(zeta)*sin(theta)*sin(theta)
		+ l*cosBeta*cos(eta)*sinAlpha*sinAlpha*cos(theta)*cos(theta)*sin(eta)*sin(zeta)*sin(zeta)
		+ l*cosAlpha*cosAlpha*cosBeta*cos(eta)*sin(eta)*sin(theta)*sin(theta)*sin(zeta)*sin(zeta)
		+ l*cosBeta*cos(eta)*sinAlpha*sinAlpha*sin(eta)*sin(theta)*sin(theta)*sin(zeta)*sin(zeta));
		
	invP[1][2] = (cosAlpha*cos(theta)*cos(zeta))/(l*cosBeta*sinAlpha*sinAlpha*sin(eta)*cos(zeta)*cos(zeta)*sin(theta)*sin(theta)
		+ l*cosBeta*sinAlpha*sinAlpha*cos(theta)*cos(theta)*sin(eta)*sin(zeta)*sin(zeta)
		+ l*cosAlpha*cosAlpha*cosBeta*sin(eta)*sin(theta)*sin(theta)*sin(zeta)*sin(zeta)
		+ l*cosBeta*sinAlpha*sinAlpha*sin(eta)*sin(theta)*sin(theta)*sin(zeta)*sin(zeta)
		+ l*cosAlpha*cosAlpha*cos(eta)*sinBeta*cos(theta)*cos(zeta) 
		+ l*cos(eta)*sinAlpha*sinAlpha*sinBeta*cos(theta)*cos(zeta) 
		- l*cosAlpha*cosAlpha*cos(eta)*sinBeta*sin(theta)*sin(zeta) 
		- l*cos(eta)*sinAlpha*sinAlpha*sinBeta*sin(theta)*sin(zeta) 
		+ l*cosAlpha*cosAlpha*cosBeta*cos(theta)*cos(theta)*sin(eta)*cos(zeta)*cos(zeta) 
		+ l*cosBeta*sinAlpha*sinAlpha*cos(theta)*cos(theta)*sin(eta)*cos(zeta)*cos(zeta) 
		+ l*cosAlpha*cosAlpha*cosBeta*sin(eta)*cos(zeta)*cos(zeta)*sin(theta)*sin(theta)
		+ l*cosAlpha*cosAlpha*cosBeta*cos(theta)*cos(theta)*sin(eta)*sin(zeta)*sin(zeta)) 
		- (cosAlpha*sin(theta)*sin(zeta))/(l*cosBeta*sinAlpha*sinAlpha*sin(eta)*cos(zeta)*cos(zeta)*sin(theta)*sin(theta)
		+ l*cosBeta*sinAlpha*sinAlpha*cos(theta)*cos(theta)*sin(eta)*sin(zeta)*sin(zeta)
		+ l*cosAlpha*cosAlpha*cosBeta*sin(eta)*sin(theta)*sin(theta)*sin(zeta)*sin(zeta)
		+ l*cosBeta*sinAlpha*sinAlpha*sin(eta)*sin(theta)*sin(theta)*sin(zeta)*sin(zeta)
		+ l*cosAlpha*cosAlpha*cos(eta)*sinBeta*cos(theta)*cos(zeta) 
		+ l*cos(eta)*sinAlpha*sinAlpha*sinBeta*cos(theta)*cos(zeta) 
		- l*cosAlpha*cosAlpha*cos(eta)*sinBeta*sin(theta)*sin(zeta) 
		- l*cos(eta)*sinAlpha*sinAlpha*sinBeta*sin(theta)*sin(zeta) 
		+ l*cosAlpha*cosAlpha*cosBeta*cos(theta)*cos(theta)*sin(eta)*cos(zeta)*cos(zeta) 
		+ l*cosBeta*sinAlpha*sinAlpha*cos(theta)*cos(theta)*sin(eta)*cos(zeta)*cos(zeta) 
		+ l*cosAlpha*cosAlpha*cosBeta*sin(eta)*cos(zeta)*cos(zeta)*sin(theta)*sin(theta)
		+ l*cosAlpha*cosAlpha*cosBeta*cos(theta)*cos(theta)*sin(eta)*sin(zeta)*sin(zeta)) 
		- (cosBeta*sinAlpha*cos(theta)*sin(zeta))/(l*cosBeta*sinAlpha*sinAlpha*sin(eta)*cos(zeta)*cos(zeta)*sin(theta)*sin(theta)
		+ l*cosBeta*sinAlpha*sinAlpha*cos(theta)*cos(theta)*sin(eta)*sin(zeta)*sin(zeta)
		+ l*cosAlpha*cosAlpha*cosBeta*sin(eta)*sin(theta)*sin(theta)*sin(zeta)*sin(zeta)
		+ l*cosBeta*sinAlpha*sinAlpha*sin(eta)*sin(theta)*sin(theta)*sin(zeta)*sin(zeta)
		+ l*cosAlpha*cosAlpha*cos(eta)*sinBeta*cos(theta)*cos(zeta) 
		+ l*cos(eta)*sinAlpha*sinAlpha*sinBeta*cos(theta)*cos(zeta) 
		- l*cosAlpha*cosAlpha*cos(eta)*sinBeta*sin(theta)*sin(zeta) 
		- l*cos(eta)*sinAlpha*sinAlpha*sinBeta*sin(theta)*sin(zeta) 
		+ l*cosAlpha*cosAlpha*cosBeta*cos(theta)*cos(theta)*sin(eta)*cos(zeta)*cos(zeta) 
		+ l*cosBeta*sinAlpha*sinAlpha*cos(theta)*cos(theta)*sin(eta)*cos(zeta)*cos(zeta) 
		+ l*cosAlpha*cosAlpha*cosBeta*sin(eta)*cos(zeta)*cos(zeta)*sin(theta)*sin(theta)
		+ l*cosAlpha*cosAlpha*cosBeta*cos(theta)*cos(theta)*sin(eta)*sin(zeta)*sin(zeta)) 
		- (cosBeta*sinAlpha*cos(zeta)*sin(theta))/(l*cosBeta*sinAlpha*sinAlpha*sin(eta)*cos(zeta)*cos(zeta)*sin(theta)*sin(theta)
		+ l*cosBeta*sinAlpha*sinAlpha*cos(theta)*cos(theta)*sin(eta)*sin(zeta)*sin(zeta)
		+ l*cosAlpha*cosAlpha*cosBeta*sin(eta)*sin(theta)*sin(theta)*sin(zeta)*sin(zeta)
		+ l*cosBeta*sinAlpha*sinAlpha*sin(eta)*sin(theta)*sin(theta)*sin(zeta)*sin(zeta)
		+ l*cosAlpha*cosAlpha*cos(eta)*sinBeta*cos(theta)*cos(zeta) 
		+ l*cos(eta)*sinAlpha*sinAlpha*sinBeta*cos(theta)*cos(zeta) 
		- l*cosAlpha*cosAlpha*cos(eta)*sinBeta*sin(theta)*sin(zeta) 
		- l*cos(eta)*sinAlpha*sinAlpha*sinBeta*sin(theta)*sin(zeta) 
		+ l*cosAlpha*cosAlpha*cosBeta*cos(theta)*cos(theta)*sin(eta)*cos(zeta)*cos(zeta) 
		+ l*cosBeta*sinAlpha*sinAlpha*cos(theta)*cos(theta)*sin(eta)*cos(zeta)*cos(zeta) 
		+ l*cosAlpha*cosAlpha*cosBeta*sin(eta)*cos(zeta)*cos(zeta)*sin(theta)*sin(theta)
		+ l*cosAlpha*cosAlpha*cosBeta*cos(theta)*cos(theta)*sin(eta)*sin(zeta)*sin(zeta)); 
			
	invP[2][0] = 0;

	invP[2][1] = (sinAlpha*cos(theta)*sin(eta)*sin(zeta))/(l*cosAlpha*cosAlpha*cos(eta)*cos(eta)*sinBeta*cos(theta)*cos(zeta) 
		+ l*cos(eta)*cos(eta)*sinAlpha*sinAlpha*sinBeta*cos(theta)*cos(zeta) 
		- l*cosAlpha*cosAlpha*cos(eta)*cos(eta)*sinBeta*sin(theta)*sin(zeta) 
		- l*cos(eta)*cos(eta)*sinAlpha*sinAlpha*sinBeta*sin(theta)*sin(zeta) 
		+ l*cosAlpha*cosAlpha*cosBeta*cos(eta)*cos(theta)*cos(theta)*sin(eta)*cos(zeta)*cos(zeta) 
		+ l*cosBeta*cos(eta)*sinAlpha*sinAlpha*cos(theta)*cos(theta)*sin(eta)*cos(zeta)*cos(zeta) 
		+ l*cosAlpha*cosAlpha*cosBeta*cos(eta)*sin(eta)*cos(zeta)*cos(zeta)*sin(theta)*sin(theta)
		+ l*cosAlpha*cosAlpha*cosBeta*cos(eta)*cos(theta)*cos(theta)*sin(eta)*sin(zeta)*sin(zeta)
		+ l*cosBeta*cos(eta)*sinAlpha*sinAlpha*sin(eta)*cos(zeta)*cos(zeta)*sin(theta)*sin(theta)
		+ l*cosBeta*cos(eta)*sinAlpha*sinAlpha*cos(theta)*cos(theta)*sin(eta)*sin(zeta)*sin(zeta)
		+ l*cosAlpha*cosAlpha*cosBeta*cos(eta)*sin(eta)*sin(theta)*sin(theta)*sin(zeta)*sin(zeta)
		+ l*cosBeta*cos(eta)*sinAlpha*sinAlpha*sin(eta)*sin(theta)*sin(theta)*sin(zeta)*sin(zeta)) 
		- (cosAlpha*cos(eta)*sinBeta)/(l*cosAlpha*cosAlpha*cos(eta)*cos(eta)*sinBeta*cos(theta)*cos(zeta) 
		+ l*cos(eta)*cos(eta)*sinAlpha*sinAlpha*sinBeta*cos(theta)*cos(zeta) 
		- l*cosAlpha*cosAlpha*cos(eta)*cos(eta)*sinBeta*sin(theta)*sin(zeta) 
		- l*cos(eta)*cos(eta)*sinAlpha*sinAlpha*sinBeta*sin(theta)*sin(zeta) 
		+ l*cosAlpha*cosAlpha*cosBeta*cos(eta)*cos(theta)*cos(theta)*sin(eta)*cos(zeta)*cos(zeta) 
		+ l*cosBeta*cos(eta)*sinAlpha*sinAlpha*cos(theta)*cos(theta)*sin(eta)*cos(zeta)*cos(zeta) 
		+ l*cosAlpha*cosAlpha*cosBeta*cos(eta)*sin(eta)*cos(zeta)*cos(zeta)*sin(theta)*sin(theta)
		+ l*cosAlpha*cosAlpha*cosBeta*cos(eta)*cos(theta)*cos(theta)*sin(eta)*sin(zeta)*sin(zeta)
		+ l*cosBeta*cos(eta)*sinAlpha*sinAlpha*sin(eta)*cos(zeta)*cos(zeta)*sin(theta)*sin(theta)
		+ l*cosBeta*cos(eta)*sinAlpha*sinAlpha*cos(theta)*cos(theta)*sin(eta)*sin(zeta)*sin(zeta)
		+ l*cosAlpha*cosAlpha*cosBeta*cos(eta)*sin(eta)*sin(theta)*sin(theta)*sin(zeta)*sin(zeta)
		+ l*cosBeta*cos(eta)*sinAlpha*sinAlpha*sin(eta)*sin(theta)*sin(theta)*sin(zeta)*sin(zeta)) 
		+ (sinAlpha*sin(eta)*cos(zeta)*sin(theta))/(l*cosAlpha*cosAlpha*cos(eta)*cos(eta)*sinBeta*cos(theta)*cos(zeta) 
		+ l*cos(eta)*cos(eta)*sinAlpha*sinAlpha*sinBeta*cos(theta)*cos(zeta) 
		- l*cosAlpha*cosAlpha*cos(eta)*cos(eta)*sinBeta*sin(theta)*sin(zeta) 
		- l*cos(eta)*cos(eta)*sinAlpha*sinAlpha*sinBeta*sin(theta)*sin(zeta) 
		+ l*cosAlpha*cosAlpha*cosBeta*cos(eta)*cos(theta)*cos(theta)*sin(eta)*cos(zeta)*cos(zeta) 
		+ l*cosBeta*cos(eta)*sinAlpha*sinAlpha*cos(theta)*cos(theta)*sin(eta)*cos(zeta)*cos(zeta) 
		+ l*cosAlpha*cosAlpha*cosBeta*cos(eta)*sin(eta)*cos(zeta)*cos(zeta)*sin(theta)*sin(theta)
		+ l*cosAlpha*cosAlpha*cosBeta*cos(eta)*cos(theta)*cos(theta)*sin(eta)*sin(zeta)*sin(zeta)
		+ l*cosBeta*cos(eta)*sinAlpha*sinAlpha*sin(eta)*cos(zeta)*cos(zeta)*sin(theta)*sin(theta)
		+ l*cosBeta*cos(eta)*sinAlpha*sinAlpha*cos(theta)*cos(theta)*sin(eta)*sin(zeta)*sin(zeta)
		+ l*cosAlpha*cosAlpha*cosBeta*cos(eta)*sin(eta)*sin(theta)*sin(theta)*sin(zeta)*sin(zeta)
		+ l*cosBeta*cos(eta)*sinAlpha*sinAlpha*sin(eta)*sin(theta)*sin(theta)*sin(zeta)*sin(zeta)) 
		- (cosAlpha*cosBeta*cos(theta)*sin(eta)*cos(zeta))/(l*cosAlpha*cosAlpha*cos(eta)*cos(eta)*sinBeta*cos(theta)*cos(zeta) 
		+ l*cos(eta)*cos(eta)*sinAlpha*sinAlpha*sinBeta*cos(theta)*cos(zeta) 
		- l*cosAlpha*cosAlpha*cos(eta)*cos(eta)*sinBeta*sin(theta)*sin(zeta) 
		- l*cos(eta)*cos(eta)*sinAlpha*sinAlpha*sinBeta*sin(theta)*sin(zeta) 
		+ l*cosAlpha*cosAlpha*cosBeta*cos(eta)*cos(theta)*cos(theta)*sin(eta)*cos(zeta)*cos(zeta) 
		+ l*cosBeta*cos(eta)*sinAlpha*sinAlpha*cos(theta)*cos(theta)*sin(eta)*cos(zeta)*cos(zeta) 
		+ l*cosAlpha*cosAlpha*cosBeta*cos(eta)*sin(eta)*cos(zeta)*cos(zeta)*sin(theta)*sin(theta)
		+ l*cosAlpha*cosAlpha*cosBeta*cos(eta)*cos(theta)*cos(theta)*sin(eta)*sin(zeta)*sin(zeta)
		+ l*cosBeta*cos(eta)*sinAlpha*sinAlpha*sin(eta)*cos(zeta)*cos(zeta)*sin(theta)*sin(theta)
		+ l*cosBeta*cos(eta)*sinAlpha*sinAlpha*cos(theta)*cos(theta)*sin(eta)*sin(zeta)*sin(zeta)
		+ l*cosAlpha*cosAlpha*cosBeta*cos(eta)*sin(eta)*sin(theta)*sin(theta)*sin(zeta)*sin(zeta)
		+ l*cosBeta*cos(eta)*sinAlpha*sinAlpha*sin(eta)*sin(theta)*sin(theta)*sin(zeta)*sin(zeta)) 
		+ (cosAlpha*cosBeta*sin(eta)*sin(theta)*sin(zeta))/(l*cosAlpha*cosAlpha*cos(eta)*cos(eta)*sinBeta*cos(theta)*cos(zeta) 
		+ l*cos(eta)*cos(eta)*sinAlpha*sinAlpha*sinBeta*cos(theta)*cos(zeta) 
		- l*cosAlpha*cosAlpha*cos(eta)*cos(eta)*sinBeta*sin(theta)*sin(zeta) 
		- l*cos(eta)*cos(eta)*sinAlpha*sinAlpha*sinBeta*sin(theta)*sin(zeta) 
		+ l*cosAlpha*cosAlpha*cosBeta*cos(eta)*cos(theta)*cos(theta)*sin(eta)*cos(zeta)*cos(zeta) 
		+ l*cosBeta*cos(eta)*sinAlpha*sinAlpha*cos(theta)*cos(theta)*sin(eta)*cos(zeta)*cos(zeta) 
		+ l*cosAlpha*cosAlpha*cosBeta*cos(eta)*sin(eta)*cos(zeta)*cos(zeta)*sin(theta)*sin(theta)
		+ l*cosAlpha*cosAlpha*cosBeta*cos(eta)*cos(theta)*cos(theta)*sin(eta)*sin(zeta)*sin(zeta)
		+ l*cosBeta*cos(eta)*sinAlpha*sinAlpha*sin(eta)*cos(zeta)*cos(zeta)*sin(theta)*sin(theta)
		+ l*cosBeta*cos(eta)*sinAlpha*sinAlpha*cos(theta)*cos(theta)*sin(eta)*sin(zeta)*sin(zeta)
		+ l*cosAlpha*cosAlpha*cosBeta*cos(eta)*sin(eta)*sin(theta)*sin(theta)*sin(zeta)*sin(zeta)
		+ l*cosBeta*cos(eta)*sinAlpha*sinAlpha*sin(eta)*sin(theta)*sin(theta)*sin(zeta)*sin(zeta));
		
	invP[2][2] = (sinAlpha*cos(theta)*cos(zeta))/(l*cosBeta*sinAlpha*sinAlpha*sin(eta)*cos(zeta)*cos(zeta)*sin(theta)*sin(theta)
		+ l*cosBeta*sinAlpha*sinAlpha*cos(theta)*cos(theta)*sin(eta)*sin(zeta)*sin(zeta)
		+ l*cosAlpha*cosAlpha*cosBeta*sin(eta)*sin(theta)*sin(theta)*sin(zeta)*sin(zeta)
		+ l*cosBeta*sinAlpha*sinAlpha*sin(eta)*sin(theta)*sin(theta)*sin(zeta)*sin(zeta)
		+ l*cosAlpha*cosAlpha*cos(eta)*sinBeta*cos(theta)*cos(zeta) 
		+ l*cos(eta)*sinAlpha*sinAlpha*sinBeta*cos(theta)*cos(zeta) 
		- l*cosAlpha*cosAlpha*cos(eta)*sinBeta*sin(theta)*sin(zeta) 
		- l*cos(eta)*sinAlpha*sinAlpha*sinBeta*sin(theta)*sin(zeta) 
		+ l*cosAlpha*cosAlpha*cosBeta*cos(theta)*cos(theta)*sin(eta)*cos(zeta)*cos(zeta) 
		+ l*cosBeta*sinAlpha*sinAlpha*cos(theta)*cos(theta)*sin(eta)*cos(zeta)*cos(zeta) 
		+ l*cosAlpha*cosAlpha*cosBeta*sin(eta)*cos(zeta)*cos(zeta)*sin(theta)*sin(theta)
		+ l*cosAlpha*cosAlpha*cosBeta*cos(theta)*cos(theta)*sin(eta)*sin(zeta)*sin(zeta)) 
		- (sinAlpha*sin(theta)*sin(zeta))/(l*cosBeta*sinAlpha*sinAlpha*sin(eta)*cos(zeta)*cos(zeta)*sin(theta)*sin(theta)
		+ l*cosBeta*sinAlpha*sinAlpha*cos(theta)*cos(theta)*sin(eta)*sin(zeta)*sin(zeta)
		+ l*cosAlpha*cosAlpha*cosBeta*sin(eta)*sin(theta)*sin(theta)*sin(zeta)*sin(zeta)
		+ l*cosBeta*sinAlpha*sinAlpha*sin(eta)*sin(theta)*sin(theta)*sin(zeta)*sin(zeta)
		+ l*cosAlpha*cosAlpha*cos(eta)*sinBeta*cos(theta)*cos(zeta) 
		+ l*cos(eta)*sinAlpha*sinAlpha*sinBeta*cos(theta)*cos(zeta) 
		- l*cosAlpha*cosAlpha*cos(eta)*sinBeta*sin(theta)*sin(zeta) 
		- l*cos(eta)*sinAlpha*sinAlpha*sinBeta*sin(theta)*sin(zeta) 
		+ l*cosAlpha*cosAlpha*cosBeta*cos(theta)*cos(theta)*sin(eta)*cos(zeta)*cos(zeta) 
		+ l*cosBeta*sinAlpha*sinAlpha*cos(theta)*cos(theta)*sin(eta)*cos(zeta)*cos(zeta) 
		+ l*cosAlpha*cosAlpha*cosBeta*sin(eta)*cos(zeta)*cos(zeta)*sin(theta)*sin(theta)
		+ l*cosAlpha*cosAlpha*cosBeta*cos(theta)*cos(theta)*sin(eta)*sin(zeta)*sin(zeta)) 
		+ (cosAlpha*cosBeta*cos(theta)*sin(zeta))/(l*cosBeta*sinAlpha*sinAlpha*sin(eta)*cos(zeta)*cos(zeta)*sin(theta)*sin(theta)
		+ l*cosBeta*sinAlpha*sinAlpha*cos(theta)*cos(theta)*sin(eta)*sin(zeta)*sin(zeta)
		+ l*cosAlpha*cosAlpha*cosBeta*sin(eta)*sin(theta)*sin(theta)*sin(zeta)*sin(zeta)
		+ l*cosBeta*sinAlpha*sinAlpha*sin(eta)*sin(theta)*sin(theta)*sin(zeta)*sin(zeta)
		+ l*cosAlpha*cosAlpha*cos(eta)*sinBeta*cos(theta)*cos(zeta) 
		+ l*cos(eta)*sinAlpha*sinAlpha*sinBeta*cos(theta)*cos(zeta) 
		- l*cosAlpha*cosAlpha*cos(eta)*sinBeta*sin(theta)*sin(zeta) 
		- l*cos(eta)*sinAlpha*sinAlpha*sinBeta*sin(theta)*sin(zeta) 
		+ l*cosAlpha*cosAlpha*cosBeta*cos(theta)*cos(theta)*sin(eta)*cos(zeta)*cos(zeta) 
		+ l*cosBeta*sinAlpha*sinAlpha*cos(theta)*cos(theta)*sin(eta)*cos(zeta)*cos(zeta) 
		+ l*cosAlpha*cosAlpha*cosBeta*sin(eta)*cos(zeta)*cos(zeta)*sin(theta)*sin(theta)
		+ l*cosAlpha*cosAlpha*cosBeta*cos(theta)*cos(theta)*sin(eta)*sin(zeta)*sin(zeta)) 
		+ (cosAlpha*cosBeta*cos(zeta)*sin(theta))/(l*cosBeta*sinAlpha*sinAlpha*sin(eta)*cos(zeta)*cos(zeta)*sin(theta)*sin(theta)
		+ l*cosBeta*sinAlpha*sinAlpha*cos(theta)*cos(theta)*sin(eta)*sin(zeta)*sin(zeta)
		+ l*cosAlpha*cosAlpha*cosBeta*sin(eta)*sin(theta)*sin(theta)*sin(zeta)*sin(zeta)
		+ l*cosBeta*sinAlpha*sinAlpha*sin(eta)*sin(theta)*sin(theta)*sin(zeta)*sin(zeta)
		+ l*cosAlpha*cosAlpha*cos(eta)*sinBeta*cos(theta)*cos(zeta) 
		+ l*cos(eta)*sinAlpha*sinAlpha*sinBeta*cos(theta)*cos(zeta) 
		- l*cosAlpha*cosAlpha*cos(eta)*sinBeta*sin(theta)*sin(zeta) 
		- l*cos(eta)*sinAlpha*sinAlpha*sinBeta*sin(theta)*sin(zeta) 
		+ l*cosAlpha*cosAlpha*cosBeta*cos(theta)*cos(theta)*sin(eta)*cos(zeta)*cos(zeta) 
		+ l*cosBeta*sinAlpha*sinAlpha*cos(theta)*cos(theta)*sin(eta)*cos(zeta)*cos(zeta) 
		+ l*cosAlpha*cosAlpha*cosBeta*sin(eta)*cos(zeta)*cos(zeta)*sin(theta)*sin(theta)
		+ l*cosAlpha*cosAlpha*cosBeta*cos(theta)*cos(theta)*sin(eta)*sin(zeta)*sin(zeta));
}

double OneLimb::CalculateZeta(double Theta)/* Verified */{
	return -2*atan(
				sqrt(2*l*l*l*l 
					- 4*b1*b1*b1*b1*cosBetaHalf*cosBetaHalf*cosBetaHalf*cosBetaHalf 
					- 4*l*l*l*l*cosBetaHalf*cosBetaHalf*cosBetaHalf*cosBetaHalf 
					+ b1*b1*b1*b1 
					- 7*l*l*b1*b1 
					+ 2*l*l*l*l*cosBeta 
					+ 2*b1*b1*b1*b1*cosBeta 
					+ 16*l*l*b1*b1*cosBetaHalf*cosBetaHalf*cosBetaHalf*cosBetaHalf 
					+ 16*l*l*b1*b1*cos(theta/2)*cos(theta/2)*cos(theta/2)*cos(theta/2) 
					- 8*l*l*b1*b1*cosBeta 
					- 8*l*l*b1*b1*cos(theta) 
					+ 16*l*l*b1*b1*cosBetaHalf*cosBetaHalf*cosBetaHalf*cosBetaHalf*cos(theta) 
					+ 16*l*l*b1*b1*cos(theta/2)*cos(theta/2)*cos(theta/2)*cos(theta/2)*cosBeta 
					- 8*l*l*b1*b1*cosBeta*cos(theta) 
					- 32*l*l*b1*b1*cosBetaHalf*cosBetaHalf*cosBetaHalf*cosBetaHalf*cos(theta/2)*cos(theta/2)*cos(theta/2)*cos(theta/2) 
					+ (8*l*l*b1*cosBetaHalf*cosBetaHalf*cosBetaHalf*sinBetaHalf*sqrt(b1*b1*cos(theta)*cos(theta) 
						+ l*l - b1*b1 - b1*b1*cosBeta*cosBeta*cos(theta)*cos(theta)))/(cosBeta + 1) 
					- (32*l*l*b1*cosBetaHalf*cosBetaHalf*cosBetaHalf*sinBetaHalf*cos(theta/2)*cos(theta/2)*cos(theta/2)*cos(theta/2)*sqrt(b1*b1*cos(theta)*cos(theta) 
					+ l*l - b1*b1 - b1*b1*cosBeta*cosBeta*cos(theta)*cos(theta)))
						/(cosBeta + cos(theta) + cosBeta*cos(theta) + 1))
				/(4*(l*l*cosBetaHalf*cosBetaHalf*tanBetaHalf*cos(theta/2)*cos(theta/2)*tan(theta/2) 
				- b1*b1*cosBetaHalf*cosBetaHalf*tanBetaHalf*cos(theta/2)*cos(theta/2)*tan(theta/2))) 
				- (l*l*tanBetaHalf)/(2*(tanBetaHalf*tan(theta/2)*l*l - tanBetaHalf*tan(theta/2)*b1*b1)) 
				+ (b1*sqrt(2*l*l*tanBetaHalf*tanBetaHalf 
					+ l*l*tanBetaHalf*tanBetaHalf*tanBetaHalf*tanBetaHalf + l*l 
					+ 2*l*l*tan(theta/2)*tan(theta/2) + l*l*tan(theta/2)*tan(theta/2)*tan(theta/2)*tan(theta/2) 
					+ 2*b1*b1*tanBetaHalf*tanBetaHalf - b1*b1*tanBetaHalf*tanBetaHalf*tanBetaHalf*tanBetaHalf - b1*b1 
					- 2*b1*b1*tan(theta/2)*tan(theta/2) - b1*b1*tan(theta/2)*tan(theta/2)*tan(theta/2)*tan(theta/2) 
					+ 4*l*l*tanBetaHalf*tanBetaHalf*tan(theta/2)*tan(theta/2) 
					+ 2*l*l*tanBetaHalf*tanBetaHalf*tan(theta/2)*tan(theta/2)*tan(theta/2)*tan(theta/2) 
					+ 2*l*l*tanBetaHalf*tanBetaHalf*tanBetaHalf*tanBetaHalf*tan(theta/2)*tan(theta/2) 
					+ l*l*tanBetaHalf*tanBetaHalf*tanBetaHalf*tanBetaHalf*tan(theta/2)*tan(theta/2)*tan(theta/2)*tan(theta/2) 
					- 12*b1*b1*tanBetaHalf*tanBetaHalf*tan(theta/2)*tan(theta/2) 
					+ 2*b1*b1*tanBetaHalf*tanBetaHalf*tan(theta/2)*tan(theta/2)*tan(theta/2)*tan(theta/2) 
					- 2*b1*b1*tanBetaHalf*tanBetaHalf*tanBetaHalf*tanBetaHalf*tan(theta/2)*tan(theta/2) 
					- b1*b1*tanBetaHalf*tanBetaHalf*tanBetaHalf*tanBetaHalf*tan(theta/2)*tan(theta/2)*tan(theta/2)*tan(theta/2)))
				/(4*(tanBetaHalf*tan(theta/2)*l*l - tanBetaHalf*tan(theta/2)*b1*b1)) 
				+ (l*l*tanBetaHalf*tan(theta/2)*tan(theta/2))/(2*(tanBetaHalf*tan(theta/2)*l*l - tanBetaHalf*tan(theta/2)*b1*b1)));

}

double OneLimb::CalculateEta(double Theta) /* Verified */ {
	return -2*atan(
				sqrt(2*l*l*l*l 
					- 4*b1*b1*b1*b1*cosBetaHalf*cosBetaHalf*cosBetaHalf*cosBetaHalf 
					- 4*l*l*l*l*cosBetaHalf*cosBetaHalf*cosBetaHalf*cosBetaHalf + b1*b1*b1*b1 - 7*l*l*b1*b1 + 2*l*l*l*l*cosBeta 
					+ 2*b1*b1*b1*b1*cosBeta + 16*l*l*b1*b1*cosBetaHalf*cosBetaHalf*cosBetaHalf*cosBetaHalf 
					+ 16*l*l*b1*b1*cos(theta/2)*cos(theta/2)*cos(theta/2)*cos(theta/2) - 8*l*l*b1*b1*cosBeta 
					- 8*l*l*b1*b1*cos(theta) + 16*l*l*b1*b1*cosBetaHalf*cosBetaHalf*cosBetaHalf*cosBetaHalf*cos(theta) 
					+ 16*l*l*b1*b1*cos(theta/2)*cos(theta/2)*cos(theta/2)*cos(theta/2)*cosBeta - 8*l*l*b1*b1*cosBeta*cos(theta) 
					- 32*l*l*b1*b1*cosBetaHalf*cosBetaHalf*cosBetaHalf*cosBetaHalf*cos(theta/2)*cos(theta/2)*cos(theta/2)*cos(theta/2) 
					+ (8*l*l*b1*cosBetaHalf*cosBetaHalf*cosBetaHalf*sinBetaHalf*sqrt(b1*b1*cos(theta)*cos(theta) + l*l - b1*b1 
					- b1*b1*cosBeta*cosBeta*cos(theta)*cos(theta)))/(cosBeta + 1) 
					- (32*l*l*b1*cosBetaHalf*cosBetaHalf*cosBetaHalf*sinBetaHalf*cos(theta/2)*cos(theta/2)*cos(theta/2)*cos(theta/2)
						*sqrt(b1*b1*cos(theta)*cos(theta) + l*l - b1*b1 - b1*b1*cosBeta*cosBeta*cos(theta)*cos(theta)))
						/(cosBeta + cos(theta) + cosBeta*cos(theta) + 1))
				/(2*(l*l*cosBetaHalf*cosBetaHalf*cos(theta/2)*cos(theta/2) 
						- b1*b1*cosBetaHalf*cosBetaHalf*cos(theta/2)*cos(theta/2) 
						- l*l*cosBetaHalf*cosBetaHalf*tanBetaHalf*tanBetaHalf*cos(theta/2)*cos(theta/2) 
						+ b1*b1*cosBetaHalf*cosBetaHalf*tanBetaHalf*tanBetaHalf*cos(theta/2)*cos(theta/2))) 
				+ (cos(theta)
					*sqrt(2*l*l*l*l - 4*b1*b1*b1*b1*cosBetaHalf*cosBetaHalf*cosBetaHalf*cosBetaHalf 
						- 4*l*l*l*l*cosBetaHalf*cosBetaHalf*cosBetaHalf*cosBetaHalf + b1*b1*b1*b1 - 7*l*l*b1*b1 
						+ 2*l*l*l*l*cosBeta + 2*b1*b1*b1*b1*cosBeta + 16*l*l*b1*b1*cosBetaHalf*cosBetaHalf*cosBetaHalf*cosBetaHalf 
						+ 16*l*l*b1*b1*cos(theta/2)*cos(theta/2)*cos(theta/2)*cos(theta/2) - 8*l*l*b1*b1*cosBeta 
						- 8*l*l*b1*b1*cos(theta) + 16*l*l*b1*b1*cosBetaHalf*cosBetaHalf*cosBetaHalf*cosBetaHalf*cos(theta) 
						+ 16*l*l*b1*b1*cos(theta/2)*cos(theta/2)*cos(theta/2)*cos(theta/2)*cosBeta 
						- 8*l*l*b1*b1*cosBeta*cos(theta) 
						- 32*l*l*b1*b1*cosBetaHalf*cosBetaHalf*cosBetaHalf*cosBetaHalf*cos(theta/2)*cos(theta/2)*cos(theta/2)*cos(theta/2) 
						+ (8*l*l*b1*cosBetaHalf*cosBetaHalf*cosBetaHalf*sinBetaHalf
								*sqrt(b1*b1*cos(theta)*cos(theta) + l*l - b1*b1 - b1*b1*cosBeta*cosBeta*cos(theta)*cos(theta)))
							/(cosBeta + 1) 
						- (32*l*l*b1*cosBetaHalf*cosBetaHalf*cosBetaHalf*sinBetaHalf*cos(theta/2)*cos(theta/2)*cos(theta/2)*cos(theta/2)
								*sqrt(b1*b1*cos(theta)*cos(theta) + l*l - b1*b1 - b1*b1*cosBeta*cosBeta*cos(theta)*cos(theta)))
							/(cosBeta + cos(theta) + cosBeta*cos(theta) + 1)))
					/(2*(l*l*cosBetaHalf*cosBetaHalf*cos(theta/2)*cos(theta/2) 
						- b1*b1*cosBetaHalf*cosBetaHalf*cos(theta/2)*cos(theta/2) 
						- l*l*cosBetaHalf*cosBetaHalf*tanBetaHalf*tanBetaHalf*cos(theta/2)*cos(theta/2) 
						+ b1*b1*cosBetaHalf*cosBetaHalf*tanBetaHalf*tanBetaHalf*cos(theta/2)*cos(theta/2))) 
						+ (l*sqrt(2*l*l*tanBetaHalf*tanBetaHalf + l*l*tanBetaHalf*tanBetaHalf*tanBetaHalf*tanBetaHalf 
								+ l*l + 2*l*l*tan(theta/2)*tan(theta/2) 
								+ l*l*tan(theta/2)*tan(theta/2)*tan(theta/2)*tan(theta/2) 
								+ 2*b1*b1*tanBetaHalf*tanBetaHalf - b1*b1*tanBetaHalf*tanBetaHalf*tanBetaHalf*tanBetaHalf 
								- b1*b1 - 2*b1*b1*tan(theta/2)*tan(theta/2) 
								- b1*b1*tan(theta/2)*tan(theta/2)*tan(theta/2)*tan(theta/2) 
								+ 4*l*l*tanBetaHalf*tanBetaHalf*tan(theta/2)*tan(theta/2) 
								+ 2*l*l*tanBetaHalf*tanBetaHalf*tan(theta/2)*tan(theta/2)*tan(theta/2)*tan(theta/2) 
								+ 2*l*l*tanBetaHalf*tanBetaHalf*tanBetaHalf*tanBetaHalf*tan(theta/2)*tan(theta/2) 
								+ l*l*tanBetaHalf*tanBetaHalf*tanBetaHalf*tanBetaHalf*tan(theta/2)*tan(theta/2)*tan(theta/2)*tan(theta/2) 
								- 12*b1*b1*tanBetaHalf*tanBetaHalf*tan(theta/2)*tan(theta/2) 
								+ 2*b1*b1*tanBetaHalf*tanBetaHalf*tan(theta/2)*tan(theta/2)*tan(theta/2)*tan(theta/2) 
								- 2*b1*b1*tanBetaHalf*tanBetaHalf*tanBetaHalf*tanBetaHalf*tan(theta/2)*tan(theta/2) 
								- b1*b1*tanBetaHalf*tanBetaHalf*tanBetaHalf*tanBetaHalf*tan(theta/2)*tan(theta/2)*tan(theta/2)*tan(theta/2)))
							/(2*(l*l*tanBetaHalf*tanBetaHalf - l*l - b1*b1*tanBetaHalf*tanBetaHalf + b1*b1))
					- (l*b1*tanBetaHalf)/(l*l*tanBetaHalf*tanBetaHalf - l*l - b1*b1*tanBetaHalf*tanBetaHalf + b1*b1) 
					+ (l*cos(theta)*
							sqrt(2*l*l*tanBetaHalf*tanBetaHalf + l*l*tanBetaHalf*tanBetaHalf*tanBetaHalf*tanBetaHalf 
								+ l*l + 2*l*l*tan(theta/2)*tan(theta/2) + l*l*tan(theta/2)*tan(theta/2)*tan(theta/2)*tan(theta/2) 
								+ 2*b1*b1*tanBetaHalf*tanBetaHalf - b1*b1*tanBetaHalf*tanBetaHalf*tanBetaHalf*tanBetaHalf - b1*b1 
								- 2*b1*b1*tan(theta/2)*tan(theta/2) - b1*b1*tan(theta/2)*tan(theta/2)*tan(theta/2)*tan(theta/2) 
								+ 4*l*l*tanBetaHalf*tanBetaHalf*tan(theta/2)*tan(theta/2) 
								+ 2*l*l*tanBetaHalf*tanBetaHalf*tan(theta/2)*tan(theta/2)*tan(theta/2)*tan(theta/2) 
								+ 2*l*l*tanBetaHalf*tanBetaHalf*tanBetaHalf*tanBetaHalf*tan(theta/2)*tan(theta/2) 
								+ l*l*tanBetaHalf*tanBetaHalf*tanBetaHalf*tanBetaHalf*tan(theta/2)*tan(theta/2)*tan(theta/2)*tan(theta/2) 
								- 12*b1*b1*tanBetaHalf*tanBetaHalf*tan(theta/2)*tan(theta/2) 
								+ 2*b1*b1*tanBetaHalf*tanBetaHalf*tan(theta/2)*tan(theta/2)*tan(theta/2)*tan(theta/2) 
								- 2*b1*b1*tanBetaHalf*tanBetaHalf*tanBetaHalf*tanBetaHalf*tan(theta/2)*tan(theta/2) 
								- b1*b1*tanBetaHalf*tanBetaHalf*tanBetaHalf*tanBetaHalf*tan(theta/2)*tan(theta/2)*tan(theta/2)*tan(theta/2)))
						/(2*(l*l*tanBetaHalf*tanBetaHalf - l*l - b1*b1*tanBetaHalf*tanBetaHalf + b1*b1)) 
					+ (l*b1*tanBetaHalf*tan(theta/2)*tan(theta/2))
						/(l*l*tanBetaHalf*tanBetaHalf - l*l - b1*b1*tanBetaHalf*tanBetaHalf + b1*b1) 
					- (l*b1*tanBetaHalf*cos(theta))/(l*l*tanBetaHalf*tanBetaHalf - l*l - b1*b1*tanBetaHalf*tanBetaHalf + b1*b1) 
					+ (l*b1*tanBetaHalf*tan(theta/2)*tan(theta/2)*cos(theta))
						/(l*l*tanBetaHalf*tanBetaHalf - l*l - b1*b1*tanBetaHalf*tanBetaHalf + b1*b1));

}

double OneLimb::CalculateZ(double Theta) /* Verified */{
	return -sqrt(b1*b1*cos(theta)*cos(theta) 
	+ l*l - b1*b1 
	- b1*b1*cosBeta*cosBeta*cos(theta)*cos(theta)) 
	- b1*sinBeta*cos(theta);
}

void OneLimb::qUpdate(double AbsMotorPosition){

	// Old q is now qz1 (q delayed)
	std::copy(&q[0][0], &q[0][0]+N, &qz1[0][0]);

	// Measured theta
	theta = motorPosToRad(AbsMotorPosition);
	
	// Forward inverse kinematic.
	zeta = CalculateZeta(theta);
	eta = CalculateEta(theta);
	z = CalculateZ(theta);

	x[2][0] = z;

	q[0][0] = theta;
	q[1][0] = zeta;
	q[2][0] = eta;

	// Simple differentiation
	dtheta = (q[0][0] - qz1[0][0])/SAMPLETIME;
	dzeta = (q[1][0] - qz1[1][0])/SAMPLETIME;
	deta = (q[2][0] - qz1[2][0])/SAMPLETIME;

	dq[0][0] = dtheta;
	dq[1][0] = dzeta;
	dq[2][0] = deta;
}

void OneLimb::CUpdate(){
	// TODO: Implement coriolis effect.
	C[0][0] = 0;
}

void OneLimb::GUpdate()/* Verified */{
	
	G[0][0] = b1*g*mL*sinBeta*sin(theta) 
        + (b1*g*mb*sinBeta*sin(theta))/2 
        + b1*g*mp*sinBeta*sin(theta) 
        + (L*g*mL*cos(eta)*sinBeta*cos(theta)*sin(zeta))/2 
        + (L*g*mL*cos(eta)*sinBeta*cos(zeta)*sin(theta))/2 
        + L*g*mp*cos(eta)*sinBeta*cos(theta)*sin(zeta) 
        + L*g*mp*cos(eta)*sinBeta*cos(zeta)*sin(theta);
        
    G[1][0] = (L*g*mL*cos(eta)*sinBeta*cos(theta)*sin(zeta))/2 
        + (L*g*mL*cos(eta)*sinBeta*cos(zeta)*sin(theta))/2 
        + L*g*mp*cos(eta)*sinBeta*cos(theta)*sin(zeta) 
        + L*g*mp*cos(eta)*sinBeta*cos(zeta)*sin(theta);
        
    G[2][0] = (L*g*mL*sinBeta*cos(theta)*sin(eta)*cos(zeta))/2 
        - L*g*mp*cosBeta*cos(eta) 
        - (L*g*mL*sinBeta*sin(eta)*sin(theta)*sin(zeta))/2 
        - L*g*mp*sinBeta*sin(eta)*sin(theta)*sin(zeta) 
        - (L*g*mL*cosBeta*cos(eta))/2 
        + L*g*mp*sinBeta*cos(theta)*sin(eta)*cos(zeta);
}

void OneLimb::HUpdate(){
	CUpdate();
	GUpdate();
	Matrix.Add(*C, *G, N, N, *H);
}

void OneLimb::exUpdate(){
	Matrix.Subtract(*xr, *x, N, 1, *ex);
}

void OneLimb::JUpdate() /* Have been verified */ {

	J[0][0] = l*cos(eta)*sinAlpha*sin(theta)*sin(zeta) 
	- b1*cosAlpha*cosBeta*sin(theta) 
	- b1*sinAlpha*cos(theta)
	- l*cos(eta)*sinAlpha*cos(theta)*cos(zeta) 
	- l*cosAlpha*cosBeta*cos(eta)*cos(theta)*sin(zeta) 
	- l*cosAlpha*cosBeta*cos(eta)*cos(zeta)*sin(theta);

	J[1][0] = l*(cos((eta))*cos((zeta))*(cos((theta))*cosAlpha 
	- sin((theta))*cosBeta*sinAlpha) 
	- cos((eta))*sin((zeta))*(sin((theta))*cosAlpha 
	+ cos((theta))*cosBeta*sinAlpha)) 
	+ b1*(cos((theta))*cosAlpha 
	- sin((theta))*cosBeta*sinAlpha);

	J[2][0] = sinBeta*(b1*sin((theta)) 
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

void OneLimb::dJUpdate() /* Have been verified */ {
    dJ[0][0] = b1*(sin(theta)*sinAlpha*dtheta 
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

    dJ[1][0] = -b1*(sin(theta)*cosAlpha*dtheta 
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
	- b1*cos(theta)*dtheta + l*sin(eta)*cos(zeta)*sin(theta)*deta 
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

void OneLimb::JtUpdate(){
	Matrix.Transpose(*J, N, N, *Jt);
}

int sign(float x) {
  if (x > 0) return 1;
  if (x < 0) return -1;
  return 0;
}

void LED(bool on){
	digitalWrite(13,on);
}