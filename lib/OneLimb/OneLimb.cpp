
#include "OneLimb.h"

OneLimb::OneLimb() :
    J{0,0,0, 0,0,0, 0,0,0},
    dJ{0,0,0, 0,0,0, 0,0,0},
    Jt{0,0,0, 0,0,0, 0,0,0},
    H{0,0,0},
    C{0,0,0},
    G{0,0,0},
    P{1,0,0, 0,0,0, 0,0,0},
    Phit{0,0,0, 0,0,0},
    invP{0,0,0, 0,0,0, 0,0,0},
    Md{0,0,0, 0,0,0, 0,0,0},
    Kv{Kvx,0,0, 0,Kvy,0, 0,0,Kvz},
    Kp{Kpx,0,0, 0,Kpy,0, 0,0,Kpz},
    q{-1,-1,-1},
    dq{-1,-1,-1},
    x{0,0,0},
    xr{0,0,0},
    ex{0,0,0},
    res{0,0,0}
{
    
}
OneLimb::~OneLimb(){

}

void OneLimb::setZref(double Z){
	// Only values between -0.4 and -1.0 is allowed.
	if (Z > -0.4) {
		Z = -0.4;
	} else if (Z < -1.0) {
		Z = 1.0;
	}
	
	xr[2][0] = Z;
}

double OneLimb::getTorque(){
    qUpdate();

    JUpdate();
    Matrix.Multiply(*Kv, *J, N, N, N, *KvJ);

	dJUpdate();
	Matrix.Multiply(*Md, *dJ, N, N, N, *MddJ);

	Matrix.Add(*MddJ, *KvJ, N, N, *MddJ_p_KvJ);

	dqUpdate();
	Matrix.Multiply(*MddJ_p_KvJ, *dq, N, N, 1, *MddJ_p_KvJ_t_dq);

	exUpdate();
	Matrix.Multiply(*Kp, *ex, N, N, N, *Kpex);

	Matrix.Subtract(*MddJ_p_KvJ_t_dq, *Kpex, N, N, *MddJ_p_KvJ_t_dq_m_Kpex);

	JtUpdate();
	Matrix.Multiply(*Jt, *MddJ_p_KvJ_t_dq_m_Kpex, N, N, 1, *Jt_t_MddJ_p_KvJ_t_dq_m_Kpex);
	
	HUpdate();
	Matrix.Subtract(*H, *MddJ_p_KvJ_t_dq_m_Kpex, N, N, *H_m_Jt_t_MddJ_p_KvJ_t_dq_m_Kpex);

	invPUpdate();
	Matrix.Multiply(*invP, *H_m_Jt_t_MddJ_p_KvJ_t_dq_m_Kpex, N, N, 1, *res);

	return res[0][0];
}

void OneLimb::JtUpdate(){
	Matrix.Transpose(*J, N, N, *Jt);
}

void OneLimb::invPUpdate() // TODO: Verify
{
	double theta = q[0][0];
	double zeta  = q[1][0];
	double eta   = q[2][0];

	theta = 1;
	zeta = 1;
	eta = 1;


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
		- (b*cosBeta*cos(theta)*cos(theta)*sin(eta)*cos(zeta))/(l*cos(eta)*cos(eta)*sinBeta*cos(theta)*cos(zeta) 
		- l*cos(eta)*cos(eta)*sinBeta*sin(theta)*sin(zeta) 
		+ l*cosBeta*cos(eta)*cos(theta)*cos(theta)*sin(eta)*cos(zeta)*cos(zeta) 
		+ l*cosBeta*cos(eta)*sin(eta)*cos(zeta)*cos(zeta)*sin(theta)*sin(theta)
		+ l*cosBeta*cos(eta)*cos(theta)*cos(theta)*sin(eta)*sin(zeta)*sin(zeta)
		+ l*cosBeta*cos(eta)*sin(eta)*sin(theta)*sin(theta)*sin(zeta)*sin(zeta)) 
		- (b*cos(eta)*sinBeta*cos(theta))/(l*cos(eta)*cos(eta)*sinBeta*cos(theta)*cos(zeta) 
		- l*cos(eta)*cos(eta)*sinBeta*sin(theta)*sin(zeta) 
		+ l*cosBeta*cos(eta)*cos(theta)*cos(theta)*sin(eta)*cos(zeta)*cos(zeta) 
		+ l*cosBeta*cos(eta)*sin(eta)*cos(zeta)*cos(zeta)*sin(theta)*sin(theta)
		+ l*cosBeta*cos(eta)*cos(theta)*cos(theta)*sin(eta)*sin(zeta)*sin(zeta)
		+ l*cosBeta*cos(eta)*sin(eta)*sin(theta)*sin(theta)*sin(zeta)*sin(zeta)) 
		- (b*cosBeta*sin(eta)*cos(zeta)*sin(theta)*sin(theta))/(l*cos(eta)*cos(eta)*sinBeta*cos(theta)*cos(zeta) 
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
		
	invP[0][2] = (b*cosBeta*cos(theta)*cos(theta)*sin(zeta))/(l*cosBeta*cos(theta)*cos(theta)*sin(eta)*cos(zeta)*cos(zeta) 
		- l*cos(eta)*sinBeta*sin(theta)*sin(zeta) 
		+ l*cosBeta*sin(eta)*cos(zeta)*cos(zeta)*sin(theta)*sin(theta)
		+ l*cosBeta*cos(theta)*cos(theta)*sin(eta)*sin(zeta)*sin(zeta)
		+ l*cosBeta*sin(eta)*sin(theta)*sin(theta)*sin(zeta)*sin(zeta)
		+ l*cos(eta)*sinBeta*cos(theta)*cos(zeta)) 
		+ (b*cosBeta*sin(theta)*sin(theta)*sin(zeta))/(l*cosBeta*cos(theta)*cos(theta)*sin(eta)*cos(zeta)*cos(zeta) 
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

void OneLimb::qUpdate(){
	// TODO: update q. Canbus data and FK?

	theta = q[0][0];
	zeta = q[1][0];
	eta = q[2][0];
}

void OneLimb::dqUpdate(){
	// TODO: update dq. Look what Juan have done.

	dtheta = dq[0][0];
	dzeta = dq[1][0];
	deta = dq[2][0];
}

void OneLimb::HUpdate(){
	CUpdate();
	GUpdate();
	Matrix.Add(*C, *G, N, N, *H);
}

void OneLimb::CUpdate(){

}

void OneLimb::GUpdate(){
	
	G[0][0] = b*g*mL*sinBeta*sin(theta) 
        + (b*g*mb*sinBeta*sin(theta))/2 
        + b*g*mp*sinBeta*sin(theta) 
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

void OneLimb::exUpdate(){

	Matrix.Subtract(*x, *xr, N, 1, *ex);
}

void OneLimb::dJUpdate() /* Have been verified */ {
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

void OneLimb::JUpdate() /* Have been verified */ {

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

