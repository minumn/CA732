#ifndef OneLimb_h
#define OneLimb_h

#include <iostream>
#include <Arduino.h>
#include <MatrixMath.h>
#include <math.h>
#include <string.h>

#include <SD.h>
#include <SPI.h>


#define OFF 0
#define ON 1

#define N 3

#define Kvx 40.83
#define Kvy 52.5
#define Kvz 41.25

#define Kpx 1000
#define Kpy 500
#define Kpz 200 

#define l 0.55 // meter
#define L 0.55 // meter
#define b1 0.3 // meter
#define g 9.81
#define mL 0.12 // kg
#define mp 0.25 // kg
#define mb 0.3 // kg

#define ZUPPERLIMIT -0.43 // meter
#define ZLOWERLIMIT -0.65 // meter

#define CONTROLDELAY 5000 // us -> 20 ms
#define SAMPLETIME 0.005

// #define alpha 0.2618
// #define beta 0.7854
#define sinAlpha 0.2588
#define cosAlpha 0.9659
#define cosBeta 0.7071
#define sinBeta 0.7071
#define cosBetaHalf 0.9239
#define sinBetaHalf 0.3827
#define tanBetaHalf 0.4142


/**
 * OneLimb is used to calculate the required torque to make 
 * a constrained limb of the Ragner robot move along the z axis.
 */
class OneLimb
{
public:
    OneLimb(); 
    ~OneLimb(); 
    double getTorque(); // Gets the torque value.
    double getTorque(double AbsMotorPos); // Feed new position value and get the new torque value.
    void setZref(double Z); // Set the z reference
    void newData(double AbsMotorPos); // Feed new data to the model.
    double motorPosToRad(double MotorPos); // Convert from motor ticks to angle in radian (theta)
    double motorPosToDeg(double MotorPos); // Convert from motor ticks to angle in degrees (theta)
    void setMotorPositionOffset(double MotorPosOffset); // The motor can get an undesired offset due to gearing.
    void writeToFile(const char* fileName); // Saved data to SD. SLOW (5-8 ms)
    double CalculateZ(double Theta); // Forward kinematic for z

    void setStiffness(float kpz); // Set stiffness value for the Impedance Controller
    void setDamping(float kvz); // Set Damping value for the Impedance Controller
    void setSampleTime(int ts);

    double z, zr;

private:
    double theta, zeta, eta, dtheta, dzeta, deta, _MotorPosOffset;
    long int timer;
    mtx_type J[N][N]; // Jacobian
    mtx_type dJ[N][N]; // Derrivitive Jacobian
    mtx_type Jt[N][N]; // Jacobian transposed
    mtx_type H[N][1]; // Coriolis and Gravity
    mtx_type C[N][1]; // Coriolis Matrix
    mtx_type G[N][1]; // Gravity Matrix
    mtx_type invP[N][N]; // P Inversed
    mtx_type Md[N][N];
    mtx_type Kv[N][N];
    mtx_type Kp[N][N];
    mtx_type q[N][1]; // [theta zeta eta]'
    mtx_type qz1[N][1]; // [theta zeta eta]' delayed
    mtx_type dq[N][1]; // Derrivative q
    mtx_type x[N][1]; // x 
    mtx_type xr[N][1]; // Ref x 
    mtx_type ex[N][1]; // Error x 
    mtx_type res[N][1]; // [tau lamba1 lambda2]'

    mtx_type KvJ[N][N]; // Kv * J
    mtx_type MddJ[N][N]; // Md * dJ
    mtx_type MddJ_p_KvJ[N][N]; // (Md * dJ) + (Kv * J)
    mtx_type MddJ_p_KvJ_t_dq[N][1];  // ((Md * dJ) + (Kv * J)) * dq
    mtx_type Kpex[N][1]; // Kp * ex
    mtx_type MddJ_p_KvJ_t_dq_m_Kpex[N][1]; // ((Md * dJ) + (Kv * J)) * dq) - (Kp * ex)
    mtx_type Jt_t_MddJ_p_KvJ_t_dq_m_Kpex[N][1]; // Jt * (((Md * dJ) + (Kv * J)) * dq) - (Kp * ex))
    mtx_type H_m_Jt_t_MddJ_p_KvJ_t_dq_m_Kpex[N][1]; // H - Jt * (((Md * dJ) + (Kv * J)) * dq) - (Kp * ex))

    double CalculateZeta(double Theta); 
    double CalculateEta(double Theta);
    
    void JUpdate();
    void dJUpdate();
    void JtUpdate();
    void invPUpdate();
    void qUpdate(double AbsMotorPos);
    void HUpdate();
    void exUpdate();
    void GUpdate();
    void CUpdate();
    void TorqueUpdate();
};


int sign(float x);
void LED(bool on);

#endif