#ifndef OneLimb_h
#define OneLimb_h

#include <iostream>
#include <Arduino.h>
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

class OneLimb
{
public:
    OneLimb();
    ~OneLimb();
    double getTorque();

private:
    mtx_type J[N][N]; // Jacobian
    mtx_type dJ[N][N]; // Derrivitive Jacobian
    mtx_type Jt[N][N]; // Jacobian transposed
    mtx_type H[N][1];
    mtx_type P[N][N];
    mtx_type Pt[N][N]; // P transposed
    mtx_type Md[N][N];
    mtx_type Kv[N][N];
    mtx_type Kp[N][N];
    mtx_type q[N][1]; // [theta zeta eta]'
    mtx_type dq[N][1]; // Derrivative q
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

    void JUpdate();
    void dJUpdate();
    void JtUpdate();
    void PtUpdate();
    void qUpdate();
    void dqUpdate();
    void HUpdate();
    void exUpdate();
};

#endif