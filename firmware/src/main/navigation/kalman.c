/*
 * kalman.c
 *
 *  Created on: 2026. 2. 11.
 *      Author: 왕학승
 */


#include "navigation/kalman.h"

#include <math.h>
#include <string.h>


/*
 * ekf_xy_bias.c
 *
 * 6-State EKF for Drone XY Position
 *
 * State:
 *   x = [ px, py, vx, vy, bax, bay ]
 *
 * Input:
 *   ax_meas, ay_meas (earth frame accel measurement)
 *
 * Measurement:
 *   Optical Flow velocity (vx, vy)
 *
 * Features:
 *   - Acceleration bias estimation
 *   - Full covariance prediction (FPF' + Q)
 *   - Physically derived process noise
 *   - Joseph form covariance update
 *   - Innovation gating
 *
 * Author: 왕학승
 */

/* ============================================================ */
/* ================= MATRIX HELPERS ============================ */
/* ============================================================ */

static void mat6_mult(float A[6][6], float B[6][6], float C[6][6])
{
    for (int i=0;i<6;i++)
        for (int j=0;j<6;j++)
        {
            C[i][j]=0;
            for (int k=0;k<6;k++)
                C[i][j]+=A[i][k]*B[k][j];
        }
}

static void mat6_transpose(float A[6][6], float AT[6][6])
{
    for (int i=0;i<6;i++)
        for (int j=0;j<6;j++)
            AT[j][i]=A[i][j];
}

/* ============================================================ */
/* ================= INITIALIZE ================================ */
/* ============================================================ */

void ekf_init(EKF_State *f)
{
    memset(f,0,sizeof(EKF_State));

    for(int i=0;i<STATE_DIM;i++)
        f->P[i][i]=1.0f;

    f->R[0]=0.05f;
    f->R[1]=0.05f;

    f->accel_noise=0.8f;   // tune
    f->bias_noise =0.02f;  // tune (very small)
}

/* ============================================================ */
/* ================= BUILD PROCESS NOISE ======================= */
/* ============================================================ */

static void ekf_build_Q(EKF_State *f, float dt)
{
    float dt2=dt*dt;
    float dt3=dt2*dt;
    float dt4=dt3*dt;

    float var_a=f->accel_noise*f->accel_noise;
    float var_b=f->bias_noise*f->bias_noise;

    memset(f->Q,0,sizeof(f->Q));

    /* Acceleration driven noise (position/velocity) */
    f->Q[0][0]=0.25f*dt4*var_a;
    f->Q[1][1]=0.25f*dt4*var_a;
    f->Q[0][2]=0.5f*dt3*var_a;
    f->Q[1][3]=0.5f*dt3*var_a;
    f->Q[2][0]=0.5f*dt3*var_a;
    f->Q[3][1]=0.5f*dt3*var_a;
    f->Q[2][2]=dt2*var_a;
    f->Q[3][3]=dt2*var_a;

    /* Bias random walk */
    f->Q[4][4]=dt*var_b;
    f->Q[5][5]=dt*var_b;
}

/* ============================================================ */
/* ======================= PREDICT ============================= */
/* ============================================================ */

void ekf_predict(EKF_State *f, float ax_meas, float ay_meas, float dt)
{
    float bax=f->x[4];
    float bay=f->x[5];

    float ax=ax_meas - bax;
    float ay=ay_meas - bay;

    /* ---- State prediction ---- */

    f->x[0]+=f->x[2]*dt + 0.5f*ax*dt*dt;
    f->x[1]+=f->x[3]*dt + 0.5f*ay*dt*dt;
    f->x[2]+=ax*dt;
    f->x[3]+=ay*dt;

    /* bias assumed random walk (no deterministic change) */

    /* ---- State transition matrix F ---- */

    float F[6][6]={0};

    for(int i=0;i<6;i++) F[i][i]=1.0f;

    F[0][2]=dt;
    F[1][3]=dt;

    F[0][4]=-0.5f*dt*dt;
    F[1][5]=-0.5f*dt*dt;

    F[2][4]=-dt;
    F[3][5]=-dt;

    float FT[6][6];
    float FP[6][6];
    float FPFt[6][6];

    mat6_transpose(F,FT);

    ekf_build_Q(f,dt);

    mat6_mult(F,f->P,FP);
    mat6_mult(FP,FT,FPFt);

    for(int i=0;i<6;i++)
        for(int j=0;j<6;j++)
            f->P[i][j]=FPFt[i][j]+f->Q[i][j];
}

/* ============================================================ */
/* ======================== UPDATE ============================= */
/* ============================================================ */

void ekf_update(EKF_State *f, float flow_vx, float flow_vy)
{
    float z[2]={flow_vx,flow_vy};

    for(int axis=0;axis<2;axis++)
    {
        int vel=axis+2;

        float y=z[axis]-f->x[vel];
        float S=f->P[vel][vel]+f->R[axis];

        if((y*y)/S > INNOVATION_GATE)
            continue;

        float K[6];
        for(int i=0;i<6;i++)
            K[i]=f->P[i][vel]/S;

        /* State update */
        for(int i=0;i<6;i++)
            f->x[i]+=K[i]*y;

        /* Joseph form */
        float I_KH[6][6]={0};
        for(int i=0;i<6;i++) I_KH[i][i]=1.0f;
        for(int i=0;i<6;i++) I_KH[i][vel]-=K[i];

        float temp[6][6];
        float temp2[6][6];
        float I_KH_T[6][6];

        mat6_mult(I_KH,f->P,temp);
        mat6_transpose(I_KH,I_KH_T);
        mat6_mult(temp,I_KH_T,temp2);

        for(int i=0;i<6;i++)
            for(int j=0;j<6;j++)
                f->P[i][j]=temp2[i][j] + K[i]*f->R[axis]*K[j];
    }
}


