#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "qst_convert.h"
#include "QstAlgoLog.h"

//#define DEBUG_LOG
#ifdef DEBUG_LOG
#undef LOG_TAG
#define LOG_TAG "qst-convert"
#define	QLOG		LOGD
#else
#define	QLOG(...)
#endif

#ifndef PI
#define PI					3.14159265358979f
#endif
#define RADTODEG			(180.0f/PI)
#define DEGTORAD			(PI/180.0f)
#define clamp(v)			((v) < 0 ? 0 : (v))

void android_quat_to_matrix(float quat[4], float mat[][3])
{
    float q0 = quat[3];	// w
    float q1 = quat[0];	// x
    float q2 = quat[1];	// y
    float q3 = quat[2];	// z
    float sq_q1 = 2 * q1 * q1;
    float sq_q2 = 2 * q2 * q2;
    float sq_q3 = 2 * q3 * q3;
    float q1_q2 = 2 * q1 * q2;
    float q3_q0 = 2 * q3 * q0;
    float q1_q3 = 2 * q1 * q3;
    float q2_q0 = 2 * q2 * q0;
    float q2_q3 = 2 * q2 * q3;
    float q1_q0 = 2 * q1 * q0;

	mat[0][0] = 1 - sq_q2 - sq_q3;
	mat[0][1] = q1_q2 - q3_q0;
	mat[0][2] = q1_q3 + q2_q0;
	mat[1][0] = q1_q2 + q3_q0;
	mat[1][1] = 1 - sq_q1 - sq_q3;
	mat[1][2] = q2_q3 - q1_q0;
	mat[2][0] = q1_q3 - q2_q0;
	mat[2][1] = q2_q3 + q1_q0;
	mat[2][2] = 1 - sq_q1 - sq_q2;
}

void android_quat_to_euler(float quat[4], float euler[3])
{
	float mat[3][3];
    float q0 = quat[3];	// w
    float q1 = quat[0];	// x
    float q2 = quat[1];	// y
    float q3 = quat[2];	// z
    float sq_q1 = 2 * q1 * q1;
    float sq_q2 = 2 * q2 * q2;
    float sq_q3 = 2 * q3 * q3;
    float q1_q2 = 2 * q1 * q2;
    float q3_q0 = 2 * q3 * q0;
    float q1_q3 = 2 * q1 * q3;
    float q2_q0 = 2 * q2 * q0;
    float q2_q3 = 2 * q2 * q3;
    float q1_q0 = 2 * q1 * q0;

    mat[0][0] = 1 - sq_q2 - sq_q3;
    mat[0][1] = q1_q2 - q3_q0;
    mat[0][2] = q1_q3 + q2_q0;
    mat[1][0] = q1_q2 + q3_q0;
    mat[1][1] = 1 - sq_q1 - sq_q3;
    mat[1][2] = q2_q3 - q1_q0;
    mat[2][0] = q1_q3 - q2_q0;
    mat[2][1] = q2_q3 + q1_q0;
    mat[2][2] = 1 - sq_q1 - sq_q2;

	euler[0] = atan2f(-mat[1][0], mat[0][0]) * RADTODEG;
	euler[1] = atan2f(-mat[2][1], mat[2][2]) * RADTODEG;
	euler[2] = asinf ( mat[2][0])			* RADTODEG;
	if(euler[0] < 0)
	{
		euler[0] += 360;
	}
}


void android_matrix_to_euler(float mat[][3], float euler[3])
{
	euler[0] = atan2f(-mat[1][0], mat[0][0]) * RADTODEG;
	euler[1] = atan2f(-mat[2][1], mat[2][2]) * RADTODEG;
	euler[2] = asinf ( mat[2][0])			* RADTODEG;
	if(euler[0] < 0)
	{
		euler[0] += 360;
	}
}

void android_matrix_to_quat(float mat[][3], float quat[4])
{
	const float Hx = mat[0][0];
	const float My = mat[1][1];
	const float Az = mat[2][2];

	quat[0] = sqrtf(clamp(Hx - My - Az + 1) * 0.25f);
	quat[1] = sqrtf(clamp(-Hx + My - Az + 1) * 0.25f);
	quat[2] = sqrtf(clamp(-Hx - My + Az + 1) * 0.25f);
	quat[3] = sqrtf(clamp(Hx + My + Az + 1) * 0.25f);
	quat[0] = copysignf(quat[0], mat[2][1] - mat[1][2]);
	quat[1] = copysignf(quat[1], mat[0][2] - mat[2][0]);
	quat[2] = copysignf(quat[2], mat[1][0] - mat[0][1]);

	//float q_length =  sqrtf(quat[0] * quat[0] + quat[1] * quat[1] + quat[2] * quat[2] + quat[3] * quat[3]);
	//quat[0] = quat[0] / q_length;
	//quat[1] = quat[1] / q_length;
	//quat[2] = quat[2] / q_length;
	//quat[3] = quat[3] / q_length;
}

void android_euler_to_matrix(float euler[3], float mat[][3])
{
	float yaw = euler[0]*DEGTORAD;
	float pitch = euler[0]*DEGTORAD;
	float roll = euler[0]*DEGTORAD;
	float cp = cosf(pitch);
	float sp = sinf(pitch); 	
	float sr = sinf(roll);
	float cr = cosf(roll);
	float sy = sinf(yaw);
	float cy = cosf(yaw);

	mat[0][0] = cy * cr;
	mat[0][1] = (cy * sr * sp + cp * sy);
	mat[0][2] = sy * sp - cy * cp * sr;

	mat[1][0] = -(cr * sy);
	mat[1][1] = -(sy * sr * sp -cy * cp);
	mat[1][2] = cp*sr*sy + sp*cy;

	mat[2][0] = sr;
	mat[2][1] = -(cr * sp);
	mat[2][2] = cr * cp;
}


void android_euler_to_quat(float euler[3], float quat[4])
{
	float yaw = euler[0]*DEGTORAD;
	float pitch = euler[0]*DEGTORAD;
	float roll = euler[0]*DEGTORAD;
	float cp = cosf(pitch);
	float sp = sinf(pitch); 	
	float sr = sinf(roll);
	float cr = cosf(roll);
	float sy = sinf(yaw);
	float cy = cosf(yaw);
	float mat[3][3];
#if 0
	mat33_t_R_yzw[0][0] = cy * cr;
	mat33_t_R_yzw[0][1] = -(cy * sr * sp - cp * sy);
	mat33_t_R_yzw[0][2] = sy * sp + cy * cp * sy;

	mat33_t_R_yzw[1][0] = -(cr * sy);
	mat33_t_R_yzw[1][1] = cy * cp + sy * sr * sp;
	mat33_t_R_yzw[1][2] = -(cp * sy * sr - cy * sp);

	mat33_t_R_yzw[2][0] = -sr;
	mat33_t_R_yzw[2][1] = -(cr * sp);
	mat33_t_R_yzw[2][2] = cr * cp;
#endif
	mat[0][0] = cy * cr;
	mat[0][1] = (cy * sr * sp + cp * sy);
	mat[0][2] = sy * sp - cy * cp * sr;

	mat[1][0] = -(cr * sy);
	mat[1][1] = -(sy * sr * sp -cy * cp);
	mat[1][2] = cp*sr*sy + sp*cy;

	mat[2][0] = sr;
	mat[2][1] = -(cr * sp);
	mat[2][2] = cr * cp;
	
	const float Hx = mat[0][0];
	const float My = mat[1][1];
	const float Az = mat[2][2];

	quat[0] = sqrtf(clamp(Hx - My - Az + 1) * 0.25f);
	quat[1] = sqrtf(clamp(-Hx + My - Az + 1) * 0.25f);
	quat[2] = sqrtf(clamp(-Hx - My + Az + 1) * 0.25f);
	quat[3] = sqrtf(clamp(Hx + My + Az + 1) * 0.25f);
	quat[0] = copysignf(quat[0], mat[2][1] - mat[1][2]);
	quat[1] = copysignf(quat[1], mat[0][2] - mat[2][0]);
	quat[2] = copysignf(quat[2], mat[1][0] - mat[0][1]);

	//float q_length =  sqrtf(quat[0] * quat[0] + quat[1] * quat[1] + quat[2] * quat[2] + quat[3] * quat[3]);
	//quat[0] = quat[0] / q_length;
	//quat[1] = quat[1] / q_length;
	//quat[2] = quat[2] / q_length;
	//quat[3] = quat[3] / q_length;
}


void qst_quat_to_matrix(float quat[4], float mat[][3])
{
    float q0 = quat[0];	// w
    float q1 = quat[1];	// x
    float q2 = quat[2];	// y
    float q3 = quat[3];	// z
    float sq_q1 = 2 * q1 * q1;
    float sq_q2 = 2 * q2 * q2;
    float sq_q3 = 2 * q3 * q3;
    float q1_q2 = 2 * q1 * q2;
    float q3_q0 = 2 * q3 * q0;
    float q1_q3 = 2 * q1 * q3;
    float q2_q0 = 2 * q2 * q0;
    float q2_q3 = 2 * q2 * q3;
    float q1_q0 = 2 * q1 * q0;

	mat[0][0] = 1 - sq_q2 - sq_q3;
	mat[0][1] = q1_q2 - q3_q0;
	mat[0][2] = q1_q3 + q2_q0;
	mat[1][0] = q1_q2 + q3_q0;
	mat[1][1] = 1 - sq_q1 - sq_q3;
	mat[1][2] = q2_q3 - q1_q0;
	mat[2][0] = q1_q3 - q2_q0;
	mat[2][1] = q2_q3 + q1_q0;
	mat[2][2] = 1 - sq_q1 - sq_q2;
}


#define YAW_AVG_FILTER

#ifdef YAW_AVG_FILTER
#define YAW_AVG_SIZE	8
static float filter_ori[YAW_AVG_SIZE] = { 0 };
static float filter_ori_temp[YAW_AVG_SIZE] = { 0 };
#endif
float qst_yaw_improve(float yaw)
{
#ifdef YAW_AVG_FILTER
	int i = 0;
	int j = 0;
	float	filter_ori_sum = 0;

	for(j = 0; j < (YAW_AVG_SIZE-1); j++)
	{
		filter_ori[j] = filter_ori[j + 1];

	}
	filter_ori[YAW_AVG_SIZE-1] = yaw;

	if(yaw >= 330.f || yaw <= 30.f)
	{
		for(i = 1; i <= (YAW_AVG_SIZE-1); i++)
		{
			double dtHead = filter_ori[0] - filter_ori[i];
			if(dtHead > 330.f)
				filter_ori_temp[i] = 360.f  + filter_ori[i];
			else if(dtHead <= -330.f)
				filter_ori_temp[i] = filter_ori[i] - 360.f;
			else
				filter_ori_temp[i] = filter_ori[i];
		}

		filter_ori_temp[0] =filter_ori[0];

		for (i = 0; i < YAW_AVG_SIZE; i++)
		{
			filter_ori_sum += filter_ori_temp[i];
		}
	}else{
		for (i = 0; i < YAW_AVG_SIZE; i++){
			filter_ori_sum += filter_ori[i];
		}
	}

	yaw = filter_ori_sum / YAW_AVG_SIZE;

	if(yaw >= 360.f)
		yaw = yaw - 360.f;
	else if(yaw < 0)
		yaw = yaw + 360.f;
#endif
	
#ifdef YAW_HOPE_DEGREE
	int hope =0;
	hope = (int)((yaw / 45) + 0.5) * 45;

	if(yaw >= (hope - 10) && yaw <= (hope + 10))
	   yaw = hope * 0.5 + 0.5 * yaw;
	else if(yaw >= (hope - 20) && yaw < (hope - 10))
	   yaw = (hope -5) - ((hope - 10) - yaw) * 1.5;
	else if(yaw > (hope + 10) && yaw < (hope + 20))
	   yaw = (hope + 5) + (yaw - (hope + 10)) * 1.5;
#endif

	return yaw;
}

void android_raw_to_euler(float acc[3], float mag[3], float euler[3])
{
#if 0
	float pitch,roll,yaw;
	float Ax,Ay,Az,av;
	float mMagData[3] = {0.0f};
	float hx,hy;
	float cp,sp,sr,cr;
	float normalize = sqrtf(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);

	Ax = acc[0]/normalize;
	Ay = acc[1]/normalize;
	Az = acc[2]/normalize;

	av = sqrtf(Ax*Ax + Ay*Ay + Az*Az);
	pitch = atan2f(Ay, Az);
	roll  = asinf(Ax / av);

	cp = cosf(pitch);
	sp = sinf(pitch);
	sr = sinf(roll);
	cr = cosf(roll);

	normalize = sqrtf(mag[0]*mag[0] + mag[1]*mag[1] + mag[2]*mag[2]);
	if(normalize > 1.0f)
	{
		mMagData[0] = mag[0] / normalize;
		mMagData[1] = mag[1] / normalize;
		mMagData[2] = mag[2] / normalize;
		hx = mMagData[0] * cr + mMagData[1] * sp*sr+mMagData[2] *(sp*sr+cp*sr);
		hy = mMagData[1] * cp - mMagData[2] * sp;
		yaw = -atan2f(hx, hy);
	}
	else
	{
		yaw = 0;
	}

	euler[0] = yaw * RADTODEG;
	euler[1] = -pitch * RADTODEG;
	euler[2] = roll * RADTODEG;
	if(euler[0] < 0)
	{
		euler[0] += 360.0f;
	}
#else
	float accel_data[3] = {0, 0, 0};
	float mag_data[3] = {0, 0, 0};
	
	float mat33_t_R[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

	accel_data[0] = acc[0];
	accel_data[1] = acc[1];
	accel_data[2] = acc[2];
	
	float	normalize = sqrtf(accel_data[0] * accel_data[0] + accel_data[1] * accel_data[1] + accel_data[2] * accel_data[2]);
	accel_data[0] = accel_data[0] / normalize;
	accel_data[1] = accel_data[1] / normalize;
	accel_data[2] = accel_data[2] / normalize;
	
	float pitch = -atan2f(accel_data[1], accel_data[2]);
	euler[1] = pitch * RADTODEG;
	
	normalize = sqrtf(accel_data[0] * accel_data[0] + accel_data[1] * accel_data[1] + accel_data[2] * accel_data[2]);

	float roll = asinf((accel_data[0] / normalize));
	euler[2] = roll * RADTODEG;
	
	float cp = cosf(pitch);   //roll=0,  pitch=0;      //1
	float sp = sinf(pitch);                                 //0
	float sr = sinf(roll);                                     //0
	float cr = cosf(roll);                                   //  1
	
	mag_data[0] = mag[0];
	mag_data[1] = mag[1];
	mag_data[2] = mag[2];
	
	normalize = sqrtf(mag_data[0] * mag_data[0] + mag_data[1] * mag_data[1] + mag_data[2] * mag_data[2]);
	mag_data[0] = mag_data[0] / normalize;
	mag_data[1] = mag_data[1] / normalize;
	mag_data[2] = mag_data[2] / normalize;

	float hx = mag_data[0] * cr - mag_data[2] * sr;
	float hy = mag_data[0] * sp * sr + mag_data[1] * cp + mag_data[2] * cr * sp;
	float yaw = -atan2f(hx, hy);
	
	euler[0] = yaw * RADTODEG;
	if(euler[0] < 0.0f)
	{
		euler[0] += 360.0f;
	}

//	euler[0] = fusion_9axis_init_yaw;
//	euler[1] = fusion_9axis_init_pitch;
//	euler[2] = fusion_9axis_init_roll;
#endif
    QLOG("qst_raw_to_euler %f %f %f\n",euler[0],euler[1],euler[2]);
}


