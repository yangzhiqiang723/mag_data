

#ifndef IMU_SOFT_ST_H_
#define IMU_SOFT_ST_H_

#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

#define QST_ABS(X) 				((X) < 0.0f ? (-1.0f * (X)) : (X))

#define MAX_CALI_COUNT			200		// number of data for calibration
#define STD_ARRAY_SIZE			32		// min number of data for Computational noise
#define ACC_OFFSET_MAX			5.0f	// 500mg
#define GYR_OFFSET_MAX			1.2f	// about 68dps
#define ACC_DIFF_MAX			1.5f	// 150mg, acc 差值超过150mg，认为设备未静止，或者数据异常

//#define ACC_STD_THRESHOLD		0.030f	// variance, about 3mg, (uint m/s2)
//#define GYR_STD_THRESHOLD		0.0035f	// variance, about 20mdps, (uint rad/s)
//#define ACC_STD_THRESHOLD		0.040f	// variance, about 4mg, (uint m/s2)
//#define GYR_STD_THRESHOLD		0.006f	// variance, about 35mdps, (uint rad/s)

#define ACC_STD_THRESHOLD		450.0f	// uG/sqrt(bw)
#define GYR_STD_THRESHOLD		45.0f	// mdps/sqrt(bw)

#define IMU_BANDWIDTH			(125/4)		//62.5f	//

#define MOTION_DATA_COUNT			300		// (20ms调度),如果其他调度时间，请自行修改
#define MOTION_DIFF_THRESHOLD		5.0f	// (uint rad/s)

//#define ACC_TO_EULER_STATIC_NUM		30		// 计算初始欧拉角需要的次数，约1秒静止
#define ACC_TO_EULER_TILT_NUM		500		// 计算倾斜欧拉角需要的次数，俯仰角，翻滚角,(20ms调度)如果其他调度时间，请自行修改
#define ACC_TO_EULER_TILT_DIFF		15		// 倾斜角度阈值，超过这个阈值认为acc OK

#define ZERO_VALUE					1.0f	//(float)1e-6

#define QMI8658_SOFT_ST_FAIL		-1
#define QMI8658_SOFT_ST_ONGOING		0
#define QMI8658_SOFT_ST_PASS		1

typedef struct
{
	unsigned int	count;
	float			gyo_x_min;
	float			gyo_x_max;
	float			gyo_y_min;
	float			gyo_y_max;
	float			gyo_z_min;
	float			gyo_z_max;
}qst_gyro_motion_t;

typedef struct
{
	unsigned int	count;
	unsigned int	status;
	float			bias[6];
	float			pitch_1;
	float			roll_1;
	float			pitch_2;
	float			roll_2;
	float			pitch_diff;
	float			roll_diff;
}acc_to_euler_t;

typedef struct
{
	float	data_x[STD_ARRAY_SIZE];
	float	data_y[STD_ARRAY_SIZE];
	float	data_z[STD_ARRAY_SIZE];
	float	sum_x;
	float	sum_y;
	float	sum_z;
	short	size;
	short	index;
}QstStd3;

typedef struct
{
	float	data_1[STD_ARRAY_SIZE];
	float	data_2[STD_ARRAY_SIZE];
	float	data_3[STD_ARRAY_SIZE];
	float	data_4[STD_ARRAY_SIZE];
	float	data_5[STD_ARRAY_SIZE];
	float	data_6[STD_ARRAY_SIZE];
	float	sum_1;
	float	sum_2;
	float	sum_3;
	float	sum_4;
	float	sum_5;
	float	sum_6;
	short	size;
	short	index;
}QstStd6;


extern void imu_soft_st_init(void);
extern int imu_static_cali(float acc[3],float gyro[3],float a_bias[3],float g_bias[3],int side);
extern void getStandardDeviation6(float in[6], float out[6]);
//extern void getStandardDeviation3(float in[3], float out[3]);
extern int imu_static_soft_st(float imu[6]);
extern int imu_motion_soft_st(float imu[6]);
extern int imu_acc_to_euler_st(float imu[6]);

#ifdef __cplusplus
}
#endif

#endif /* IMU_SOFT_ST_H_ */
