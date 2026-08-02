#include "stdafx.h"
#include <io.h>
#include "conio.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "imu_soft_st.h"

#define LOGD		_cprintf

static QstStd6 q_std6;
//static QstStd3 q_std3;
static qst_gyro_motion_t gyro_motion;
static acc_to_euler_t acc_euler;

void imu_soft_st_init(void)
{
	memset(&q_std6, 0, sizeof(q_std6));
	//memset(&q_std3, 0, sizeof(q_std3));
	memset(&gyro_motion, 0, sizeof(gyro_motion));
	memset(&acc_euler, 0, sizeof(acc_euler));
}

int imu_static_cali(float acc[3],float gyro[3],float a_bias[3],float g_bias[3],int side)
{
	static int cali_discard_count = 10;
	static int	cali_count = 0;
	static float acc_abs_min, acc_abs_max;
	static float acc_bias_sum[3], gyr_bias_sum[3];

	if(cali_discard_count > 0)
	{
		cali_discard_count--;
		//LOGD("sample discard\n");
		return 0;
	}
	//LOGD("imu_cali input: acc:%f  %f  %f gyro:%f  %f  %f\n", acc[0], acc[1], acc[2], gyro[0],gyro[1],gyro[2]);
    if(cali_count == 0)
    {
		acc_abs_min = acc_abs_max = QST_ABS(acc[0])+ QST_ABS(acc[1])+ QST_ABS(acc[2]);
        acc_bias_sum[0] = acc[0];
        acc_bias_sum[1] = acc[1];
        acc_bias_sum[2] = acc[2];
        gyr_bias_sum[0] = gyro[0];
        gyr_bias_sum[1] = gyro[1];
        gyr_bias_sum[2] = gyro[2];

        cali_count++;
    }
    else if(cali_count < MAX_CALI_COUNT)
    {
    	float acc_abs_curr = QST_ABS(acc[0])+ QST_ABS(acc[1])+ QST_ABS(acc[2]);

        acc_bias_sum[0] += acc[0];
        acc_bias_sum[1] += acc[1];
        acc_bias_sum[2] += acc[2];
        gyr_bias_sum[0] += gyro[0];
        gyr_bias_sum[1] += gyro[1];
        gyr_bias_sum[2] += gyro[2];
		acc_abs_min = (acc_abs_min > acc_abs_curr) ? acc_abs_curr : acc_abs_min;
		acc_abs_max = (acc_abs_max < acc_abs_curr) ? acc_abs_curr : acc_abs_max;

        cali_count++;
    }
    else if(cali_count == MAX_CALI_COUNT)
    {
		float avg_acc[3] = {0.0, 0.0, 0.0};
		float avg_gyro[3] = {0.0, 0.0, 0.0};
		//float abs_avg_acc_z;

		// reset parameter
		cali_count = 0;
		cali_discard_count = 10;
		// reset parameter
		avg_acc[0] = acc_bias_sum[0] / (MAX_CALI_COUNT);
		avg_acc[1] = acc_bias_sum[1] / (MAX_CALI_COUNT);
		avg_acc[2] = acc_bias_sum[2] / (MAX_CALI_COUNT);
		avg_gyro[0] = gyr_bias_sum[0] / (MAX_CALI_COUNT);
		avg_gyro[1] = gyr_bias_sum[1] / (MAX_CALI_COUNT);
		avg_gyro[2] = gyr_bias_sum[2] / (MAX_CALI_COUNT);

		if((QST_ABS(acc_abs_max-acc_abs_min) > ACC_DIFF_MAX))
		{
			LOGD("calibration fail! device is not static! \n");
			return -2;
		}

		//abs_avg_acc_z = QST_ABS(avg_acc[2]);
		
		a_bias[0] = (0.0f-avg_acc[0]);
        a_bias[1] = (0.0f-avg_acc[1]);
		if(side)
		{
			a_bias[2] = (9.807f-avg_acc[2]);
		}
		else
		{
			a_bias[2] = (-9.807f-avg_acc[2]);
		}

        g_bias[0] = (0.0f-avg_gyro[0]);
        g_bias[1] = (0.0f-avg_gyro[1]);
        g_bias[2] = (0.0f-avg_gyro[2]);
//		LOGD("calibration done: acc:%f %f %f gyro:%f %f %f\n", a_bias[0],a_bias[1],a_bias[2],g_bias[0],g_bias[1],g_bias[2]);

		if((QST_ABS(a_bias[0])>ACC_OFFSET_MAX) ||(QST_ABS(a_bias[1])>ACC_OFFSET_MAX)||(QST_ABS(a_bias[2])>ACC_OFFSET_MAX))
		{
//			LOGD("calibration fail! Acc Offset too large! \n");
			return 1;	//-1;
		}
		if((QST_ABS(g_bias[0])>GYR_OFFSET_MAX) ||(QST_ABS(g_bias[1])>GYR_OFFSET_MAX)||(QST_ABS(g_bias[2])>GYR_OFFSET_MAX))
		{
//			LOGD("calibration fail! Gyr Offset too large! \n");
			return 1;	//-1;
		}

        return 1;
    }

    return 0;
}


void getStandardDeviation6(float in[6], float out[6])
{
	float avg_1,avg_2,avg_3,avg_4,avg_5,avg_6;
	float toatl_1,toatl_2,toatl_3,toatl_4,toatl_5,toatl_6;
	short index = 0;

	if(q_std6.size < STD_ARRAY_SIZE)
	{
		index = q_std6.size;
		if(index == 0)
		{
			q_std6.sum_1=q_std6.sum_2=q_std6.sum_3=q_std6.sum_4=q_std6.sum_5=q_std6.sum_6 = 0.0f;
			q_std6.index = 0;
		}
		q_std6.data_1[index] = in[0];
		q_std6.data_2[index] = in[1];
		q_std6.data_3[index] = in[2];
		q_std6.data_4[index] = in[3];
		q_std6.data_5[index] = in[4];
		q_std6.data_6[index] = in[5];
		q_std6.sum_1 += in[0];
		q_std6.sum_2 += in[1];
		q_std6.sum_3 += in[2];		
		q_std6.sum_4 += in[3];
		q_std6.sum_5 += in[4];
		q_std6.sum_6 += in[5];
		q_std6.size++;
		out[0] = 0.0f;
		out[1] = 0.0f;
		out[2] = 0.0f;
		out[3] = 0.0f;
		out[4] = 0.0f;
		out[5] = 0.0f;
	}
	else
	{
		index = q_std6.index;
		q_std6.sum_1 = q_std6.sum_1 - q_std6.data_1[index] + in[0];
		q_std6.sum_2 = q_std6.sum_2 - q_std6.data_2[index] + in[1];
		q_std6.sum_3 = q_std6.sum_3 - q_std6.data_3[index] + in[2];
		q_std6.sum_4 = q_std6.sum_4 - q_std6.data_4[index] + in[3];
		q_std6.sum_5 = q_std6.sum_5 - q_std6.data_5[index] + in[4];
		q_std6.sum_6 = q_std6.sum_6 - q_std6.data_6[index] + in[5];

		q_std6.data_1[index] = in[0];
		q_std6.data_2[index] = in[1];
		q_std6.data_3[index] = in[2];
		q_std6.data_4[index] = in[3];
		q_std6.data_5[index] = in[4];
		q_std6.data_6[index] = in[5];

		q_std6.index = (++q_std6.index%STD_ARRAY_SIZE);
		avg_1 = q_std6.sum_1/STD_ARRAY_SIZE;
		avg_2 = q_std6.sum_2/STD_ARRAY_SIZE;
		avg_3 = q_std6.sum_3/STD_ARRAY_SIZE;
		avg_4 = q_std6.sum_4/STD_ARRAY_SIZE;
		avg_5 = q_std6.sum_5/STD_ARRAY_SIZE;
		avg_6 = q_std6.sum_6/STD_ARRAY_SIZE;
		toatl_1 = toatl_2 = toatl_3 = toatl_4 = toatl_5 = toatl_6 = 0.0f;

		for(int i = 0; i < STD_ARRAY_SIZE; i++)
		{
			toatl_1 += (q_std6.data_1[i] - avg_1) * (q_std6.data_1[i] - avg_1);
			toatl_2 += (q_std6.data_2[i] - avg_2) * (q_std6.data_2[i] - avg_2);
			toatl_3 += (q_std6.data_3[i] - avg_3) * (q_std6.data_3[i] - avg_3);
			toatl_4 += (q_std6.data_4[i] - avg_4) * (q_std6.data_4[i] - avg_4);
			toatl_5 += (q_std6.data_5[i] - avg_5) * (q_std6.data_5[i] - avg_5);
			toatl_6 += (q_std6.data_6[i] - avg_6) * (q_std6.data_6[i] - avg_6);
		}

		out[0] = (float)sqrt(toatl_1 / (STD_ARRAY_SIZE));
		out[1] = (float)sqrt(toatl_2 / (STD_ARRAY_SIZE));
		out[2] = (float)sqrt(toatl_3 / (STD_ARRAY_SIZE));
		out[3] = (float)sqrt(toatl_4 / (STD_ARRAY_SIZE));
		out[4] = (float)sqrt(toatl_5 / (STD_ARRAY_SIZE));
		out[5] = (float)sqrt(toatl_6 / (STD_ARRAY_SIZE));
	}
}

#if 0
void getStandardDeviation3(float in[3], float out[3])
{
	float avg_x,avg_y,avg_z;
	float toatl_x,toatl_y,toatl_z;
	short index;

	if(q_std3.size < STD_ARRAY_SIZE)
	{
		index = q_std3.size;		
		if(index == 0)
		{
			q_std3.sum_x=q_std3.sum_y=q_std3.sum_z=0.0f;
			q_std3.index = 0;
		}
		q_std3.data_x[index] = in[0];
		q_std3.data_y[index] = in[1];
		q_std3.data_z[index] = in[2];
		q_std3.sum_x += in[0];
		q_std3.sum_y += in[1];
		q_std3.sum_z += in[2];
		q_std3.size++;
		out[0] = 0.0f;
		out[1] = 0.0f;
		out[2] = 0.0f;
	}
	else
	{
		index = q_std3.index;
		q_std3.sum_x = q_std3.sum_x - q_std3.data_x[index] + in[0];
		q_std3.sum_y = q_std3.sum_y - q_std3.data_y[index] + in[1];
		q_std3.sum_z = q_std3.sum_z - q_std3.data_z[index] + in[2];
		q_std3.data_x[index] = in[0];
		q_std3.data_y[index] = in[1];
		q_std3.data_z[index] = in[2];
		q_std3.index = (++q_std3.index%STD_ARRAY_SIZE);
		avg_x = q_std3.sum_x/STD_ARRAY_SIZE;
		avg_y = q_std3.sum_y/STD_ARRAY_SIZE;
		avg_z = q_std3.sum_z/STD_ARRAY_SIZE;
		toatl_x = toatl_y = toatl_z = 0.0f;

		for(int i = 0; i < STD_ARRAY_SIZE; i++)
		{
			toatl_x += (q_std3.data_x[i] - avg_x) * (q_std3.data_x[i] - avg_x);
			toatl_y += (q_std3.data_y[i] - avg_y) * (q_std3.data_y[i] - avg_y);
			toatl_z += (q_std3.data_z[i] - avg_z) * (q_std3.data_z[i] - avg_z);
		}

		out[0] = (float)sqrt(toatl_x / (STD_ARRAY_SIZE));
		out[1] = (float)sqrt(toatl_y / (STD_ARRAY_SIZE));
		out[2] = (float)sqrt(toatl_z / (STD_ARRAY_SIZE));
	}
}
#endif

int imu_static_soft_st(float imu[6])
{
	int cali_ret = 0;
	float bias[6];
	float std[6];
	float bw_sq = sqrtf(IMU_BANDWIDTH);	// sqrt(bandwith)

	cali_ret = imu_static_cali(&imu[0],&imu[3],&bias[0],&bias[3],1);
	getStandardDeviation6(&imu[0], &std[0]);
	std[0] = (std[0]*1000000.0f)/(9.807f*bw_sq);
	std[1] = (std[1]*1000000.0f)/(9.807f*bw_sq);
	std[2] = (std[2]*1000000.0f)/(9.807f*bw_sq);
	std[3] = (std[3]*1000.0f*57.3f)/(bw_sq);
	std[4] = (std[4]*1000.0f*57.3f)/(bw_sq);
	std[5] = (std[5]*1000.0f*57.3f)/(bw_sq);

	if(cali_ret == 1)
	{
		char acc_noise_flag = 0;
		char gyr_noise_flag = 0;

		LOGD("qmi8658 cali OK\n");
		LOGD("offset	[%f	%f	%f]	[%f	%f	%f]\n",bias[0],bias[1],bias[2],bias[3],bias[4],bias[5]);	
		LOGD("noise	[%f	%f	%f]	[%f	%f	%f]\n",std[0],std[1],std[2],std[3],std[4],std[5]);
		// acc
		if((std[0]>ACC_STD_THRESHOLD)||(std[1]>ACC_STD_THRESHOLD)||(std[2]>ACC_STD_THRESHOLD))
		{
			LOGD("accel noise too large!\n");
			acc_noise_flag = 0;
		}
		else if((std[0]<ZERO_VALUE)||(std[1]<ZERO_VALUE)||(std[2]<ZERO_VALUE))
		{
			LOGD("accel noise too small!\n");
			acc_noise_flag = 0;
		}
		else
		{
			acc_noise_flag = 1;
		}
		// acc
		
		// gyr
		if((std[3]>GYR_STD_THRESHOLD)||(std[4]>GYR_STD_THRESHOLD)||(std[5]>GYR_STD_THRESHOLD))
		{
			LOGD("gyro noise too large!\n");
			gyr_noise_flag = 0;
		}		
		else if((std[3]<ZERO_VALUE)||(std[4]<ZERO_VALUE)||(std[5]<ZERO_VALUE))
		{
			LOGD("gyro noise too small!\n");
			gyr_noise_flag = 0;
		}
		else
		{
			gyr_noise_flag = 1;
		}
		// gyr

		if(acc_noise_flag && gyr_noise_flag)
		{
			LOGD("static selftest PASS!\n");
			return QMI8658_SOFT_ST_PASS;
		}
		else
		{
			LOGD("static selftest FAIL!\n");
			return QMI8658_SOFT_ST_FAIL;
		}
	}
	else if(cali_ret == -1)
	{
		LOGD("qmi8658 cali fail, static selftest FAIL!\n");
		return QMI8658_SOFT_ST_FAIL;
	}
	else
	{
		//LOGD("qmi8658 cali ongoing\n");
		return QMI8658_SOFT_ST_ONGOING;
	}
}


int imu_motion_soft_st(float imu[6])
{	
	gyro_motion.count++;
	if(gyro_motion.count == 1)
	{
		gyro_motion.gyo_x_max = gyro_motion.gyo_x_min = imu[3];
		gyro_motion.gyo_y_max = gyro_motion.gyo_y_min = imu[4];
		gyro_motion.gyo_z_max = gyro_motion.gyo_z_min = imu[5];

		return QMI8658_SOFT_ST_ONGOING;
	}
	else
	{
		if(imu[3]>gyro_motion.gyo_x_max)
		{
			gyro_motion.gyo_x_max = imu[3];
		}
		else if(imu[3]<gyro_motion.gyo_x_min)
		{
			gyro_motion.gyo_x_min = imu[3];
		}

		if(imu[4]>gyro_motion.gyo_y_max)
		{
			gyro_motion.gyo_y_max = imu[4];
		}
		else if(imu[4]<gyro_motion.gyo_y_min)
		{
			gyro_motion.gyo_y_min = imu[4];
		}
		
		if(imu[5]>gyro_motion.gyo_z_max)
		{
			gyro_motion.gyo_z_max = imu[5];
		}
		else if(imu[5]<gyro_motion.gyo_z_min)
		{
			gyro_motion.gyo_z_min = imu[5];
		}

		if(gyro_motion.count >= MOTION_DATA_COUNT)
		{
			float gyr_x_diff,gyr_y_diff,gyr_z_diff;

			gyr_x_diff = QST_ABS(gyro_motion.gyo_x_max-gyro_motion.gyo_x_min);
			gyr_y_diff = QST_ABS(gyro_motion.gyo_y_max-gyro_motion.gyo_y_min);
			gyr_z_diff = QST_ABS(gyro_motion.gyo_z_max-gyro_motion.gyo_z_min);

			LOGD("qmi8658 motion st, diff[%f	%f	%f]\n",gyr_x_diff,gyr_y_diff,gyr_z_diff);
#if 0
			if((gyr_x_diff>MOTION_DIFF_THRESHOLD)&&(gyr_y_diff>MOTION_DIFF_THRESHOLD)&&(gyr_z_diff>MOTION_DIFF_THRESHOLD))
#else
			if((gyr_z_diff>MOTION_DIFF_THRESHOLD))		// qst0103 test axis-z only
#endif
			{
				LOGD("imu_motion_soft_st PASS\n");
				return QMI8658_SOFT_ST_PASS;
			}
			else
			{
				LOGD("imu_motion_soft_st FAIL\n");
				return QMI8658_SOFT_ST_FAIL;
			}
		}
		else
		{
			return QMI8658_SOFT_ST_ONGOING;
		}
	}
}


int imu_acc_to_euler_st(float imu[6])
{
	int cali_ret = 0;
	float acc[3];
	float norm = 0;

	switch(acc_euler.status)
	{
		case 0:				// static
			cali_ret = imu_static_cali(&imu[0], &imu[3], &acc_euler.bias[0], &acc_euler.bias[3], 1);
			if(cali_ret == 1)
			{
				acc[0] = imu[0] + acc_euler.bias[0];
				acc[1] = imu[1] + acc_euler.bias[1];
				acc[2] = imu[2] + acc_euler.bias[2];
				norm = sqrtf(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);

				acc_euler.pitch_1 = (atan2f(acc[1],acc[2]))*180.0f/3.1415926f;
				acc_euler.roll_1 = (asinf(acc[0]/norm))*180.0f/3.1415926f;
				acc_euler.status = 1;
				LOGD("imu_acc_to_euler_st static cali done! pitch[%f] roll[%f]\n",acc_euler.pitch_1,acc_euler.roll_1);
				LOGD("!!!Star to tilt !!!\n");

				return QMI8658_SOFT_ST_ONGOING;
			}
			else
			{
				return cali_ret;
			}
			break;
		case 1:				// tilt	
			acc_euler.count++;
			if(acc_euler.count <= ACC_TO_EULER_TILT_NUM)
			{
				acc[0] = imu[0] + acc_euler.bias[0];
				acc[1] = imu[1] + acc_euler.bias[1];
				acc[2] = imu[2] + acc_euler.bias[2];
				norm = sqrtf(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);

				acc_euler.pitch_2 = (atan2f(acc[1],acc[2]))*180.0f/3.1415926f;
				acc_euler.roll_2 = (asinf(acc[0]/norm))*180.0f/3.1415926f;
				//LOGD("imu_acc_to_euler_st pitch[%f] roll[%f]\n",acc_euler.pitch_2,acc_euler.roll_2);
				if(QST_ABS(acc_euler.pitch_2-acc_euler.pitch_1) > acc_euler.pitch_diff)
				{
					acc_euler.pitch_diff = QST_ABS(acc_euler.pitch_2-acc_euler.pitch_1);
				}
				if(QST_ABS(acc_euler.roll_2-acc_euler.roll_1) > acc_euler.roll_diff)
				{
					acc_euler.roll_diff = QST_ABS(acc_euler.roll_2-acc_euler.roll_1);
				}				
			}
			else
			{
				LOGD("imu_acc_to_euler_st tilt result pitch[%f] roll[%f]\n",acc_euler.pitch_diff,acc_euler.roll_diff);
				if((acc_euler.pitch_diff > ACC_TO_EULER_TILT_DIFF) && (acc_euler.roll_diff >  ACC_TO_EULER_TILT_DIFF))
				{
					return QMI8658_SOFT_ST_PASS;
				}
				else
				{
					return QMI8658_SOFT_ST_FAIL;
				}
			}

			break;
		default:
			break;
	}
	
	return QMI8658_SOFT_ST_ONGOING;
}

