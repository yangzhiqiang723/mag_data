#include <stdio.h>
#include <math.h>
#include <string.h>
#include "MyFilter.h"
#include <io.h>
#include "conio.h"

#ifdef LOG_TAG
#undef LOG_TAG
#define LOG_TAG "qst-imucali"
#endif

#define QLOGD		_cprintf

#define CALI_ABS(X) 				((X) < 0.0f ? (-1.0f * (X)) : (X))
//#define CALI_FILE_NAME           "/storage/emulated/0/Documents/imu.txt"

enum
{
	IMU_CALI_FAIL = -1,
	IMU_CALI_ONGOING = 0,
	IMU_CALI_DONE = 1
};

typedef struct
{
    unsigned short		static_flag;
    unsigned short		index;
    float				sum[3];
    float				g_bias_auto[3];
} imu_auto_cali;

typedef struct
{
    float *a_bias;
    float *g_bias;
} imu_bias;

static QstStd3 gyr_std;
static imu_auto_cali gyr_cali;
static imu_bias cali_p = {NULL, NULL};

int qst_imu_cali_write_file(void)
{
#ifdef CALI_FILE_NAME
    FILE    *fp = NULL;
    int ret = 0;

	if((cali_p.a_bias==NULL)||(cali_p.g_bias == NULL))
	{
		QLOGD("qst_imu_cali_write_file oft ptr is NULL\n");
		return 0;
	}

    fp = fopen(CALI_FILE_NAME, "wt");
    if(fp){
        ret = fprintf(fp, "%f %f %f %f %f %f", cali_p.a_bias[0],cali_p.a_bias[1],cali_p.a_bias[2],cali_p.g_bias[0],cali_p.g_bias[1],cali_p.g_bias[2]);
        fclose(fp);
        QLOGD("qst_imu_cali_write_file ret=%d", ret);
        return 1;
    }
    else {
        QLOGD("qst_imu_cali_write_file fopen error!");
        return 0;
    }
#else
        return 0;
#endif
}

int qst_imu_cali_read_file(void)
{
#ifdef CALI_FILE_NAME
    int ret = 0;
    FILE    *fp = NULL;
    char buf[256];

	if((cali_p.a_bias==NULL)||(cali_p.g_bias == NULL))
	{
		QLOGD("qst_imu_cali_read_file oft ptr is NULL\n");
		return 0;
	}

    fp = fopen(CALI_FILE_NAME, "r");
    if(fp){
        memset(buf, 0, sizeof(buf));
        ret = fscanf(fp, "%f %f %f %f %f %f", &cali_p.a_bias[0], &cali_p.a_bias[1],&cali_p.a_bias[2],&cali_p.g_bias[0], &cali_p.g_bias[1], &cali_p.g_bias[2]);
        QLOGD("qst_imu_cali_read_file ret=%d [%f %f %f %f %f %f]", ret, cali_p.a_bias[0],cali_p.a_bias[1],cali_p.a_bias[2],cali_p.g_bias[0],cali_p.g_bias[1],cali_p.g_bias[2]);
        fclose(fp);
        return 1;
    }
    else{
        QLOGD("qst_imu_cali_write_file fopen error!");
        return 0;
    }
#else
    return 0;
#endif
}

void qst_imu_cali_init(float *a_bisa, float *g_bias)
{
    memset(&gyr_std, 0, sizeof(gyr_std));
    memset(&gyr_cali, 0, sizeof(gyr_cali));
    memset(&cali_p, 0, sizeof(cali_p));
	cali_p.a_bias = a_bisa;
	cali_p.g_bias = g_bias;
    qst_imu_cali_read_file();
}

#define MAX_CALI_COUNT			60
#define ACC_OFFSET_MAX			5.0f	// 500mg
#define GYR_OFFSET_MAX			0.55f	// about 30dps
#define ACC_DIFF_MAX			1.5f	// 150mg, acc
#define GYR_DIFF_MAX			0.2f	// 1dps, gyr
#define DISCARD_COUNT			100


static unsigned short cali_count = 0;
static unsigned discard_count = DISCARD_COUNT;

int qst_algo_acc_cali(float acc[3])
{
	static float acc_abs_min, acc_abs_max;
	static float acc_bias_sum[3];

	if(!cali_p.a_bias)
	{
		QLOGD("qst_algo_acc_cali cali_p.a_bias is NULL\n");
		return -1;
	}
	//QLOGD("qst_algo_acc_cali input: %f  %f  %f\n", acc[0], acc[1], acc[2]);
    if(cali_count == 0)
    {
		acc_abs_min = acc_abs_max = CALI_ABS(acc[0])+ CALI_ABS(acc[1])+ CALI_ABS(acc[2]);
        acc_bias_sum[0] = acc[0];
        acc_bias_sum[1] = acc[1];
        acc_bias_sum[2] = acc[2];
    }
    else if(cali_count < MAX_CALI_COUNT)
    {
    	float acc_abs_curr = CALI_ABS(acc[0])+ CALI_ABS(acc[1])+ CALI_ABS(acc[2]);

        acc_bias_sum[0] += acc[0];
        acc_bias_sum[1] += acc[1];
        acc_bias_sum[2] += acc[2];
		acc_abs_min = (acc_abs_min > acc_abs_curr) ? acc_abs_curr : acc_abs_min;
		acc_abs_max = (acc_abs_max < acc_abs_curr) ? acc_abs_curr : acc_abs_max;
    }
    else if(cali_count == MAX_CALI_COUNT)
    {
		float avg_acc[3] = {0.0f, 0.0f, 0.0f};
		float ref_value[3] = {0.0f, 0.0f, 9.807f};
		float abs_avg_acc[3];

		avg_acc[0] = acc_bias_sum[0] / (MAX_CALI_COUNT);
		avg_acc[1] = acc_bias_sum[1] / (MAX_CALI_COUNT);
		avg_acc[2] = acc_bias_sum[2] / (MAX_CALI_COUNT);

		if((CALI_ABS(acc_abs_max-acc_abs_min) > ACC_DIFF_MAX))
		{
			QLOGD("cali fail! device is not static! \n");
			return -1;
		}
		abs_avg_acc[0] = CALI_ABS(avg_acc[0]);
		abs_avg_acc[1] = CALI_ABS(avg_acc[1]);
		abs_avg_acc[2] = CALI_ABS(avg_acc[2]);

		if(abs_avg_acc[2] > (abs_avg_acc[0]+abs_avg_acc[1]))
		{
			ref_value[0] = 0.0f;
			ref_value[1] = 0.0f;
			if(avg_acc[2] > 0)
				ref_value[2] = 9.807f;
			else
				ref_value[2] = -9.807f;
		}
		else if(abs_avg_acc[0] > (abs_avg_acc[1]+abs_avg_acc[2]))
		{
			ref_value[1] = 0.0f;
			ref_value[2] = 0.0f;
			if(avg_acc[0] > 0)
				ref_value[0] = 9.807f;
			else
				ref_value[0] = -9.807f;
		}
		else if(abs_avg_acc[1] > (abs_avg_acc[0]+abs_avg_acc[2]))
		{
			ref_value[0] = 0.0f;
			ref_value[2] = 0.0f;
			if(avg_acc[1] > 0)
				ref_value[1] = 9.807f;
			else
				ref_value[1] = -9.807f;
		}

        cali_p.a_bias[0] = (ref_value[0]-avg_acc[0]);
        cali_p.a_bias[1] = (ref_value[1]-avg_acc[1]);
        cali_p.a_bias[2] = (ref_value[2]-avg_acc[2]);

		QLOGD("acc offset: %f %f %f\n", cali_p.a_bias[0],cali_p.a_bias[1],cali_p.a_bias[2]);

		if((CALI_ABS(cali_p.a_bias[0])>ACC_OFFSET_MAX) ||(CALI_ABS(cali_p.a_bias[1])>ACC_OFFSET_MAX)||(CALI_ABS(cali_p.a_bias[2])>ACC_OFFSET_MAX))
		{		
			cali_p.a_bias[0] = cali_p.a_bias[1] = cali_p.a_bias[2] = 0.0f;
			QLOGD("cali fail! Acc Offset too large! \n");
			return -1;
		}

        return 1;
    }

    return 0;
}

int qst_algo_gyr_cali(float gyr[3])
{
	static float gyr_abs_min, gyr_abs_max;
	static float gyr_bias_sum[3];

	if(!cali_p.g_bias)
	{	
		QLOGD("qst_algo_gyr_cali cali_p.g_bias is NULL\n");
		return -1;
	}

	//QLOGD("qst_algo_gyr_cali input: %f  %f  %f\n", gyr[0], gyr[1], gyr[2]);
    if(cali_count == 0)
    {
		gyr_abs_max = gyr_abs_min = CALI_ABS(gyr[0])+ CALI_ABS(gyr[1])+ CALI_ABS(gyr[2]);
        gyr_bias_sum[0] = gyr[0];
        gyr_bias_sum[1] = gyr[1];
        gyr_bias_sum[2] = gyr[2];
    }
    else if(cali_count < MAX_CALI_COUNT)
    {
    	float gyr_abs_curr = CALI_ABS(gyr[0])+ CALI_ABS(gyr[1])+ CALI_ABS(gyr[2]);

        gyr_bias_sum[0] += gyr[0];
        gyr_bias_sum[1] += gyr[1];
        gyr_bias_sum[2] += gyr[2];
		gyr_abs_min = (gyr_abs_min > gyr_abs_curr) ? gyr_abs_curr : gyr_abs_min;
		gyr_abs_max = (gyr_abs_max < gyr_abs_curr) ? gyr_abs_curr : gyr_abs_max;
    }
    else if(cali_count == MAX_CALI_COUNT)
    {
		float avg_gyro[3] = {0.0f, 0.0f, 0.0f};

		avg_gyro[0] = gyr_bias_sum[0] / (MAX_CALI_COUNT);
		avg_gyro[1] = gyr_bias_sum[1] / (MAX_CALI_COUNT);
		avg_gyro[2] = gyr_bias_sum[2] / (MAX_CALI_COUNT);

		if((CALI_ABS(gyr_abs_max-gyr_abs_min) > GYR_DIFF_MAX))
		{
			QLOGD("cali fail! device is not static! \n");
			return -1;
		}

        cali_p.g_bias[0] = (0.0f - avg_gyro[0]);
        cali_p.g_bias[1] = (0.0f - avg_gyro[1]);
        cali_p.g_bias[2] = (0.0f - avg_gyro[2]);

		QLOGD("gyro offset: %f %f %f\n", cali_p.g_bias[0],cali_p.g_bias[1],cali_p.g_bias[2]);

		if((CALI_ABS(cali_p.g_bias[0])>GYR_OFFSET_MAX) ||(CALI_ABS(cali_p.g_bias[1])>GYR_OFFSET_MAX)||(CALI_ABS(cali_p.g_bias[2])>GYR_OFFSET_MAX))
		{		
			cali_p.g_bias[0] = cali_p.g_bias[1] = cali_p.g_bias[2] = 0.0f;
			QLOGD("cali fail! Gyr Offset too large! \n");
			return -1;
		}

        return 1;
    }

    return 0;
}

int qst_algo_imu_cali(float acc[3], float gyr[3])
{
	int acc_ret = 0;
	int gyr_ret = 0;
	// reset parameter
	if (discard_count > 0)
	{
		discard_count--;
		QLOGD("imu cali sample discard\n");
		return IMU_CALI_ONGOING;
	}

	acc_ret = qst_algo_acc_cali(acc);
	gyr_ret = qst_algo_gyr_cali(gyr);
	if ((acc_ret == IMU_CALI_DONE) && (gyr_ret == IMU_CALI_DONE))
	{
		cali_count = 0;
		discard_count = DISCARD_COUNT;
		return IMU_CALI_DONE;
	}
	else if ((acc_ret == IMU_CALI_ONGOING) && (gyr_ret == IMU_CALI_ONGOING))
	{
		cali_count++;
		return IMU_CALI_ONGOING;
	}
	else
	{
		cali_count = 0;
		discard_count = DISCARD_COUNT;
		return IMU_CALI_FAIL;
	}
}

void qst_imu_auto_cali(float d[3])
{
#define QST_IMU_CALI_COUNT			100
	float std[3];

	getStandardDeviation3(&gyr_std, d, std);	// < 0.005f
	if((std[0] < 0.005f)&&(std[1] < 0.005f)&&(std[2] < 0.005f))
	{
		if(gyr_cali.index == 0)
		{
			gyr_cali.sum[0] = 0;
			gyr_cali.sum[1] = 0;
			gyr_cali.sum[2] = 0;
		}
		if(gyr_cali.index < QST_IMU_CALI_COUNT)
		{
			gyr_cali.sum[0] += d[0];
			gyr_cali.sum[1] += d[1];
			gyr_cali.sum[2] += d[2];
			gyr_cali.index++;
			if(gyr_cali.index >= QST_IMU_CALI_COUNT)
			{
				// update avg
                gyr_cali.g_bias_auto[0] = gyr_cali.sum[0]/QST_IMU_CALI_COUNT;
                gyr_cali.g_bias_auto[1] = gyr_cali.sum[1]/QST_IMU_CALI_COUNT;
                gyr_cali.g_bias_auto[2] = gyr_cali.sum[2]/QST_IMU_CALI_COUNT;
				gyr_cali.static_flag = 1;
                // write bias to fs
				QLOGD("gyro auto cali bias: %f %f %f\n", gyr_cali.g_bias_auto[0], gyr_cali.g_bias_auto[1], gyr_cali.g_bias_auto[2]);
                qst_imu_cali_write_file();
                // write bias to fs
			}
		}
	}
	else
	{
		gyr_cali.index = 0;
		gyr_cali.static_flag = 0;
		gyr_cali.sum[0] = 0;
		gyr_cali.sum[1] = 0;
		gyr_cali.sum[2] = 0;
	}
	// remove offset 
	d[0] = d[0] - gyr_cali.g_bias_auto[0];
	d[1] = d[1] - gyr_cali.g_bias_auto[1];
	d[2] = d[2] - gyr_cali.g_bias_auto[2];
    //QLOGD("[%d]gyro_std[%f  %f  %f]data[%f  %f  %f]",gyr_cali.static_flag,std[0],std[1],std[2],d[0],d[1],d[2]);
#if 0
	if(gyr_cali.static_flag)
	{
		//QLOGD("gyr-static	");
		d[0] = d[0]*0.1f;
		d[1] = d[1]*0.1f;
		d[2] = d[2]*0.1f;
	}
#endif
}


