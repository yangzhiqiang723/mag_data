#include <stdio.h>
#include <stdlib.h>
//#include <errno.h>
#include <math.h>
#include <string.h>

//#include "butterworth.h"
#include "dummyGyro.h"
#include "QstAlgoLog.h"

//#define DUMMYGYRO_LOG

#ifdef DUMMYGYRO_LOG
#undef LOG_TAG
#define LOG_TAG "qst-dummy-g"
#define	DGLOG		LOGD
#else
#define	DGLOG(...)
#endif

typedef struct
{
	char	init;
	short	size;
	float	*x;
	float	*y;
	float	*z;
} Axis3Filter_t;

#define THRESH_ZERO_X		8.0f
#define THRESH_ZERO_Y		8.0f
#define THRESH_ZERO_Z		8.0f

//#define MAG_BTWZ_FILTER
#define ACC_FILTER_SIZE		30
#define MAG_FILTER_SIZE		30
#define GYR_FILTER_SIZE		30
#if ACC_FILTER_SIZE
static float acc_buf_x[ACC_FILTER_SIZE];
static float acc_buf_y[ACC_FILTER_SIZE];
static float acc_buf_z[ACC_FILTER_SIZE];
static Axis3Filter_t	g_acc_f;
#endif
#if MAG_FILTER_SIZE
static float mag_buf_x[MAG_FILTER_SIZE];
static float mag_buf_y[MAG_FILTER_SIZE];
static float mag_buf_z[MAG_FILTER_SIZE];
static Axis3Filter_t	g_mag_f;
#endif
#if GYR_FILTER_SIZE
static float gyr_buf_x[GYR_FILTER_SIZE];
static float gyr_buf_y[GYR_FILTER_SIZE];
static float gyr_buf_z[GYR_FILTER_SIZE];
static Axis3Filter_t	g_gyr_f;
#endif

static int64_t timeStamp_start = 0;
static int64_t timeStamp_old = 0;
static float Vgyro_last[3] = {0, 0, 0};
static char	qst_filter_init_flag = 0;

static struct virtual_gyro_matrix matrix_old = {{
    {1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f}
    }};
static struct virtual_gyro_matrix matrix_new = {{
    {1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f}
    }};
static struct virtual_gyro_matrix qst_matrix_transpose = {{
    {1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f}
    }};


static void qst_axis3_filter_init(void)
{
#if ACC_FILTER_SIZE
	g_acc_f.init = 0;
	g_acc_f.size = ACC_FILTER_SIZE;
	g_acc_f.x = acc_buf_x;
	g_acc_f.y = acc_buf_y;
	g_acc_f.z = acc_buf_z;
#endif
#if MAG_FILTER_SIZE
	g_mag_f.init = 0;
	g_mag_f.size = MAG_FILTER_SIZE;
	g_mag_f.x = mag_buf_x;
	g_mag_f.y = mag_buf_y;
	g_mag_f.z = mag_buf_z;
#endif
#if GYR_FILTER_SIZE
	g_gyr_f.init = 0;
	g_gyr_f.size = GYR_FILTER_SIZE;
	g_gyr_f.x = gyr_buf_x;
	g_gyr_f.y = gyr_buf_y;
	g_gyr_f.z = gyr_buf_z;
#endif
#if defined(MAG_BTWZ_FILTER)
    qst_btwz_init();
#endif
}

static void qst_axis3_filter_run(Axis3Filter_t *filter, float in[3], float out[3])
{
	int i;
	float sum[3] = {0, 0, 0};

	if(filter->init == 0)
	{
		for(i=0; i<filter->size; i++)
		{
			filter->x[i] = in[0];
			filter->y[i] = in[1];
			filter->z[i] = in[2];
		}
		filter->init = 1;
	}

	sum[0] = 0.0f;
	sum[1] = 0.0f;
	sum[2] = 0.0f;
	for(i=1; i<filter->size; i++)
	{
		sum[0] += filter->x[i];
		sum[1] += filter->y[i];
		sum[2] += filter->z[i];
		filter->x[i-1] = filter->x[i];
		filter->y[i-1] = filter->y[i];
		filter->z[i-1] = filter->z[i];
	}
	filter->x[filter->size-1] = in[0];
	filter->y[filter->size-1] = in[1];
	filter->z[filter->size-1] = in[2];
	sum[0] += in[0];
	sum[1] += in[1];
	sum[2] += in[2];

	out[0] = (float)(sum[0]/filter->size);
	out[1] = (float)(sum[1]/filter->size);
	out[2] = (float)(sum[2]/filter->size);
}

static void qst_matrix_trans(struct virtual_gyro_matrix *a, struct virtual_gyro_matrix *aT, int num)
{
    //struct virtual_gyro_matrix ori;
    int i = 0, j = 0;

    for(i = 0; i < num; i++)
    {
        for(j = 0; j < num; j++)
        {
            aT->cs[i][j] = a->cs[j][i];
        }
    }
}

static void qst_vgyro_matrix(float *Acc, float *Mag, struct virtual_gyro_matrix *matrix)
{
    float C[3][3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
    float hx_mag = 0, hy_mag = 0, hz_mag = 0;
    float cosyaw = 0, sinyaw = 0, hx_C = 0, hy_C = 0, hz_C = 0;
    float normalize = sqrtf(Acc[0] * Acc[0] + Acc[1] * Acc[1] + Acc[2] * Acc[2]);
    C[2][0] = Acc[0] / normalize;
    C[2][1] = Acc[1] / normalize;
    C[2][2] = Acc[2] / normalize;

    if(fabs(C[2][0]) > 0.5)
    {
        C[1][0] = C[2][1];
        C[1][1] = -C[2][0];
        C[1][2] = 0;
    }
    else
    {
        C[1][0] = 0;
        C[1][1] = C[2][2];
        C[1][2] = -C[2][1];
    }
    //???
    normalize = sqrtf(C[1][0] * C[1][0] + C[1][1] * C[1][1]+ C[1][2] * C[1][2]);        
    C[1][0] = C[1][0] / normalize;
    C[1][1] = C[1][1] / normalize;
    C[1][2] = C[1][2] / normalize;

    //????C0(???)
    C[0][0] = C[1][1] * C[2][2] - C[2][1] * C[1][2];
    C[0][1] = -(C[1][0] * C[2][2] - C[2][0] * C[1][2]);
    C[0][2] = C[1][0] * C[2][1] - C[2][0] * C[1][1];

    normalize = sqrtf(Mag[0] * Mag[0] + Mag[1] * Mag[1] + Mag[2] * Mag[2]);
    hx_mag = Mag[0] / normalize;
    hy_mag = Mag[1] / normalize;
    hz_mag = Mag[2] / normalize;

    //????
    hx_C = C[0][0] * hx_mag + C[0][1] * hy_mag + C[0][2] * hz_mag;
    hy_C = C[1][0] * hx_mag + C[1][1] * hy_mag + C[1][2] * hz_mag;
    hz_C = C[2][0] * hx_mag + C[2][1] * hy_mag + C[2][2] * hz_mag;
	((void)hz_C);
    
    sinyaw = hx_C / sqrtf(hx_C * hx_C + hy_C * hy_C);
    cosyaw = hy_C / sqrtf(hx_C * hx_C + hy_C * hy_C);

    if(sinyaw <= -1.0f)
    {
        sinyaw = -1;
    }
    else if(sinyaw >= 1.0f)
    {
        sinyaw = 1;
    }
    if(cosyaw <= -1.0f)
    {
        cosyaw = -1;
    }
    else if(cosyaw >= 1.0f)
    {
        cosyaw = 1;
    }
    //Cnb = Cnh * Chb
    matrix->cs[0][0] = cosyaw * C[0][0] - sinyaw * C[1][0];
    matrix->cs[0][1] = cosyaw * C[0][1] - sinyaw * C[1][1];
    matrix->cs[0][2] = cosyaw * C[0][2] - sinyaw * C[1][2];

    matrix->cs[1][0] = sinyaw * C[0][0] + cosyaw * C[1][0];
    matrix->cs[1][1] = sinyaw * C[0][1] + cosyaw * C[1][1];
    matrix->cs[1][2] = sinyaw * C[0][2] + cosyaw * C[1][2];

    matrix->cs[2][0] = C[2][0];
    matrix->cs[2][1] = C[2][1];
    matrix->cs[2][2] = C[2][2];
}


void qst_fusion_dummy_gyro(float Acc[3], float Mag[3], float Vgyro[3], int64_t timeStamp)
{
    int i = 0, j = 0, x = 0;
    float acc_data[3] = {0, 0, 0};
    float mag_data[3] = {0, 0, 0};
    float gyro_data[3] = {0, 0, 0};
    float T_delta = (float)(timeStamp - timeStamp_old) / 1000000000.0f;
    float acc_nomal = 0;
    float mag_nomal = 0;
    unsigned char vgyro_init = 0;
    struct virtual_gyro_matrix matrix_fin;

    timeStamp_old = timeStamp;
	if(T_delta > 0.2f)
	{
	    Vgyro[0] = 0.0001;
        Vgyro[1] = 0.0001;
        Vgyro[2] = 0.0001;
        qst_filter_init_flag = 0;
        T_delta = 0.02f;
		timeStamp_start = timeStamp;
		return;
	}
	else if(T_delta <= 0.0f)
	{
        Vgyro[0] = 0.0001;
        Vgyro[1] = 0.0001;
        Vgyro[2] = 0.0001;
		timeStamp_start = timeStamp;
        return;
    }
    else if(T_delta <= 1e-6f)
    {
        T_delta = 0.02f;
    }

    memset(&matrix_fin, 0, sizeof(struct virtual_gyro_matrix));

    acc_nomal = sqrtf(Acc[0] * Acc[0] + Acc[1] * Acc[1] + Acc[2] * Acc[2]);
    mag_nomal = sqrtf(Mag[0] * Mag[0] + Mag[1] * Mag[1] + Mag[2] * Mag[2]);
    
    if((acc_nomal > 1.0f) && (mag_nomal > 1.0f))
    {
		if(qst_filter_init_flag == 0)
		{
			qst_filter_init_flag = 1;
			qst_axis3_filter_init();
		}
#if ACC_FILTER_SIZE
		qst_axis3_filter_run(&g_acc_f, Acc, acc_data);
#else
		acc_data[0] = Acc[0];
		acc_data[1] = Acc[1];
		acc_data[2] = Acc[2];
#endif

#if defined(MAG_BTWZ_FILTER)
		qst_btwz_apply_1(Mag);
#endif
#if MAG_FILTER_SIZE
		qst_axis3_filter_run(&g_mag_f, Mag, mag_data);
#else
		mag_data[0] = Mag[0];
		mag_data[1] = Mag[1];
		mag_data[2] = Mag[2];
#endif

#ifdef DUMMYGYRO_LOG
		DGLOG("%s %f, %f, %f, %f, %f, %f [%f]\n", __func__,
											acc_data[0], acc_data[1], acc_data[2], 
											mag_data[0], mag_data[1], mag_data[2],
                                            T_delta);
#endif
        qst_vgyro_matrix(acc_data, mag_data, &matrix_new);

#if 0 //def DUMMYGYRO_LOG
		DGLOG("%s,%s %f, %f, %f, %f, %f, %f, %f, %f, %f\n", __func__,LOG_TAG,
					matrix_new.cs[0][0], matrix_new.cs[0][1], matrix_new.cs[0][2],
					matrix_new.cs[1][0], matrix_new.cs[1][1], matrix_new.cs[1][2],
					matrix_new.cs[2][0], matrix_new.cs[2][1], matrix_new.cs[2][2]);
#endif

        qst_matrix_trans(&matrix_new, &qst_matrix_transpose, 3);
            
        for(i = 0; i < 3; i++) 
        {
            for(j = 0; j < 3; j++) 
            {
                for(x = 0; x < 3; x++) 
                {
                    matrix_fin.cs[i][j] += qst_matrix_transpose.cs[i][x] * matrix_old.cs[x][j];
                }
            }
        }
        matrix_fin.cs[0][0] = matrix_fin.cs[0][0] - 1;
        matrix_fin.cs[1][1] = matrix_fin.cs[1][1] - 1;
        matrix_fin.cs[2][2] = matrix_fin.cs[2][2] - 1;

#if 0//def DUMMYGYRO_LOG
		DGLOG("%s,%s %f, %f, %f, %f, %f, %f, %f, %f, %f\n", __func__,LOG_TAG,
	      matrix_fin.cs[0][0], matrix_fin.cs[0][1], matrix_fin.cs[0][2],
	      matrix_fin.cs[1][0], matrix_fin.cs[1][1], matrix_fin.cs[1][2],
	      matrix_fin.cs[2][0], matrix_fin.cs[2][1], matrix_fin.cs[2][2]);
		DGLOG("%s,%s: %f,%f,%f,%f,%f\n", __func__, LOG_TAG,
			matrix_fin.cs[2][1], matrix_fin.cs[1][2], matrix_fin.cs[0][2], matrix_fin.cs[2][0], T_delta);
#endif
        gyro_data[0] = -(matrix_fin.cs[2][1] - matrix_fin.cs[1][2]) / (2 * T_delta);
        gyro_data[1] = -(matrix_fin.cs[0][2] - matrix_fin.cs[2][0]) / (2 * T_delta);
        gyro_data[2] = -(matrix_fin.cs[1][0] - matrix_fin.cs[0][1]) / (2 * T_delta);

#if 0//def DUMMYGYRO_LOG
		DGLOG("%s,%s: %f,%f,%f,%f\n", __func__,LOG_TAG, gyro_data[0], gyro_data[1], gyro_data[2], T_delta);
#endif
        matrix_old = matrix_new;
        //timeStamp_old = timeStamp;
#if GYR_FILTER_SIZE
		qst_axis3_filter_run(&g_gyr_f, gyro_data, Vgyro);
#else
		Vgyro[0] = gyro_data[0];
		Vgyro[1] = gyro_data[1];
		Vgyro[2] = gyro_data[2];
#endif
        float gyro_normal = sqrtf(Vgyro[0] * Vgyro[0] + Vgyro[1] * Vgyro[1] + Vgyro[2] * Vgyro[2]);
        
        if(gyro_normal > 1e-6f)
        {
            vgyro_init = 1;
        }
        if(vgyro_init == 0)
        {
            Vgyro[0] = 0;
            Vgyro[1] = 0;
            Vgyro[2] = 0;
        }
        
        if(fabs(Vgyro[0]*RadToDeg) < THRESH_ZERO_X )
        {
            Vgyro[0] = Vgyro[0] * 0.01f; 
        }
        if(fabs(Vgyro[1]*RadToDeg) < THRESH_ZERO_Y)
        {
            Vgyro[1] = Vgyro[1] * 0.01f;
        }
        if(fabs(Vgyro[2]*RadToDeg) < THRESH_ZERO_Z)
        {
            Vgyro[2] = Vgyro[2] * 0.01f; 
        }
    
        Vgyro_last[0] = Vgyro[0];
        Vgyro_last[1] = Vgyro[1];
        Vgyro_last[2] = Vgyro[2];
    }
    else
    {
        Vgyro[0] = Vgyro_last[0];
        Vgyro[1] = Vgyro_last[1];
        Vgyro[2] = Vgyro_last[2];
    }

	if(((timeStamp-timeStamp_start)/1000000.0f) < 300.0f)
	{
		static float dummy_chara = 1.0f;

		Vgyro[0] = 0.0001*dummy_chara;
		Vgyro[1] = 0.0001*dummy_chara;
		Vgyro[2] = 0.0001*dummy_chara;
		dummy_chara = -1.0f*dummy_chara;
	}
}

