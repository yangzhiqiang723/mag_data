
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include "ical_qst.h"
#include "qstFastCali.h"

#include "qst_convert.h"
#include "dummyGyro.h"
#include "fusion_interface.h"
#include "QstAlgoLog.h"

//#define ANDROID_FS
#ifdef ANDROID_FS
#include <fcntl.h>
#include <unistd.h>
#endif

#define QST_FUSION_VER2_EN			0
#define QST_FUSION_EN				0
#define QST_MAG_CALI_EN				1
#define QST_DUMMY_GYRO_EN			0

//#define FUSION_DEBUG_LOG
#ifdef FUSION_DEBUG_LOG
#undef LOG_TAG
#define LOG_TAG "qst-interface"
#define	FLOG		LOGD
#else
#define	FLOG(...)
#endif

#if QST_FUSION_EN
#include "qst_fusion_9axis.h"
#endif
#if QST_FUSION_VER2_EN
#include "imualgo_axis9.h"
#endif

typedef struct
{
	int			mode;
	float		acc[3];
	float		acc_f[3];
	float		gyr[3];
	float		gyr_f[3];
	float		mag[3];
	float		mag_cali[3];
	float		mag_f[3];
	float		mag_offset[3];
	float		mag_rr;
	float		gravity[3];
	float		dummy_gyr[3];
	char		mag_accury;
	int64_t		tm_acc;
	int64_t		tm_gyr;
	int64_t		tm_mag;
	bool		acc_drdy;
	bool		mag_drdy;
}qst_fusion_t;

#define ACC_AVG_SIZE		6
#define MAG_AVG_SIZE		0	//4

#if MAG_AVG_SIZE
typedef struct
{
	char	init;
	float	x[MAG_AVG_SIZE];
	float	y[MAG_AVG_SIZE];
	float	z[MAG_AVG_SIZE];
}MagAxis3Avg_t;
static MagAxis3Avg_t	mag_avg;
#endif

#if ACC_AVG_SIZE
typedef struct
{
	char	init;
	float	x[ACC_AVG_SIZE];
	float	y[ACC_AVG_SIZE];
	float	z[ACC_AVG_SIZE];
}AccAxis3Avg_t;
static AccAxis3Avg_t	acc_avg;
#endif

static qst_fusion_t	gFusion;

#if defined(ANDROID_FS)
static const char* file_cali_para = "/mnt/vendor/nvcfg/sensor/qst.json";
//static const char* file_cali_para = "/mnt/sdcard/qst.json";
#endif

#if ACC_AVG_SIZE
static void InitAccAvg(AccAxis3Avg_t *filter)
{
	filter->init = 0;
}

static void RunAccAvg(AccAxis3Avg_t *filter, const float in[3], float out[3])
{
	int i;
	float sum[3] = {0, 0, 0};

	if(filter->init == 0)
	{
		for(i=0; i<ACC_AVG_SIZE; i++)
		{
			filter->x[i] = in[0];
			filter->y[i] = in[1];
			filter->z[i] = in[2];
		}
		filter->init = 1;
	}

	sum[0] = sum[1] = sum[2] = 0.0f;
	for(i=1; i<ACC_AVG_SIZE; i++)
	{
		sum[0] += filter->x[i];
		sum[1] += filter->y[i];
		sum[2] += filter->z[i];
		filter->x[i-1] = filter->x[i];
		filter->y[i-1] = filter->y[i];
		filter->z[i-1] = filter->z[i];
	}
	filter->x[ACC_AVG_SIZE-1] = in[0];
	filter->y[ACC_AVG_SIZE-1] = in[1];
	filter->z[ACC_AVG_SIZE-1] = in[2];
	sum[0] += in[0];
	sum[1] += in[1];
	sum[2] += in[2];
	out[0] = (float)(sum[0]/ACC_AVG_SIZE);
	out[1] = (float)(sum[1]/ACC_AVG_SIZE);
	out[2] = (float)(sum[2]/ACC_AVG_SIZE);

}
#endif

#if MAG_AVG_SIZE
static void InitMagAvg(MagAxis3Avg_t *filter)
{
	filter->init = 0;
}

static void RunMagAvg(MagAxis3Avg_t *filter, const float in[3], float out[3])
{
	int i;
	float sum[3] = {0, 0, 0};

	if(filter->init == 0)
	{
		for(i=0; i<MAG_AVG_SIZE; i++)
		{
			filter->x[i] = in[0];
			filter->y[i] = in[1];
			filter->z[i] = in[2];
		}
		filter->init = 1;
	}

	sum[0] = sum[1] = sum[2] = 0.0f;
	for(i=1; i<MAG_AVG_SIZE; i++)
	{
		sum[0] += filter->x[i];
		sum[1] += filter->y[i];
		sum[2] += filter->z[i];
		filter->x[i-1] = filter->x[i];
		filter->y[i-1] = filter->y[i];
		filter->z[i-1] = filter->z[i];
	}
	filter->x[MAG_AVG_SIZE-1] = in[0];
	filter->y[MAG_AVG_SIZE-1] = in[1];
	filter->z[MAG_AVG_SIZE-1] = in[2];
	sum[0] += in[0];
	sum[1] += in[1];
	sum[2] += in[2];
	out[0] = (float)(sum[0]/MAG_AVG_SIZE);
	out[1] = (float)(sum[1]/MAG_AVG_SIZE);
	out[2] = (float)(sum[2]/MAG_AVG_SIZE);

}
#endif

static int qst_calc_gravity(const float acc[3])
{
#if !QST_FUSION_EN
	static unsigned char gravity_init = 0;
	
    if(gravity_init == 0)
    {
        gravity_init = 1;
        gFusion.gravity[0] = acc[0];
        gFusion.gravity[1] = acc[1];
        gFusion.gravity[2] = acc[2];
    }
    else
    {
        gFusion.gravity[0] = gFusion.gravity[0]*0.8f + acc[0]*0.2f;
        gFusion.gravity[1] = gFusion.gravity[1]*0.8f + acc[1]*0.2f;
        gFusion.gravity[2] = gFusion.gravity[2]*0.8f + acc[2]*0.2f;
    }
#else
	((void)(acc));
#endif
	return 0;
}

static void qst_fusion_reset(void)
{
#if ACC_AVG_SIZE
	InitAccAvg(&acc_avg);
#endif
#if MAG_AVG_SIZE
	InitMagAvg(&mag_avg);
#endif
	//memset(&gFusion, 0, sizeof(gFusion));
	gFusion.acc[0] = gFusion.acc_f[0] = 0.0f;
	gFusion.acc[1] = gFusion.acc_f[1] = 0.0f;
	gFusion.acc[2] = gFusion.acc_f[2] = 9.807f;
	
	gFusion.gyr[0] = gFusion.gyr_f[0] = 0.001f;
	gFusion.gyr[1] = gFusion.gyr_f[1] = 0.001f;
	gFusion.gyr[2] = gFusion.gyr_f[2] = 0.001f;
	
	gFusion.mag[0] = gFusion.mag_f[0] = 0.1f;
	gFusion.mag[1] = gFusion.mag_f[1] = -30.0f;
	gFusion.mag[2] = gFusion.mag_f[2] = -36.0f;

	gFusion.acc_drdy = false;
	gFusion.mag_drdy = false;
}

static int qst_fusion_write_nvram(float pData[4])
{
#if defined(ANDROID_FS)
    int pFile;
    long modeval;
    long filemode;
    int w_len;
    char w_buf[128];

    FLOG("qst_fusion_write_nvram para[%f	%f	%f	%f]\n",pData[0],pData[1],pData[2],pData[3]);
    modeval = O_WRONLY|O_CREAT|O_TRUNC;
	filemode = S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP|S_IROTH;

    pFile = open(file_cali_para, modeval, filemode);
    if(pFile > 0)
    {
        memset(w_buf, 0, sizeof(w_buf));
        int len = sprintf(w_buf, "%f %f %f %f", pData[0],pData[1],pData[2],pData[3]);
        w_len = write(pFile, w_buf, len);
        close(pFile);
        FLOG("qst_fusion_write_nvram successed write_len = %d %d\n", w_len, len);
        return 1;
    }
    else
    {
        FLOG("qst_fusion_write_nvram open file failed\n");
        return 0;
    }
#else
	((void)(pData));
	return 0;
#endif
}

static int qst_fusion_read_nvram(float pData[4])
{
#if defined(ANDROID_FS)
	int pFile;
	int r_len;
	char r_buf[128];

	if(!pData)
		return 0;

	pFile = open(file_cali_para, O_RDONLY, 0);
	if(pFile>0)
	{

		memset(r_buf, 0, sizeof(r_buf));
		r_len = read(pFile, r_buf, sizeof(r_buf));
		if(r_len > 0)
		{
			sscanf(r_buf, "%f %f %f %f", &pData[0], &pData[1], &pData[2], &pData[3]);
		}
		FLOG("qst_fusion_read_nvram para[%f	%f	%f	%f]\n",pData[0],pData[1],pData[2],pData[3]);
		close(pFile);
		return 1;
	}
	else
	{
		FLOG("qst_fusion_read_nvram open file failed\n");
		return 0;
	}
#else
	((void)(pData));
	return 0;
#endif
}

static int qst_mag_cali_init(void)
{
#if QST_MAG_CALI_EN
	float magPara[4] = {0.0f, 0.0f, 0.0f, 0.0f};

	qst_fusion_read_nvram(magPara);
	gFusion.mag_accury = (char)qst_ical_init(magPara);
#endif
	return 0;
}

static int qst_fusion_enableLib(int mode)
{
	float para[6] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
    FLOG("qst_fusion_enableLib mode %d\n", mode);
	qst_fusion_reset();
	qst_mag_cali_init();
	qst_fastcali_init(para);
	gFusion.mode = mode;
	return 0;
}

static int qst_fusion_SetGyroData(libData *inputData)
{
	float dt=0.0f;

	gFusion.gyr[0] = inputData->x;
	gFusion.gyr[1] = inputData->y;
	gFusion.gyr[2] = inputData->z;

	dt = (float)(inputData->timeStamp-gFusion.tm_gyr)/1000000000.0f;
	gFusion.tm_gyr = inputData->timeStamp;
	FLOG("qst_fusion_SetGyroData [%f %f %f] tm[%lld] dt[%f]",gFusion.gyr[0],gFusion.gyr[1],gFusion.gyr[2],inputData->timeStamp, dt);
	if(dt > 0.1f || dt < 0.0f)
	{
		return 0;
	}

#if QST_FUSION_EN
	if((gFusion.mode & FUSION_9AXIS)&&gFusion.acc_drdy&&gFusion.mag_drdy)
	{
		qst_fusion_9axis_update(&gFusion.acc_f[0], &gFusion.gyr[0], &gFusion.mag_f[0], dt, 0);
	}
	if((gFusion.mode & FUSION_NOMAG)&&gFusion.acc_drdy)
	{
		qst_fusion_nomag_update(&gFusion.acc_f[0], &gFusion.gyr[0], dt, 0);
	}
#endif
#if	QST_FUSION_VER2_EN
	if((gFusion.mode & FUSION_9AXIS)&&gFusion.acc_drdy&&gFusion.mag_drdy)
	{
		qst_fusion_9axis_update(&gFusion.acc_f[0], &gFusion.gyr[0], &gFusion.mag_f[0], &dt, 0);
	}
#endif

	return 0;
}

static int qst_fusion_SetAccData(libData *inputData)
{
	float dt=0.0f;

	gFusion.acc[0] = inputData->x;
	gFusion.acc[1] = inputData->y;
	gFusion.acc[2] = inputData->z;
#if ACC_AVG_SIZE
	RunAccAvg(&acc_avg, gFusion.acc, gFusion.acc_f);
	// add by yzq chage original acc data
	inputData->x = gFusion.acc_f[0];
	inputData->y = gFusion.acc_f[1];
	inputData->z = gFusion.acc_f[2];
#else
	gFusion.acc_f[0] = gFusion.acc[0];
	gFusion.acc_f[1] = gFusion.acc[1];
	gFusion.acc_f[2] = gFusion.acc[2];
#endif
	dt = (float)(inputData->timeStamp-gFusion.tm_acc)/1000000000.0f;
	gFusion.tm_acc = inputData->timeStamp;
	if(!gFusion.acc_drdy)
	{
		gFusion.acc_drdy = true;
	}
	FLOG("qst_fusion_SetAccData [%f %f %f] tm[%lld] dt[%f]",gFusion.acc_f[0],gFusion.acc_f[1],gFusion.acc_f[2],inputData->timeStamp,dt);
	if(dt > 0.1f || dt <0.0f)
	{
		return 0;
	}

	qst_calc_gravity(gFusion.acc_f);
#if QST_FUSION_EN
	if((gFusion.mode & FUSION_NOGYRO) && gFusion.mag_drdy)
	{
		qst_fusion_nogyro_update(&gFusion.acc_f[0], &gFusion.mag_f[0]);
	}
#endif
#if QST_DUMMY_GYRO_EN
	if(gFusion.mag_drdy)	// (gFusion.mode & FUSION_NOGYRO)
	{
		float acc[3];
		float mag[3];

		acc[0] = gFusion.acc[0];
		acc[1] = gFusion.acc[1];
		acc[2] = gFusion.acc[2];
		mag[0] = gFusion.mag_cali[0];
		mag[1] = gFusion.mag_cali[1];
		mag[2] = gFusion.mag_cali[2];

		qst_fusion_dummy_gyro(acc, mag, gFusion.dummy_gyr, gFusion.tm_acc);
		//FLOG("qst_fusion_dummy_gyro %f,%f,%f    %f,%f,%f",gFusion.gyr[0],gFusion.gyr[1],gFusion.gyr[2],gFusion.dummy_gyr[0],gFusion.dummy_gyr[1],gFusion.dummy_gyr[2]);
	}
#endif

	return 0;
}

static int qst_fusion_SetMagData(libData *inputData)
{
#if 0
	// set mag calied data
	gFusion.mag[0] = inputData->x;
	gFusion.mag[1] = inputData->y;
	gFusion.mag[2] = inputData->z;
#if MAG_AVG_SIZE
	RunMagAvg(&mag_avg, gFusion.mag, gFusion.mag_f);
#else
	gFusion.mag_f[0] = gFusion.mag[0];
	gFusion.mag_f[1] = gFusion.mag[1];
	gFusion.mag_f[2] = gFusion.mag[2];
#endif

#else
	((void)(inputData));
#endif

	return 0;
}

static int qst_fusion_GetMagOffset(float offset[3], float *rr)
{
	if(offset)
	{
		offset[0] = gFusion.mag_offset[0];
		offset[1] = gFusion.mag_offset[1];
		offset[2] = gFusion.mag_offset[2];
	}
	if(rr)
	{
		*rr = gFusion.mag_rr;
	}

	return 0;
}

static int qst_fusion_doCali(libData *inputData, libData *outputData)
{
#if QST_MAG_CALI_EN
	float mag[3], mag_cal[3], offset[3];
	float rr;
	signed char accuracy;
	int cali_ret = 0;

	mag_cal[0] = mag[0] = inputData->x;
	mag_cal[1] = mag[1] = inputData->y;
	mag_cal[2] = mag[2] = inputData->z;
	cali_ret = convert_magnetic(mag, mag_cal, offset, &rr, &accuracy);
	gFusion.tm_mag = inputData->timeStamp;
	gFusion.mag_cali[0] = mag_cal[0];
	gFusion.mag_cali[1] = mag_cal[1];
	gFusion.mag_cali[2] = mag_cal[2];
	gFusion.mag_offset[0] = offset[0];
	gFusion.mag_offset[1] = offset[1];
	gFusion.mag_offset[2] = offset[2];
	gFusion.mag_rr = rr;
#if MAG_AVG_SIZE
	RunMagAvg(&mag_avg, mag_cal, gFusion.mag_f);
#else
	gFusion.mag_f[0] = mag_cal[0];
	gFusion.mag_f[1] = mag_cal[1];
	gFusion.mag_f[2] = mag_cal[2];
#endif
	//FLOG("qst_fusion_doCali out[%f %f %f]",gFusion.mag_f[0],gFusion.mag_f[1],gFusion.mag_f[2]);
	outputData->x = gFusion.mag_f[0];
	outputData->y = gFusion.mag_f[1];
	outputData->z = gFusion.mag_f[2];
	outputData->w = rr;
	outputData->status = accuracy;
	outputData->timeStamp = inputData->timeStamp;

	//if((gFusion.mag_accury < 3) && (accuracy >= 3))
	if((cali_ret==1) && (accuracy >= 3))
	{
		float magPara[4] = {0.0f, 0.0f, 0.0f, 0.0f};

		magPara[0] = gFusion.mag_offset[0];
		magPara[1] = gFusion.mag_offset[1];
		magPara[2] = gFusion.mag_offset[2];
		magPara[3] = rr;
		qst_fusion_write_nvram(magPara);
	}
	gFusion.mag_accury = accuracy;
#else
	outputData->status = inputData->status;
	outputData->timeStamp = inputData->timeStamp;
	outputData->x = inputData->x;
	outputData->y = inputData->y;
	outputData->z = inputData->z;
	gFusion.mag_f[0] = gFusion.mag_cali[0] = inputData->x;
	gFusion.mag_f[1] = gFusion.mag_cali[1] = inputData->y;
	gFusion.mag_f[2] = gFusion.mag_cali[2] = inputData->z;
#endif
	if(!gFusion.mag_drdy)
	{
		gFusion.mag_drdy = true;
	}

	return 0;
}


static int qst_fusion_doFastCali(libData *inputData, libData *outputData)
{
#if QST_MAG_CALI_EN
	float mag[3], mag_cal[3], offset[6];
//	float rr;
	int accuracy;
	float	dt = 0.0f;

	mag_cal[0] = mag[0] = inputData->x;
	mag_cal[1] = mag[1] = inputData->y;
	mag_cal[2] = mag[2] = inputData->z;
	dt = (inputData->timeStamp - gFusion.tm_mag)/1000000000.0f;
	gFusion.tm_mag = inputData->timeStamp;

	convert_magnetic_fastcali(mag, gFusion.gyr, mag_cal, offset, dt, &accuracy, 0);	

	gFusion.mag_cali[0] = mag_cal[0];
	gFusion.mag_cali[1] = mag_cal[1];
	gFusion.mag_cali[2] = mag_cal[2];
	gFusion.mag_offset[0] = offset[0];
	gFusion.mag_offset[1] = offset[1];
	gFusion.mag_offset[2] = offset[2];
	FLOG("qst_fusion_doFastCali [%f %f %f] tm[%lld] dt[%f] status[%d]",gFusion.mag_cali[0],gFusion.mag_cali[1],gFusion.mag_cali[2],inputData->timeStamp,dt, accuracy);

#if MAG_AVG_SIZE
	RunMagAvg(&mag_avg, mag_cal, gFusion.mag_f);
#else
	gFusion.mag_f[0] = mag_cal[0];
	gFusion.mag_f[1] = mag_cal[1];
	gFusion.mag_f[2] = mag_cal[2];
#endif
	outputData->x = gFusion.mag_f[0];
	outputData->y = gFusion.mag_f[1];
	outputData->z = gFusion.mag_f[2];
	outputData->status = accuracy;
	outputData->timeStamp = inputData->timeStamp;

	if((gFusion.mag_accury < 3) && (accuracy >= 3))
	{
		float magPara[6] = {0.0f, 0.0f, 0.0f, 0.0f};

		magPara[0] = offset[0];
		magPara[1] = offset[1];
		magPara[2] = offset[2];
		magPara[3] = offset[3];
		magPara[4] = offset[4];
		magPara[5] = offset[5];
		qst_fusion_write_nvram(magPara);
	}
	gFusion.mag_accury = accuracy;
#else
	outputData->status = inputData->status;
	outputData->timeStamp = inputData->timeStamp;
	outputData->x = inputData->x;
	outputData->y = inputData->y;
	outputData->z = inputData->z;
	gFusion.mag_f[0] = gFusion.mag_cali[0] = inputData->x;
	gFusion.mag_f[1] = gFusion.mag_cali[1] = inputData->y;
	gFusion.mag_f[2] = gFusion.mag_cali[2] = inputData->z;
#endif
	if(!gFusion.mag_drdy)
	{
		gFusion.mag_drdy = true;
	}

	return 0;

}

static int qst_fusion_getGravity(libData *outputData)
{
	float gravity[3];

#if QST_FUSION_EN
	if(gFusion.mode & FUSION_9AXIS)
	{
		qst_fusion_get_gravity(gravity, 1);
	}	
	else if(gFusion.mode & FUSION_NOMAG)
	{
		qst_fusion_get_gravity(gravity, 2);
	}
	else if(gFusion.mode & FUSION_NOGYRO)
	{
		qst_fusion_get_gravity(gravity, 3);
	}
	gFusion.gravity[0] = gravity[0];
	gFusion.gravity[1] = gravity[1];
	gFusion.gravity[2] = gravity[2];
#else
	gravity[0] = gFusion.gravity[0];
	gravity[1] = gFusion.gravity[1];
	gravity[2] = gFusion.gravity[2];
#endif
	outputData->status = 3;
	outputData->timeStamp = gFusion.tm_acc;
	outputData->x = gravity[0];
	outputData->y = gravity[1];
	outputData->z = gravity[2];
	
	return 0;
}

static int qst_fusion_getRotationVector(libData *outputData)
{
#if QST_FUSION_EN
	float quat[4] = {0,0,0,1};

	qst_fusion_9axis_get_quaternion(quat);
	outputData->timeStamp = gFusion.tm_gyr;

	//qst_fusion_nomag_get_quaternion(quat);		
	//outputData->timeStamp = gFusion.tm_gyr;

	//qst_fusion_nogyro_get_quaternion(quat);
	//outputData->timeStamp = gFusion.tm_acc;
	
	outputData->x = quat[0];
	outputData->y = quat[1];
	outputData->z = quat[2];
	outputData->w = quat[3];
	outputData->status = 3;
#endif

	return 0;
}

static int qst_fusion_getOrientation(libData *outputData)
{
	float euler[3];
#if QST_FUSION_EN
	float quat[4];

	if(gFusion.mode & FUSION_9AXIS)
	{
		qst_fusion_9axis_get_quaternion(quat);		
	}	
	else if(gFusion.mode & FUSION_NOMAG)
	{
		qst_fusion_nomag_get_quaternion(quat);
	}
	else if(gFusion.mode & FUSION_NOGYRO)
	{			
		qst_fusion_nogyro_get_quaternion(quat);
	}
	android_quat_to_euler(quat, euler);
	euler[0] = qst_yaw_improve(euler[0]);
	//FLOG("qst_fusion_getOrientation %f", euler[0]);
#elif QST_FUSION_VER2_EN
	float quat[4];
	if(gFusion.mode & FUSION_9AXIS)
	{
		qst_fusion_9axis_euler_to_quaternion(quat);
	}
	android_quat_to_euler(quat, euler);
	euler[0] = qst_yaw_improve(euler[0]);
#else
	#if QST_MAG_CALI_EN
//		signed char accuracy = 0;
//		qst_ori_progress(gFusion.acc_f, gFusion.mag_f, euler, &accuracy);
		android_raw_to_euler(gFusion.acc_f, gFusion.mag_f, euler);
		euler[0] = qst_yaw_improve(euler[0]);
	#endif	
#endif
	
	outputData->status = gFusion.mag_accury;
	outputData->timeStamp = gFusion.tm_mag;
	outputData->x = euler[0];
	outputData->y = euler[1];
	outputData->z = euler[2];

	return 0;
}

static int qst_fusion_getLinearAccel(libData *outputData)
{
	float gravity[3];

#if QST_FUSION_EN
	if(gFusion.mode & FUSION_9AXIS)
	{
		qst_fusion_get_gravity(gravity, 1);
	}	
	else if(gFusion.mode & FUSION_NOMAG)
	{
		qst_fusion_get_gravity(gravity, 2);
	}
	else if(gFusion.mode & FUSION_NOGYRO)
	{
		qst_fusion_get_gravity(gravity, 3);
	}
	gFusion.gravity[0] = gravity[0];
	gFusion.gravity[1] = gravity[1];
	gFusion.gravity[2] = gravity[2];
#endif

	gravity[0] = gFusion.acc[0] - gFusion.gravity[0];
	gravity[1] = gFusion.acc[1] - gFusion.gravity[1];
	gravity[2] = gFusion.acc[2] - gFusion.gravity[2];
	outputData->status = 3;
	outputData->timeStamp = gFusion.tm_acc;
	outputData->x = gravity[0];
	outputData->y = gravity[1];
	outputData->z = gravity[2];
	
	return 0;
}

static int qst_fusion_getGameRotationVector(libData *outputData)
{
#if QST_FUSION_EN
	float quat[4];
	qst_fusion_nomag_get_quaternion(quat);		
	outputData->x = quat[0];
	outputData->y = quat[1];
	outputData->z = quat[2];
	outputData->w = quat[3];
	outputData->status = 3;
	outputData->timeStamp = gFusion.tm_gyr;
#endif
	return 0;
}

static int qst_fusion_getGeoMagnetic(libData *outputData)
{
#if QST_FUSION_EN
	float quat[4];
	qst_fusion_nogyro_get_quaternion(quat);
	outputData->x = quat[0];
	outputData->y = quat[1];
	outputData->z = quat[2];
	outputData->w = quat[3];
	outputData->status = 3;
	outputData->timeStamp = gFusion.tm_acc;
#endif
	return 0;
}

static int qst_fusion_getVirtualGyro(libData *outputData)
{
#if QST_DUMMY_GYRO_EN
	outputData->status = 3;
	outputData->timeStamp = gFusion.tm_acc;
	outputData->x = gFusion.dummy_gyr[0];
	outputData->y = gFusion.dummy_gyr[1];
	outputData->z = gFusion.dummy_gyr[2];
#else
	outputData->status = 0;
	outputData->timeStamp = gFusion.tm_acc;
	outputData->x = 0.0f;
	outputData->y = 0.0f;
	outputData->z = 0.0f;
#endif

	return 0;
}

struct qst_fusion_interface_t algo_qst_fusion = 
{
	.version = "1.01",
	.enableLib = qst_fusion_enableLib,
	.SetGyroData = qst_fusion_SetGyroData,
	.SetAccData = qst_fusion_SetAccData,
	.SetMagData = qst_fusion_SetMagData,
	.GetMagOffset = qst_fusion_GetMagOffset,
	.doCali = qst_fusion_doCali,
	.doFastCali = qst_fusion_doFastCali,
	.getGravity = qst_fusion_getGravity,
	.getRotationVector = qst_fusion_getRotationVector,
	.getOrientation = qst_fusion_getOrientation,
	.getLinearAccel = qst_fusion_getLinearAccel,
	.getGameRotationVector = qst_fusion_getGameRotationVector,
	.getGeoMagnetic = qst_fusion_getGeoMagnetic,
	.getVirtualGyro = qst_fusion_getVirtualGyro
};

void qst_fusion_get_interface(struct qst_fusion_interface_t **p_algo) 
{
	*p_algo = &algo_qst_fusion;
}
