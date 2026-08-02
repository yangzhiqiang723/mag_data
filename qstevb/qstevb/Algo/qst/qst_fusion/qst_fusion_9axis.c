#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "qst_fusion_9axis.h"
#include "qst_convert.h"
#include "QstAlgoLog.h"

typedef struct
{
	float	q[4];
	float	mat[3][3];
}qst_fusion_data;

#define STD_ARRAY_SIZE		12
typedef struct
{
	float	data_x[STD_ARRAY_SIZE];
	float	sum_x;
	short	size;
	short	index;
}QstStd1;

#define LPF_ALPHA           0.8f
#define QST_FABS(v)         ((v<0)?(-1.0f*v):(v))
#define QST_ABS(v)          ((v<0)?(-1*v):(v))
#define QST_MIN(a, b)		(((a) < (b)) ? (a) : (b))
#define Pi					3.14159265358979f
#define TODEG               57.2957796f
#define TORAD               Pi / 180.0f
#define GRAVITY_MSS         9.80665f
#define clamp(v)			((v) < 0 ? 0 : (v))

//#define QST_FUSION_9AXIS_LOCK_EULER

static float fusion_9axis_kp = 4.0f;
static float fusion_9axis_ki = 0.1f;
static float fusion_9axis_mag_kp = 6.0f;

static float fusion_9axis_Int_max = 0.05f;//0.15f;//
static float fusion_9axis_exInt = 0;
static float fusion_9axis_eyInt = 0;
static float fusion_9axis_ezInt = 0;

/****************** 根据初始化欧拉角初始化四元数 *****************************/
static unsigned char fusion_9axis_inited = 0;
//static float fusion_9axis_q_init[4] = {0, 0, 0, 0};

typedef struct
{
    float a1;
    float a2;
    float b0;
    float b1;
    float b2;
}QST_Filter_Parameter;

typedef struct
{
    float _delay_element_1;
    float _delay_element_2;
}QST_Filter_Buffer;

//1hz
//QST_Filter_Parameter gyro_filter_parameter = {-1.82269502,   0.837181747,   0.00362168206,    0.00724336412,   0.00362168206};
//2hz
//QST_Filter_Parameter gyro_filter_parameter = {-1.64745998,   0.7008968,   0.0133592002,    0.0267184004,   .0133592002};
//5hz
static QST_Filter_Parameter gyro_filter_parameter = {-1.14298046,   0.412801653,   0.0674552768,    0.134910554,   0.0674552768};    //10
//10hz
//QST_Filter_Parameter gyro_filter_parameter = {-0.369527251,   0.195815727,   0.206572115,    0.413144231,   0.206572115};    //10

//QST_Filter_Parameter gyro_filter_parameter = {-1.14298046,   0.412801653,   0.0674552768,   0.134910554,  0.0674552768};    //20
//QST_Filter_Parameter accel_filter_parameter = {-1.56101811,   0.641351581,   0.020083366,    0.0401667319,   0.020083366};    //10

//QST_Filter_Parameter gyro_filter_parameter = {-1.56101811,   0.641351581,   0.020083366,    0.0401667319,   0.020083366};        //10
//QST_Filter_Parameter accel_filter_parameter = {-1.77863193,   0.800802648,   0.00554271694,   0.0110854339,   0.00554271694};    //5

//QST_Filter_Parameter gyro_filter_parameter = {-1.77863193,   0.800802648,   0.00554271694,   0.0110854339,   0.00554271694};////5
//QST_Filter_Parameter accel_filter_parameter = {-1.91119695,   0.914975762,   0.000944691768,    0.00188938354,   0.000944691768};////2


//QST_Filter_Parameter gyro_filter_parameter = {-1.91119695,   0.914975762,   0.000944691768,    0.00188938354,   0.000944691768};////2
//QST_Filter_Parameter accel_filter_parameter = {-1.91119695,   0.914975762,   0.000944691768,    0.00188938354,   0.000944691768};////2

//QST_Filter_Parameter gyro_filter_parameter = {-1.95557833,   0.956543684,   0.000241359114,   0.000482718227,   0.000241359114};////1
//QST_Filter_Parameter accel_filter_parameter = {-1.95557833,   0.956543684,   0.000241359114,   0.000482718227,   0.000241359114};////1

static QST_Filter_Buffer gyro_buffer[3] = {{0,0}};
//static QST_Filter_Buffer accel_buffer[3] = {{0,0}};

static float gyro_max = 0.005f;
static unsigned char fusion_9axis_mode = 1;
static unsigned char fusion_9axis_mode_last = 1;
#if defined(QST_FUSION_9AXIS_LOCK_EULER)
static unsigned int static_delay = 0;
static unsigned int static_flag = 0;
static QstStd1 stdAcc;
static QstStd1 stdGyr;
#endif
static qst_fusion_data	axis9;


//#define QST_FUSION_9D_USE_MAG_RR

#if defined(QST_FUSION_9D_USE_MAG_RR)
void rr_sync_to_fusion(float *rr);
void ical_update_offset_flag_sync_to_fusion(int *flag);
#endif

/*!
 * \brief set second order butterworth filtering Parameters.
* \input float sample:accel and gyro filter input value.
* \input Butterworth_Buffer *buffer:accel and gyro filter buffer value.
* \input float cutoff_freq:accel and gyro filter cut-off frequency.
* \return filter output value
 */
static float Filter_Apply(float sample, QST_Filter_Buffer *buffer, QST_Filter_Parameter *filter) 
{
	//do the filtering
	float delay_element_0 = sample - buffer->_delay_element_1 * filter->a1 - buffer->_delay_element_2 * filter->a2;

	const float output = delay_element_0 * filter->b0 + buffer->_delay_element_1 * filter->b1 + buffer->_delay_element_2 * filter->b2;

	buffer->_delay_element_2 = buffer->_delay_element_1;
	buffer->_delay_element_1 = delay_element_0;

	return output;
}

//constrain a value
static float constrain_float(float amt, float low, float high) 
{
	return ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)));
}

#if defined(QST_FUSION_9AXIS_LOCK_EULER)
static void getStandardDeviation1(QstStd1 *pStd1, float in, float *out)
{
	float avg_x;
	float toatl_x;
	short index;

	if(pStd1->size < STD_ARRAY_SIZE)
	{
		index = pStd1->size;		
		if(index == 0)
		{
			pStd1->sum_x = 0.0f;
			pStd1->index = 0;
		}
		pStd1->data_x[index] = in;
		pStd1->sum_x += in;
		pStd1->size++;
		pStd1->index = 0;
		*out = 100.0f;
	}
	else
	{
		index = pStd1->index;
		pStd1->sum_x = pStd1->sum_x - pStd1->data_x[index] + in;
		pStd1->data_x[index] = in;
		pStd1->index = (++pStd1->index%STD_ARRAY_SIZE);
		avg_x = pStd1->sum_x/STD_ARRAY_SIZE;

		toatl_x = 0.0f;
		for(int i = 0; i < STD_ARRAY_SIZE; i++)
		{
			toatl_x += (pStd1->data_x[i] - avg_x) * (pStd1->data_x[i] - avg_x);
		}

		*out = (float)sqrtf(toatl_x / (STD_ARRAY_SIZE));
	}
}
#endif

static void qst_fusion_9axis_rotation_matrix(void)
{
    float q1q1 = axis9.q[1] * axis9.q[1];
    float q2q2 = axis9.q[2] * axis9.q[2];
    float q3q3 = axis9.q[3] * axis9.q[3];

    float q1q2 = axis9.q[1] * axis9.q[2];
    float q0q3 = axis9.q[0] * axis9.q[3];
    float q1q3 = axis9.q[1] * axis9.q[3];

    float q0q2 = axis9.q[0] * axis9.q[2];
    float q2q3 = axis9.q[2] * axis9.q[3];
    float q0q1 = axis9.q[0] * axis9.q[1];

    axis9.mat[0][0] = 1 - 2 * (q2q2 + q3q3);
    axis9.mat[0][1] = 2 * (q1q2 - q0q3);
    axis9.mat[0][2] = 2 * (q1q3 + q0q2);
  
    axis9.mat[1][0] = 2 * (q1q2 + q0q3);
    axis9.mat[1][1] = 1 - 2 * (q1q1 + q3q3);
    axis9.mat[1][2] = 2 * (q2q3 - q0q1);
  
    axis9.mat[2][0] = 2 * (q1q3 - q0q2);
    axis9.mat[2][1] = 2 * (q2q3 + q0q1);
    axis9.mat[2][2] = 1 - 2 * (q1q1 + q2q2);
}


unsigned char qst_fusion_9axis_init(float accel_in[3], float mag_in[3]) 
{
		float accel_data[3] = {0, 0, 0};
		float mag_data[3] = {0, 0, 0};
		//	float euler[3] = {0, 0, 0};
		float mat33_t_R[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
		float quat_init[4] = {0, 0, 0, 0};

		axis9.q[0] = 1.0f;
		axis9.q[1] = 0.0f;
		axis9.q[2] = 0.0f;
		axis9.q[3] = 0.0f;

		axis9.mat[0][0] = 1.0f;
		axis9.mat[0][1] = 0.0f;
		axis9.mat[0][2] = 0.0f;
		axis9.mat[1][0] = 0.0f;
		axis9.mat[1][1] = 1.0f;
		axis9.mat[1][2] = 0.0f;
		axis9.mat[2][0] = 0.0f;
		axis9.mat[2][1] = 0.0f;
		axis9.mat[2][2] = 1.0f;

    accel_data[0] = accel_in[0];
    accel_data[1] = accel_in[1];
    accel_data[2] = accel_in[2];
	
    mag_data[0] = mag_in[0];
    mag_data[1] = mag_in[1];
    mag_data[2] = mag_in[2];

    float normalize = sqrtf(accel_data[0] * accel_data[0] + accel_data[1] * accel_data[1] + accel_data[2] * accel_data[2]);
    accel_data[0] = accel_data[0] / normalize;
    accel_data[1] = accel_data[1] / normalize;
    accel_data[2] = accel_data[2] / normalize;
    
    float pitch = -atan2f(accel_data[1], accel_data[2]);
    normalize = sqrtf(accel_data[0] * accel_data[0] + accel_data[1] * accel_data[1] + accel_data[2] * accel_data[2]);
    float roll = asinf((accel_data[0] / normalize));

    float cp = cosf(pitch);
    float sp = sinf(pitch);
    float sr = sinf(roll);
    float cr = cosf(roll);


    normalize = sqrtf(mag_data[0] * mag_data[0] + mag_data[1] * mag_data[1] + mag_data[2] * mag_data[2]);
    mag_data[0] = mag_data[0] / normalize;
    mag_data[1] = mag_data[1] / normalize;
    mag_data[2] = mag_data[2] / normalize;

    float hx = mag_data[0] * cr - mag_data[2] * sr;
    float hy = mag_data[0] * sp * sr + mag_data[1] * cp + mag_data[2] * cr * sp;
    float yaw = -atan2f(hx, hy);
    float sy = sinf(yaw);
    float cy = cosf(yaw);

    //    1 yaw       2 roll   3 pitch
    mat33_t_R[0][0] = cy * cr;
    mat33_t_R[0][1] = (cy * sr * sp + cp * sy);
    mat33_t_R[0][2] = sy * sp - cy * cp * sr;

    mat33_t_R[1][0] = -(cr * sy);
    mat33_t_R[1][1] = -(sy * sr * sp -cy * cp);
    mat33_t_R[1][2] =  cp * sr * sy + sp * cy;

    mat33_t_R[2][0] = sr;
    mat33_t_R[2][1] = -(cr * sp);
    mat33_t_R[2][2] = cr * cp;

    quat_init[0] = sqrtf(clamp( mat33_t_R[0][0] - mat33_t_R[1][1] - mat33_t_R[2][2] + 1) * 0.25f);
    quat_init[1] = sqrtf(clamp(-mat33_t_R[0][0] + mat33_t_R[1][1] - mat33_t_R[2][2] + 1) * 0.25f);
    quat_init[2] = sqrtf(clamp(-mat33_t_R[0][0] - mat33_t_R[1][1] + mat33_t_R[2][2] + 1) * 0.25f);
    quat_init[3] = sqrtf(clamp( mat33_t_R[0][0] + mat33_t_R[1][1] + mat33_t_R[2][2] + 1) * 0.25f);

    quat_init[0] = copysignf(quat_init[0], mat33_t_R[2][1] - mat33_t_R[1][2]);
    quat_init[1] = copysignf(quat_init[1], mat33_t_R[0][2] - mat33_t_R[2][0]);
    quat_init[2] = copysignf(quat_init[2], mat33_t_R[1][0] - mat33_t_R[0][1]);

    float q_length =  sqrtf(quat_init[0] * quat_init[0] + quat_init[1] * quat_init[1] + quat_init[2] * quat_init[2] + quat_init[3] * quat_init[3]);//归一化
    if(q_length > 0.95f && q_length < 1.05f)
    {
        fusion_9axis_inited = 1;
    } 
    else
    {
        fusion_9axis_inited = 0;
    }

    axis9.q[0] = quat_init[3] / q_length;
    axis9.q[1] = quat_init[0] / q_length;
    axis9.q[2] = quat_init[1] / q_length;
    axis9.q[3] = quat_init[2] / q_length;

	qst_fusion_9axis_rotation_matrix();

#if defined(QST_FUSION_9AXIS_LOCK_EULER)
	stdAcc.size = 0;
	stdGyr.size = 0;
#endif
    return fusion_9axis_inited;
}


void qst_fusion_9axis_set_yaw(float angle_in[3])
{
	float mat33_t_R[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    float fusion_9axis_q_init[4];

	float yaw = angle_in[0];
	float pitch = angle_in[1];
	float roll = angle_in[2];

	float cp = cosf(pitch);
	float sp = sinf(pitch);
	float sr = sinf(roll);
	float cr = cosf(roll);
	float sy = sinf(yaw);
	float cy = cosf(yaw);

	//	  1 yaw 	  2 roll   3 pitch
	mat33_t_R[0][0] = cy * cr;
	mat33_t_R[0][1] = (cy * sr * sp + cp * sy);
	mat33_t_R[0][2] = sy * sp - cy * cp * sr;

	mat33_t_R[1][0] = -(cr * sy);
	mat33_t_R[1][1] = -(sy * sr * sp -cy * cp);
	mat33_t_R[1][2] =  cp * sr * sy + sp * cy;

	mat33_t_R[2][0] = sr;
	mat33_t_R[2][1] = -(cr * sp);
	mat33_t_R[2][2] = cr * cp;

	fusion_9axis_q_init[0] = sqrtf(clamp( mat33_t_R[0][0] - mat33_t_R[1][1] - mat33_t_R[2][2] + 1) * 0.25f);
	fusion_9axis_q_init[1] = sqrtf(clamp(-mat33_t_R[0][0] + mat33_t_R[1][1] - mat33_t_R[2][2] + 1) * 0.25f);
	fusion_9axis_q_init[2] = sqrtf(clamp(-mat33_t_R[0][0] - mat33_t_R[1][1] + mat33_t_R[2][2] + 1) * 0.25f);
	fusion_9axis_q_init[3] = sqrtf(clamp( mat33_t_R[0][0] + mat33_t_R[1][1] + mat33_t_R[2][2] + 1) * 0.25f);


	fusion_9axis_q_init[0] = copysignf(fusion_9axis_q_init[0], mat33_t_R[2][1] - mat33_t_R[1][2]);
	fusion_9axis_q_init[1] = copysignf(fusion_9axis_q_init[1], mat33_t_R[0][2] - mat33_t_R[2][0]);
	fusion_9axis_q_init[2] = copysignf(fusion_9axis_q_init[2], mat33_t_R[1][0] - mat33_t_R[0][1]);

	float q_length =  sqrtf(fusion_9axis_q_init[0] * fusion_9axis_q_init[0] + fusion_9axis_q_init[1] * fusion_9axis_q_init[1] + fusion_9axis_q_init[2] * fusion_9axis_q_init[2] + fusion_9axis_q_init[3] * fusion_9axis_q_init[3]);//归一化
	fusion_9axis_q_init[0] = fusion_9axis_q_init[0] / q_length;
	fusion_9axis_q_init[1] = fusion_9axis_q_init[1] / q_length;
	fusion_9axis_q_init[2] = fusion_9axis_q_init[2] / q_length;
	fusion_9axis_q_init[3] = fusion_9axis_q_init[3] / q_length;

    axis9.q[0] = fusion_9axis_q_init[3];
    axis9.q[1] = fusion_9axis_q_init[0];
    axis9.q[2] = fusion_9axis_q_init[1];
    axis9.q[3] = fusion_9axis_q_init[2];
}

#if defined(QST_FUSION_9AXIS_LOCK_EULER)
static float rpy_out[3];
static float rpy_out_lock[3];
#endif
//static int yaw_mag_timer;
static float yaw_gyro_recovery = 0;
static float mag_R_init = 0;
#define GYRO_THRESH 2.0
#define BUFFSER_SIZE 10
static float gyro_buffer2[BUFFSER_SIZE] = {0};
static float mag_buffer2[3] = {0, 0, 0};

void qst_fusion_9axis_update(float acc[3], float gyr[3], float mag[3], float dt, unsigned char reset) {
    float ex_9axis = 0.0f;
    float ey_9axis = 0.0f;
    float ez_9axis = 0.0f;
    float integral_dt = dt;
    float halfT = 0.5f * integral_dt;
    float accel_tmp[3] = {0.0f, 0.0f, 0.0f};
    float gyro_tmp[3] = {0.0f, 0.0f, 0.0f};
    float mag_tmp[3] = {0.0f, 0.0f, 0.0f};

    unsigned char axis = 0;

    float mag_norm = sqrtf(mag[0] * mag[0] + mag[1] * mag[1] + mag[2] * mag[2]);
    if(mag_norm < 1.0f)
    {
        return;
    }
    float acc_norm = sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
    if(acc_norm < 1.0f)
    {
        return;
    }
    float gyr_norm = sqrtf(gyr[0] * gyr[0] + gyr[1] * gyr[1] + gyr[2] * gyr[2]);

    if(integral_dt > 0.5f)
    {
        fusion_9axis_inited = 0;
    }
    else
    {
        if(integral_dt < 1.e-6f || integral_dt > 0.2f)
        {
            integral_dt = 0.02f;
            halfT = 0.5f * integral_dt;
        }
    }

    if(reset)
    {
        fusion_9axis_inited = 0;
    }

    for(axis = 0; axis < 3; axis++)
    {
        accel_tmp[axis] = acc[axis];
        gyro_tmp[axis] = gyr[axis];
        mag_tmp[axis] = mag[axis];
    }

#ifdef ENBALE_DEBUG
    LOGD("qst_log0: %f %f %f %f %f %f %f %f %f\n", accel_tmp[0],accel_tmp[1],accel_tmp[2],gyro_tmp[0],gyro_tmp[1],gyro_tmp[2],mag_tmp[0],mag_tmp[1],mag_tmp[2]);
#endif

    if(!fusion_9axis_inited)
    {
        qst_fusion_9axis_init(accel_tmp, mag_tmp);
#if defined(QST_FUSION_9D_USE_MAG_RR)
        rr_sync_to_fusion(&mag_R_init);
#else
		mag_R_init = 0.0f;
#endif
        return;
    }

#if defined(QST_FUSION_9D_USE_MAG_RR)
    int update_offset_flag = 0;
    ical_update_offset_flag_sync_to_fusion(&update_offset_flag);
    
    if(update_offset_flag > 0)
    {
        fusion_9axis_mode = 1;
        if(update_offset_flag < 3)
        {
            rr_sync_to_fusion(&mag_R_init);
        }
    }
#endif
    float gz = axis9.mat[2][0]*gyro_tmp[0] + axis9.mat[2][1]*gyro_tmp[1] + axis9.mat[2][2]*gyro_tmp[2];
    gz = Filter_Apply(gz, &gyro_buffer[2], &gyro_filter_parameter);
    float recip_norm = sqrtf(mag_tmp[0] * mag_tmp[0] + mag_tmp[1] * mag_tmp[1] + mag_tmp[2] * mag_tmp[2]);
    if((mag_R_init>1.0f)&&(recip_norm < (mag_R_init - 6) || recip_norm >  (mag_R_init + 6)))
    {
        /***********9切6判断:超过磁场半径范围**************************/
#if defined(QST_FUSION_9D_USE_MAG_RR)
        if(update_offset_flag < 1)
        {
            if(fusion_9axis_mode == 1)
            {
                fusion_9axis_mode = 0;
				#ifdef ENBALE_DEBUG
					LOGD("qst_log6: 8888 \n");
				#endif
            }
        }
        yaw_gyro_recovery = 0;
#endif
        /***********9切6判断:超过磁场半径范围**************************/
    }else{
        /***********6切9恢复判断**************************/
        if(fabs(gz) > gyro_max * 5)//0.02)
        {
            yaw_gyro_recovery += -gz * integral_dt * TODEG;

            if(fabs(yaw_gyro_recovery) > 89)
            {
                if(fusion_9axis_mode == 0)
                {
                    fusion_9axis_mode = 1;
                }
            }
        }else{
            yaw_gyro_recovery = 0;
        }
        /*************6切9恢复************************/
    }

    for(axis = 0; axis < BUFFSER_SIZE - 1; axis++)
    {
        gyro_buffer2[axis] = gyro_buffer2[axis + 1];
    }
    gyro_buffer2[axis] = gyr_norm;	//sqrtf(gyro_tmp[0] * gyro_tmp[0] + gyro_tmp[1] * gyro_tmp[1] + gyro_tmp[2] * gyro_tmp[2]);//gz;

#if defined(QST_FUSION_9D_USE_MAG_RR)
    if(update_offset_flag < 1)
#endif
    {
        if(fabs(gyro_buffer2[0]) < GYRO_THRESH && fabs(gyro_buffer2[1]) < GYRO_THRESH && fabs(gyro_buffer2[2]) < GYRO_THRESH && fabs(gyro_buffer2[3]) < GYRO_THRESH && fabs(gyro_buffer2[4]) < GYRO_THRESH
            && fabs(gyro_buffer2[5]) < GYRO_THRESH && fabs(gyro_buffer2[6]) < GYRO_THRESH && fabs(gyro_buffer2[7]) < GYRO_THRESH && fabs(gyro_buffer2[8]) < GYRO_THRESH && fabs(gyro_buffer2[9]) < GYRO_THRESH)
        {
            float mag_normal = fabs(mag_tmp[0] - mag_buffer2[0]) + fabs(mag_tmp[1] - mag_buffer2[1]) + fabs(mag_tmp[2] - mag_buffer2[2]);
            if(mag_normal > 5)
            {
                if(fusion_9axis_mode == 1)
                {
                    fusion_9axis_mode = 0;
					#ifdef ENBALE_DEBUG
						LOGD("qst_log7: 9999 \n");
					#endif
                    //LOGD("ma_q2 8888 \n");
                }
            }
        }
    }

    mag_buffer2[0] = mag_tmp[0];
    mag_buffer2[1] = mag_tmp[1];
    mag_buffer2[2] = mag_tmp[2];

    if(fusion_9axis_mode == 0)
    {
        if(fusion_9axis_mode_last == 1)
        {
            fusion_9axis_exInt = 0.0f;
            fusion_9axis_eyInt = 0.0f;
            fusion_9axis_ezInt = 0.0f;
        }
    }

    fusion_9axis_mode_last = fusion_9axis_mode;

	float spinRate = sqrtf(gyro_tmp[0] * gyro_tmp[0] + gyro_tmp[1] * gyro_tmp[1] + gyro_tmp[2] * gyro_tmp[2]);
    if(fusion_9axis_mode)
    {
        // Normalise magnetometer measurement
        mag_tmp[0] = mag_tmp[0] / recip_norm;
        mag_tmp[1] = mag_tmp[1] / recip_norm;
        mag_tmp[2] = mag_tmp[2] / recip_norm;

        // Magnetometer correction
        // Project mag field vector to global frame and extract XY component
        float hx = axis9.mat[0][0] * mag_tmp[0] + axis9.mat[0][1] * mag_tmp[1] + axis9.mat[0][2] * mag_tmp[2];
        float hy = axis9.mat[1][0] * mag_tmp[0] + axis9.mat[1][1] * mag_tmp[1] + axis9.mat[1][2] * mag_tmp[2];
        float hz = axis9.mat[2][0] * mag_tmp[0] + axis9.mat[2][1] * mag_tmp[1] + axis9.mat[2][2] * mag_tmp[2];

        float by = sqrtf(hx * hx + hy * hy);
        float bz = hz;

        //n系中的地磁向量[bx,by,bz]转换到b系中，得到[wx,wy,wz]
        float wx = axis9.mat[1][0] * by + axis9.mat[2][0] * bz;
        float wy = axis9.mat[1][1] * by + axis9.mat[2][1] * bz;
        float wz = axis9.mat[1][2] * by + axis9.mat[2][2] * bz;

		float fifty_dps =  0.873f;//50dps
		float gainMult = 1.0f;
		if(spinRate > fifty_dps)
		{
			gainMult = QST_MIN((spinRate / fifty_dps), 2.50f);
		}
		ex_9axis = (mag_tmp[1] * wz - mag_tmp[2] * wy) * fusion_9axis_mag_kp * gainMult;
		ey_9axis = (mag_tmp[2] * wx - mag_tmp[0] * wz) * fusion_9axis_mag_kp * gainMult;
		ez_9axis = (mag_tmp[0] * wy - mag_tmp[1] * wx) * fusion_9axis_mag_kp * gainMult;
    }

    //recip_norm = sqrtf(accel_tmp[0] * accel_tmp[0] + accel_tmp[1] * accel_tmp[1] + accel_tmp[2] * accel_tmp[2]);
	if(acc_norm > GRAVITY_MSS * 0.9f && acc_norm <  GRAVITY_MSS * 1.1f)
	{
        accel_tmp[0] = accel_tmp[0] / acc_norm;
        accel_tmp[1] = accel_tmp[1] / acc_norm;
        accel_tmp[2] = accel_tmp[2] / acc_norm;

        ex_9axis += (accel_tmp[1] * axis9.mat[2][2] - accel_tmp[2] * axis9.mat[2][1]) * fusion_9axis_kp;
        ey_9axis += (accel_tmp[2] * axis9.mat[2][0] - accel_tmp[0] * axis9.mat[2][2]) * fusion_9axis_kp;
        ez_9axis += (accel_tmp[0] * axis9.mat[2][1] - accel_tmp[1] * axis9.mat[2][0]) * fusion_9axis_kp;
    }

    if(gyro_tmp[0] > -gyro_max && gyro_tmp[0] <  gyro_max)
    {
        gyro_tmp[0] = 0;
    }
    if(gyro_tmp[1] > -gyro_max && gyro_tmp[1] <  gyro_max)
    {
        gyro_tmp[1] = 0;
    }
    if(gyro_tmp[2] > -gyro_max && gyro_tmp[2] <  gyro_max)
    {
        gyro_tmp[2] = 0;
    }

	spinRate = sqrtf(gyro_tmp[0] * gyro_tmp[0] + gyro_tmp[1] * gyro_tmp[1] + gyro_tmp[2] * gyro_tmp[2]);
	if(spinRate < 0.175f)///0.873f)//0.175f)//0.34f)//1.075f)
	{
		fusion_9axis_exInt += ex_9axis * fusion_9axis_ki * integral_dt;
		fusion_9axis_eyInt += ey_9axis * fusion_9axis_ki * integral_dt;
		fusion_9axis_ezInt += ez_9axis * fusion_9axis_ki * integral_dt;

		fusion_9axis_exInt = constrain_float(fusion_9axis_exInt, -fusion_9axis_Int_max, fusion_9axis_Int_max);
		fusion_9axis_eyInt = constrain_float(fusion_9axis_eyInt, -fusion_9axis_Int_max, fusion_9axis_Int_max);
		fusion_9axis_ezInt = constrain_float(fusion_9axis_ezInt, -fusion_9axis_Int_max, fusion_9axis_Int_max);
	}
	else
	{
		fusion_9axis_exInt = 0.0f;
		fusion_9axis_eyInt = 0.0f;
		fusion_9axis_ezInt = 0.0f;
	}

    #ifdef ENBALE_DEBUG
    LOGD("qst_log1: %f %f %f\n", fusion_9axis_exInt, fusion_9axis_eyInt, fusion_9axis_ezInt);
    #endif

    gyro_tmp[0] = gyro_tmp[0] + ex_9axis + fusion_9axis_exInt;
    gyro_tmp[1] = gyro_tmp[1] + ey_9axis + fusion_9axis_eyInt;
    gyro_tmp[2] = gyro_tmp[2] + ez_9axis + fusion_9axis_ezInt;

    float qw = axis9.q[0];
    float qx = axis9.q[1];
    float qy = axis9.q[2];
    float qz = axis9.q[3];

    axis9.q[0] += (-qx * gyro_tmp[0] - qy * gyro_tmp[1] - qz * gyro_tmp[2]) * halfT;
    axis9.q[1] += (+qw * gyro_tmp[0] + qy * gyro_tmp[2] - qz * gyro_tmp[1]) * halfT;
    axis9.q[2] += (+qw * gyro_tmp[1] - qx * gyro_tmp[2] + qz * gyro_tmp[0]) * halfT;
    axis9.q[3] += (+qw * gyro_tmp[2] + qx * gyro_tmp[1] - qy * gyro_tmp[0]) * halfT;

	//qst_quat_to_matrix(axis9.q, axis9.mat);

    recip_norm = sqrtf(axis9.q[0] * axis9.q[0] + axis9.q[1] * axis9.q[1] + axis9.q[2] * axis9.q[2] + axis9.q[3] * axis9.q[3]);
    axis9.q[0] = axis9.q[0] / recip_norm;
    axis9.q[1] = axis9.q[1] / recip_norm;
    axis9.q[2] = axis9.q[2] / recip_norm;
    axis9.q[3] = axis9.q[3] / recip_norm;

#if defined(QST_FUSION_9AXIS_LOCK_EULER)
	float accel_standard_deviation = 0.0f;
	float gyro_standard_deviation = 0.0f;

	getStandardDeviation1(&stdAcc, acc_norm, &accel_standard_deviation);
	getStandardDeviation1(&stdGyr, gyr_norm, &gyro_standard_deviation);
	//LOGD("%f %f\n", gyro_norm, accel_standard_deviation);

    if(spinRate > gyro_max * 4)    //
    {
        static_flag = 0;
        static_delay = 0;
    }
	if(accel_standard_deviation < 0.6f)
    {
		if(gyro_standard_deviation < 0.02f)
		{
			if(static_flag == 0)
			{
				if(static_delay < 80)
				{
					static_delay++;
					if(static_delay == 60)
					{
						if(fusion_9axis_mode)
						{
							qst_fusion_9axis_init(accel_tmp, mag_tmp);
						}
					}
				}
				else
				{
					static_delay = 0;
					static_flag = 1;
					rpy_out_lock[0] = rpy_out[0];
				}
			}
			else
			{
				rpy_out_lock[1] = rpy_out[1];
				rpy_out_lock[2] = rpy_out[2];
                qst_fusion_9axis_set_yaw(rpy_out_lock);
                #ifdef ENBALE_DEBUG
                LOGD("qst_log4: %f %f %f\n", rpy_out_lock[0], rpy_out_lock[1],rpy_out_lock[2]);
                #endif
            }
		}
	}
	else
	{
		static_delay = 0;
		static_flag = 0;
	}
#endif
	
	qst_fusion_9axis_rotation_matrix();
#if defined(QST_FUSION_9AXIS_LOCK_EULER)
    rpy_out[2]  = asinf(axis9.mat[2][0]);
    rpy_out[1] = atan2f(-axis9.mat[2][1], axis9.mat[2][2]);
    rpy_out[0] = atan2f(-axis9.mat[1][0], axis9.mat[0][0]);
#endif
    #ifdef ENBALE_DEBUG
    LOGD("qst_log2: %d %f %f %f %f\n", fusion_9axis_mode,integral_dt,  rpy_out[0] * TODEG, rpy_out[1] * TODEG, rpy_out[2] * TODEG);
    #endif
}


void qst_fusion_9axis_get_quaternion(float quat[4])
{
    quat[0] = sqrtf(clamp( axis9.mat[0][0] - axis9.mat[1][1] - axis9.mat[2][2] + 1) * 0.25f);
    quat[1] = sqrtf(clamp(-axis9.mat[0][0] + axis9.mat[1][1] - axis9.mat[2][2] + 1) * 0.25f);
    quat[2] = sqrtf(clamp(-axis9.mat[0][0] - axis9.mat[1][1] + axis9.mat[2][2] + 1) * 0.25f);
    quat[3] = sqrtf(clamp( axis9.mat[0][0] + axis9.mat[1][1] + axis9.mat[2][2] + 1) * 0.25f);
    quat[0] = copysignf(quat[0], axis9.mat[2][1] - axis9.mat[1][2]);
    quat[1] = copysignf(quat[1], axis9.mat[0][2] - axis9.mat[2][0]);
    quat[2] = copysignf(quat[2], axis9.mat[1][0] - axis9.mat[0][1]);
}


static qst_fusion_data noMag;

#define fusion_nomag_kp					(6.0f)
#define fusion_nomag_ki					(0.1f)
#define fusion_nomag_Int_max			(0.05f)
static float fusion_nomag_exInt = 0.0f;
static float fusion_nomag_eyInt = 0.0f;
static float fusion_nomag_ezInt = 0.0f;

static unsigned char fusion_nomag_inited = 0;

#if 0
void qst_fusion_nomag_quat_to_matrix(float quat[4], float mat[][3])
{
    float q2q2 = quat[2] * quat[2];
    float q2q3 = quat[2] * quat[3];
    float q1q1 = quat[1] * quat[1];
    float q1q2 = quat[1] * quat[2];
    float q1q3 = quat[1] * quat[3];
    float q0q1 = quat[0] * quat[1];
    float q0q2 = quat[0] * quat[2];
    float q0q3 = quat[0] * quat[3];
    float q3q3 = quat[3] * quat[3];

    mat[0][0] = 1 - 2 * (q2q2 + q3q3);
    mat[0][1] = 2 * (q1q2 - q0q3);
    mat[0][2] = 2 * (q1q3 + q0q2);

    mat[1][0] = 2 * (q1q2 + q0q3);
    mat[1][1] = 1 - 2 * (q1q1 + q3q3);
    mat[1][2] = 2 * (q2q3 - q0q1);

    mat[2][0] = 2 * (q1q3 - q0q2);
    mat[2][1] = 2 * (q2q3 + q0q1);
    mat[2][2] = 1-2 * (q1q1 + q2q2);
}
#endif

void qst_fusion_nomag_init(float acc[3])
{
    //float accel_data[3] = {0, 0, 0};
    float mag_data[3] = {0, 0, 0};
    float euler[3] = {0, 0, 0};
//    float mat33_t_R[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
	float quat_init[4] = {0, 0, 0, 0};

	noMag.q[0] = 1.0f;		// w
	noMag.q[1] = 0.0f;		// x
	noMag.q[2] = 0.0f;		// y
	noMag.q[3] = 0.0f;		// z

	noMag.mat[0][0] = 1.0f;
	noMag.mat[0][1] = 0.0f;
	noMag.mat[0][2] = 0.0f;
	noMag.mat[1][0] = 0.0f;
	noMag.mat[1][1] = 1.0f;
	noMag.mat[1][2] = 0.0f;
	noMag.mat[2][0] = 0.0f;
	noMag.mat[2][1] = 0.0f;
	noMag.mat[2][2] = 1.0f;

	//accel_data[0] = acc[0];
	//accel_data[1] = acc[1];
	//accel_data[2] = acc[2];

	android_raw_to_euler(acc, mag_data, euler);
	android_euler_to_matrix(euler, noMag.mat);
	android_matrix_to_quat(noMag.mat, quat_init);
    float q_length = sqrtf(quat_init[0]*quat_init[0] + quat_init[1]*quat_init[1] + quat_init[2]*quat_init[2] + quat_init[3]*quat_init[3]);//归一化

    if(q_length > 0.95f && q_length < 1.05f)
    {
        fusion_nomag_inited = 1;
    } 
    else
    {
        fusion_nomag_inited = 0;
    }
	// android quat to qst quat
    noMag.q[0] = quat_init[3]/q_length;
    noMag.q[1] = quat_init[0]/q_length;
    noMag.q[2] = quat_init[1]/q_length;
    noMag.q[3] = quat_init[2]/q_length;
}


void qst_fusion_nomag_update(float acc[3], float gyr[3], float dt, unsigned char reset)
{
    float ex = 0.0f;
    float ey = 0.0f;
    float ez = 0.0f;

    float halfT = 0.5f * dt;

    float accel_tmp[3] = {0.0f, 0.0f, 0.0f};
    float gyro_tmp[3] = {0.0f, 0.0f, 0.0f};

    accel_tmp[0] = acc[0];
    accel_tmp[1] = acc[1];
    accel_tmp[2] = acc[2];
    
    gyro_tmp[0] = gyr[0];
    gyro_tmp[1] = gyr[1];
    gyro_tmp[2] = gyr[2];

    if(dt > 0.5f)
    {
        fusion_nomag_inited = 0;
    }
    else
    {
        if(dt < 0.0f || dt > 0.04f)
        {
            dt = 0.005f;
            halfT = 0.5f * dt;
        }
    }

    if(reset)
    {
        fusion_nomag_inited = 0;
    }

    if(!fusion_nomag_inited)
    {
        qst_fusion_nomag_init(accel_tmp);
        return ;
    }

    float recip_norm = sqrtf(accel_tmp[0] * accel_tmp[0] + accel_tmp[1] * accel_tmp[1] + accel_tmp[2] * accel_tmp[2]);
    if(recip_norm > GRAVITY_MSS * 0.9f && recip_norm <  GRAVITY_MSS * 1.1f)
    {
        accel_tmp[0] = accel_tmp[0] / recip_norm;
        accel_tmp[1] = accel_tmp[1] / recip_norm;
        accel_tmp[2] = accel_tmp[2] / recip_norm;

        ex = (accel_tmp[1] * noMag.mat[2][2] - accel_tmp[2] * noMag.mat[2][1]) * fusion_nomag_kp;
        ey = (accel_tmp[2] * noMag.mat[2][0] - accel_tmp[0] * noMag.mat[2][2]) * fusion_nomag_kp;
        ez = (accel_tmp[0] * noMag.mat[2][1] - accel_tmp[1] * noMag.mat[2][0]) * fusion_nomag_kp;
    }


    float spinRate = sqrtf(gyro_tmp[0] * gyro_tmp[0] + gyro_tmp[1] * gyro_tmp[1] + gyro_tmp[2] * gyro_tmp[2]);
    if(spinRate < 0.35f) //20
    {
        fusion_nomag_exInt += ex * fusion_nomag_ki * dt;
        fusion_nomag_eyInt += ey * fusion_nomag_ki * dt;
        fusion_nomag_ezInt += ez * fusion_nomag_ki * dt;
        fusion_nomag_exInt = constrain_float(fusion_nomag_exInt, -fusion_nomag_Int_max, fusion_nomag_Int_max);
        fusion_nomag_eyInt = constrain_float(fusion_nomag_eyInt, -fusion_nomag_Int_max, fusion_nomag_Int_max);
        fusion_nomag_ezInt = constrain_float(fusion_nomag_ezInt, -fusion_nomag_Int_max, fusion_nomag_Int_max);
    }
    else
    {
        fusion_nomag_exInt = 0.0f;
        fusion_nomag_eyInt = 0.0f;
        fusion_nomag_ezInt = 0.0f;
    }

    gyro_tmp[0] = gyro_tmp[0] + ex + fusion_nomag_exInt;
    gyro_tmp[1] = gyro_tmp[1] + ey + fusion_nomag_eyInt;
    gyro_tmp[2] = gyro_tmp[2] + ez + fusion_nomag_ezInt;

    float qw = noMag.q[0];
    float qx = noMag.q[1];
    float qy = noMag.q[2];
    float qz = noMag.q[3];

    noMag.q[0] += (-qx * gyro_tmp[0] - qy * gyro_tmp[1] - qz * gyro_tmp[2]) * halfT;
    noMag.q[1] += (+qw * gyro_tmp[0] + qy * gyro_tmp[2] - qz * gyro_tmp[1]) * halfT;
    noMag.q[2] += (+qw * gyro_tmp[1] - qx * gyro_tmp[2] + qz * gyro_tmp[0]) * halfT;
    noMag.q[3] += (+qw * gyro_tmp[2] + qx * gyro_tmp[1] - qy * gyro_tmp[0]) * halfT;

	qst_quat_to_matrix(noMag.q, noMag.mat);

    recip_norm = sqrtf(noMag.q[0]*noMag.q[0] + noMag.q[1]*noMag.q[1] + noMag.q[2]*noMag.q[2] + noMag.q[3]*noMag.q[3]);
	noMag.q[0] = noMag.q[0] / recip_norm;
	noMag.q[1] = noMag.q[1] / recip_norm;
	noMag.q[2] = noMag.q[2] / recip_norm;
	noMag.q[3] = noMag.q[3] / recip_norm;
	//qst_fusion_nomag_quat_to_matrix(noMag.q, noMag.mat);
}

void qst_fusion_nomag_get_quaternion(float quat[4])
{
    const float Hx = noMag.mat[0][0];
    const float My = noMag.mat[1][1];
    const float Az = noMag.mat[2][2];
    quat[0] = sqrtf( clamp( Hx - My - Az + 1) * 0.25f );
    quat[1] = sqrtf( clamp(-Hx + My - Az + 1) * 0.25f );
    quat[2] = sqrtf( clamp(-Hx - My + Az + 1) * 0.25f );
    quat[3] = sqrtf( clamp( Hx + My + Az + 1) * 0.25f );
    quat[0] = copysignf(quat[0], noMag.mat[2][1] - noMag.mat[1][2]);
    quat[1] = copysignf(quat[1], noMag.mat[0][2] - noMag.mat[2][0]);
    quat[2] = copysignf(quat[2], noMag.mat[1][0] - noMag.mat[0][1]);
}



static qst_fusion_data noGyr;
static unsigned char fusion_nogyro_inited = 0;

void qst_fusion_nogyro_init(void)
{
	noGyr.q[0] = 1.0f;
	noGyr.q[1] = 0.0f;
	noGyr.q[2] = 0.0f;
	noGyr.q[3] = 0.0f;

	noGyr.mat[0][0] = 1.0f;
	noGyr.mat[0][1] = 0.0f;
	noGyr.mat[0][2] = 0.0f;
	noGyr.mat[1][0] = 0.0f;
	noGyr.mat[1][1] = 1.0f;
	noGyr.mat[1][2] = 0.0f;
	noGyr.mat[2][0] = 0.0f;
	noGyr.mat[2][1] = 0.0f;
	noGyr.mat[2][2] = 1.0f;
}

void qst_fusion_nogyro_update(float acc[3], float mag[3])
{
    float accel_data[3] = {0.0f, 0.0f, 0.0f};
    float mag_data[3] = {0.0f, 0.0f, 0.0f};
    float mag_data_temp[3] = {0, 0, 0};
    float sinyaw = 0;
    float cosyaw = 0;
    float fusion_rot_mat1[3][3] = {{1.0f, 0.f, 0.f}, {0.0f, 1.0f, 0.f}, {0.0f, 0.f, 1.0f}};
    accel_data[0] = acc[0];
    accel_data[1] = acc[1];
    accel_data[2] = acc[2];

	if(fusion_nogyro_inited == 0)
	{
		qst_fusion_nogyro_init();
	}
    float normalize = sqrtf(accel_data[0] * accel_data[0] + accel_data[1] * accel_data[1] + accel_data[2] * accel_data[2]);
    if(normalize > GRAVITY_MSS * 0.8f && normalize <  GRAVITY_MSS * 1.2f){
        accel_data[0] = accel_data[0] / normalize;
        accel_data[1] = accel_data[1] / normalize;
        accel_data[2] = accel_data[2] / normalize;

        fusion_rot_mat1[2][0] = accel_data[0];
        fusion_rot_mat1[2][1] = accel_data[1]; 
        fusion_rot_mat1[2][2] = accel_data[2];

        if(fabs(fusion_rot_mat1[2][0]) > 0.5) 
        {  
            fusion_rot_mat1[1][0] = fusion_rot_mat1[2][1];  
            fusion_rot_mat1[1][1] = -fusion_rot_mat1[2][0];  
            fusion_rot_mat1[1][2] = 0; 
        }else{  
            fusion_rot_mat1[1][0] = 0;  
            fusion_rot_mat1[1][1] = fusion_rot_mat1[2][2]; 
            fusion_rot_mat1[1][2] = -fusion_rot_mat1[2][1];  
        }

        // normalization
        normalize = sqrtf(fusion_rot_mat1[1][0] * fusion_rot_mat1[1][0] + fusion_rot_mat1[1][1] * fusion_rot_mat1[1][1]+ fusion_rot_mat1[1][2] * fusion_rot_mat1[1][2]); 
        fusion_rot_mat1[1][0] = fusion_rot_mat1[1][0] / normalize; 
        fusion_rot_mat1[1][1] = fusion_rot_mat1[1][1] / normalize; 
        fusion_rot_mat1[1][2] = fusion_rot_mat1[1][2] / normalize;  

        //做叉积求C0(第一行) 
        fusion_rot_mat1[0][0] = fusion_rot_mat1[1][1] * fusion_rot_mat1[2][2] - fusion_rot_mat1[2][1] * fusion_rot_mat1[1][2]; 
        fusion_rot_mat1[0][1] = -(fusion_rot_mat1[1][0] * fusion_rot_mat1[2][2] - fusion_rot_mat1[2][0] * fusion_rot_mat1[1][2]); 
        fusion_rot_mat1[0][2] = fusion_rot_mat1[1][0] * fusion_rot_mat1[2][1] - fusion_rot_mat1[2][0] * fusion_rot_mat1[1][1];
        //构造水平姿态对准结束
        //计算磁力计的方位对准开始
        mag_data[0] = mag[0];
        mag_data[1] = mag[1];
        mag_data[2] = mag[2];

        normalize = sqrtf(mag_data[0] * mag_data[0] + mag_data[1] * mag_data[1] + mag_data[2] * mag_data[2]);
        if(normalize > 0){
            mag_data[0] = mag_data[0] / normalize;
            mag_data[1] = mag_data[1] / normalize;
            mag_data[2] = mag_data[2] / normalize;
            //转置
            mag_data_temp[0] = fusion_rot_mat1[0][0] * mag_data[0] + fusion_rot_mat1[0][1] * mag_data[1] + fusion_rot_mat1[0][2] * mag_data[2];
            mag_data_temp[1] = fusion_rot_mat1[1][0] * mag_data[0] + fusion_rot_mat1[1][1] * mag_data[1] + fusion_rot_mat1[1][2] * mag_data[2];
            mag_data_temp[2] = fusion_rot_mat1[2][0] * mag_data[0] + fusion_rot_mat1[2][1] * mag_data[1] + fusion_rot_mat1[2][2] * mag_data[2];
        }
            
        sinyaw = mag_data_temp[0] / sqrtf(mag_data_temp[0] * mag_data_temp[0] + mag_data_temp[1] * mag_data_temp[1]);    
        cosyaw = mag_data_temp[1] / sqrtf(mag_data_temp[0] * mag_data_temp[0] + mag_data_temp[1] * mag_data_temp[1]);

        if(sinyaw <= -1){
            sinyaw = -1;
        }else if(sinyaw >= 1){
            sinyaw = 1;
        }    
        if(cosyaw <= -1){
            cosyaw = -1;
        }else if(cosyaw >= 1){
            cosyaw = 1;
        }
        //计算磁力计的方位对准结束

        noGyr.mat[0][0] = cosyaw * fusion_rot_mat1[0][0] - sinyaw * fusion_rot_mat1[1][0];
        noGyr.mat[0][1] = cosyaw * fusion_rot_mat1[0][1] - sinyaw * fusion_rot_mat1[1][1];
        noGyr.mat[0][2] = cosyaw * fusion_rot_mat1[0][2] - sinyaw * fusion_rot_mat1[1][2];

        noGyr.mat[1][0] = sinyaw * fusion_rot_mat1[0][0] + cosyaw * fusion_rot_mat1[1][0];
        noGyr.mat[1][1] = sinyaw * fusion_rot_mat1[0][1] + cosyaw * fusion_rot_mat1[1][1];
        noGyr.mat[1][2] = sinyaw * fusion_rot_mat1[0][2] + cosyaw * fusion_rot_mat1[1][2];

        noGyr.mat[2][0] = fusion_rot_mat1[2][0];
        noGyr.mat[2][1] = fusion_rot_mat1[2][1];
        noGyr.mat[2][2] = fusion_rot_mat1[2][2];
#if 0
        if(noGyr.mat[0][0] >= noGyr.mat[1][1] + noGyr.mat[2][2]){
            noGyr.q[1] = 0.5 * sqrtf(1 + noGyr.mat[0][0] - noGyr.mat[1][1] - noGyr.mat[2][2]);
            noGyr.q[0] = (noGyr.mat[2][1] - noGyr.mat[1][2]) / (4 * noGyr.q[1]);
            noGyr.q[2] = (noGyr.mat[0][1] + noGyr.mat[1][0]) / (4 * noGyr.q[1]);
            noGyr.q[3] = (noGyr.mat[0][2] + noGyr.mat[2][0]) / (4 * noGyr.q[1]);
        }else if(noGyr.mat[1][1] >= noGyr.mat[0][0] + noGyr.mat[2][2]){
            noGyr.q[2] = 0.5 * sqrtf(1 - noGyr.mat[0][0] + noGyr.mat[1][1] - noGyr.mat[2][2]);
            noGyr.q[0] = (noGyr.mat[0][2] - noGyr.mat[2][0]) / (4 * noGyr.q[2]);
            noGyr.q[1] = (noGyr.mat[0][1] + noGyr.mat[1][0]) / (4 * noGyr.q[2]);
            noGyr.q[3] = (noGyr.mat[1][2] + noGyr.mat[2][1]) / (4 * noGyr.q[2]);
        }else if(noGyr.mat[2][2] >= noGyr.mat[0][0] + noGyr.mat[1][1]){
            noGyr.q[3] = 0.5 * sqrtf(1 - noGyr.mat[0][0] - noGyr.mat[1][1] + noGyr.mat[2][2]);
            noGyr.q[0] = (noGyr.mat[1][0] - noGyr.mat[0][1]) / (4 * noGyr.q[3]);
            noGyr.q[1] = (noGyr.mat[0][2] + noGyr.mat[2][0]) / (4 * noGyr.q[3]);
            noGyr.q[2] = (noGyr.mat[1][2] + noGyr.mat[2][1]) / (4 * noGyr.q[3]);
        }else{
            noGyr.q[0] = 0.5 * sqrtf(1 + noGyr.mat[0][0] + noGyr.mat[1][1] + noGyr.mat[2][2]);
            noGyr.q[1] = (noGyr.mat[2][1] - noGyr.mat[1][2]) / (4 * noGyr.q[0]);
            noGyr.q[2] = (noGyr.mat[0][2] - noGyr.mat[2][0]) / (4 * noGyr.q[0]);
            noGyr.q[3] = (noGyr.mat[1][0] - noGyr.mat[0][1]) / (4 * noGyr.q[0]);
        }
#endif
    }
}

void qst_fusion_nogyro_get_quaternion(float quat[4])
{
    const float Hx = noGyr.mat[0][0];
    const float My = noGyr.mat[1][1];
    const float Az = noGyr.mat[2][2];
    quat[0] = sqrtf( clamp( Hx - My - Az + 1) * 0.25f );
    quat[1] = sqrtf( clamp(-Hx + My - Az + 1) * 0.25f );
    quat[2] = sqrtf( clamp(-Hx - My + Az + 1) * 0.25f );
    quat[3] = sqrtf( clamp( Hx + My + Az + 1) * 0.25f );
    quat[0] = copysignf(quat[0], noGyr.mat[2][1] - noGyr.mat[1][2]);
    quat[1] = copysignf(quat[1], noGyr.mat[0][2] - noGyr.mat[2][0]);
    quat[2] = copysignf(quat[2], noGyr.mat[1][0] - noGyr.mat[0][1]);
}

void qst_fusion_get_gravity(float gra_data[3], int mode)
{
	if(mode == 1)
	{
        gra_data[0] = axis9.mat[2][0] * GRAVITY_MSS;
        gra_data[1] = axis9.mat[2][1] * GRAVITY_MSS;
        gra_data[2] = axis9.mat[2][2] * GRAVITY_MSS;
	}
    else if(mode == 2)
	{
        gra_data[0] = noMag.mat[2][0] * GRAVITY_MSS;
        gra_data[1] = noMag.mat[2][1] * GRAVITY_MSS;
        gra_data[2] = noMag.mat[2][2] * GRAVITY_MSS;
    }
	else
	{
        gra_data[0] = noGyr.mat[2][0] * GRAVITY_MSS;
        gra_data[1] = noGyr.mat[2][1] * GRAVITY_MSS;
        gra_data[2] = noGyr.mat[2][2] * GRAVITY_MSS;
    }
}

