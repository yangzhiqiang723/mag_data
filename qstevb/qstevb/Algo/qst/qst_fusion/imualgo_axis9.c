
#include "imualgo_axis9.h"
//#define IMUALGO_LOG

// #ifdef ENBALE_DEBUG
// #define GYRO_LOG            printf
// #endif

#define LPF_ALPHA           0.8f
#define MAX_CALI_COUNT      50
#define QST_FABS(v)         ((v<0)?(-1.0f*v):(v))
#define QST_ABS(v)          ((v<0)?(-1*v):(v))
#define Pi					3.14159265f
#define TODEG               57.2957796f
#define TORAD               Pi / 180.0f
#define GRAVITY_MSS         9.80665f





#define clamp(v) ((v) < 0 ? 0.0000001 : (v))

float fusion_9axis_q0 = 1.0f;
float fusion_9axis_q1 = 0.0f;
float fusion_9axis_q2 = 0.0f;
float fusion_9axis_q3 = 0.0f;


float fusion_9axis_kp = 4.0f;
float fusion_9axis_ki = 0.1f;


float fusion_9axis_mag_kp = 4.0f;

float fusion_9axis_Int_max = 0.05f;//0.15f;//
float fusion_9axis_exInt = 0;
float fusion_9axis_eyInt = 0;
float fusion_9axis_ezInt = 0;

float fusion_9axis_rot_mat[3][3] = {{1.0f, 0.f, 0.f}, {0.0f, 1.0f, 0.f}, {0.0f, 0.f, 1.0f}};

float fusion_9axis_quaternion[4]= {1, 0, 0, 0};

//extern int32_t AvgMagR;

/****************** 根据初始化欧拉角初始化四元数 *****************************/
unsigned char fusion_9axis_inited = 0;
float fusion_9axis_q_init[4] = {0, 0, 0, 0};
void qst_fusion_9axis_angle(float *rpy);

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



QST_Filter_Buffer gyro_buffer[3] = {{0,0}};
QST_Filter_Buffer accel_buffer[3] = {{0,0}};

float gyro_max = 0.005f;

#define BUFFSER_SIZE 10
float accel_buffer_speed[BUFFSER_SIZE] = {0, 0};

//#define BUFFSER_SIZE 10
float gyro_buffer_speed[BUFFSER_SIZE] = {0, 0};

uint32_t static_delay = 0;
uint32_t static_flag = 0;
uint8_t static_interference_flag = 0;
uint8_t fusion_9axis_mode = 1;
uint8_t fusion_9axis_mode_last = 1;
uint32_t interference_delay = 0;



/*!
 * \brief set second order butterworth filtering Parameters.
* \input float sample:accel and gyro filter input value.
* \input Butterworth_Buffer *buffer:accel and gyro filter buffer value.
* \input float cutoff_freq:accel and gyro filter cut-off frequency.
* \return filter output value
 */
float Filter_Apply(float sample, QST_Filter_Buffer *buffer, QST_Filter_Parameter *filter) 
{
    //do the filtering
    float delay_element_0 = sample - buffer->_delay_element_1 * filter->a1 - buffer->_delay_element_2 * filter->a2;

    const float output = delay_element_0 * filter->b0 + buffer->_delay_element_1 * filter->b1 + buffer->_delay_element_2 * filter->b2;

    buffer->_delay_element_2 = buffer->_delay_element_1;
    buffer->_delay_element_1 = delay_element_0;

    //return the value. Should be no need to check limits
    return output;
}



float fusion_min(float a, float b)
{
	return (a < b) ? a : b;
}


//constrain a value
float constrain_float(float amt, float low, float high) 
{
    return ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)));
}

/*
  wrap an angle defined in radians to -PI ~ PI (equivalent to +- 180 degrees)
 */
#if 0
static float wrap_PI(float angle_in_radians)
{
    while (angle_in_radians > Pi) 
    {
        angle_in_radians -= 2.0f * Pi;
    }
    while (angle_in_radians < -Pi) 
    {
        angle_in_radians += 2.0f * Pi;
    }
    return angle_in_radians;
}
#endif


void qst_fusion_9axis_rotation_matrix(void)
{
    float q1q1 = fusion_9axis_q1 * fusion_9axis_q1;
    float q2q2 = fusion_9axis_q2 * fusion_9axis_q2;
    float q3q3 = fusion_9axis_q3 * fusion_9axis_q3;

    float q1q2 = fusion_9axis_q1 * fusion_9axis_q2;
    float q0q3 = fusion_9axis_q0 * fusion_9axis_q3;
    float q1q3 = fusion_9axis_q1 * fusion_9axis_q3;

    float q0q2 = fusion_9axis_q0 * fusion_9axis_q2;
    float q2q3 = fusion_9axis_q2 * fusion_9axis_q3;
    float q0q1 = fusion_9axis_q0 * fusion_9axis_q1;

    fusion_9axis_rot_mat[0][0] = 1 - 2 * (q2q2 + q3q3);
    fusion_9axis_rot_mat[0][1] = 2 * (q1q2 - q0q3);
    fusion_9axis_rot_mat[0][2] = 2 * (q1q3 + q0q2);
  
    fusion_9axis_rot_mat[1][0] = 2 * (q1q2 + q0q3);
    fusion_9axis_rot_mat[1][1] = 1 - 2 * (q1q1 + q3q3);
    fusion_9axis_rot_mat[1][2] = 2 * (q2q3 - q0q1);
  
    fusion_9axis_rot_mat[2][0] = 2 * (q1q3 - q0q2);
    fusion_9axis_rot_mat[2][1] = 2 * (q2q3 + q0q1);
    fusion_9axis_rot_mat[2][2] = 1 - 2 * (q1q1 + q2q2);
}

#if 0
unsigned char qst_fusion_9axis_init(float accel_in[3], float mag_in[3]) 
{
    float accel_data[3] = {0, 0, 0};
    float mag_data[3] = {0, 0, 0};
    
    float mat33_t_R[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

    accel_data[0] = accel_in[0];
    accel_data[1] = accel_in[1];
    accel_data[2] = accel_in[2];
    
    float normalize = sqrtf(accel_data[0] * accel_data[0] + accel_data[1] * accel_data[1] + accel_data[2] * accel_data[2]);
    accel_data[0] = accel_data[0] / normalize;
    accel_data[1] = accel_data[1] / normalize;
    accel_data[2] = accel_data[2] / normalize;
    
    float pitch = -atan2f(accel_data[1], accel_data[2]);
    //fusion_9axis_init_pitch = pitch * TODEG;////俯仰角
    
    normalize = sqrtf(accel_data[0] * accel_data[0] + accel_data[1] * accel_data[1] + accel_data[2] * accel_data[2]);

    float roll = asinf((accel_data[0] / normalize));//横滚角
    //fusion_9axis_init_roll = roll * TODEG;//横滚角

    float cp = cosf(pitch);   //roll=0,  pitch=0;      //1
    float sp = sinf(pitch);   //0
    float sr = sinf(roll);    //0
    float cr = cosf(roll);    //  1

    mag_data[0] = mag_in[0];
    mag_data[1] = mag_in[1];
    mag_data[2] = mag_in[2];

    normalize = sqrtf(mag_data[0] * mag_data[0] + mag_data[1] * mag_data[1] + mag_data[2] * mag_data[2]);
    mag_data[0] = mag_data[0] / normalize;
    mag_data[1] = mag_data[1] / normalize;
    mag_data[2] = mag_data[2] / normalize;

    float hx = mag_data[0] * cr - mag_data[2] * sr;
    float hy = mag_data[0] * sp * sr + mag_data[1] * cp + mag_data[2] * cr * sp;
    float yaw = -atan2f(hx, hy);
    
    //    fusion_9axis_init_yaw = yaw * TODEG;
    //    if(fusion_9axis_init_yaw < 0.0f)
    //    {
    //        fusion_9axis_init_yaw += 360.0f;
    //    }

    //    printf("%d; %d; %d\r\n", (int32_t)fusion_9axis_init_pitch, (int32_t)fusion_9axis_init_roll, (int32_t)fusion_9axis_init_yaw);
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

    if(q_length > 0.95f && q_length < 1.05f)
    {
        fusion_9axis_inited = 1;
    } 
    else
    {
        fusion_9axis_inited = 0;
    }

    fusion_9axis_q0 = fusion_9axis_q_init[3];
    fusion_9axis_q1 = fusion_9axis_q_init[0];
    fusion_9axis_q2 = fusion_9axis_q_init[1];
    fusion_9axis_q3 = fusion_9axis_q_init[2];

    qst_fusion_9axis_rotation_matrix();

    return fusion_9axis_inited;
}
#else
// static void Qst_ori_matrix(float *Acc_data, float *Mag_data, struct qst_matrix *matrix)
static void Qst_9axis_matrix(float *Acc_data, float *Mag_data, float matrix[3][3])
{
    float C[3][3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
    float hx_mag = 0, hy_mag = 0, hz_mag = 0;
    float cosyaw = 0, sinyaw = 0, hx_C = 0, hy_C = 0;//, hz_C = 0;
    float normalize = sqrtf(Acc_data[0] * Acc_data[0] + Acc_data[1] * Acc_data[1] + Acc_data[2] * Acc_data[2]);
    C[2][0] = Acc_data[0] / normalize;
    C[2][1] = Acc_data[1] / normalize;
    C[2][2] = Acc_data[2] / normalize;

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
    //归一化
    normalize = sqrtf(C[1][0] * C[1][0] + C[1][1] * C[1][1]+ C[1][2] * C[1][2]);        
    C[1][0] = C[1][0] / normalize;
    C[1][1] = C[1][1] / normalize;
    C[1][2] = C[1][2] / normalize;

    //做叉积求C0(第一行)
    C[0][0] = C[1][1] * C[2][2] - C[2][1] * C[1][2];
    C[0][1] = -(C[1][0] * C[2][2] - C[2][0] * C[1][2]);
    C[0][2] = C[1][0] * C[2][1] - C[2][0] * C[1][1];

    normalize = sqrtf(Mag_data[0] * Mag_data[0] + Mag_data[1] * Mag_data[1] + Mag_data[2] * Mag_data[2]);
    hx_mag = Mag_data[0] / normalize;
    hy_mag = Mag_data[1] / normalize;
    hz_mag = Mag_data[2] / normalize;

    //地磁矢量
    hx_C = C[0][0] * hx_mag + C[0][1] * hy_mag + C[0][2] * hz_mag;
    hy_C = C[1][0] * hx_mag + C[1][1] * hy_mag + C[1][2] * hz_mag;
    //hz_C = C[2][0] * hx_mag + C[2][1] * hy_mag + C[2][2] * hz_mag;
    
    sinyaw = hx_C / sqrtf(hx_C * hx_C + hy_C * hy_C);
    cosyaw = hy_C / sqrtf(hx_C * hx_C + hy_C * hy_C);

    if(sinyaw <= -1.0)
    {
        sinyaw = -1;
    }
    else if(sinyaw >= 1.0)
    {
        sinyaw = 1;
    }
    if(cosyaw <= -1.0)
    {
        cosyaw = -1;
    }
    else if(cosyaw >= 1.0)
    {
        cosyaw = 1;
    }
    //Cnb = Cnh * Chb
    // matrix->cs[0][0] = cosyaw * C[0][0] - sinyaw * C[1][0];
    // matrix->cs[0][1] = cosyaw * C[0][1] - sinyaw * C[1][1];
    // matrix->cs[0][2] = cosyaw * C[0][2] - sinyaw * C[1][2];

    // matrix->cs[1][0] = sinyaw * C[0][0] + cosyaw * C[1][0];
    // matrix->cs[1][1] = sinyaw * C[0][1] + cosyaw * C[1][1];
    // matrix->cs[1][2] = sinyaw * C[0][2] + cosyaw * C[1][2];

    // matrix->cs[2][0] = C[2][0];
    // matrix->cs[2][1] = C[2][1];
    // matrix->cs[2][2] = C[2][2];

    
    matrix[0][0] = cosyaw * C[0][0] - sinyaw * C[1][0];
    matrix[0][1] = cosyaw * C[0][1] - sinyaw * C[1][1];
    matrix[0][2] = cosyaw * C[0][2] - sinyaw * C[1][2];

    matrix[1][0] = sinyaw * C[0][0] + cosyaw * C[1][0];
    matrix[1][1] = sinyaw * C[0][1] + cosyaw * C[1][1];
    matrix[1][2] = sinyaw * C[0][2] + cosyaw * C[1][2];

    matrix[2][0] = C[2][0];
    matrix[2][1] = C[2][1];
    matrix[2][2] = C[2][2];
}


int qst_fusion_9axis_init(float accel_in[3], float mag_in[3]) 
{
    
    float accel_data[3] = {0, 0, 0};
    float mag_data[3] = {0, 0, 0};
    float mat33_t_R[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

    accel_data[0] = accel_in[0];
    accel_data[1] = accel_in[1];
    accel_data[2] = accel_in[2];

    mag_data[0] = mag_in[0];
    mag_data[1] = mag_in[1];
    mag_data[2] = mag_in[2];
    #if 1
    Qst_9axis_matrix(accel_data, mag_data, mat33_t_R);
    #else
    accel_data[0] = accel_in[0];
    accel_data[1] = accel_in[1];
    accel_data[2] = accel_in[2];
    
    float normalize = sqrtf(accel_data[0] * accel_data[0] + accel_data[1] * accel_data[1] + accel_data[2] * accel_data[2]);
    accel_data[0] = accel_data[0] / normalize;
    accel_data[1] = accel_data[1] / normalize;
    accel_data[2] = accel_data[2] / normalize;
    
    float pitch = -atan2f(accel_data[1], accel_data[2]);
    //fusion_9axis_init_pitch = pitch * TODEG;////俯仰角
    
    normalize = sqrtf(accel_data[0] * accel_data[0] + accel_data[1] * accel_data[1] + accel_data[2] * accel_data[2]);

    float roll = asinf((accel_data[0] / normalize));//横滚角
    //fusion_9axis_init_roll = roll * TODEG;//横滚角

    float cp = cosf(pitch);   //roll=0,  pitch=0;      //1
    float sp = sinf(pitch);   //0
    float sr = sinf(roll);    //0
    float cr = cosf(roll);    //  1

    mag_data[0] = mag_in[0];
    mag_data[1] = mag_in[1];
    mag_data[2] = mag_in[2];

    normalize = sqrtf(mag_data[0] * mag_data[0] + mag_data[1] * mag_data[1] + mag_data[2] * mag_data[2]);
    mag_data[0] = mag_data[0] / normalize;
    mag_data[1] = mag_data[1] / normalize;
    mag_data[2] = mag_data[2] / normalize;

    float hx = mag_data[0] * cr - mag_data[2] * sr;
    float hy = mag_data[0] * sp * sr + mag_data[1] * cp + mag_data[2] * cr * sp;
    float yaw = -atan2f(hx, hy);
    
    //    fusion_9axis_init_yaw = yaw * TODEG;
    //    if(fusion_9axis_init_yaw < 0.0f)
    //    {
    //        fusion_9axis_init_yaw += 360.0f;
    //    }

    //    printf("%d; %d; %d\r\n", (int32_t)fusion_9axis_init_pitch, (int32_t)fusion_9axis_init_roll, (int32_t)fusion_9axis_init_yaw);
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
    #endif
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

    if(q_length > 0.95f && q_length < 1.05f)
    {
        fusion_9axis_inited = 1;
    } 
    else
    {
        fusion_9axis_inited = 0;
    }

    fusion_9axis_q0 = fusion_9axis_q_init[3];
    fusion_9axis_q1 = fusion_9axis_q_init[0];
    fusion_9axis_q2 = fusion_9axis_q_init[1];
    fusion_9axis_q3 = fusion_9axis_q_init[2];

    qst_fusion_9axis_rotation_matrix();

    return fusion_9axis_inited;
}

#endif

void qst_fusion_9axis_set_yaw(float angle_in[3])
{
    float mat33_t_R[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

    float yaw = angle_in[0];
    float pitch = angle_in[1];
    float roll = angle_in[2];    //横滚角

    float cp = cosf(pitch);   //roll=0,  pitch=0;      //1
    float sp = sinf(pitch);                                 //0
    float sr = sinf(roll);                                     //0
    float cr = cosf(roll);                                   //  1
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

    ///*//和安卓坐标系一样
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

    fusion_9axis_q0 = fusion_9axis_q_init[3];
    fusion_9axis_q1 = fusion_9axis_q_init[0];
    fusion_9axis_q2 = fusion_9axis_q_init[1];
    fusion_9axis_q3 = fusion_9axis_q_init[2];
}




float rpy_out[3];
float rpy_out_lock[3];
int yaw_mag_timer;
float yaw_gyro_recovery = 0;

float mag_R_init = 0;


uint8_t imu_offset_static = 0;
uint16_t cali_count_static = 0;
float gyro_staticn_sum[3] = {0.0, 0.0, 0.0};
#define MAX_STATIC_COUNT 50
float gyro_static_offset[3];

uint8_t static_flag2 = 0;
uint16_t static_delay2 = 0;
#define GYRO_THRESH 2.0
float gyro_buffer2[BUFFSER_SIZE] = {0, 0};

float mag_buffer2[3] = {0, 0};
static int reopen_flag = 0;
static float mag_tems[3] ={0};
static float tems_mags[3] = {0};
static int flags_enable = 0;
static int lx_flag = 0;
static int gyro_quiet_flag = 0;
static int open_large_mag = 0;
static int open_large_mag_num = 0;


#if 1
//矩阵归一化
static void matrix_normal(float Weight[3][3])
{
	float temp_matrix[3][3];
	float normlize = 0;
	int i, j;
	float temp_data = 0;
	//第一列
	temp_data = Weight[0][0] * Weight[0][0] + Weight[1][0] * Weight[1][0] + Weight[2][0] * Weight[2][0];
	if (temp_data < 0)
	{
		temp_data = 1.0;
	}
	normlize = sqrtf(temp_data);
	for (i = 0;i < 3;i++)
	{
		temp_matrix[i][0] = Weight[i][0] / normlize;
	}
	//第二列
	temp_data = Weight[0][1] * Weight[0][1] + Weight[1][1] * Weight[1][1] + Weight[2][1] * Weight[2][1];
	if (temp_data < 0)
	{
		temp_data = 1.0;
	}
	normlize = sqrtf(temp_data);
	for (i = 0;i < 3;i++)
	{
		temp_matrix[i][1] = Weight[i][1] / normlize;
	}
	//第三列
	temp_data = Weight[0][2] * Weight[0][2] + Weight[1][2] * Weight[1][2] + Weight[2][2] * Weight[2][2];
	if (temp_data < 0)
	{
		temp_data = 1.0;
	}
	normlize = sqrtf(temp_data);
	for (i = 0;i < 3;i++)
	{
		temp_matrix[i][2] = Weight[i][2] / normlize;
	}
	for (i = 0;i < 3;i++)
	{
		for (j = 0;j < 3;j++)
		{
			Weight[i][j] = temp_matrix[i][j];
		}
	}


}

//创造矩阵微分方程
static void CreateFin(float gyro[3], float dt, float Fin[3][3])
{
	float Weight[3][3] = { 0 };
	int i = 0, j = 0;
	float gyro_norm = 0;
	float Deg = 0;
	gyro_norm = gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2];
	gyro_norm = sqrtf(gyro_norm)*dt;
	Deg = sin(gyro_norm) / gyro_norm;
	//第一行
	Weight[0][0] = 1;
	Weight[0][1] = -gyro[2] * dt*Deg;
	Weight[0][2] = gyro[1] * dt*Deg;
	//第二行
	Weight[1][0] = gyro[2] * dt*Deg;
	Weight[1][1] = 1;
	Weight[1][2] = -gyro[0] * dt*Deg;
	//第三行
	Weight[2][0] = -gyro[1] * dt*Deg;
	Weight[2][1] = gyro[0] * dt*Deg;
	Weight[2][2] = 1;

	matrix_normal(Weight);
	for (i = 0;i < 3;i++)
	{
		for (j = 0;j < 3;j++)
		{
			Fin[j][i] = Weight[i][j];
		}
	}
}

//判断可靠性
int MagTrue(float mag_last_time[3], float mag_now_time[3],float Fin[3][3],float gyro[3])
{
	int i = 0;//, j = 0;
	float mag_last_time_no_offset[3] = { 0 };
	float mag_now_time_no_offset[3] = { 0 };
	float mag_now_time_no_offset_predicted[3] = { 0 };
	float DiffData[3] = { 0 };
	//float DiffDataNorm = 0;
	//float R = 0;
	int MagTrueFlag = 0;
    float gyro_norm = 0;
    float GyroThresholdValue = 0.09;//unit:rad/s //0.045
	float MagChangeThresholdValue = 1; //unit:ut
	for (i = 0;i < 3;i++)
	{
		mag_last_time_no_offset[i] = mag_last_time[i];
		mag_now_time_no_offset[i] = mag_now_time[i];		
	}
	//进行预测
	mag_now_time_no_offset_predicted[0] = Fin[0][0] * mag_last_time_no_offset[0] + Fin[0][1] * mag_last_time_no_offset[1] + Fin[0][2] * mag_last_time_no_offset[2];
	mag_now_time_no_offset_predicted[1] = Fin[1][0] * mag_last_time_no_offset[0] + Fin[1][1] * mag_last_time_no_offset[1] + Fin[1][2] * mag_last_time_no_offset[2];
	mag_now_time_no_offset_predicted[2] = Fin[2][0] * mag_last_time_no_offset[0] + Fin[2][1] * mag_last_time_no_offset[1] + Fin[2][2] * mag_last_time_no_offset[2];
	//预测-真实值作差
	for (i = 0;i < 3;i++)
	{
		DiffData[i] =fabs (mag_now_time_no_offset_predicted[i] - mag_now_time_no_offset[i]);

	}
	gyro_norm = gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2];
    if(gyro_norm < 0){
        gyro_norm = 0;
    }
	gyro_norm = sqrtf(gyro_norm);
	
	if (gyro_norm < GyroThresholdValue)
		MagChangeThresholdValue = 2;//0.5f;//0.3//0.065
	else
		MagChangeThresholdValue = 4;//1.0f;

	//对三轴差分别判断
    //ALOGD("lcx MagTrue DiffData:%f %f %f",DiffData[0],DiffData[1],DiffData[2]);
	//if (DiffData[0] < 1 && DiffData[1] < 1 && DiffData[2] < 1)
    //if (DiffData[0] < 0.8f && DiffData[1] < 0.8f && DiffData[2] < 0.8f)
    if (DiffData[0] < MagChangeThresholdValue && DiffData[1] < MagChangeThresholdValue && DiffData[2] < MagChangeThresholdValue)
	{
		MagTrueFlag = 1;
	}
	else
	{
		MagTrueFlag = 0;
	}
	#if 0
	//对模值判断
	DiffDataNorm = DiffData[0] * DiffData[0] + DiffData[1] * DiffData[1] + DiffData[2] * DiffData[2];
	DiffDataNorm = sqrtf(DiffDataNorm);
	WriteData(DiffDataNorm);
	if (DiffDataNorm <= 2)    //小于2ut
	{
		MagTrueFlag = 1;
	}
	else
	{
		MagTrueFlag = 0;
	}
	#endif
	return MagTrueFlag;
}
#endif

static int bleave_mag_num = 0;

void qst_fusion_9axis_update(float fusion_accel[3], float fusion_gyro[3], float fusion_mag[3], float *fusion_dt, unsigned char *restart)
{
    float ex_9axis = 0.0f;
    float ey_9axis = 0.0f;
    float ez_9axis = 0.0f;

    float integral_dt = *fusion_dt;
    float halfT = 0.5f * integral_dt;

    float accel_tmp[3] = {0.0f, 0.0f, 0.0f};
    float gyro_tmp[3] = {0.0f, 0.0f, 0.0f};
    float mag_tmp[3] = {0.0f, 0.0f, 0.0f};
    float mag_lx[3] = {0};
    unsigned char axis = 0;
    float accel_tm_lx[3] = {0.0f, 0.0f, 0.0f};
    float gyro_tm_lx[3] = {0.0f, 0.0f, 0.0f};


    mag_lx[0] = fusion_mag[0];
    mag_lx[1] = fusion_mag[1];
    mag_lx[2] = fusion_mag[2];

    int update_offset_flag = 0;
		//ical_update_offset_flag_sync_to_fusion(&update_offset_flag);

    //int mag_lv = 0;
    //lv_sync_to_fusion(&mag_lv);
    if(integral_dt >= 0.5)
    {   
//        rr_sync_to_fusion(&mag_R_init);
        float open_recip_norm = sqrtf(mag_lx[0] * mag_lx[0] + mag_lx[1] * mag_lx[1] + mag_lx[2] * mag_lx[2]);
        // if(fabsf(mag_tems[0] - mag_lx[0]) < 2.0f && fabsf(mag_tems[1] - mag_lx[1]) < 2.0f && update_offset_flag == 0)
        // {
        //     fusion_9axis_inited = 1;
        // }else{
        //     fusion_9axis_inited = 0;
        // }
       mag_tems[0] = mag_lx[0];
       mag_tems[1] = mag_lx[1];
       mag_tems[2] = mag_lx[2];
        if(fabs(open_recip_norm - mag_R_init) >= 20 && update_offset_flag == 0)
        {
            //磁场不好的环境下暂时不切
            if(fabsf(tems_mags[0] - mag_lx[0]) > 15.0f && fabsf(tems_mags[1] - mag_lx[1]) > 15.0f)
            {
                fusion_9axis_mode = 1;
                static_delay = 0;
                static_flag = 0;
                fusion_9axis_inited = 0;
            }else{
                open_large_mag = 1;
                open_large_mag_num = 0;
            }

        }else if(fabs(open_recip_norm - mag_R_init) < 20 && update_offset_flag == 0){
            //在磁场环境好的环境下强制切到9轴
            fusion_9axis_mode = 1;
            static_delay = 0;
            static_flag = 0;
            fusion_9axis_inited = 0;
        }

        reopen_flag = 1;
        flags_enable = 1;
        lx_flag = 0;
        gyro_quiet_flag = 0;
    }
    else
    {
        reopen_flag = 0;
        if(integral_dt < 0 || integral_dt > 0.2)
        {
            integral_dt = 0.02f;
            halfT = 0.5f * integral_dt;
        }
    }

    if(restart && *restart)
    {
        fusion_9axis_inited = 0;
        fusion_9axis_mode =1;//add by lx 220920
        static_delay = 0;//add by lx 220920
		static_flag = 0;//add by lx 220920
        flags_enable = 1;
        lx_flag = 0;
        gyro_quiet_flag = 0;
    }


    for(axis = 0; axis < 3; axis++)
    {
        accel_tmp[axis] = fusion_accel[axis];
        accel_tm_lx[axis] = fusion_accel[axis];
        //gyro_tmp[axis] = fusion_gyro[axis] + gyro_static_offset[axis]; 
        gyro_tmp[axis] = fusion_gyro[axis]; //add by lx 210928
        gyro_tm_lx[axis] = fusion_gyro[axis]; //add by lx 210928
        mag_tmp[axis] = fusion_mag[axis];
        //mag_tems[axis] = fusion_mag[axis];
        tems_mags[axis] = fusion_mag[axis];
        
    }

    #ifdef ENBALE_DEBUG
    printf("qst_log0: %f %f %f %f %f %f %f %f %f\n", accel_tmp[0], accel_tmp[1], accel_tmp[2], 
                    gyro_tmp[0], gyro_tmp[1], gyro_tmp[2], mag_tmp[0], mag_tmp[1], mag_tmp[2]);
    #endif

    if(!fusion_9axis_inited)
    {
        qst_fusion_9axis_init(accel_tmp, mag_tmp);
//        rr_sync_to_fusion(&mag_R_init);
        return;
    }

    // int update_offset_flag = 0;
    // ical_update_offset_flag_sync_to_fusion(&update_offset_flag);
    
    if(update_offset_flag > 0)
    {
        fusion_9axis_mode = 1;
        if(update_offset_flag < 3)
        {
//            rr_sync_to_fusion(&mag_R_init);
        }

        if(fusion_9axis_inited ==1 && (update_offset_flag == 80 || update_offset_flag ==10))
        {   
            qst_fusion_9axis_init(accel_tmp, mag_tmp);
        }
    }

    float gz = fusion_9axis_rot_mat[2][0] * gyro_tmp[0] + fusion_9axis_rot_mat[2][1] * gyro_tmp[1] + fusion_9axis_rot_mat[2][2] * gyro_tmp[2];
    gz = Filter_Apply(gz, &gyro_buffer[2], &gyro_filter_parameter);
    float recip_norm = sqrtf(mag_tmp[0] * mag_tmp[0] + mag_tmp[1] * mag_tmp[1] + mag_tmp[2] * mag_tmp[2]);
    #if 0
    if(recip_norm < (mag_R_init - 8) || recip_norm >  (mag_R_init + 8))
    {
        /***********9切6判断:超过磁场半径范围**************************/
        if(update_offset_flag < 1)
        {
            if(fusion_9axis_mode == 1)
            {
                fusion_9axis_mode = 0;
				#ifdef ENBALE_DEBUG
					printf("qst_log6: 8888 \n");
				#endif
            }
        }
        yaw_gyro_recovery = 0;
        /***********9切6判断:超过磁场半径范围**************************/
    }else{
        /***********6切9恢复判断**************************/
        if(fabs(gz) > gyro_max * 4)//0.02)
        {
            yaw_gyro_recovery += -gz * integral_dt * TODEG;

            if(recip_norm > (mag_R_init - 4) || recip_norm < (mag_R_init + 4))
            {
                if(fabs(yaw_gyro_recovery) >= 10)//89
                {
                    if(fusion_9axis_mode == 0)
                    {
                        fusion_9axis_mode = 1;
                    }
                }
            }else{
                if(fabs(yaw_gyro_recovery) >= 30)//89
                {
                    if(fusion_9axis_mode == 0)
                    {
                        fusion_9axis_mode = 1;
                    }
                }
            }

        }else{
            yaw_gyro_recovery = 0;
        }
        /*************6切9恢复************************/
    }
    #else
    if(recip_norm < (mag_R_init - 7.0f) || recip_norm >  (mag_R_init + 7.0f))
    {
        /***********9切6判断:超过磁场半径范围**************************/
        if(update_offset_flag < 1)
        {
            if(fusion_9axis_mode == 1)
            {
                fusion_9axis_mode = 0;
            }
        }
    }

    int bleavemag = -1;
    float gyro_norms = gyro_tm_lx[0] * gyro_tm_lx[0] + gyro_tm_lx[1] * gyro_tm_lx[1] + gyro_tm_lx[2] * gyro_tm_lx[2];
    if(gyro_norms < 0){
        gyro_norms = 0;
    }
    float gyro_norms_rcm = sqrtf(gyro_norms);
    float Fin[3][3] = { 0 };//变换矩阵
    if (gyro_norms_rcm >= 0.05f) //大于阈值，进入验证//0.045 //0.05
    {
        CreateFin(gyro_tm_lx, integral_dt, Fin); //本次采样陀螺仪值
        bleavemag=MagTrue(mag_tems, mag_lx,Fin,gyro_tm_lx);//判断真值
       
    }
    
    if(bleavemag == 1){
        bleave_mag_num ++;
    }else if(bleavemag == 0){
        //切到6d
        bleave_mag_num = 0;
        if(fabs(recip_norm - mag_R_init) >4){
            fusion_9axis_mode = 0;
        }
        //fusion_9axis_mode = 0;
    }else if(bleavemag == -1){
        bleave_mag_num = 0;
    }

    if(bleave_mag_num >12){
        bleave_mag_num = 0;
        if(fusion_9axis_mode == 0 && fabs(recip_norm - mag_R_init) <=15 && update_offset_flag == 0){//30

            fusion_9axis_mode = 1;
        }
    }
    //ALOGD("lcx gyro_norms_rcm:%f bleave_mag_num:%d bleavemag:%d 9axis_mode:%d rr:%f R_init:%f\n",gyro_norms_rcm,bleave_mag_num,bleavemag,fusion_9axis_mode,recip_norm,mag_R_init);
    mag_tems[0] = mag_lx[0];
    mag_tems[1] = mag_lx[1];
    mag_tems[2] = mag_lx[2];
    #endif
    if(open_large_mag ==1 )
    {
        //打开是强磁环境下时候在移动到环境好的环境下强制切到9d
        if(recip_norm > (mag_R_init - 7) && recip_norm <  (mag_R_init + 7))
        {
            open_large_mag_num ++;
        }

        if(open_large_mag_num > 15) //40
        {
            open_large_mag =0;
            open_large_mag_num = 0;
            if(fusion_9axis_mode == 0)
            {
                fusion_9axis_mode = 1;
            }
        }
    }

    for(axis = 0; axis < BUFFSER_SIZE - 1; axis++)
    {
        gyro_buffer2[axis] = gyro_buffer2[axis + 1];
    }
    gyro_buffer2[axis] = sqrtf(gyro_tmp[0] * gyro_tmp[0] + gyro_tmp[1] * gyro_tmp[1] + gyro_tmp[2] * gyro_tmp[2]);//gz;

    if(update_offset_flag < 1)
    {
        if(fabs(gyro_buffer2[0]) < GYRO_THRESH && fabs(gyro_buffer2[1]) < GYRO_THRESH && fabs(gyro_buffer2[2]) < GYRO_THRESH && fabs(gyro_buffer2[3]) < GYRO_THRESH && fabs(gyro_buffer2[4]) < GYRO_THRESH
            && fabs(gyro_buffer2[5]) < GYRO_THRESH && fabs(gyro_buffer2[6]) < GYRO_THRESH && fabs(gyro_buffer2[7]) < GYRO_THRESH && fabs(gyro_buffer2[8]) < GYRO_THRESH && fabs(gyro_buffer2[9]) < GYRO_THRESH)
        {
            //float mag_normal = sqrtf((mag_tmp[0] - mag_buffer2[0]) * (mag_tmp[0] - mag_buffer2[0]) + (mag_tmp[1] - mag_buffer2[1]) * (mag_tmp[1] - mag_buffer2[1]) + (mag_tmp[2] - mag_buffer2[2]) * (mag_tmp[2] - mag_buffer2[2]));
            float mag_normal = fabs(mag_tmp[0] - mag_buffer2[0]) + fabs(mag_tmp[1] - mag_buffer2[1]) + fabs(mag_tmp[2] - mag_buffer2[2]);
            if(mag_normal > 5)
            {
                if(fusion_9axis_mode == 1)
                {
                    fusion_9axis_mode = 0;
					#ifdef ENBALE_DEBUG
						printf("qst_log7: 9999 \n");
					#endif
                    //GYRO_LOG("ma_q2 8888 \n");
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

    //fusion_9axis_mode = 0;

    //GYRO_LOG("maxianhu_status2 %d  %d %f %f\n", fusion_9axis_mode, update_offset_flag, mag_R_init, recip_norm);
	float spinRate = sqrtf(gyro_tmp[0] * gyro_tmp[0] + gyro_tmp[1] * gyro_tmp[1] + gyro_tmp[2] * gyro_tmp[2]);
    #if 1
    float gyro_H_Now[3];
    gyro_H_Now[0] = fusion_9axis_rot_mat[0][0] * gyro_tmp[0] + fusion_9axis_rot_mat[0][1] * gyro_tmp[1] + fusion_9axis_rot_mat[0][2] * gyro_tmp[2];
    gyro_H_Now[1] = fusion_9axis_rot_mat[1][0] * gyro_tmp[0] + fusion_9axis_rot_mat[1][1] * gyro_tmp[1] + fusion_9axis_rot_mat[1][2] * gyro_tmp[2];
    gyro_H_Now[2] = fusion_9axis_rot_mat[2][0] * gyro_tmp[0] + fusion_9axis_rot_mat[2][1] * gyro_tmp[1] + fusion_9axis_rot_mat[2][2] * gyro_tmp[2];
                
    float Gyr_Deg = gyro_H_Now[2] * integral_dt *TODEG;
    int move_gyro = 0;
    if(QST_FABS(Gyr_Deg)> 0.02){
        move_gyro = 1;
    }else{
        move_gyro = 0;
    }
    //ALOGD("lcx Gyr_Deg:%f\n",Gyr_Deg);
    #endif

		fusion_9axis_mode = 1;//add by lx 220920
		move_gyro = 1;
    //if(fusion_9axis_mode)
    //if(fusion_9axis_mode && spinRate >= 0.028)//0.038
    if(fusion_9axis_mode && move_gyro)//0.038
    {
        // Normalise magnetometer measurement
        mag_tmp[0] = mag_tmp[0] / recip_norm;
        mag_tmp[1] = mag_tmp[1] / recip_norm;
        mag_tmp[2] = mag_tmp[2] / recip_norm;

        // Magnetometer correction
        // Project mag field vector to global frame and extract XY component
        float hx = fusion_9axis_rot_mat[0][0] * mag_tmp[0] + fusion_9axis_rot_mat[0][1] * mag_tmp[1] + fusion_9axis_rot_mat[0][2] * mag_tmp[2];
        float hy = fusion_9axis_rot_mat[1][0] * mag_tmp[0] + fusion_9axis_rot_mat[1][1] * mag_tmp[1] + fusion_9axis_rot_mat[1][2] * mag_tmp[2];
        float hz = fusion_9axis_rot_mat[2][0] * mag_tmp[0] + fusion_9axis_rot_mat[2][1] * mag_tmp[1] + fusion_9axis_rot_mat[2][2] * mag_tmp[2];

        float by = sqrtf(hx * hx + hy * hy);
        float bz = hz;

        //n系中的地磁向量[bx,by,bz]转换到b系中，得到[wx,wy,wz]
        float wx = fusion_9axis_rot_mat[1][0] * by + fusion_9axis_rot_mat[2][0] * bz;
        float wy = fusion_9axis_rot_mat[1][1] * by + fusion_9axis_rot_mat[2][1] * bz;
        float wz = fusion_9axis_rot_mat[1][2] * by + fusion_9axis_rot_mat[2][2] * bz;

		float fifty_dps =  0.873f;//50dps
		float gainMult = 1.0f;
		if(spinRate > fifty_dps)
		{
			gainMult = fusion_min(spinRate / fifty_dps, 2.50f);
		}
		ex_9axis = (mag_tmp[1] * wz - mag_tmp[2] * wy) * fusion_9axis_mag_kp * gainMult;
		ey_9axis = (mag_tmp[2] * wx - mag_tmp[0] * wz) * fusion_9axis_mag_kp * gainMult;
		ez_9axis = (mag_tmp[0] * wy - mag_tmp[1] * wx) * fusion_9axis_mag_kp * gainMult;
    }

    recip_norm = sqrtf(accel_tmp[0] * accel_tmp[0] + accel_tmp[1] * accel_tmp[1] + accel_tmp[2] * accel_tmp[2]);
	if(recip_norm > GRAVITY_MSS * 0.9f && recip_norm <  GRAVITY_MSS * 1.1f)
	{
        accel_tmp[0] = accel_tmp[0] / recip_norm;
        accel_tmp[1] = accel_tmp[1] / recip_norm;
        accel_tmp[2] = accel_tmp[2] / recip_norm;

        ex_9axis += (accel_tmp[1] * fusion_9axis_rot_mat[2][2] - accel_tmp[2] * fusion_9axis_rot_mat[2][1]) * fusion_9axis_kp;
        ey_9axis += (accel_tmp[2] * fusion_9axis_rot_mat[2][0] - accel_tmp[0] * fusion_9axis_rot_mat[2][2]) * fusion_9axis_kp;
        ez_9axis += (accel_tmp[0] * fusion_9axis_rot_mat[2][1] - accel_tmp[1] * fusion_9axis_rot_mat[2][0]) * fusion_9axis_kp;
    }

    //GYRO_LOG("maxianhu_status,%d %d  dt:%f %f %f %f\n", *mag_status, fusion_9axis_mode, integral_dt, gyro_tmp[0], gyro_tmp[1], gyro_tmp[2]);

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
    printf("qst_log1: %f %f %f\n", fusion_9axis_exInt, fusion_9axis_eyInt, fusion_9axis_ezInt);
    #endif

    gyro_tmp[0] = gyro_tmp[0] + ex_9axis + fusion_9axis_exInt;
    gyro_tmp[1] = gyro_tmp[1] + ey_9axis + fusion_9axis_eyInt;
    gyro_tmp[2] = gyro_tmp[2] + ez_9axis + fusion_9axis_ezInt;

    float qw = fusion_9axis_q0;
    float qx = fusion_9axis_q1;
    float qy = fusion_9axis_q2;
    float qz = fusion_9axis_q3;

    fusion_9axis_q0 += (-qx * gyro_tmp[0] - qy * gyro_tmp[1] - qz * gyro_tmp[2]) * halfT;
    fusion_9axis_q1 += (+qw * gyro_tmp[0] + qy * gyro_tmp[2] - qz * gyro_tmp[1]) * halfT;
    fusion_9axis_q2 += (+qw * gyro_tmp[1] - qx * gyro_tmp[2] + qz * gyro_tmp[0]) * halfT;
    fusion_9axis_q3 += (+qw * gyro_tmp[2] + qx * gyro_tmp[1] - qy * gyro_tmp[0]) * halfT;

    recip_norm = sqrtf(fusion_9axis_q0 * fusion_9axis_q0 + fusion_9axis_q1 * fusion_9axis_q1 + fusion_9axis_q2 * fusion_9axis_q2 + fusion_9axis_q3 * fusion_9axis_q3);
    fusion_9axis_q0 = fusion_9axis_q0 / recip_norm;
    fusion_9axis_q1 = fusion_9axis_q1 / recip_norm;
    fusion_9axis_q2 = fusion_9axis_q2 / recip_norm;
    fusion_9axis_q3 = fusion_9axis_q3 / recip_norm;

    recip_norm = sqrtf(fusion_accel[0] * fusion_accel[0] + fusion_accel[1] * fusion_accel[1] + fusion_accel[2] * fusion_accel[2]);
    float gyro_recip_norm = sqrtf(fusion_gyro[0] * fusion_gyro[0] + fusion_gyro[1] * fusion_gyro[1] + fusion_gyro[2] * fusion_gyro[2]);

    float accel_sum = 0;
    float gyro_sum = 0;
    for(axis = 0; axis < BUFFSER_SIZE - 1; axis++)
    {
        accel_buffer_speed[axis] = accel_buffer_speed[axis + 1];
        accel_sum = accel_sum + accel_buffer_speed[axis];

        gyro_buffer_speed[axis] = gyro_buffer_speed[axis + 1];
        gyro_sum = gyro_sum + gyro_buffer_speed[axis];
    }
    accel_buffer_speed[axis] = recip_norm;
    accel_sum = accel_sum + accel_buffer_speed[axis];
    float accel_average = accel_sum / BUFFSER_SIZE;

    gyro_buffer_speed[axis] = gyro_recip_norm;
    gyro_sum = gyro_sum + gyro_buffer_speed[axis];
    float gyro_average = gyro_sum / BUFFSER_SIZE;

    accel_sum = 0;
    gyro_sum = 0;
    for(axis = 0; axis < BUFFSER_SIZE; axis++)
    {
        accel_sum = accel_sum + (accel_buffer_speed[axis] - accel_average) * (accel_buffer_speed[axis] - accel_average);
        gyro_sum = gyro_sum + (gyro_buffer_speed[axis] - gyro_average) * (gyro_buffer_speed[axis] - gyro_average);
    }
    float accel_standard_deviation = sqrtf(accel_sum / BUFFSER_SIZE);
    float gyro_standard_deviation = sqrtf(gyro_sum / BUFFSER_SIZE);

    //GYRO_LOG("%f %f\n", gyro_norm, accel_standard_deviation);
    if(spinRate > gyro_max * 5)    //4
    {
        static_flag = 0;
        static_delay = 0;
    }
    //GYRO_LOG("maxianhu_status %d %f %f %f %f\n", static_flag, accel_standard_deviation, gyro_static_offset[0], gyro_static_offset[1], gyro_static_offset[2]);
	

    //ALOGD("lx qst_fusion_9axis_update accel_standard_deviation:%f gyro_standard_deviation:%f\n",accel_standard_deviation,gyro_standard_deviation);
    float gyro_sqrts_lx = gyro_tm_lx[0] * gyro_tm_lx[0] + gyro_tm_lx[1] * gyro_tm_lx[1] + gyro_tm_lx[2] * gyro_tm_lx[2];
    if(gyro_sqrts_lx < 0){
        gyro_sqrts_lx = 0;
    }
    //float recip_gyro_lx = sqrtf(gyro_sqrts_lx);
    //ALOGD("lx qst_fusion_9axis_update recip_gyro_lx:%f static_flag:%d static_delay:%d\n",recip_gyro_lx,static_flag,static_delay);
    
    if(accel_standard_deviation < 0.2f)//\u0178???u0152¨¬?? //230421 0.6
    {

        float mag_sqrts_lx = mag_lx[0] * mag_lx[0] + mag_lx[1] * mag_lx[1] + mag_lx[2] * mag_lx[2];
        if(mag_sqrts_lx < 0){
            mag_sqrts_lx = 0;
        }
        float recip_norm_lx = sqrtf(mag_sqrts_lx);
//        rr_sync_to_fusion(&mag_R_init);

		if(gyro_standard_deviation < 0.02f)//0.02
		{
			if(static_flag == 0)
			{
				if(static_delay < 60)//80 
				{
					static_delay++;
					if(static_delay == 40)//60
					{
						//if(fusion_9axis_mode)
                        if(fabs(recip_norm_lx - mag_R_init) < 5)
						{
							//qst_fusion_9axis_init(accel_tmp, mag_tmp);
                            //ALOGD("lcx qst_fusion_9axis_update goto static 1");
                            qst_fusion_9axis_init(accel_tm_lx, mag_lx);
						}
					}
				}else{
					static_delay = 0;
					static_flag = 1;
                    gyro_quiet_flag =0;
					rpy_out_lock[0] = rpy_out[0];
				}
			}else{

                if(fabs(recip_norm_lx - mag_R_init) < 4 && gyro_quiet_flag <=10)
                {
                    qst_fusion_9axis_init(accel_tm_lx,mag_lx);
                    //ALOGD("lcx qst_fusion_9axis_update goto static 2");
                    rpy_out_lock[0] = rpy_out[0];
                    gyro_quiet_flag ++;
                }

                if(fabs(recip_norm_lx - mag_R_init) < 2 && gyro_quiet_flag >=10 && gyro_quiet_flag <=20)
                {
                    qst_fusion_9axis_init(accel_tm_lx,mag_lx);
                    //ALOGD("lcx qst_fusion_9axis_update goto static 2");
                    rpy_out_lock[0] = rpy_out[0];
                    gyro_quiet_flag ++;
                }


				rpy_out_lock[1] = rpy_out[1];
				rpy_out_lock[2] = rpy_out[2];
                //ALOGD("lx qst_fusion_9axis_update gyro_quiet_flag:%d\n",gyro_quiet_flag);
                #ifdef ENBALE_DEBUG
				printf("xiaomi7 %d %f %f %f %f %f %f\n", fusion_9axis_mode, rpy_out_lock[0],
						 rpy_out_lock[1], rpy_out_lock[2], fusion_gyro[0], fusion_gyro[1],
						 fusion_gyro[2]);
                #endif
                #if 0
                //if(fusion_9axis_mode == 0)
                if(reopen_flag == 1)
                {
                    reopen_flag = 0;
                    //qst_fusion_9axis_rotation_matrix();
                    qst_fusion_9axis_init(accel_tmp, mag_tmp);
                    qst_fusion_9axis_angle(rpy_out);
                    if(fabsf(rpy_out[0]-rpy_out_lock[0]) < 10.0f){
                        //小余10度时候，显示上一组角度值
                        qst_fusion_9axis_set_yaw(rpy_out_lock);
                    }else{
                        rpy_out_lock[0] = rpy_out[0];
                        //重新更新
                    }
                }else{

                    //add by lx 230525

                    if(recip_gyro_lx <=0.32f)
                    {
                        gyro_quiet_flag ++;
                    }else{
                        gyro_quiet_flag = 0;
                    }

                    if(gyro_quiet_flag > 100)
                    {
                        gyro_quiet_flag = 100;
                    }

                    if( flags_enable <=5 && gyro_quiet_flag >= 6)
                    {

                        if(fabs(accel_tm_lx[0]) <= 0.6f && fabs(accel_tm_lx[1]) <= 3.0f)
                        {
                            if(lx_flag < 2 )
                            {
                                
                                //if(recip_norm_lx >= (mag_R_init - 3) && recip_norm_lx <=  (mag_R_init + 3))
                                if(fabs(recip_norm_lx - mag_R_init) <=4)
                                {
                                    qst_fusion_9axis_init(accel_tmp, mag_tmp);
                                    qst_fusion_9axis_angle(rpy_out);
                                    rpy_out_lock[0] = rpy_out[0];
                                    rpy_out_lock[1] = rpy_out[1];
                                    rpy_out_lock[2] = rpy_out[2];
                                    //lx_flag = 1;
                                    lx_flag++;
                                }
                                flags_enable++;
                            }   
                            //if(lx_flag >= 1 && lx_flag < 2 && fabs(accel_tm_lx[0]) < 0.3 && fabs(accel_tm_lx[1]) < 0.3)
                            if(lx_flag >= 2 && lx_flag < 4 &&fabs(accel_tm_lx[0]) <= 0.3 && fabs(accel_tm_lx[1]) <= 0.3)
                            {
                                //rr_sync_to_fusion(&mag_R_init);
                                //if(recip_norm_lx >= (mag_R_init - 3) && recip_norm_lx <=  (mag_R_init + 3))
                                if(fabs(recip_norm_lx - mag_R_init) <= 2)
                                {
                                    qst_fusion_9axis_init(accel_tmp, mag_tmp);
                                    qst_fusion_9axis_angle(rpy_out);
                                    rpy_out_lock[0] = rpy_out[0];
                                    rpy_out_lock[1] = rpy_out[1];
                                    rpy_out_lock[2] = rpy_out[2];
                                    lx_flag++;
                                }
                                flags_enable++;                   
                            }               
                        }
                    }
                    
                    rpy_out_lock[1] = rpy_out[1];
				    rpy_out_lock[2] = rpy_out[2];
                    #endif
                    qst_fusion_9axis_set_yaw(rpy_out_lock);
                    //ALOGD("lcx qst_fusion_9axis_update goto static 3");
                //}
                //qst_fusion_9axis_set_yaw(rpy_out_lock);
                #ifdef ENBALE_DEBUG
                printf("qst_log4: %f %f %f\n", rpy_out_lock[0], rpy_out_lock[1],rpy_out_lock[2]);
                #endif


            }
		}
	}else{
		static_delay = 0;
		static_flag = 0;
        gyro_quiet_flag = 0;     
	}

    qst_fusion_9axis_rotation_matrix();

    qst_fusion_9axis_angle(rpy_out);

    #ifdef ENBALE_DEBUG
    printf("qst_log2: %d %f %f %f %f\n", fusion_9axis_mode,integral_dt,  rpy_out[0] * TODEG, rpy_out[1] * TODEG, rpy_out[2] * TODEG);
    printf("qst_log3: %d %d %f %f %f %f %f\n", static_flag, static_flag2, accel_standard_deviation, gyro_standard_deviation, gyro_static_offset[0], gyro_static_offset[1], gyro_static_offset[2]);
    #endif
}

float ical_pitch = 0;
float ical_roll = 0;

float test_pitch = 0;
float test_roll = 0;
float test_yaw = 0;

//extern float test_angle;

void qst_fusion_9axis_angle(float *rpy)
{

    // if(matrix_ori.cs[2][1]<-1.0f){
    //     matrix_ori.cs[2][1] = -1.0f;
    // }else if(matrix_ori.cs[2][1]>1.0f){
    //     matrix_ori.cs[2][1] = 1.0f;
    // }
    // //正欧拉角
    // float pitch_special = asinf(matrix_ori.cs[2][1]);
    // float roll_special= atan2f(-matrix_ori.cs[2][0],matrix_ori.cs[2][2]);
    // float yaw_special= -atan2f(-matrix_ori.cs[0][1],matrix_ori.cs[1][1]);

    // //反欧拉角
    // if(matrix_ori.cs[2][0]<-1.0f){
    //     matrix_ori.cs[2][0] = -1.0;
    // }else if(matrix_ori.cs[2][0]>1.0f){
    //     matrix_ori.cs[2][0] = 1.0f;
    // }
    // float pitch_anti =atan2f(matrix_ori.cs[2][1],matrix_ori.cs[2][2]);
    // float roll_anti =asinf(-matrix_ori.cs[2][0]);
    // float yaw_anti=-atan2f(matrix_ori.cs[1][0],matrix_ori.cs[0][0]); 

    float pitch = atan2f(-fusion_9axis_rot_mat[2][1], fusion_9axis_rot_mat[2][2]);
    float roll  = asinf(fusion_9axis_rot_mat[2][0]);
    float yaw = atan2f(-fusion_9axis_rot_mat[1][0], fusion_9axis_rot_mat[0][0]);

    rpy[1] = pitch;
    rpy[2] = roll;
    rpy[0] = yaw;

    roll  = roll * TODEG;
    pitch = pitch * TODEG;
    yaw = yaw * TODEG;

    if(yaw < 0.0f)
    {
        yaw += 360.0f;
    }
		
		printf("%f %f %f %f\n", roll, pitch, yaw);

    ical_pitch = pitch;
    ical_roll = roll;
		
		test_pitch = pitch;
		test_roll = roll;
		test_yaw = yaw;

		
    //GYRO_LOG("maxianhu, %d %d %d %f %f\n", update_offset_flag, charge_init, charge_flag, yaw, yaw_6axis);
}


void qst_fusion_9axis_euler_to_quaternion(float quat[4])
{
    quat[0] = sqrtf(clamp( fusion_9axis_rot_mat[0][0] - fusion_9axis_rot_mat[1][1] - fusion_9axis_rot_mat[2][2] + 1) * 0.25f);
    quat[1] = sqrtf(clamp(-fusion_9axis_rot_mat[0][0] + fusion_9axis_rot_mat[1][1] - fusion_9axis_rot_mat[2][2] + 1) * 0.25f);
    quat[2] = sqrtf(clamp(-fusion_9axis_rot_mat[0][0] - fusion_9axis_rot_mat[1][1] + fusion_9axis_rot_mat[2][2] + 1) * 0.25f);
    quat[3] = sqrtf(clamp( fusion_9axis_rot_mat[0][0] + fusion_9axis_rot_mat[1][1] + fusion_9axis_rot_mat[2][2] + 1) * 0.25f);
    quat[0] = copysignf(quat[0], fusion_9axis_rot_mat[2][1] - fusion_9axis_rot_mat[1][2]);
    quat[1] = copysignf(quat[1], fusion_9axis_rot_mat[0][2] - fusion_9axis_rot_mat[2][0]);
    quat[2] = copysignf(quat[2], fusion_9axis_rot_mat[1][0] - fusion_9axis_rot_mat[0][1]);
}


