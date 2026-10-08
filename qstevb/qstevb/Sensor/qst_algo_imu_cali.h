

#ifndef GYRO_ALGO_IMU_CALI_H_
#define GYRO_ALGO_IMU_CALI_H_

#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

extern void qst_imu_cali_init(float *a_bisa, float *g_bias);
//extern void qst_imu_get_offset(float a_bias[3],float g_bias[3]);    // after cali OK, get offset
//extern int qst_imu_cali(float acc[3],float gyro[3]);                // imu cali OK:retrun 1
extern int qst_algo_imu_cali(float acc[3], float gyr[3]);
extern void qst_imu_auto_cali(float d[3]);                          // gyro auto cali

#ifdef __cplusplus
}
#endif

#endif /* GYRO_ALGO_IMU_CALI_H_ */
