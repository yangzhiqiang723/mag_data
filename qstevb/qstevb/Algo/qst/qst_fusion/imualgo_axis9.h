#ifndef IMU_ALGO_AXIS9_H_
#define IMU_ALGO_AXIS9_H_
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif
//function declare
float constrain_float(float amt, float low, float high);
float wrap_PI(float angle_in_radians);
void qst_fusion_9axis_rotation_matrix(void);
int qst_fusion_9axis_init(float accel_in[3], float mag_in[3]);


void qst_fusion_9axis_update(float fusion_accel[3], float fusion_gyro[3], float fusion_mag[3], float *fusion_dt, unsigned char *restart);
//void qst_fusion_9axis_angle(float *rpy);
void qst_fusion_9axis_euler_to_quaternion(float	quat[4]);

//void qst_fusion_get_quat(float quat[4],int mode);
//void qst_fusion_get_matrix(float matrix[9],int mode);
//extern void rr_sync_to_fusion(float *rr);
//extern void ical_update_offset_flag_sync_to_fusion(int *flag);
//extern void lv_sync_to_fusion(float *lv);

#ifdef __cplusplus
}
#endif

#endif /* IMU_ALGO_AXIS9_H_ */
