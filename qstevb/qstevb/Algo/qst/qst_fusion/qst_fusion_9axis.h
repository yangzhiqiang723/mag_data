

#ifndef QST_FUSION_AXIS9_H_
#define QST_FUSION_AXIS9_H_

#ifdef __cplusplus
extern "C" {
#endif

void qst_fusion_9axis_update(float acc[3], float gyr[3], float mag[3], float dt, unsigned char reset);
void qst_fusion_9axis_get_quaternion(float	quat[4]);

void qst_fusion_nomag_init(float acc[3]);
void qst_fusion_nomag_update(float acc[3], float gyr[3], float dt, unsigned char reset);
void qst_fusion_nomag_get_quaternion(float quat[4]);

void qst_fusion_nogyro_init(void);
void qst_fusion_nogyro_update(float acc[3], float mag[3]);
void qst_fusion_nogyro_get_quaternion(float quat[4]);

void qst_fusion_get_gravity(float gra_data[3],int mode);

#ifdef __cplusplus
}
#endif

#endif /* IMU_ALGO_AXIS9_H_ */
