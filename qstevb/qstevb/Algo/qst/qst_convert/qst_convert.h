

#ifndef QST_CONVERT_H_
#define QST_CONVERT_H_

#ifdef __cplusplus
extern "C" {
#endif

void android_quat_to_matrix(float quat[4], float mat[][3]);
void android_quat_to_euler(float quat[4], float euler[3]);
void android_matrix_to_euler(float mat[][3], float euler[3]);
void android_matrix_to_quat(float mat[][3], float quat[4]);
void android_euler_to_matrix(float euler[3], float mat[][3]);
void android_euler_to_quat(float euler[3], float quat[4]);

void android_raw_to_euler(float acc[3], float mag[3], float euler[3]);
float qst_yaw_improve(float yaw);
void qst_quat_to_matrix(float quat[4], float mat[][3]);

#ifdef __cplusplus
}
#endif

#endif /* QST_CONVERT_H_ */
