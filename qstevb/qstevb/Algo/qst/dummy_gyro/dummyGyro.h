#ifndef DUMMYGYRO_H
#define DUMMYGYRO_H

#undef LOG_TAG
#define LOG_TAG "dummy-gyro"

#ifndef PI
#define PI					3.1415926535897932384626433832795f
#endif
#define DegToRad                0.0174533f
#define RadToDeg                57.29578f

#ifndef int64_t
typedef long long				int64_t;
#endif

struct virtual_gyro_matrix{
	float cs[3][3];
};

#ifdef __cplusplus
extern "C" {
#endif
void qst_fusion_dummy_gyro(float Acc[3], float Mag[3], float Vgyro[3], int64_t timeStamp);
#ifdef __cplusplus
}
#endif

#endif 
