#ifndef FUSION_INTERFACE_H
#define FUSION_INTERFACE_H

enum
{
	FUSION_NONE = 0,
	FUSION_9AXIS = 1, // use accel gyro mag    
	FUSION_NOMAG = 1<<1, // use accel gyro (game rotation, gravity)    
	FUSION_NOGYRO = 1<<2, // use accel mag (geomag rotation)    
	NUM_FUSION_MODE

};

typedef struct
{
    int64_t timeStamp;
    char status;
    float x, y, z, w;
}libData;

struct qst_fusion_interface_t
{
	const char *version;
	int (*enableLib)(int mode);
	int (*SetGyroData)(libData *inputData);
	int (*SetAccData)(libData *inputData);
	int (*SetMagData)(libData *inputData);
	int (*GetMagOffset)(float offset[3], float *rr);
	int (*doCali)(libData *inputData, libData *outputData);
	int (*doFastCali)(libData *inputData, libData *outputData);
	int (*getGravity)(libData *outputData);
	int (*getRotationVector)(libData *outputData);
	int (*getOrientation)(libData *outputData);
	int (*getLinearAccel)(libData *outputData);
	int (*getGameRotationVector)(libData *outputData);
	int (*getGeoMagnetic)(libData *outputData);
	int (*getVirtualGyro)(libData *outputData);
};

#ifdef __cplusplus
extern "C" {
#endif
void qst_fusion_get_interface(struct qst_fusion_interface_t **p_algo);
#ifdef __cplusplus
}
#endif

#endif
