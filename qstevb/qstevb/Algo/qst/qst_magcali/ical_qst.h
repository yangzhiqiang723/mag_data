#ifndef ICAL_QST_H_
#define ICAL_QST_H_
//#include <sys/types.h>

//#define USING_ACC_LOCK

// Internal Type Definitions
#if 1
typedef unsigned char		UCHAR;
typedef unsigned char		UINT8;
typedef unsigned char		uint8_t;

typedef signed char			INT8;
typedef signed char			int8_t;

typedef short				INT16;
typedef short				int16_t;

typedef unsigned short		UINT16;
typedef unsigned short		uint16_t;

typedef int					INT32;
typedef int					int32_t;

typedef unsigned int		UINT32;
typedef unsigned int		uint32_t;

typedef float					REAL;
typedef long long				int64_t;
typedef int64_t					INT64;
typedef unsigned long long		uint64_t;
typedef uint64_t				UINT64;
#endif

/* DEBUG LEVEL definition*/
#define LOG_DEBUG  3
#define LOG_INFO   2
#define LOG_WARN   1
#define LOG_ERROR  0

typedef struct
{
	int32_t	Offset[3];		// Magnetic Hard Iron x, y, z offsets
	int32_t rr;
} TRANSFORM_T;

#ifdef __cplusplus
extern "C" {
#endif

int qst_ical_init(REAL *calipara);
int convert_magnetic(REAL *raw, REAL *result,REAL *offset,REAL *RR,int8_t *accuracy);
void get_ical_accuracy(int8_t *accuracy);
int qst_ori_progress(float *Acc,float *Mag,float *Ori,int8_t *accuracy);
//#ifdef USING_ACC_LOCK
//void get_acc_lock_status(int *acc_lock_en);
//#endif

#ifdef __cplusplus
}
#endif

#endif /* ICAL_QST_H_ */
