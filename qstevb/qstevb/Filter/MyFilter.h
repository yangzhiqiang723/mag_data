
#ifndef __QST_MY_FILTER_H__
#define __QST_MY_FILTER_H__

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
	char   init;
	float  x_now;
	float  x_mid;
	float  p_mid;
	float  x_last;
	float  p_last;
	float  p_now;
	float  kg;
}MyFilter_t;


typedef struct
{
	char	init;
	float	alpha;
	float	last;
}LpfFilter_t;

#define AVG_ARRAY_SIZE		12
typedef struct
{
	char	init;
	short	size;
	float	array[AVG_ARRAY_SIZE];
}Avg1AxisFilter_t;

typedef struct
{
	char	init;
	short	size;
	float	x[AVG_ARRAY_SIZE];
	float	y[AVG_ARRAY_SIZE];
	float	z[AVG_ARRAY_SIZE];
}Avg3AxisFilter_t;

#define STD_ARRAY_SIZE		64	//128
typedef struct
{
	float	data_x[STD_ARRAY_SIZE];
	float	data_y[STD_ARRAY_SIZE];
	float	data_z[STD_ARRAY_SIZE];
	float	sum_x;
	float	sum_y;
	float	sum_z;
	short	size;
	short	index;
}QstStd3;

typedef struct
{
	float	data_1[STD_ARRAY_SIZE];
	float	data_2[STD_ARRAY_SIZE];
	float	data_3[STD_ARRAY_SIZE];
	float	data_4[STD_ARRAY_SIZE];
	float	data_5[STD_ARRAY_SIZE];
	float	data_6[STD_ARRAY_SIZE];
	float	sum_1;
	float	sum_2;
	float	sum_3;
	float	sum_4;
	float	sum_5;
	float	sum_6;
	short	size;
	short	index;
}QstStd6;

//#define QST_STD_6
#define QST_STD_3

void InitMyFilter(MyFilter_t *filter);
float ExeMyFilter(MyFilter_t *filter, float in);

void InitLpfFilter(LpfFilter_t *filter);
float ExeLpfFilter(LpfFilter_t *filter, float in);

void InitAvg1AxisFilter(Avg1AxisFilter_t *filter, short size);
float ExeAvg1AxisFilter(Avg1AxisFilter_t *filter, float in);

void InitAvg3AxisFilter(Avg3AxisFilter_t *filter, short size);
void ExeAvg3AxisFilter(Avg3AxisFilter_t *filter, float in[3], float out[3]);

#if defined(QST_STD_6)
void getStandardDeviation6(QstStd6 *pStd6, float in[6], float out[6]);
#endif
#if defined(QST_STD_3)
void getStandardDeviation3(QstStd3 *pStd3, float in[3], float out[3]);
#endif

#ifdef __cplusplus
}
#endif

#endif

