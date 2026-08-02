#include "stdafx.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "MyFilter.h"

//#define Factor		0.01f
//#define R				0.000000414975f
#define Factor			0.01f
#define R				0.00005f
#define Q				(R*Factor)

void InitMyFilter(MyFilter_t *filter)
{
	memset(filter, 0, sizeof(MyFilter_t));
}

float ExeMyFilter(MyFilter_t *filter, float in)
{
	float out;

	if(filter->init == 0)
	{
		filter->x_last = in;
		filter->init = 1;
	}
    filter->x_mid=filter->x_last; 
    filter->p_mid=filter->p_last + Q; 
    filter->kg=filter->p_mid/(filter->p_mid+R); 
    filter->x_now=filter->x_mid + filter->kg*(in -filter->x_mid); 
    filter->p_now=(1-filter->kg)*filter->p_mid;
    filter->p_last = filter->p_now; 
    filter->x_last = filter->x_now; 
    out = filter->x_now;

	return out;
}


void InitLpfFilter(LpfFilter_t *filter)
{
	filter->init = 0;
	filter->alpha = 0.85f;
	filter->last = 0.0f;
}

float ExeLpfFilter(LpfFilter_t *filter, float in)
{
	if(filter->init == 0)
	{
		filter->last = in;
		filter->init = 1;
	}
	filter->last = (filter->alpha*filter->last) + (1-filter->alpha)*in;

	return filter->last;
}


void InitAvg1AxisFilter(Avg1AxisFilter_t *filter, short size)
{
	filter->init = 0;
	if(size <= AVG_ARRAY_SIZE)
		filter->size = size;
	else
		filter->size = AVG_ARRAY_SIZE;
}

float ExeAvg1AxisFilter(Avg1AxisFilter_t *filter, float in)
{
	int i;
	float sum = 0.0f;

	if(filter->init == 0)
	{
		for(i=0; i<filter->size; i++)
		{
			filter->array[i] = in;
		}
		filter->init = 1;
	}

	sum = 0.0f;
	for(i=1; i<filter->size; i++)
	{
		sum += filter->array[i];
		filter->array[i-1] = filter->array[i];
	}
	filter->array[filter->size-1] = in;
	sum += in;

	return sum/filter->size;
}

void InitAvg3AxisFilter(Avg3AxisFilter_t *filter, short size)
{
	filter->init = 0;
	if(size <= AVG_ARRAY_SIZE)
		filter->size = size;
	else
		filter->size = AVG_ARRAY_SIZE;
}

void ExeAvg3AxisFilter(Avg3AxisFilter_t *filter, float in[3], float out[3])
{
	int i;
	float sum[3] = {0, 0, 0};

	if(filter->init == 0)
	{
		for(i=0; i<filter->size; i++)
		{
			filter->x[i] = in[0];
			filter->y[i] = in[1];
			filter->z[i] = in[2];
		}
		filter->init = 1;
	}

	sum[0] = 0.0f;
	sum[1] = 0.0f;
	sum[2] = 0.0f;
	for(i=1; i<filter->size; i++)
	{
		sum[0] += filter->x[i];
		sum[1] += filter->y[i];
		sum[2] += filter->z[i];
		filter->x[i-1] = filter->x[i];
		filter->y[i-1] = filter->y[i];
		filter->z[i-1] = filter->z[i];
	}
	filter->x[filter->size-1] = in[0];
	filter->y[filter->size-1] = in[1];
	filter->z[filter->size-1] = in[2];
	sum[0] += in[0];
	sum[1] += in[1];
	sum[2] += in[2];

	out[0] = (float)(sum[0]/filter->size);
	out[1] = (float)(sum[1]/filter->size);
	out[2] = (float)(sum[2]/filter->size);
}

#if defined(QST_STD_6)
void getStandardDeviation6(QstStd6 *pStd6, float in[6], float out[6])
{
	float avg_1,avg_2,avg_3,avg_4,avg_5,avg_6;
	float toatl_1,toatl_2,toatl_3,toatl_4,toatl_5,toatl_6;
	short index = 0;

	if(pStd6->size < STD_ARRAY_SIZE)
	{
		index = pStd6->size;
		if(index == 0)
		{
			pStd6->sum_1=pStd6->sum_2=pStd6->sum_3=pStd6->sum_4=pStd6->sum_5=pStd6->sum_6 = 0.0f;
			pStd6->index = 0;
		}
		pStd6->data_1[index] = in[0];
		pStd6->data_2[index] = in[1];
		pStd6->data_3[index] = in[2];
		pStd6->data_4[index] = in[3];
		pStd6->data_5[index] = in[4];
		pStd6->data_6[index] = in[5];
		pStd6->sum_1 += in[0];
		pStd6->sum_2 += in[1];
		pStd6->sum_3 += in[2];
		pStd6->sum_4 += in[3];
		pStd6->sum_5 += in[4];
		pStd6->sum_6 += in[5];
		pStd6->size++;
		out[0] = 0.0f;
		out[1] = 0.0f;
		out[2] = 0.0f;
		out[3] = 0.0f;
		out[4] = 0.0f;
		out[5] = 0.0f;
	}
	else
	{
		index = pStd6->index;
		pStd6->sum_1 = pStd6->sum_1 - pStd6->data_1[index] + in[0];
		pStd6->sum_2 = pStd6->sum_2 - pStd6->data_2[index] + in[1];
		pStd6->sum_3 = pStd6->sum_3 - pStd6->data_3[index] + in[2];
		pStd6->sum_4 = pStd6->sum_4 - pStd6->data_4[index] + in[3];
		pStd6->sum_5 = pStd6->sum_5 - pStd6->data_5[index] + in[4];
		pStd6->sum_6 = pStd6->sum_6 - pStd6->data_6[index] + in[5];

		pStd6->data_1[index] = in[0];
		pStd6->data_2[index] = in[1];
		pStd6->data_3[index] = in[2];
		pStd6->data_4[index] = in[3];
		pStd6->data_5[index] = in[4];
		pStd6->data_6[index] = in[5];

		pStd6->index = (++pStd6->index%STD_ARRAY_SIZE);
		avg_1 = pStd6->sum_1/STD_ARRAY_SIZE;
		avg_2 = pStd6->sum_2/STD_ARRAY_SIZE;
		avg_3 = pStd6->sum_3/STD_ARRAY_SIZE;
		avg_4 = pStd6->sum_4/STD_ARRAY_SIZE;
		avg_5 = pStd6->sum_5/STD_ARRAY_SIZE;
		avg_6 = pStd6->sum_6/STD_ARRAY_SIZE;
		toatl_1 = toatl_2 = toatl_3 = toatl_4 = toatl_5 = toatl_6 = 0.0f;

		for(int i = 0; i < STD_ARRAY_SIZE; i++)
		{
			toatl_1 += (pStd6->data_1[i] - avg_1) * (pStd6->data_1[i] - avg_1);
			toatl_2 += (pStd6->data_2[i] - avg_2) * (pStd6->data_2[i] - avg_2);
			toatl_3 += (pStd6->data_3[i] - avg_3) * (pStd6->data_3[i] - avg_3);
			toatl_4 += (pStd6->data_4[i] - avg_4) * (pStd6->data_4[i] - avg_4);
			toatl_5 += (pStd6->data_5[i] - avg_5) * (pStd6->data_5[i] - avg_5);
			toatl_6 += (pStd6->data_6[i] - avg_6) * (pStd6->data_6[i] - avg_6);
		}

		out[0] = (float)sqrt(toatl_1 / (STD_ARRAY_SIZE));
		out[1] = (float)sqrt(toatl_2 / (STD_ARRAY_SIZE));
		out[2] = (float)sqrt(toatl_3 / (STD_ARRAY_SIZE));
		out[3] = (float)sqrt(toatl_4 / (STD_ARRAY_SIZE));
		out[4] = (float)sqrt(toatl_5 / (STD_ARRAY_SIZE));
		out[5] = (float)sqrt(toatl_6 / (STD_ARRAY_SIZE));
	}
}
#endif

#if defined(QST_STD_3)
void getStandardDeviation3(QstStd3 *pStd3, float in[3], float out[3])
{
	float avg_x,avg_y,avg_z;
	float toatl_x,toatl_y,toatl_z;
	short index;

	if(pStd3->size < STD_ARRAY_SIZE)
	{
		index = pStd3->size;		
		if(index == 0)
		{
			pStd3->sum_x=pStd3->sum_y=pStd3->sum_z=0.0f;
			pStd3->index = 0;
		}
		pStd3->data_x[index] = in[0];
		pStd3->data_y[index] = in[1];
		pStd3->data_z[index] = in[2];
		pStd3->sum_x += in[0];
		pStd3->sum_y += in[1];
		pStd3->sum_z += in[2];
		pStd3->size++;
		out[0] = 255.0f;
		out[1] = 255.0f;
		out[2] = 255.0f;
	}
	else
	{
		index = pStd3->index;
		pStd3->sum_x = pStd3->sum_x - pStd3->data_x[index] + in[0];
		pStd3->sum_y = pStd3->sum_y - pStd3->data_y[index] + in[1];
		pStd3->sum_z = pStd3->sum_z - pStd3->data_z[index] + in[2];
		pStd3->data_x[index] = in[0];
		pStd3->data_y[index] = in[1];
		pStd3->data_z[index] = in[2];
		pStd3->index = (++pStd3->index%STD_ARRAY_SIZE);
		avg_x = pStd3->sum_x/STD_ARRAY_SIZE;
		avg_y = pStd3->sum_y/STD_ARRAY_SIZE;
		avg_z = pStd3->sum_z/STD_ARRAY_SIZE;
		toatl_x = toatl_y = toatl_z = 0.0f;

		for(int i = 0; i < STD_ARRAY_SIZE; i++)
		{
			toatl_x += (pStd3->data_x[i] - avg_x) * (pStd3->data_x[i] - avg_x);
			toatl_y += (pStd3->data_y[i] - avg_y) * (pStd3->data_y[i] - avg_y);
			toatl_z += (pStd3->data_z[i] - avg_z) * (pStd3->data_z[i] - avg_z);
		}

		out[0] = (float)sqrt(toatl_x / (STD_ARRAY_SIZE));
		out[1] = (float)sqrt(toatl_y / (STD_ARRAY_SIZE));
		out[2] = (float)sqrt(toatl_z / (STD_ARRAY_SIZE));
	}
}
#endif

