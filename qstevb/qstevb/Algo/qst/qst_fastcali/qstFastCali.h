#ifndef qstFastCali_H
#define qstFastCali_H

#include<stdio.h>
#include<stdlib.h>
#include<math.h>


extern void convert_magnetic_fastcali(float *uncalimag,float *gyro,float *calimag,float *offset,float dt,int *lv,int restart);
extern void qst_fastcali_init(float offset[6]);

#endif

