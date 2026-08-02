
#include "math.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "ical_qst.h"
#include "icfg.h"
#include "QstAlgoLog.h"

#define ICAL_GDF_LOG
#ifdef ICAL_GDF_LOG
#undef LOG_TAG
#define LOG_TAG "qst-ical"
#define ICAL_LOG  	LOGD
#else
#define ICAL_LOG(...)
#endif

// Internal Definitions
#define ENS3D_STANDOFF				0.6 // Minimum distance between samples in radians
#define DIM_A						4
#define PARA_3D						4

#define MAXIMUM_DATA_SAMPLES        50

#define ACCURACY_UNRELIABLE      0
#define ACCURACY_LOW             1
#define ACCURACY_MEDIUM          2
#define ACCURACY_HIGH            3

#define MAG_Z_THRESH  400
#define MAG_RES  120

// Internal Structural Type Definitions
typedef struct
{
   INT32 mag[MAXIMUM_DATA_SAMPLES][3];
} CAL_ENSEMBLE_T;

typedef struct
{
   INT32 trialMag[3];
   INT32 nsamps;//Count of collection data
   INT32 standoffCounts;
} ICAL_MGR_T;

typedef struct
{
   REAL MatA[DIM_A][DIM_A];
   REAL invMatA[DIM_A][DIM_A];

} MA_INVERT_T;

// Internal Variable Definitions
static ICAL_MGR_T icalMgr;
static CAL_ENSEMBLE_T icalEns;
static MA_INVERT_T ma;

static REAL *Jpt[PARA_3D];
static REAL JTJ[PARA_3D][PARA_3D];
static REAL JTK[PARA_3D];
static REAL Jout[PARA_3D];
static REAL magX[MAXIMUM_DATA_SAMPLES] = {0};
static REAL magY[MAXIMUM_DATA_SAMPLES] = {0};
static REAL magZ[MAXIMUM_DATA_SAMPLES] = {0};
static REAL SQ_rr[MAXIMUM_DATA_SAMPLES] = {0};

// 3D
static TRANSFORM_T newCal;
static INT16 nDiffAccuracy_3D =0;
static int32_t ofsX = 0, ofsY = 0, ofsZ = 0;
static int32_t AvgMagR = (int32_t)DEFAULT_NORMAL;
static int32_t lastCalibCount = 0;
static INT32 vd = 0;
static INT32 vd_xy = 0;
static int32_t rd = 0;
static int8_t lastStatus  = ACCURACY_UNRELIABLE;
static int8_t curAccuracy = ACCURACY_UNRELIABLE;
static int32_t lastHard[3] = { 0, 0, 0 };
static int32_t lastR = 0;
static int32_t diffSameCount = 0;
static int32_t IsFirstCalAfterReboot = 1;

#define OFFSET_AVG_N	4
static TRANSFORM_T offset_n[OFFSET_AVG_N];
static int16_t offset_avgN = 0;

INT32 vec_magnitude(INT32 *v0, INT32 *v1, INT32 *diff);

INT32 ical_qualify( INT32 raw545, INT32 rawMagY, INT32 rawMagZ);
void ical_addSample(INT32* mag, INT32 sampIX);
void ical_shiftData(INT32 length);

INT32 em_invert(INT32 size);
void  em_average_demean(INT32(*xyz)[3],INT32 *iavg, INT32 npts);
INT32 em_center(INT32(*xyz)[3], INT32 *outrxyz, INT32 npts);
#ifdef ALGO_9D_FUSION
static int ical_update_offset_flag = 0;
void ical_update_offset_flag_sync_to_fusion(int *flag);
void rr_sync_to_fusion(float *rr);
void rr_sync_to_fusion(float *rr)
{
	*rr = 	AvgMagR/31.25;
}

void ical_update_offset_flag_sync_to_fusion(int *flag)
{
	*flag = ical_update_offset_flag;
}

#endif
static INT32 mSabs(INT32 a){
    return (a > 0)?(a):(-a);
}

static void *mMemset(void *ptr, int value, uint16_t num)
{
    char *xs = (char*) ptr;
    while (num --)
        *xs++ = value;
    return ptr;
}

static void ical_Init(void)
{
    // Zero the managers
    mMemset(&icalMgr, 0, sizeof(ICAL_MGR_T));
    mMemset(&icalEns, 0, sizeof(CAL_ENSEMBLE_T));
    mMemset(offset_n, 0, OFFSET_AVG_N*sizeof(TRANSFORM_T));
    // Initialize the stand off counts
    icalMgr.standoffCounts = (INT32)(ENS3D_STANDOFF * 562.5);

    offset_avgN = 0;
}

static void removePointAt(INT32 num)
{
    INT32 i=0;
    if(num>=icalMgr.nsamps || num == -1)
        return;

    for(i= num + 1; i < icalMgr.nsamps; i++ )
    {
        icalEns.mag[i-1][0] = icalEns.mag[i][0];
        icalEns.mag[i-1][1] = icalEns.mag[i][1];
        icalEns.mag[i-1][2] = icalEns.mag[i][2];
    }
    icalMgr.nsamps--;
}

/*
    delete 2 points,max value and min value from the offset
*/
static void ical_clearFarPoint(int32_t rr,int32_t ofx, int32_t ofy, int32_t ofz)
{
    INT32 i=0;

    if(icalMgr.nsamps > FIRST_CALIB_NUM){
        INT32 maxIndex = 0;
        INT32 minIndex = 0;
        INT32 maxVar,minVar;

        maxVar = minVar = mSabs(icalEns.mag[0][0]-ofx) + mSabs(icalEns.mag[0][1]-ofy) + mSabs(icalEns.mag[0][2]-ofz);

        for (i = 1; i < icalMgr.nsamps; i++) {
            INT32 temp = mSabs(icalEns.mag[i][0]-ofx) + mSabs(icalEns.mag[i][1]-ofy) + mSabs(icalEns.mag[i][2]-ofz);
            if(temp > maxVar){
                maxVar = temp;
                maxIndex = i;
            }
            if(temp < minVar){
                minVar = temp;
                minIndex = i;
            }
        }

        if(maxIndex > minIndex){
            removePointAt(maxIndex);
            removePointAt(minIndex);
        }else{
            removePointAt(minIndex);
            removePointAt(maxIndex);
        }
    }
	
    ((void)(rr));
}

/*************************************************************************
*    This function is used to pass raw (uncalibrated) magnetic data into the
*    calibration's internal data storage buffers. It also does a quality
*    check to determine of the 3D data point is good to use.
*
*    Inputs:
*
*    rawMagX:            - Raw magnetic data value (+- range) sensed from the X axes.
*
*    rawMagY:            - Raw magnetic data value (+- range) sensed from the Y axes.
*
*    rawMagZ:            - Raw magnetic data value (+- range) sensed from the Z axes.
*
*    Return Value:        - 0: Data point was rejected.    1: Data point was used.
*
*
*    Note 1:                - The function ical_readCheck() should always be called
*                          after this function is called to determine if enough
*                          data has been collected to generate calibration
*                          coefficients.
*
*/
static INT32 ical_collectDataPoint(INT32 rawMagX, INT32 rawMagY, INT32 rawMagZ )
{
    if (0 == ical_qualify(rawMagX, rawMagY, rawMagZ))
        return 0;
    ical_addSample(&icalMgr.trialMag[0], icalMgr.nsamps);
    return 1;
}

static INT32 ical_computeCalibration(TRANSFORM_T* xfp)
{
   INT32    irxyz[4];
   INT32 mag_z_Max = icalEns.mag[0][2];
   INT32 mag_z_Min = icalEns.mag[0][2];
   INT32 mag_x_diff = 0;
   INT32 mag_y_diff = 0;
   INT32 mag_z_diff = 0;
   float mag_res = 0;
   INT32 i = 0;

   if(!xfp)
   {
	   return 0;
   }

	for(i = 0; i < icalMgr.nsamps; i++)
	{
		if(icalEns.mag[i][2] > mag_z_Max)
		{
            mag_z_Max = icalEns.mag[i][2];
        }

        if(icalEns.mag[i][2] < mag_z_Min){
            mag_z_Min = icalEns.mag[i][2];
        }
        //ICAL_LOG("%s, %d %f %f %f",__func__, ix, magX[ix] / 31.25, magY[ix] / 31.25, magZ[ix] / 31.25);
        //ICAL_LOG("%s, %d %d %d %d",__func__, i, icalEns.mag[i][0],  icalEns.mag[i][1], icalEns.mag[i][2]);
	}
	if(abs(mag_z_Max - mag_z_Min) < MAG_Z_THRESH)
	{
		ICAL_LOG("return 2 error! %d %d %d", mag_z_Max, mag_z_Min, (mag_z_Max - mag_z_Min));
		return 2;
	}   
   if (1 != em_center(&icalEns.mag[0], &irxyz[0], icalMgr.nsamps))
           return 0;

    xfp->Offset[0] = irxyz[1];
    xfp->Offset[1] = irxyz[2];
    xfp->Offset[2] = irxyz[3];
    xfp->rr = (int)irxyz[0];

	//ŒÆËã²Ð²î
	for(i = icalMgr.nsamps - 16; i < icalMgr.nsamps; i++)
	{
		mag_x_diff = icalEns.mag[i][0] - irxyz[1];
		mag_y_diff = icalEns.mag[i][1] - irxyz[2];
		mag_z_diff = icalEns.mag[i][2] - irxyz[3];
		mag_res = fabs(sqrtf(mag_x_diff * mag_x_diff + mag_y_diff * mag_y_diff + mag_z_diff * mag_z_diff) - irxyz[0]);

		if(mag_res > MAG_RES)
		{
			ICAL_LOG("return 2 error! mag_res = %f ",mag_res);
			//return 2;		// qst0103 remove
		}
		//ICAL_LOG("%s %d %f %f %f",__func__, ix, magX[ix] / 31.25, magY[ix] / 31.25, magZ[ix] / 31.25);
		//ICAL_LOG("%s %f %d %d %d %d",__func__, mag_res, i, icalEns.mag[i][0],  icalEns.mag[i][1], icalEns.mag[i][2]);
	}
	//ICAL_LOG("vivo_qst %d %d %d", mag_z_Max, mag_z_Min, (mag_z_Max - mag_z_Min));

    return 1;
}

//////////////////////////////////////////////////////////////////////
////////////////// START OF INTERNAL LIBRARY CODE ////////////////////
//////////////////////////////////////////////////////////////////////

/*************************************************************************
*    This function is used to qualify a magnetic data point in both time and
*    direction/displacement.
*
*    Inputs:
*
*        rawMagX                - Raw magnetic field strength in the X axis direction.
*
*        rawMagY                - Raw magnetic field strength in the Y axis direction.
*
*        rawMagZ                - Raw magnetic field strength in the Z axis direction.
*
*
*    Return Value:            - 0: Data is rejected.    1: Data is valid.
*
*/
INT32 ical_qualify(INT32 rawMagX, INT32 rawMagY, INT32 rawMagZ)
{
    INT32 ii;
    INT32 vecDiff[3]={0};
    INT32 vector_magnitude;

    icalMgr.trialMag[0] = rawMagX;
    icalMgr.trialMag[1] = rawMagY;
    icalMgr.trialMag[2] = rawMagZ;

    if (icalMgr.nsamps == 0)
        return 1;

    ii  = icalMgr.nsamps;

    vector_magnitude = vec_magnitude(&(icalMgr.trialMag[0]), &(icalEns.mag[ii-1][0]), &vecDiff[0]);

    if (vector_magnitude < icalMgr.standoffCounts )
        return 0;

    while (ii > 0)
    {
        ii--;
        vector_magnitude = vec_magnitude(&(icalMgr.trialMag[0]), &(icalEns.mag[ii][0]), &vecDiff[0]);
        if (vector_magnitude < (icalMgr.standoffCounts * 0.5))
            return 0;

        if (vector_magnitude > 6000 )
        {
            ical_Init();
            return 0;
            }
    }

    return 1;
}

/*************************************************************************
*    This function simply adds the magnetic data values to the
*    internal data buffers and increments index counter.
*
*    Inputs:
*
*        mag        - Pointer to array containing magnetic data.
*
*        acc        - Pointer to array containing usage flags.
*
*        sampIX    - Current location to place data into array.
*
*/
void ical_addSample(INT32* mag, INT32 sampIX)
{
    INT32 ii;
    for (ii = 0;ii < 3;ii++)
        icalEns.mag[sampIX][ii] = mag[ii];
    icalMgr.nsamps = icalMgr.nsamps + 1;

    // Circle around if exceed buffer size
    if (icalMgr.nsamps >= MAXIMUM_DATA_SAMPLES)
        icalMgr.nsamps = 0;
}

/*
 *
 * Left shift data
 *
 * */
void ical_shiftData(INT32 length)
{
    INT32 i = 0;

    if(icalMgr.nsamps<=length || length>MAXNUM_BEFORE_ROLLBACK)
        return;

    for(i=0; i<(icalMgr.nsamps - length); i++){
        icalEns.mag[i][0] = icalEns.mag[i+length][0];
        icalEns.mag[i][1] = icalEns.mag[i+length][1];
        icalEns.mag[i][2] = icalEns.mag[i+length][2];
    }

    icalMgr.nsamps -= length;

    for(i=0; i<(MAXIMUM_DATA_SAMPLES - icalMgr.nsamps); i++){
        icalEns.mag[i+icalMgr.nsamps][0] = icalEns.mag[i+icalMgr.nsamps][1] = icalEns.mag[i+icalMgr.nsamps][2] = 0;
    }

}

/////////////////////////////////////////////////////////////////////
///////////////////// EM MATH FUNCTIONS /////////////////////////////
/////////////////////////////////////////////////////////////////////

// function to invert a matrix by Gaussian elimination
//
//    Maximum Matrix size: DIM_A
//
//    MatA is reduced to identity matrix
//    invMatA contains inverse matrix
//
//   Return Values:
//        0 = Successful inversion
//        -1 = Error: Singular Matrix

INT32 em_invert(INT32 size)
{
   REAL alpha, beta;
   INT32 ii, jj, kk;

   if( (size < 2) || (size > DIM_A))
       return 0;
   // initialize the reduction matrix--> unity matrix
   for(ii = 0; ii < size; ii++)
   {
      for(jj = 0; jj < size; jj++)
      {
         ma.invMatA[ii][jj] = 0.0f;

      }
      ma.invMatA[ii][ii] = 1.0f;
   }

   // do the reduction
   for(ii = 0; ii < size; ii++)
   {
      alpha = ma.MatA[ii][ii];
      if(alpha == 0.0f )
          return 0; // -------------------> bail out

      for(jj = 0; jj < size; jj++)
      {
         ma.MatA[ii][jj]    = ma.MatA[ii][jj] / alpha;
         ma.invMatA[ii][jj] = ma.invMatA[ii][jj] / alpha;

      } // END jj loop

      for(kk = 0; kk < size; kk++)
      {
         if((kk - ii) != 0)
         {
            beta = ma.MatA[kk][ii];
            for(jj = 0; jj < size; jj++)
            {
               ma.MatA[kk][jj]    = ma.MatA[kk][jj] - beta * ma.MatA[ii][jj];
               ma.invMatA[kk][jj] = ma.invMatA[kk][jj] - beta * ma.invMatA[ii][jj];
            }
         }
      } // END kk loop
   } // END ii loop
   return 1;
}

// Find the average of each component, report both integer and real versions
void em_average_demean(INT32 (*xyz)[3],INT32 *iavg, INT32 npts)
{
   INT32 ix;
   INT64 tempL[3] = {0,0,0};//add by lx

   for(ix = 0;ix < npts;ix++)
   {
      tempL[0] += xyz[ix][0];
      tempL[1] += xyz[ix][1];
      tempL[2] += xyz[ix][2];
   }

   iavg[0] = (INT32)(tempL[0] / npts);
   iavg[1] = (INT32)(tempL[1] / npts);
   iavg[2] = (INT32)(tempL[2] / npts);

    for (ix = 0; ix < npts; ix++) {
        xyz[ix][0] -= iavg[0];
        xyz[ix][1] -= iavg[1];
        xyz[ix][2] -= iavg[2];
    }   
}

// prepare data for hard iron estimate:
// subtract average, move data to real arrays, and initialize pointers
static void em_prepare_data(INT32 (*xyz)[3], INT32 *iavg, INT32 npts) 
{
    INT32 ix =0;
    
    // subtract average off of integer input data
    em_average_demean(xyz, iavg, npts);

    for (ix = 0; ix < npts; ix++)
    {
        magX[ix] = (REAL) xyz[ix][0];
        magY[ix] = (REAL) xyz[ix][1];
        magZ[ix] = (REAL) xyz[ix][2];
        SQ_rr[ix]   = (REAL) (magX[ix] * magX[ix] + magY[ix] * magY[ix] + magZ[ix] * magZ[ix]);
    }

    Jpt[0] = &SQ_rr[0];
    Jpt[1] = &magX[0];
    Jpt[2] = &magY[0];
    Jpt[3] = &magZ[0];
}

// J transpose times J
// This could be formed directly in ma.MatA to save a copy
static void em_JTJ(INT32 npts) {
    INT32 jrr, jcc, jkk;

    for (jrr = 0; jrr < PARA_3D; jrr++) {
        for (jcc = 0; jcc < PARA_3D; jcc++) {
            JTJ[jrr][jcc] = 0.0f;
            for (jkk = 0; jkk < npts; jkk++) {
                JTJ[jrr][jcc] += Jpt[jrr][jkk] * Jpt[jcc][jkk];
            }
        }
    }
}
// move data to matrix for inversion
static void em_moveData(void) {
    INT32 jrr, jcc;
    for (jrr = 0; jrr < PARA_3D; jrr++){
        for (jcc = 0; jcc < PARA_3D; jcc++){
            ma.MatA[jrr][jcc] = JTJ[jrr][jcc];
        }
    }
}
// vector K is a column of 1's
// so J'*K (J transpose times K --> JTK) is a column vector
// each of whose elements is the sum of one of the rows of J'
static void em_JTK(INT32 npts) {
    INT32 jrr, jkk;
    for (jrr = 0; jrr < PARA_3D; jrr++) {
        JTK[jrr] = 0.0f;
        for (jkk = 0; jkk < npts; jkk++) {
            JTK[jrr] += Jpt[jrr][jkk];
        }
    }
}

// inverse of J transpose times J ("JTJinv")
// times J'K
static void em_JTJinvJK(void) {
    INT32 jrr, jkk;
    for (jrr = 0; jrr < PARA_3D; jrr++) {
        Jout[jrr] = 0.0;
        for (jkk = 0; jkk < PARA_3D; jkk++) {
            Jout[jrr] += ma.invMatA[jrr][jkk] * JTK[jkk];
        }
    }
}

//    A(X^2 +Y^2 + Z^2) + Bx + Cy + D = 1 for sphere
// A is in inv[0]
// B is in inv[1] ...
// ...
//
// Return radius in inv[0]
// center offsets in inv[1..3] for sphere

static void  em_results(INT32 (*xyz)[3], INT32 *inavg, REAL *inv, INT32* ftoixyz,INT32 npts)
{
    REAL RR;
    REAL ofs[3];

    INT32 ii = 0;

    ofs[0] = -inv[1]/(2.0f * inv[0]);
    ofs[1] = -inv[2]/(2.0f * inv[0]);
    ofs[2] = -inv[3]/(2.0f * inv[0]);

    RR = sqrtf(ofs[0] * ofs[0] + ofs[1] * ofs[1] + ofs[2] * ofs[2] + 1/inv[0]);

    inv[0]=RR;
    inv[1]=ofs[0] + inavg[0];
    inv[2]=ofs[1] + inavg[1];
    inv[3]=ofs[2] + inavg[2];

    for(ii=0;ii < npts;ii++)
    {
        xyz[ii][0]+=inavg[0];
        xyz[ii][1]+=inavg[1];
        xyz[ii][2]+=inavg[2];
    }

    // integer output
    ftoixyz[0]=(INT32)(inv[0]);
    ftoixyz[1]=(INT32)(inv[1]);
    ftoixyz[2]=(INT32)(inv[2]);
    ftoixyz[3]=(INT32)(inv[3] );
}

INT32 em_center( INT32 (*xyz)[3], INT32 *ioutrxyz, INT32 npts)
{
   INT32 iavg[3] = {0};
   INT32 rc;

   em_prepare_data(xyz, &iavg[0], npts);    // subtract average, move to real arrays
   em_JTJ(npts);                            // form J'J
   em_JTK(npts);                            // form J'K
   em_moveData();                            // Move J'J to matrix for inversion
   rc=em_invert(PARA_3D);                    // invert J'J
   if(rc<0)
   {
      ioutrxyz[0]=0;
      return 0;
   }
   em_JTJinvJK();// Multiply J'K by inverted matrix
    // use output to find hard iron offset;
   em_results(xyz, iavg, &Jout[0], ioutrxyz,npts);
      // put average back into data and
    // move real results to integer ioutrxyz
    // hard iron offsets are in ioutrxyz[1..3]
   return 1;
}

INT32 vec_magnitude(INT32 *v0, INT32 *v1, INT32 *diff)
{
	INT64 square_sum = 0;
	if ((v0 == 0) || (v1 == 0) || (diff == 0))
		return 0;

    diff[0] = v1[0] - v0[0];
    diff[1] = v1[1] - v0[1];
    diff[2] = v1[2] - v0[2];
    
	square_sum = (diff[0]*diff[0]) + (diff[1]*diff[1]) + (diff[2]*diff[2]);
	if(square_sum < 0)
		return 0;
	return (INT32)(sqrt(square_sum));
}

static INT8 checkAccuracy(INT32 curMag, INT8 Status) {

    int8_t st = ACCURACY_UNRELIABLE;

//    ICAL_LOG("checkAccuracy, curMag = %d, AvgMagR = %d, \t|(%d)|", curMag, AvgMagR, (curMag - AvgMagR));
    if (mSabs(curMag - AvgMagR) < 1500)   //1500 //620
        st =  ACCURACY_HIGH;
    else if (mSabs(curMag - AvgMagR) < 3000)  //3000 //1240
        st = ACCURACY_MEDIUM;
    else if (mSabs(curMag - AvgMagR) < 4500)   //4500 //1500
        st = ACCURACY_LOW;
    else
        st = ACCURACY_UNRELIABLE;

    if(st != Status || st == ACCURACY_UNRELIABLE)
    {
        nDiffAccuracy_3D++;

        if (nDiffAccuracy_3D >=25)
        {
            nDiffAccuracy_3D = 0;
            ical_Init();
            diffSameCount = 0;
            IsFirstCalAfterReboot = 1;
            lastCalibCount = 0;
            st = ACCURACY_UNRELIABLE;
        }
    }

    return st;
}

static void update_offset(TRANSFORM_T *newcal)
{
    INT16 i = 0;
    INT64 offset_sum[4] = { 0, 0, 0, 0};

	
    ofsX = newcal->Offset[0];
    ofsY = newcal->Offset[1];
    ofsZ = newcal->Offset[2];
	AvgMagR = newcal->rr;

	offset_sum[0] = offset_sum[1] = offset_sum[2] = offset_sum[3] = 0;
	if(offset_avgN < OFFSET_AVG_N)
	{	
		i = offset_avgN;
		offset_n[i].Offset[0] = newcal->Offset[0];
		offset_n[i].Offset[1] = newcal->Offset[1];
		offset_n[i].Offset[2] = newcal->Offset[2];
		offset_n[i].rr = newcal->rr;
	}
	else
	{
	    for(i = 0; i < OFFSET_AVG_N-1; i++)
		{
			offset_n[i+1].Offset[0] = offset_n[i].Offset[0];
			offset_n[i+1].Offset[1] = offset_n[i].Offset[1];
			offset_n[i+1].Offset[2] = offset_n[i].Offset[2];
			offset_n[i+1].rr = offset_n[i].rr;
	    }
		offset_n[0].Offset[0] = newcal->Offset[0];
	    offset_n[0].Offset[1] = newcal->Offset[1];
	    offset_n[0].Offset[2] = newcal->Offset[2];
	    offset_n[0].rr = newcal->rr;
	}

    if(offset_avgN < OFFSET_AVG_N)
    {
        offset_avgN++;
    }
    else if(offset_avgN > OFFSET_AVG_N)
    {
        offset_avgN = OFFSET_AVG_N;
    }

	for(i = 0; i < offset_avgN; i++)
	{
		offset_sum[0] += offset_n[i].Offset[0];
		offset_sum[1] += offset_n[i].Offset[1];
		offset_sum[2] += offset_n[i].Offset[2];
		offset_sum[3] += offset_n[i].rr;
	}

    ofsX = (INT32)(offset_sum[0] / offset_avgN);
    ofsY = (INT32)(offset_sum[1] / offset_avgN);
    ofsZ = (INT32)(offset_sum[2] / offset_avgN);
	AvgMagR = (INT32)(offset_sum[3] / offset_avgN);

	ICAL_LOG("update_offset newcal[%d %d %d %d]avg[%d %d %d %d]",newcal->Offset[0],newcal->Offset[1],newcal->Offset[2],newcal->rr
																,ofsX,ofsY,ofsZ,AvgMagR);
}


int qst_ical_init(REAL *calipara)
{
    if(((mSabs(calipara[0]) < 1e-6) && (mSabs(calipara[1]) < 1e-6) && (mSabs(calipara[2]) < 1e-6))
		|| (mSabs(calipara[3]) < ONLY_UPDATE_LOWER_LIMIT) || (mSabs(calipara[3]) > ONLY_UPDATE_UPPER_LIMIT))
	{
        lastStatus = 0;
        ofsX = ofsY = ofsZ = 0;
        AvgMagR = (unsigned short)DEFAULT_NORMAL;
        ical_Init();
        lastR = 0;
        IsFirstCalAfterReboot = 1;
    } else {
        ical_Init();
        lastStatus = 3;
        ofsX = calipara[0] * 31.25f;
        ofsY = calipara[1] * 31.25f;
        ofsZ = calipara[2] * 31.25f;
        AvgMagR = lastR = calipara[3];
        IsFirstCalAfterReboot = 0;
    }

	return (int)lastStatus;
}

static int process(REAL* magData )
{
    int32_t rawMag_X = 0;
    int32_t rawMag_Y = 0;
    int32_t rawMag_Z = 0;
    int32_t ICALK_X = 0;
    int32_t ICALK_Y = 0;
    int32_t ICALK_Z = 0;
    INT32 OffsetDiff[3] = {0};

    int32_t rok; 
    int32_t ret = 0;
    INT32 MagR;

    rawMag_X = (int32_t)magData[0];
    rawMag_Y = (int32_t)magData[1];
    rawMag_Z = (int32_t)magData[2];

    ICALK_X = rawMag_X;
    ICALK_Y = rawMag_Y;
    ICALK_Z = rawMag_Z;

    magData[0] = (REAL) (rawMag_X - ofsX);
    magData[1] = (REAL) (rawMag_Y - ofsY);
    magData[2] = (REAL) (rawMag_Z - ofsZ);

    if(ical_collectDataPoint(ICALK_X, ICALK_Y, ICALK_Z))
    {
	    ICAL_LOG("ical_3D icalMgr.nsamps = %d, lastCalibCount = %d, IsFirstCalAfterReboot = %d", icalMgr.nsamps, lastCalibCount, IsFirstCalAfterReboot);
    }
    if ((icalMgr.nsamps > FIRST_CALIB_NUM && icalMgr.nsamps % CALIB_INTERVAL_NUM_DEFAULT == 0) && (lastCalibCount != icalMgr.nsamps))
    {
        rok = ical_computeCalibration(&newCal);
		ICAL_LOG("ical_computeCalibration rok=%d", rok);
        lastCalibCount =  icalMgr.nsamps;
        switch (rok)
        {
            case 1:
            ICAL_LOG("ical_3D Offset [%d %d %d]\r\n", newCal.Offset[0],newCal.Offset[1],newCal.Offset[2],newCal.rr);
            //ICAL_LOG("ical_3D Offset - Y: %d\r\n", newCal.Offset[1]);
            //ICAL_LOG("ical_3D Offset - Z: %d\r\n", newCal.Offset[2]);
            //ICAL_LOG("ical_3D Offset - R: %d\r\n", newCal.rr);

            if(newCal.rr > ONLY_UPDATE_LOWER_LIMIT && newCal.rr < ONLY_UPDATE_UPPER_LIMIT)
            {
                /*modify by Geoff;20210316
                 * Todo:Cannot recalibrate
                 * Before code: if(IsFirstCalAfterReboot == 1)
                 */
                if(1)		//IsFirstCalAfterReboot == 1)
                //End of Geoff;20210316
                {
                    update_offset(&newCal);

                    lastHard[0] = newCal.Offset[0];
                    lastHard[1] = newCal.Offset[1];
                    lastHard[2] = newCal.Offset[2];

                    lastR = AvgMagR = newCal.rr;
               #ifdef ALGO_9D_FUSION
                    ical_update_offset_flag = 150;
               #endif

                    ical_clearFarPoint(AvgMagR, ofsX, ofsY, ofsZ);
                    ical_clearFarPoint(AvgMagR, ofsX, ofsY, ofsZ);

                    diffSameCount = 1;
                    IsFirstCalAfterReboot = 0;
                    ret = 1;
                    break;
                }
            }

            ical_clearFarPoint(AvgMagR, ofsX, ofsY, ofsZ);
            ical_clearFarPoint(AvgMagR, ofsX, ofsY, ofsZ);

            vd = vec_magnitude(&newCal.Offset[0],&lastHard[0],&OffsetDiff[0]);
            vd_xy = (INT32)(sqrt(OffsetDiff[0]*OffsetDiff[0] + OffsetDiff[1]*OffsetDiff[1]));
            rd = mSabs(lastR - newCal.rr);

            //ICAL_LOG("ical_3D vd = %d, vd_xy = %d, rd = %d\r\n", (int)vd, (int)vd_xy, (int)rd);
            if (vd < VD_DIFF_MAX && vd_xy < VD_XY_DIFF_MAX && rd < RD_DIFF_MAX)            
                diffSameCount++;
            else
                diffSameCount = 0;

            //ICAL_LOG("ical_3D diffSameCount = %d \r\n", diffSameCount);
            if(newCal.rr > ONLY_UPDATE_LOWER_LIMIT && newCal.rr < ONLY_UPDATE_UPPER_LIMIT)
            {
                if (diffSameCount >= ONLY_UPDATE_DIFFSAMECOUNT)
                {
                    ret = 1;
                    //update_offset(&newCal);
                    //lastHard[0] = newCal.Offset[0];
                    //lastHard[1] = newCal.Offset[1];
                    //lastHard[2] = newCal.Offset[2];
                    //lastR = AvgMagR = newCal.rr;
                    
               #ifdef ALGO_9D_FUSION
                    ical_update_offset_flag = 150;
               #endif
                    ical_clearFarPoint(AvgMagR, ofsX, ofsY, ofsZ);
                }
                else
                {
                    lastCalibCount =0;
                    //IsFirstCalAfterReboot = 1;
                    //lastStatus = 0;
                    ical_Init();
                    diffSameCount = 0;
                    ret = 0;
                    //ICAL_LOG("ical_3D diffSameCount < ONLY_UPDATE_DIFFSAMECOUNT (%d)\r\n", diffSameCount);
                }
            }
            else
            {
                lastCalibCount =0;
                IsFirstCalAfterReboot = 1;
                lastStatus = 0;
                diffSameCount = 0;
                ical_Init();
                ret = 0;
                //ICAL_LOG("ical_3D rr is over range (%d)\r\n",newCal.rr);
            }

            if (lastCalibCount >= MAXNUM_BEFORE_ROLLBACK)
            {
                ical_shiftData(MAXNUM_BEFORE_ROLLBACK / 2);
            }
            break;

            case 2:
            break;
            default:
            {
                lastCalibCount =0;
                IsFirstCalAfterReboot = 1;
                lastStatus = 0;
                diffSameCount = 0;
                ical_Init();
                ret = 0;
                //ICAL_LOG("ical_3D Calibration Error\r\n");
            }
        }//switch
    }//if FIRST_CALIB_NUM

    MagR = (INT32)(sqrtf(magData[0]*magData[0] + magData[1]*magData[1] +magData[2]*magData[2]));
    
    if (IsFirstCalAfterReboot != 1)
    {
        lastStatus = checkAccuracy(MagR, lastStatus);
    }

    curAccuracy = lastStatus;
    //ICAL_LOG("curAccuracy = %d\r\n",curAccuracy);
#ifdef ALGO_9D_FUSION
    if(ical_update_offset_flag > 0)
	{
		ical_update_offset_flag--;
	}
#endif
    return ret;
}

int convert_magnetic(REAL *raw, REAL *result,REAL *offset,REAL *RR,int8_t *accuracy)
{
    int ret = 0;
    REAL magRaw[3] = {0.0f};

    magRaw[0] = raw[0] * 31.25f;
    magRaw[1] = raw[1] * 31.25f;
    magRaw[2] = raw[2] * 31.25f;

    ret = process(magRaw);
    
    result[0] = magRaw[0] / 31.25f;
    result[1] = magRaw[1] / 31.25f;
    result[2] = magRaw[2] / 31.25f;

    offset[0] = ofsX / 31.25f;
    offset[1] = ofsY / 31.25f;
    offset[2] = ofsZ / 31.25f;
    *RR = AvgMagR;

    *accuracy = curAccuracy;

    return ret;
}


#ifdef Head_AVG_Filter
static float filter_ori[8] = { 0 };
static float filter_ori_temp[8] = { 0 };
static float filter_ori_sum = 0;
#endif
int qst_ori_progress(float *Acc,float *Mag,float *Ori,int8_t *accuracy)
{
    float pitch,ipitch = 0;
    float roll,iroll = 0;
    float Ax,Ay,Az,av;
    float mMagData[3] = {0.0f};
    float Head = 0;
    float yaw_anti;
#ifdef VIVO_SAVE_OFFSET
    float yaw_special;
#endif
    float yaw = 0;
    float hx,hy;
    float cp,sp,sr,cr;
    float normalize = sqrtf(Acc[0] * Acc[0] + Acc[1] * Acc[1] + Acc[2] * Acc[2]);
    Ax = Acc[0]/normalize;
    Ay = Acc[1]/normalize;
    Az = Acc[2]/normalize;

    av = sqrtf(Ax*Ax + Ay*Ay + Az*Az);
    pitch = atan2f(Ay,Az);
    ipitch = pitch * 180 / PI;
    roll  = asinf(Ax / av);
    iroll = -roll * 180 / PI;

//    ICAL_LOG("qst_ori_progress acc %f %f %f %f %f\n",Ax,Ay,Az,ipitch,iroll);

    cp = cosf(pitch);
    sp = sinf(pitch);
    sr = sinf(roll);
    cr = cosf(roll);
    normalize = sqrtf(Mag[0] * Mag[0] + Mag[1] * Mag[1] + Mag[2] * Mag[2]);
    mMagData[0] = Mag[0] / normalize;
    mMagData[1] = Mag[1] / normalize;
    mMagData[2] = Mag[2] / normalize;
    hx = mMagData[0] * cr + mMagData[1] * sp*sr+mMagData[2] *(sp*sr+cp*sr);
    hy = mMagData[1] * cp - mMagData[2] * sp;
    yaw_anti = -atan2f(hx, hy);
#ifdef VIVO_SAVE_OFFSET
    float roll_special = -atan2f(Ax,Az);
    float pitch_special = asinf(Ay/av);
    cp = cosf(pitch_special);
    sp = sinf(pitch_special);
    sr = sinf(roll_special);
    cr = cosf(roll_special);
    hx = mMagData[0] * cr + mMagData[2] *sr;
    hy = mMagData[0] * sp * sr + mMagData[1] * cp - mMagData[2] * sp*cr;
    yaw_special = -atan2f(hx, hy);
    //ICAL_LOG("wzy roll_s pitch_s %f %f\n",roll_special* 180 / PI,pitch_special * 180 / PI);
    if(fabs(pitch_special * TODEG)>=45){
        //ICAL_LOG("wzy anti yaw1 %f\n",yaw_anti);
        yaw = yaw_anti;
    }else if(fabs(pitch)<fabs(roll_special) && fabs(roll_special*TODEG)>160) {
        //ICAL_LOG("wzy anti yaw2 %f\n",yaw_anti);
#endif
        yaw = yaw_anti;
#ifdef VIVO_SAVE_OFFSET
    }else{
        //ICAL_LOG("wzy special yaw %f\n",yaw_special);
        yaw = yaw_special;

    }
#endif
    Head = yaw * 180 / PI;
    //ICAL_LOG("wzy head %f\n",Head);
   
    if(Head < 0)
    {
        Head += 360.0f;
    }
#ifdef Head_AVG_Filter
    int i = 0;
    int j = 0;
    filter_ori_sum = 0;
        
    for (j = 0; j < 7; j++)
    {
        filter_ori[j] = filter_ori[j + 1];

    }
    filter_ori[7] = Head;

    if(Head >= 330  || Head <= 30)
    {
        for(i = 1; i <= 7; i++)
        {
            double dtHead = filter_ori[0] - filter_ori[i];
            if(dtHead > 330)
                filter_ori_temp[i] = 360  + filter_ori[i];
            else if(dtHead <= -330)
                filter_ori_temp[i] = filter_ori[i] - 360;
            else
                filter_ori_temp[i] = filter_ori[i];
        }

        filter_ori_temp[0] =filter_ori[0];

        for (i = 0; i < 8; i++)
        {
            filter_ori_sum += filter_ori_temp[i];
        }
    }else{
        for (i = 0; i < 8; i++){
            filter_ori_sum += filter_ori[i];
        }
    }

    Head = filter_ori_sum / 8;

    if(Head >= 360)
        Head = Head - 360;
    else if(Head < 0)
        Head = Head + 360;

#endif

#ifdef QST_SUPPORT_YAW
    int hope =0;
    hope = (int)((Head / 45) + 0.5) * 45;

    if(Head >= (hope - 10) && Head <= (hope + 10))
       Head = hope * 0.5 + 0.5 * Head;
    else if(Head >= (hope - 20) && Head < (hope - 10))
       Head = (hope -5) - ((hope - 10) - Head) * 1.5;
    else if(Head > (hope + 10) && Head < (hope + 20))
       Head = (hope + 5) + (Head - (hope + 10)) * 1.5;
#endif
    *accuracy = curAccuracy;
    Ori[0] = fmod(Head,360);
    Ori[1] = -ipitch;
    Ori[2] = -iroll;
    return 1;
}


