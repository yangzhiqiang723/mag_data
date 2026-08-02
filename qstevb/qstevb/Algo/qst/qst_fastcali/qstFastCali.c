#include "qstFastCali.h"
//#define _CRT_SECURE_NO_WARNINGS

//static float Offset_mag[3] = { 100.0f,100.0f,100.0f };//硬磁初始值
//static float last_offset[3] = {0};
#define M 1862
#define N 6
#define ArrayDataN 5
#define Max_RR 170 //160
#define Mid_RR 120 //110
#define Min_RR 90//75
//int count = 0;
//定义一个结构体
struct m_ArrayData 
{
	float data[ArrayDataN][7];
	int count;
};
static float Pk_PQR[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
static float RMatrix_PQR[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
static float QMatrix_PQR[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
static float Fin_PQR[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
static float AvgMagR=40.0f;
#define myabss(x)	((x) >= 0.0 ? (x) : -(x)) 
static int GlobalLv = 0;//手机系统当前的磁力计的Lv等级
//static int init_flag = 0;


static float MagucLastdata[3] = { 0,0,0 };
//static float MagOffset[3] = { 80.0f,80.0f,80.0f };//硬磁初始值
static float Mag_Offset[3] = { 80.0f,80.0f,80.0f };//硬磁初始值
static float used_offset[3] = {0};
// static float last_offset[3] = {0};
static float uncalimags[3] ={0};
//static float MagNow[3] = { 0 };
static float MagNows[3] = { 0 };
//static float get_offset[3] = {0};
static int flag_zero_three = 0; 
//struct m_ArrayData DataSaveReverse = { 0 };//保存动态下的陀螺仪+磁力计数据
struct m_ArrayData Data_SaveReverse = { 0 };//保存动态下的陀螺仪+磁力计数据
//void convert_magnetic_KFM(float *uncalimag,float *gyro,float *calimag,float *offset,float dt,int *lv,int restart)
//#define ALGO_9D_FUSION


#ifdef ALGO_9D_FUSION
static int ical_update_offset_flag = 0;
void ical_update_offset_flag_sync_to_fusion(int *flag);
void rr_sync_to_fusion(float *rr);
void lv_sync_to_fusion(float *lv,int *zero_to_three);
void rr_sync_to_fusion(float *rr)
{
	*rr = AvgMagR;									
}

void lv_sync_to_fusion(float *lv, int *zero_to_three)
{
	*lv =  GlobalLv;
	*zero_to_three = flag_zero_three;
}
 
void ical_update_offset_flag_sync_to_fusion(int *flag)
{
	*flag = ical_update_offset_flag;
}
#endif


//构造3*3的矩阵乘法
static void  MulMatrix(float a[3][3], float b[3][3], float c[3][3])
{
	int i = 0, j = 0, k = 0;
	for(i = 0; i < 3; i++)
	{
		for(j = 0; j < 3; j++)
		{
			c[i][j] = 0;
			for(k = 0; k < 3; k++)
			{
				c[i][j] += a[i][k] * b[k][j];
			}
		}
	}
}



//构造转置矩阵
static void MatrixTransFunction(float Matrix[3][3], float MatrixTrans[3][3])
{
	int i = 0, j = 0;
	for(i = 0;i < 3;i++)
	{
		for(j = 0;j < 3;j++)
		{
			MatrixTrans[j][i] = Matrix[i][j];
		}
	}
}

//�����һ��?
static void matrix_normal(float Weight[3][3])
{
	float temp_matrix[3][3];
	float normlize = 0;
	int i, j;
	float temp_data = 0;
	//��һ��
	temp_data = Weight[0][0] * Weight[0][0] + Weight[1][0] * Weight[1][0] + Weight[2][0] * Weight[2][0];
	if(temp_data < 0)
	{
		temp_data = 1.0;
	}
	normlize = sqrtf(temp_data);
	for(i = 0;i < 3;i++)
	{
		temp_matrix[i][0] = Weight[i][0] / normlize;
	}
	//�ڶ���
	
	temp_data = Weight[0][1] * Weight[0][1] + Weight[1][1] * Weight[1][1] + Weight[2][1] * Weight[2][1];
	if(temp_data < 0)
	{
		temp_data = 1.0;
	}
	normlize = sqrtf(temp_data);
	for(i = 0;i < 3;i++)
	{
		temp_matrix[i][1] = Weight[i][1] / normlize;
	}
	//������
	temp_data = Weight[0][2] * Weight[0][2] + Weight[1][2] * Weight[1][2] + Weight[2][2] * Weight[2][2];
	if(temp_data < 0)
	{
		temp_data = 1.0;
	}
	normlize = sqrtf(temp_data);
	for(i = 0;i < 3; i++)
	{
		temp_matrix[i][2] = Weight[i][2] / normlize;
	}
	for(i = 0; i < 3; i++)
	{
		for(j = 0;j < 3;j++)
		{
			Weight[i][j] = temp_matrix[i][j];
		}
	}
}

//创造矩阵微分方程,用作量测
static void CreateFin(float gyro[3], float dt, float Hk[3][3])
{
	float Weight[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
	int i = 0, j = 0;
	float gyro_norm = 0;
	float Deg = 0;
	gyro_norm = gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2];
	gyro_norm = sqrtf(gyro_norm)*dt;
	//if (gyro_norm == 0)
	if(myabss(gyro_norm) <= 0.0000001)
	{
		Deg = 1;
	}
	else
	{
		//Deg = sin(gyro_norm) / gyro_norm;
		Deg = sinf(gyro_norm) / gyro_norm;
	}

	//第一行
	Weight[0][0] = 1;
	Weight[0][1] = -gyro[2] * dt*Deg;
	Weight[0][2] = gyro[1] * dt*Deg;
	//第二行
	Weight[1][0] = gyro[2] * dt*Deg;
	Weight[1][1] = 1;
	Weight[1][2] = -gyro[0] * dt*Deg;
	//第三行
	Weight[2][0] = -gyro[1] * dt*Deg;
	Weight[2][1] = gyro[0] * dt*Deg;
	Weight[2][2] = 1;

	matrix_normal(Weight);

	//转置操作
	for(i = 0;i < 3;i++)
	{
		for(j = 0;j < 3;j++)
		{
			Hk[j][i] = Weight[i][j];
		}
	}
}



//卡尔曼滤波初始化
//void InitMatrix(float Pk[3][3], float RMatrix[3][3], float QMatrix[3][3], float Fin[3][3])
void InitMatrix(void)
{
	//int i = 0, j = 0;
	Pk_PQR[0][0] = 90000.0f;
	Pk_PQR[1][1] = 90000.0f;
	Pk_PQR[2][2] = 90000.0f;
	RMatrix_PQR[0][0] = 2.0f;//1.0f;
	RMatrix_PQR[1][1] = 2.0f;//1.0f;
	RMatrix_PQR[2][2] = 2.0f;//1.0f;
	QMatrix_PQR[0][0] = 0.4f;
	QMatrix_PQR[1][1] = 0.4f;
	QMatrix_PQR[2][2] = 0.4f;
	Fin_PQR[0][0] = 1.0f;
	Fin_PQR[1][1] = 1.0f;
	Fin_PQR[2][2] = 1.0f;
}


void KFilterPredict(float Xk[3], float XkPredict[3], float Pk[3][3], float PkPredict[3][3])
{
	int i = 0, j = 0;
	for(i = 0; i < 3; i++)
	{
		XkPredict[i] = Xk[i];
	}
	for(i = 0; i < 3; i++)
	{
		for(j = 0; j < 3; j++)
		{
			PkPredict[i][j] = Pk[i][j];
		}
	}
}


void Matrix_inverse(float arc[3][3], int n, float ans[3][3])//计算矩阵的逆
{
	int i, j, k;//列
	float max, tempA, tempB, P;
	int max_num;
	float arcs[3][3];
	for(i = 0; i < 3; i++)
	{
		for(j = 0; j < 3; j++)
		{
			arcs[i][j] = arc[i][j];
		}
	}
	for(i = 0; i < n; i++)
	{
		ans[i][i] = 1;
	}
	for(i = 0; i < n; i++)//第i列
	{
		max = (float)fabs(arcs[i][i]);
		max_num = i;
		for(j = i + 1; j < n; j++)//选出主元
		{
			if(fabs(arcs[j][i]) > max)
			{
				max = (float)fabs(arcs[j][i]);
				max_num = j;
			}
		}
		/*if(max == 0)
		{
		printf("i can't");
		break;
		}*/
		for(k = 0; k < n; k++)//交换行
		{
			tempA = arcs[i][k];
			arcs[i][k] = arcs[max_num][k];
			arcs[max_num][k] = tempA;
			tempB = ans[i][k];
			ans[i][k] = ans[max_num][k];
			ans[max_num][k] = tempB;
		}
		for(k = i + 1; k < n; k++)
		{
			P = arcs[k][i] / arcs[i][i];
			for (j = 0; j < n; j++)
			{
				arcs[k][j] = arcs[k][j] - arcs[i][j] * P;
				ans[k][j] = ans[k][j] - ans[i][j] * P;
			}
		}
	}
	for(i = 0; i < n; i++)//行
	{
		P = arcs[i][i];
		for(j = i; j < n; j++)
		{
			arcs[i][j] = arcs[i][j] / P;
		}
		for(j = 0; j < n; j++)
		{
			ans[i][j] = ans[i][j] / P;
		}
	}
	for(i = n - 1; i > 0; i--)
	{
		for(j = i - 1; j >= 0; j--)
		{
			for(k = 0; k < n; k++)
			{
				ans[j][k] = ans[j][k] - ans[i][k] * arcs[j][i];
			}
		}
	}
}

void CreateKk(float PkPredict[3][3], float Hk[3][3], float RMatrix[3][3], float Kk[3][3])
{
	float HkTrans[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
	float Hk_PkP[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
	float Pkp_HkT[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
	float InvPkp_HkT[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
	float PkP_HkT[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
	int i = 0, j = 0;
	MatrixTransFunction(Hk, HkTrans);//
	MulMatrix(Hk, PkPredict, Hk_PkP);
	MulMatrix(Hk_PkP, HkTrans, Pkp_HkT);
	for(i = 0;i < 3;i++)
	{
		for(j = 0;j < 3;j++)
		{
			Pkp_HkT[i][j] = Pkp_HkT[i][j] + RMatrix[i][j];
		}
	}
	Matrix_inverse(Pkp_HkT, 3, InvPkp_HkT);//矩阵求逆
	MulMatrix(PkPredict, HkTrans, PkP_HkT);
	MulMatrix(PkP_HkT, InvPkp_HkT, Kk);
}

void KFMagOffset(float MagucNow[3], float MagucLastTime[3], float gyro[3], float MagOffset[3], float Ts)
{
	float Hk[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
	float Xk[3] = { 0 }; //Ӳ��offset
	float XkPredict[3] = { 0 };
	//float Ts = 0.02;
	float PkPredict[3][3] = { {0,0,0},{0,0,0},{0,0,0} };
	float ZData[3] = { 0 };
	float ZDataMeasure[3] = { 0 };
	float Delta[3] = { 0 };
	float GetData[3] = { 0 };
	//float Temp[3][3] = { 0 };
	float KkHk[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
	int i = 0, j = 0;
	float Kk[3][3] = {{0,0,0},{0,0,0},{0,0,0}};

	//ALOGD("lx KFMagOffset in Offset:%f %f %f ts:%f\n",MagOffset[0],MagOffset[1],MagOffset[2],Ts);
	for(i = 0; i < 3; i++)
	{
		Xk[i] = MagOffset[i];
	}
	//
	CreateFin(gyro, Ts, Hk);

#if 0
	//for(j = 0;j<3;j++)
	{
		//ALOGD("lx KFMagOffset Hk[%d][]:%f %f %f\n",j,Hk[j][0],Hk[j][1],Hk[j][2]);
	}
#endif
	
	ZData[0] = Hk[0][0] * MagucLastTime[0] + Hk[0][1] * MagucLastTime[1] + Hk[0][2] * MagucLastTime[2];
	ZData[1] = Hk[1][0] * MagucLastTime[0] + Hk[1][1] * MagucLastTime[1] + Hk[1][2] * MagucLastTime[2];
	ZData[2] = Hk[2][0] * MagucLastTime[0] + Hk[2][1] * MagucLastTime[1] + Hk[2][2] * MagucLastTime[2];
	for(i = 0; i < 3; i++)
	{
		ZDataMeasure[i] = ZData[i] - MagucNow[i];
		//printf("%f ", ZDataMeasure[i]);
	}
	//预测
	KFilterPredict(Xk, XkPredict, Pk_PQR, PkPredict);
	//更改Hk
	Hk[0][0] = Hk[0][0] - 1;
	Hk[1][1] = Hk[1][1] - 1;
	Hk[2][2] = Hk[2][2] - 1;
	//卡尔曼反馈环节

	//1 计算量测值
	GetData[0] = Hk[0][0] * XkPredict[0] + Hk[0][1] * XkPredict[1] + Hk[0][2] * XkPredict[2];
	GetData[1] = Hk[1][0] * XkPredict[0] + Hk[1][1] * XkPredict[1] + Hk[1][2] * XkPredict[2];
	GetData[2] = Hk[2][0] * XkPredict[0] + Hk[2][1] * XkPredict[1] + Hk[2][2] * XkPredict[2];
	for (i = 0; i < 3; i++)
	{
		Delta[i] = ZDataMeasure[i] - GetData[i];
	}
	//2 构造增益矩阵Kk
	CreateKk(PkPredict, Hk, RMatrix_PQR, Kk);
	//for(j = 0;j<3;j++)
	//{
	//	ALOGD("lx KFMagOffset Kk[%d][]:%f %f %f\n",j,Kk[j][0],Kk[j][1],Kk[j][2]);
	//}

	//更新XK看，Pk
	Xk[0] = XkPredict[0] + Kk[0][0] * Delta[0] + Kk[0][1] * Delta[1] + Kk[0][2] * Delta[2];
	Xk[1] = XkPredict[1] + Kk[1][0] * Delta[0] + Kk[1][1] * Delta[1] + Kk[1][2] * Delta[2];
	Xk[2] = XkPredict[2] + Kk[2][0] * Delta[0] + Kk[2][1] * Delta[1] + Kk[2][2] * Delta[2];
	MulMatrix(Kk, Hk, KkHk);
	for(i = 0; i < 3; i++)
	{
		for(j = 0; j < 3; j++)
		{
			KkHk[i][j] = -KkHk[i][j];
		}
	}
	for(i = 0;i < 3;i++)
	{
		KkHk[i][i] = 1 + KkHk[i][i];
	}
	MulMatrix(KkHk, PkPredict, Pk_PQR);
	for(i = 0; i < 3;i++)
	{
		if(Pk_PQR[i][i] > 90000.0f)
		{
			Pk_PQR[i][i] = 900000.0f;
		}
		if(Pk_PQR[i][i] < 0.01f)
		{
			Pk_PQR[i][i] = 0.5f;
		}
	}

	//��ֵ��Magoffsset
	for(i = 0; i < 3; i++)
	{
		MagOffset[i] = Xk[i];
	}

	//ALOGD("lx KFMagOffset out Offset:%f %f %f\n",MagOffset[0],MagOffset[1],MagOffset[2]);
}



//求模值
float NormData(float data[3])
{
	float result = 0;
	int i = 0;
	for(i = 0;i < 3;i++)
	{
		result += data[i] * data[i];
	}
	if(result < 0) 
	{
		result = 0; 
	}
	result = sqrtf(result);
	return result;
}

//计算两个向量的减法:(A-B)

void VecSub(float result[3],float A[3],float B[3])
{
	int j = 0;
	for(j = 0; j < 3; j++)
	{
		result[j] = A[j] - B[j];
	}
}



//清空数据队列
void ClearArrayData(struct m_ArrayData *ArrayData)
{
	int i, j;
	for(i = 0; i < ArrayDataN; i++)
	{
		for(j = 0; j < 7; j++)
		{
			ArrayData->data[i][j] = 0;
		}
	}

	ArrayData->count = 0;
}

void AddArrayData(struct m_ArrayData *ArrayData, float newdata[6], float Ts)
{
	int i, j;
	for(i = 1; i < ArrayDataN; i++)
	{
		for (j = 0;j < 7;j++)
		{
			ArrayData->data[i - 1][j] = ArrayData->data[i][j];
		}
	}
	for(j = 0; j < 6; j++)
	{
		ArrayData->data[ArrayDataN - 1][j] = newdata[j];
	}
	ArrayData->data[ArrayDataN - 1][6] = Ts;
	ArrayData->count = ArrayData->count + 1;
	if(ArrayData->count >= ArrayDataN)
	{
		ArrayData->count = ArrayDataN;
	}
}


//重新初始化
void HugeMagInter(float RNows)
{
	//if ((RNow < 15 || RNow>80) && (GlobalLv == 3))
	if((RNows < 15 || RNows>Max_RR) && (GlobalLv == 3))
	{
		GlobalLv = 0;
		Pk_PQR[0][0] = 90000;
		Pk_PQR[1][1] = 90000;
		Pk_PQR[2][2] = 90000;
	}
}

//验证半径在合理范围内
int VerifyR(struct m_ArrayData DataSaveReverse, float MagOffset[3])
{
	float R[ArrayDataN] = { 0 };
	float Result[3] = { 0 };
	int i = 0,j = 0;
	int Flag = 0;
	if(DataSaveReverse.count < ArrayDataN)
	{
	  //ALOGD("lx count=%d\n",DataSaveReverse.count);
		Flag = 1;
		return Flag;
	}

	/*if (DataSaveReverse.count == ArrayDataN)
	{
			ALOGD("lx ARRIVE ArrayDataN=15 \n");
	}*/

	for(i = 0; i < ArrayDataN; i++)
	{
		for(j = 0; j < 3; j++)
		{	
//		 printf("lx VerifyR Function  DataSaveReverse.data=%f  MagOffset=%f ",DataSaveReverse.data[i][j+3],MagOffset[j]);
//			 printf("lx VerifyR Function %d DataSaveReverse.data=%f  MagOffset=%f ",j, DataSaveReverse.data[i][j],MagOffset[j]);
			Result[j] = DataSaveReverse.data[i][j+3] - MagOffset[j];
		}
		
//		printf("\n");
		
		R[i] = NormData(Result);
		
//		printf("lx VerifyR Function i=%d R[i]=%f \n",i,R[i]);
		if(R[i]< 15 || R[i] > 80)
		{
			Flag = 1;
			//ClearArrayData(&DataSaveReverse);
			return Flag;
		}
	}
	return Flag;
}


//利用陀螺仪验正offset的可靠性
float CheckTransMag(const struct m_ArrayData DataSaveReverse, float MagOffset[3])
{
	//int Flag = 0;
	int i = 0, j = 0;
	float MagNow[3] = { 0 };
	float MagLast[3] = { 0 };
	float gyro[3] = {0};
	float MagucNow[3] = {0};
	float MagucLast[3] = { 0 };
	float Fin[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
	float ZData[3] = { 0 };
	float DiffData[3] = { 0 };
	float DiffDataNorm = 0;
	float MaxDiffDataNorm = 0;
	float Ts = 0;
	for(i = ArrayDataN - 1; i <= 1; i++)
	{
		for(j = 0; j < 3; j++)
		{
			gyro[j] = DataSaveReverse.data[i][j];
			MagucNow[j] = DataSaveReverse.data[i][j + 3];//本时刻的值
			MagucLast[j] = DataSaveReverse.data[i-1][j + 3];//上一时刻的值
		}
		VecSub(MagNow, MagucNow, MagOffset);
		VecSub(MagLast, MagucLast, MagOffset);
		Ts = DataSaveReverse.data[i][6];//采样间隔
		CreateFin(gyro, Ts, Fin);
		ZData[0] = Fin[0][0] * MagLast[0] + Fin[0][1] * MagLast[1] + Fin[0][2] * MagLast[2];
		ZData[1] = Fin[1][0] * MagLast[0] + Fin[1][1] * MagLast[1] + Fin[1][2] * MagLast[2];
		ZData[2] = Fin[2][0] * MagLast[0] + Fin[2][1] * MagLast[1] + Fin[2][2] * MagLast[2];
		for(j = 0; j < 3; j++)
		{
			DiffData[j] = ZData[j] - MagNow[j];			
		}
		DiffDataNorm = NormData(DiffData);
		if (DiffDataNorm > MaxDiffDataNorm)
		{
			MaxDiffDataNorm = DiffDataNorm;
		}
	}
	return MaxDiffDataNorm;
}



//验正磁力计精确度等级
void CheckLv(const struct m_ArrayData DataSaveReverse, float MagOffset[3])
{

	int i = 0;//j = 0;
	int PkLvCount = 0;
	float MaxTransDiff = 0;
	int QualityFlag = 0;
	//static init_num = 0;
	//计算Pk收敛程度
	for(i = 0; i < 3; i++)
	{
		if(Pk_PQR[i][i] < 16)
		{
			PkLvCount = PkLvCount + 1;
		}
	}
	
	// if(init_flag ==1 && GlobalLv==3)
	// {
	// 	init_num ++ ;
	// 	if(init_num >10){
	// 		init_flag = 0;
	// 		init_num = 0;
	// 	}
	// }else{
//	printf("lx CheckLv PkLvCount=%d\n",PkLvCount);
	if(PkLvCount == 3) 
	{
		if(MaxTransDiff <= 4)
		{
			QualityFlag = 3;
		}
		else if(MaxTransDiff > 4 && MaxTransDiff <= 8)
		{
			QualityFlag = 2;
		}
		else if(MaxTransDiff > 8 && MaxTransDiff <= 10)
		{
			QualityFlag = 1;
		}
		else
		{
			QualityFlag = 0;
		}
		//ALOGD("lx CheckLv QualityFlag=%d PkLvCount=%d\n",QualityFlag,PkLvCount);
		if(QualityFlag <= PkLvCount)
		{
			GlobalLv = QualityFlag;
		}
		else
		{
			GlobalLv = PkLvCount;
		}

		float calimag_lx[3] = {0};
		VecSub(calimag_lx, uncalimags, Mag_Offset);
		AvgMagR = NormData(calimag_lx);
		
		#ifdef ALGO_9D_FUSION
		ical_update_offset_flag = 110;//150
		#endif
	}else
	{
		GlobalLv = PkLvCount;
	}
}

//
void KfLoop(struct m_ArrayData ArrayData, float MagOffset[3])
{
	//int n = ArrayData.count;
	int i = 0,j = 0;
	float MagucNow[3] = { 0 };
	float MagucLastTime[3] = { 0 };
	float gyro[3] = { 0 };
	float Ts = 0;
	//float Ts = 0;//move to func para
	for(i = ArrayDataN - 1; i < 2; i--)
	{
		Ts = ArrayData.data[i][6];
		for(j = 0; j < 3; j++)
		{
			gyro[j] = ArrayData.data[i][j];
		}

		for(j = 3; j < 6; j++)
		{
			MagucNow[j] = ArrayData.data[i][j];
			MagucLastTime[j]= ArrayData.data[i-1][j];				
		}
		KFMagOffset(MagucNow, MagucLastTime, gyro, MagOffset, Ts);
	}
}


void init_KFM(void)
{
	//InitMatrix(Pk_PQR, RMatrix_PQR, QMatrix_PQR, Fin_PQR);
	InitMatrix();
}

// static float MagucLastdata[3] = { 0,0,0 };
// //static float MagOffset[3] = { 80.0f,80.0f,80.0f };//硬磁初始值
// static float Mag_Offset[3] = { 80.0f,80.0f,80.0f };//硬磁初始值
// //static float MagNow[3] = { 0 };
// static float MagNows[3] = { 0 };
// //struct m_ArrayData DataSaveReverse = { 0 };//保存动态下的陀螺仪+磁力计数据
// struct m_ArrayData Data_SaveReverse = { 0 };//保存动态下的陀螺仪+磁力计数据
// //void convert_magnetic_KFM(float *uncalimag,float *gyro,float *calimag,float *offset,float dt,int *lv,int restart)
void convert_magnetic_fastcali(float *uncalimag,float *gyro,float *calimag,float *offset,float dt,int *lv,int restart)
{
	float MagucNow[3] = { 0,0,0 };
	//float MagucLastTime[3] = { 0,0,0 };
	float RNow = 0;
	float gyro_norm = 0;
	float NowData[6] = { 0 };
	int i = 0, j = 0;
	int Flag_Back = 1;//默认必须反馈
	int Running_Mode = 0;//默认静止	

	//float Fin[3][3] = { 0 };
	if(restart >= 1)
	{
		MagucLastdata[0] = uncalimag[0];
		MagucLastdata[1] = uncalimag[1];
		MagucLastdata[2] = uncalimag[2];
		//init_KFM();       
	}
	
	MagucNow[0] = uncalimag[0];
	MagucNow[1] = uncalimag[1];
	MagucNow[2] = uncalimag[2];

	uncalimags[0] = uncalimag[0];
	uncalimags[1] = uncalimag[1];
	uncalimags[2] = uncalimag[2];
	
	//ALOGD("lx convert_magnetic_KFM pk:%f %f %f\n",Pk_PQR[0][0],Pk_PQR[1][1],Pk_PQR[2][2]);
//	printf("lx convert_magnetic_KFM magraw:%f %f %f dt:%f \n",MagucNow[0],MagucNow[1],MagucNow[2],dt);
	//ALOGD("lx convert_magnetic_KFM magrawlast:%f %f %f\n",MagucLastdata[0],MagucLastdata[1],MagucLastdata[2]);
//	printf("lx convert_magnetic_KFM gyro:%f %f %f\n",gyro[0],gyro[1],gyro[2]);
	//KFMagOffset(MagucNow, MagucLastdata, gy, MagOffset,dt);//���������offset
	NowData[0]= gyro[0];
	NowData[1]= gyro[1];
	NowData[2]= gyro[2];
	NowData[3]= uncalimag[0];
	NowData[4]= uncalimag[1];
	NowData[5]= uncalimag[2];
 
	Flag_Back = 1;//默认必须反馈
	Running_Mode = 0;//默认静止
	VecSub(MagNows, MagucNow, Mag_Offset);
	RNow = NormData(MagNows);
	//1 干扰下初始化Lv
	HugeMagInter(RNow);
	// 2 算陀螺仪模值
	gyro_norm = NormData(gyro);
	//if(gyro_norm > 0.05) { Running_Mode = 1; }
	if(gyro_norm > 0.08) 
	{
		Running_Mode = 1; 
	}
	//3 动态下保存数据，否则清0
	if((Running_Mode == 1) && (GlobalLv < 3))
	{
		AddArrayData(&Data_SaveReverse, NowData,dt);
	}
	else
	{
		ClearArrayData(&Data_SaveReverse);
	}
	
	//4 判断是否反馈
	//if ((RNow < 15.0 || RNow>80.0)) { 
	if(RNow < 15.0 || RNow > Max_RR)
	{ 
		Flag_Back = 1;
		GlobalLv = 0;
		for(i = 0; i < 3; i++)
		{
			Pk_PQR[i][i] = 90000;
		} 
  }
	else if(RNow < Max_RR && RNow > Mid_RR && GlobalLv == 2)
	{
		Flag_Back = 1;
    GlobalLv = 1;
	}
	else if(RNow < Mid_RR && RNow > Min_RR && GlobalLv == 3)
	{
		Flag_Back = 1;
    GlobalLv = 2;
	}
	else
	{	
		//验证R
		Flag_Back = VerifyR(Data_SaveReverse, Mag_Offset);        
		//ALOGD("lx Flag_Back=%d GlobalLv=%d\n",Flag_Back,GlobalLv);
		//半径通过后，修改验正Lv等级
		if((Flag_Back == 0)&& GlobalLv < 3)
		{
			//ALOGD("lx CheckLv\n");
			//KfLoop(Data_SaveReverse, Mag_Offset);
			CheckLv(Data_SaveReverse, Mag_Offset);//gegnxin lv
			ClearArrayData(&Data_SaveReverse);
		}
	}

	//5 判断是否执行卡尔曼滤波反馈环节，更新磁力计offset
	if(Running_Mode == 1)
	{
		KFMagOffset(MagucNow, MagucLastdata, gyro, Mag_Offset, dt);
		if(GlobalLv == 3)
		{
			float calimag_lx[3] = {0};
			VecSub(calimag_lx, MagucNow, Mag_Offset);
			AvgMagR = NormData(calimag_lx);
		}
	#if 0
		if(myabss(Mag_Offset[0]) < 3000.0f && myabss(Mag_Offset[0]) > 0.0f &&
		   myabss(Mag_Offset[1]) < 3000.0f && myabss(Mag_Offset[1]) >0.0f &&
		   myabss(Mag_Offset[2]) < 3000.0f &&  myabss(Mag_Offset[2]) > 0.0f)
		{
			if(last_offset[0] != Mag_Offset[0] || last_offset[1] != Mag_Offset[1] && last_offset[2] != Mag_Offset[2])
			{

				last_offset[0] = Mag_Offset[0];
				last_offset[1] = Mag_Offset[1];
				last_offset[2] = Mag_Offset[2];

			}	
		}
		else
		{
			Mag_Offset[0] = last_offset[0];
			Mag_Offset[1] = last_offset[1];
			Mag_Offset[2] = last_offset[2];	
		}

		float calimag_lx[3] ={0};
		VecSub(calimag_lx, MagucNow, Mag_Offset);
		AvgMagR = NormData(calimag_lx);
	#endif

		// #ifdef ALGO_9D_FUSION
		// ical_update_offset_flag = 110;//150
		// #endif
			//把存储的数据再运行反馈，加速Pk收敛
			//if (Data_SaveReverse.count == ArrayDataN-1)
			{
			 //   KfLoop(Data_SaveReverse, MagOffset);
			 //   KfLoop(Data_SaveReverse, MagOffset);
			 //   KfLoop(Data_SaveReverse, MagOffset);
			}
		}
    VecSub(MagNows, MagucNow, Mag_Offset);
		RNow = NormData(MagNows);
    //if ((RNow < 15.0 || RNow>80.0))
		if(RNow < 15.0 || RNow>Max_RR)
    {
      for(i = 0; i < 3; i++)
      {
        if(Pk_PQR[i][i] < 80)
        {
					Pk_PQR[i][i] = MagucNow[i] * MagucNow[i];
        }
      } 
		}
		//我增加的，用来显示PK矩阵非主对角线的值，防止跑飞  230726   
		for(i = 0;i<3;i++)
		{
			for(j = 0;j<3;j++)
			{
				if(i != j)
				{
					if(myabss(Pk_PQR[i][j]) > 90000.0f)
					{
						Pk_PQR[i][j] = 0;
					}
				}
			}
		}

		//我增加的，用来显示PK矩阵主对角线的值，防止跑飞  230726   
		for(i = 0; i < 3; i++)
		{
			if(myabss(Pk_PQR[i][i]) > 90000.0f)
			{
				Pk_PQR[i][i] = 90000.0f;
			}
		}
    
    if(Pk_PQR[0][0] < 1 && Pk_PQR[1][1] < 1 && Pk_PQR[2][2] < 1) 
    {
			for(i = 0; i < 3; i++)
			{
				Pk_PQR[i][i] = 2;   // old 25 yzq     
			} 
    }
//    printf("lx convert_magnetic_KFM endover  pk:%f %f %f\n",Pk_PQR[0][0],Pk_PQR[1][1],Pk_PQR[2][2]);
    //ALOGD("lx liuhao720AM\n");
	MagucLastdata[0] = uncalimag[0];
	MagucLastdata[1] = uncalimag[1];
	MagucLastdata[2] = uncalimag[2];
	//ALOGD("lx convert_magnetic_KFM offset:%f %f %f\n",MagOffset[0],MagOffset[1],MagOffset[2]);
//	printf("lx GlobalLv=%d offset:%f %f %f\n",GlobalLv,Mag_Offset[0],Mag_Offset[1],Mag_Offset[2]);

	if(myabss(Mag_Offset[0]) < 2000.0f && myabss(Mag_Offset[0]) > 0.0f &&
		myabss(Mag_Offset[1]) < 2000.0f && myabss(Mag_Offset[1]) > 0.0f &&
		myabss(Mag_Offset[2]) < 2300.0f &&  myabss(Mag_Offset[2]) > 0.0f)
	{
		if(GlobalLv > 0)//add by lx 230731
		{
			if(used_offset[0] != Mag_Offset[0] || used_offset[1] != Mag_Offset[1] || used_offset[2] != Mag_Offset[2])
			{
				used_offset[0] = Mag_Offset[0];
				used_offset[1] = Mag_Offset[1];
				used_offset[2] = Mag_Offset[2];
			}
		}
		else
		{
			//add by lx 230801
			if(used_offset[0] != Mag_Offset[0] || used_offset[1] != Mag_Offset[1] || used_offset[2] != Mag_Offset[2])
			{
				if(myabss(used_offset[0] - Mag_Offset[0]) < 35.0f && myabss(used_offset[1] - Mag_Offset[1]) < 35.0f &&
					myabss(used_offset[2] - Mag_Offset[2]) < 45.0f)
				{
					used_offset[0] = Mag_Offset[0];
					used_offset[1] = Mag_Offset[1];
					used_offset[2] = Mag_Offset[2];
				}
			}			
		}
	}
	//ALOGD("lx AvgMagR=%f:%d used_offset:%f %f %f\n",AvgMagR,used_offset[0],used_offset[1],used_offset[2]);
	offset[0] = used_offset[0];
	offset[1] = used_offset[1];
	offset[2] = used_offset[2];
	offset[3] = Pk_PQR[0][0];
	offset[4] = Pk_PQR[1][1];
	offset[5] = Pk_PQR[2][2];

	calimag[0] = uncalimag[0] - used_offset[0];
	calimag[1] = uncalimag[1] - used_offset[1];
	calimag[2] = uncalimag[2] - used_offset[2];	

	*lv = GlobalLv;
	//*rr = AvgMagR;
	#ifdef ALGO_9D_FUSION
	if(ical_update_offset_flag > 0)
	{
		ical_update_offset_flag--;
	}
	#endif
}

void qst_fastcali_init(float offset[6])
{
	init_KFM();
	if((offset[0] < 1e-6 && offset[1] < 1e-6 && offset[2] < 1e-6) || offset[3] > 25 || offset[4] > 25 || offset[5] > 25)
	{
		Mag_Offset[0] = 80.0f;
		Mag_Offset[1] = 80.0f;
		Mag_Offset[2] = 80.0f;
		AvgMagR = 40.0f;
		GlobalLv = 0;
  } 
	else 
	{
		Mag_Offset[0] = offset[0];
		Mag_Offset[1] = offset[1];
		Mag_Offset[2] = offset[2];
		AvgMagR = 40.0f;
		GlobalLv = 3;
		Pk_PQR[0][0] = offset[3];
		Pk_PQR[1][1] = offset[4];
		Pk_PQR[2][2] = offset[5];
  }
}

