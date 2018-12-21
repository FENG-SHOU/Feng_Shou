//**************************************************************//
//    GNSS/INS Loose-Coupled Integrated Navigation Algorithm	//
//**************************************************************//
//	Author: 									//
//	This algorithm is used for GNSS/INS Integrated Navigation	//
//	System, its core is a Extended Kalman Filter. For Saving	//
//	RAM, P Matrix of Filter only save half triangular section,	//
//	moreover, the huge matrix calculation is expanded to some 	//
//	small pieces(3*3 matrix) for saving CPU resource(See de-	//
//	tails in Head file).										//
//									2015.4.12					//
//																//
//	Copyright  All Right Reserved.				//
//**************************************************************//

#include "GIFilter.h"
#include "GlobalVars.h"
#include "BasicFunc.h"
#include "Gnss.h"
#include "DataProc.h"
#include <string.h>
#include <math.h>


STATIC GIKF_DATA_T GIKFdata;
FLOAT64 *pBlock[15][3];
CONST U8 line_start[] = { 0, 1, 3, 6, 10, 15, 21, 28, 36, 45, 55, 66, 78, 91, 105, 120 };

//初始化kalman滤波器
void GIKFInit(void)
{
	MEMSET(GIKFdata.X, 0, SIZEOF(GIKFdata.X));
	MEMSET(GIKFdata.P, 0, SIZEOF(GIKFdata.P));
	MEMSET(GIKFdata.Q, 0, SIZEOF(GIKFdata.Q));
	MEMSET(GIKFdata.Fpp, 0, SIZEOF(GIKFdata.Fpp));
	MEMSET(GIKFdata.Fpv, 0, SIZEOF(GIKFdata.Fpv));
	MEMSET(GIKFdata.Fvp, 0, SIZEOF(GIKFdata.Fvp));
	MEMSET(GIKFdata.Fvv, 0, SIZEOF(GIKFdata.Fvv));
	MEMSET(GIKFdata.Fva, 0, SIZEOF(GIKFdata.Fva));
	MEMSET(GIKFdata.Faa, 0, SIZEOF(GIKFdata.Faa));
	MEMSET(GIKFdata.Fvba_abg, 0, SIZEOF(GIKFdata.Fvba_abg));

	//GetQMatrix();
	GIKFModularizePMatrix(pBlock, GIKFdata.P);


}

//P矩阵分块，15阶下三角矩阵分为15个3*3子矩阵
void GIKFModularizePMatrix(FLOAT64 *pBlock[][3], FLOAT64 *pmtx)
{
	S32 i, j, k;
	for (i = 0; i < 5; i++)
	{
		for (j = line_start[i]; j < line_start[i + 1]; j++)
		{
			for (k = 0; k < 3; k++)
				pBlock[j][k] = &pmtx[line_start[3 * i + k] + 3 * (j - line_start[i])];
		}
	}
}

//初始化过程噪声方差阵
void GetQMatrix(void)		// Init Q Matrix            //gty？ 仅仅初始化速度、角度、加速度和角速度的噪声，位置和高度的噪声为零？？
{
	GIKFdata.Q[0] = GIKFdata.Q[1] = POS_RAND_WALK_LEVEL*POS_RAND_WALK_LEVEL;
	GIKFdata.Q[2] = POS_RAND_WALK_ALT*POS_RAND_WALK_ALT;
	GIKFdata.Q[3] = GIKFdata.Q[4] = GIKFdata.Q[5] = VEL_RAND_WALK*VEL_RAND_WALK;
	GIKFdata.Q[6] = GIKFdata.Q[7] = GIKFdata.Q[8] = ANG_RAND_WALK*ANG_RAND_WALK;
	GIKFdata.Q[9] = GIKFdata.Q[10] = GIKFdata.Q[11] = ACC_CONST_BIAS*ACC_CONST_BIAS;
	GIKFdata.Q[12] = GIKFdata.Q[13] = GIKFdata.Q[14] = GYRO_CONST_BIAS*GYRO_CONST_BIAS;
}

//初始化P阵
void GIKFInitPMatrix(void)		// Init P Matrix
{

	GIKFdata.P[0] = GIKFdata.P[2] = 25.0;
	GIKFdata.P[5] = 64.0;
	GIKFdata.P[9] = GIKFdata.P[14] = 0.25;
	GIKFdata.P[20] = 0.64;
	GIKFdata.P[27] = GIKFdata.P[35] = GIKFdata.P[44] = 1.0*1.0*DEG2RAD*DEG2RAD;
	GIKFdata.P[54] = GIKFdata.P[65] = GIKFdata.P[77] = 0.5*0.5;//ACC_CONST_BIAS*ACC_CONST_BIAS;//
	GIKFdata.P[90] = GIKFdata.P[104] = GIKFdata.P[119] = 0.0005*0.0005;//GYRO_CONST_BIAS*GYRO_CONST_BIAS;//




}

//按照惯导误差模型计算状态矩阵的各子块
void GIKFCalcPHIMatrix(U32 MsInterval)
{
	FLOAT64 r, temp[3], dt = MsInterval / 1000.;

	// 位置vs位置
	GIKFdata.Fpp[0] = GIKFdata.Fpp[4] = GIKFdata.Fpp[8] = 1.0;
	GIKFdata.Fpp[1] = g_GINavInfo.Wen[2] * dt;		GIKFdata.Fpp[2] = -g_GINavInfo.Wen[1] * dt;
	GIKFdata.Fpp[3] = -g_GINavInfo.Wen[2] * dt;		GIKFdata.Fpp[5] = g_GINavInfo.Wen[0] * dt;
	GIKFdata.Fpp[6] = g_GINavInfo.Wen[1] * dt;		GIKFdata.Fpp[7] = -g_GINavInfo.Wen[0] * dt;
	// 位置vs速度
	GIKFdata.Fpv[0] = GIKFdata.Fpv[4] = GIKFdata.Fpv[8] = dt;
	// 速度vs位置
	r = sqrt(g_GINavInfo.Rm * g_GINavInfo.Rn);
	GIKFdata.Fvp[0] = GIKFdata.Fvp[4] = -g_GINavInfo.Gravity / (r + g_GINavInfo.Position.Alt)*dt;
	GIKFdata.Fvp[8] = -2.0 * GIKFdata.Fvp[0];
	// 速度vs速度
	temp[0] = (2.0*g_GINavInfo.Wie[0] + g_GINavInfo.Wen[0]) * dt;
	temp[1] = (2.0*g_GINavInfo.Wie[1] + g_GINavInfo.Wen[1]) * dt;
	temp[2] = (2.0*g_GINavInfo.Wie[2] + g_GINavInfo.Wen[2]) * dt;
	GIKFdata.Fvv[0] = GIKFdata.Fvv[4] = GIKFdata.Fvv[8] = 1.0;
	GIKFdata.Fvv[1] = temp[2];		GIKFdata.Fvv[2] = -temp[1];
	GIKFdata.Fvv[3] = -temp[2];		GIKFdata.Fvv[5] = temp[0];
	GIKFdata.Fvv[6] = temp[1];		GIKFdata.Fvv[7] = -temp[0];
	// 速度vs姿态
	GIKFdata.Fva[1] = -g_GINavInfo.SF_n[2];		GIKFdata.Fva[2] = g_GINavInfo.SF_n[1];
	GIKFdata.Fva[3] = g_GINavInfo.SF_n[2];		GIKFdata.Fva[5] = -g_GINavInfo.SF_n[0];
	GIKFdata.Fva[6] = -g_GINavInfo.SF_n[1];		GIKFdata.Fva[7] = g_GINavInfo.SF_n[0];
	// 姿态vs姿态
	temp[0] = (g_GINavInfo.Wie[0] + g_GINavInfo.Wen[0]) * dt;
	temp[1] = (g_GINavInfo.Wie[1] + g_GINavInfo.Wen[1]) * dt;
	temp[2] = (g_GINavInfo.Wie[2] + g_GINavInfo.Wen[2]) * dt;
	GIKFdata.Faa[0] = GIKFdata.Faa[4] = GIKFdata.Faa[8] = 1.0;
	GIKFdata.Faa[1] = temp[2];		GIKFdata.Faa[2] = -temp[1];
	GIKFdata.Faa[3] = -temp[2];		GIKFdata.Faa[5] = temp[0];
	GIKFdata.Faa[6] = temp[1];		GIKFdata.Faa[7] = -temp[0];
	// 速度vs加计bias & 姿态vs陀螺bias
	GIKFdata.Fvba_abg[0] = g_GINavInfo.CM_bn.C11 * dt;
	GIKFdata.Fvba_abg[1] = g_GINavInfo.CM_bn.C12 * dt;
	GIKFdata.Fvba_abg[2] = g_GINavInfo.CM_bn.C13 * dt;
	GIKFdata.Fvba_abg[3] = g_GINavInfo.CM_bn.C21 * dt;
	GIKFdata.Fvba_abg[4] = g_GINavInfo.CM_bn.C22 * dt;
	GIKFdata.Fvba_abg[5] = g_GINavInfo.CM_bn.C23 * dt;
	GIKFdata.Fvba_abg[6] = g_GINavInfo.CM_bn.C31 * dt;
	GIKFdata.Fvba_abg[7] = g_GINavInfo.CM_bn.C32 * dt;
	GIKFdata.Fvba_abg[8] = g_GINavInfo.CM_bn.C33 * dt;
}

void GIKFPredictPMatrix(U32 MsInterval)
{
	S32 i;
	FLOAT64 Pk_1[STATE_DIM*(STATE_DIM + 1) / 2], dt = MsInterval / 1000.;
	FLOAT64 *pBlock_1[15][3];

	MEMCPY(Pk_1, GIKFdata.P, SIZEOF(GIKFdata.P));
	MEMSET(GIKFdata.P, 0, SIZEOF(GIKFdata.P));

	//Modularize Pk_1 matrix
	GIKFModularizePMatrix(pBlock_1, Pk_1);

	//PHI*P*PHI' 展开形式，忽略0元素乘法
	// PHI*P*PHI'
	//p0 = Fpp*conj(p1)*conj(Fpv) + Fpp*p0*conj(Fpp) + Fpv*p1*conj(Fpp) + Fpv*p2*conj(Fpv)
	AddMxMtxMt(pBlock[0], GIKFdata.Fpp, pBlock_1[1], GIKFdata.Fpv, 3, 3, 3, 3, 0x02);
	AddMxMxMt(pBlock[0], GIKFdata.Fpp, pBlock_1[0], GIKFdata.Fpp, 3, 3, 3, 3, 0x03);
	AddMxMxMt(pBlock[0], GIKFdata.Fpv, pBlock_1[1], GIKFdata.Fpp, 3, 3, 3, 3, 0x02);
	AddMxMxMt(pBlock[0], GIKFdata.Fpv, pBlock_1[2], GIKFdata.Fpv, 3, 3, 3, 3, 0x03);
	//p1 = Fvp*conj(p1)*conj(Fpv) + Fvp*p0*conj(Fpp) + Fvv*p1*conj(Fpp) + Fvv*p2*conj(Fpv) + Fva*p3*conj(Fpp) + Fva*p4*conj(Fpv) + Fvba*p6*conj(Fpp) + Fvba*p7*conj(Fpv)
	AddMxMtxMt(pBlock[1], GIKFdata.Fvp, pBlock_1[1], GIKFdata.Fpv, 3, 3, 3, 3, 0x00);
	AddMxMxMt(pBlock[1], GIKFdata.Fvp, pBlock_1[0], GIKFdata.Fpp, 3, 3, 3, 3, 0x01);
	AddMxMxMt(pBlock[1], GIKFdata.Fvv, pBlock_1[1], GIKFdata.Fpp, 3, 3, 3, 3, 0x00);
	AddMxMxMt(pBlock[1], GIKFdata.Fvv, pBlock_1[2], GIKFdata.Fpv, 3, 3, 3, 3, 0x01);
	AddMxMxMt(pBlock[1], GIKFdata.Fva, pBlock_1[3], GIKFdata.Fpp, 3, 3, 3, 3, 0x00);
	AddMxMxMt(pBlock[1], GIKFdata.Fva, pBlock_1[4], GIKFdata.Fpv, 3, 3, 3, 3, 0x00);
	AddMxMxMt(pBlock[1], GIKFdata.Fvba_abg, pBlock_1[6], GIKFdata.Fpp, 3, 3, 3, 3, 0x00);
	AddMxMxMt(pBlock[1], GIKFdata.Fvba_abg, pBlock_1[7], GIKFdata.Fpv, 3, 3, 3, 3, 0x00);
	//p2 = Fvp*conj(p1)*conj(Fvv) + Fvp*conj(p3)*conj(Fva) + Fvv*conj(p4)*conj(Fva) + Fvp*conj(p6)*conj(Fvba) + Fvv*conj(p7)*conj(Fvba) + Fva*conj(p8)*conj(Fvba) + Fvp*p0*conj(Fvp) +
	//	Fvv*p1*conj(Fvp) + Fvv*p2*conj(Fvv) + Fva*p3*conj(Fvp) + Fva*p4*conj(Fvv) + Fva*p5*conj(Fva) + Fvba*p6*conj(Fvp) + Fvba*p7*conj(Fvv) + Fvba*p8*conj(Fva) + Fvba*p9*conj(Fvba)
	AddMxMtxMt(pBlock[2], GIKFdata.Fvp, pBlock_1[1], GIKFdata.Fvv, 3, 3, 3, 3, 0x02);
	AddMxMtxMt(pBlock[2], GIKFdata.Fvp, pBlock_1[3], GIKFdata.Fva, 3, 3, 3, 3, 0x02);
	AddMxMtxMt(pBlock[2], GIKFdata.Fvv, pBlock_1[4], GIKFdata.Fva, 3, 3, 3, 3, 0x02);
	AddMxMxMt(pBlock[2], GIKFdata.Fvp, pBlock_1[0], GIKFdata.Fvp, 3, 3, 3, 3, 0x03);
	AddMxMxMt(pBlock[2], GIKFdata.Fvv, pBlock_1[1], GIKFdata.Fvp, 3, 3, 3, 3, 0x02);
	AddMxMxMt(pBlock[2], GIKFdata.Fvv, pBlock_1[2], GIKFdata.Fvv, 3, 3, 3, 3, 0x03);
	AddMxMxMt(pBlock[2], GIKFdata.Fva, pBlock_1[3], GIKFdata.Fvp, 3, 3, 3, 3, 0x02);
	AddMxMxMt(pBlock[2], GIKFdata.Fva, pBlock_1[4], GIKFdata.Fvv, 3, 3, 3, 3, 0x02);
	AddMxMxMt(pBlock[2], GIKFdata.Fva, pBlock_1[5], GIKFdata.Fva, 3, 3, 3, 3, 0x03);
	AddMxMtxMt(pBlock[2], GIKFdata.Fvp, pBlock_1[6], GIKFdata.Fvba_abg, 3, 3, 3, 3, 0x02);
	AddMxMtxMt(pBlock[2], GIKFdata.Fvv, pBlock_1[7], GIKFdata.Fvba_abg, 3, 3, 3, 3, 0x02);
	AddMxMtxMt(pBlock[2], GIKFdata.Fva, pBlock_1[8], GIKFdata.Fvba_abg, 3, 3, 3, 3, 0x02);
	AddMxMxMt(pBlock[2], GIKFdata.Fvba_abg, pBlock_1[6], GIKFdata.Fvp, 3, 3, 3, 3, 0x02);
	AddMxMxMt(pBlock[2], GIKFdata.Fvba_abg, pBlock_1[7], GIKFdata.Fvv, 3, 3, 3, 3, 0x02);
	AddMxMxMt(pBlock[2], GIKFdata.Fvba_abg, pBlock_1[8], GIKFdata.Fva, 3, 3, 3, 3, 0x02);
	AddMxMxMt(pBlock[2], GIKFdata.Fvba_abg, pBlock_1[9], GIKFdata.Fvba_abg, 3, 3, 3, 3, 0x03);
	//p3 = Fabg*p10*conj(Fpp) + Fabg*p11*conj(Fpv) + Faa*p3*conj(Fpp) + Faa*p4*conj(Fpv)
	AddMxMxMt(pBlock[3], GIKFdata.Fvba_abg, pBlock_1[10], GIKFdata.Fpp, 3, 3, 3, 3, 0x00);
	AddMxMxMt(pBlock[3], GIKFdata.Fvba_abg, pBlock_1[11], GIKFdata.Fpv, 3, 3, 3, 3, 0x00);
	AddMxMxMt(pBlock[3], GIKFdata.Faa, pBlock_1[3], GIKFdata.Fpp, 3, 3, 3, 3, 0x00);
	AddMxMxMt(pBlock[3], GIKFdata.Faa, pBlock_1[4], GIKFdata.Fpv, 3, 3, 3, 3, 0x00);
	//p4 = Faa*conj(p8)*conj(Fvba) + Fabg*p12*conj(Fva) + Fabg*p13*conj(Fvba) + Fabg*p10*conj(Fvp) + Fabg*p11*conj(Fvv) + Faa*p3*conj(Fvp) + Faa*p4*conj(Fvv) + Faa*p5*conj(Fva)
	AddMxMxMt(pBlock[4], GIKFdata.Faa, pBlock_1[3], GIKFdata.Fvp, 3, 3, 3, 3, 0x00);
	AddMxMxMt(pBlock[4], GIKFdata.Faa, pBlock_1[4], GIKFdata.Fvv, 3, 3, 3, 3, 0x00);
	AddMxMxMt(pBlock[4], GIKFdata.Faa, pBlock_1[5], GIKFdata.Fva, 3, 3, 3, 3, 0x01);
	AddMxMtxMt(pBlock[4], GIKFdata.Faa, pBlock_1[8], GIKFdata.Fvba_abg, 3, 3, 3, 3, 0x00);
	AddMxMxMt(pBlock[4], GIKFdata.Fvba_abg, pBlock_1[10], GIKFdata.Fvp, 3, 3, 3, 3, 0x00);
	AddMxMxMt(pBlock[4], GIKFdata.Fvba_abg, pBlock_1[11], GIKFdata.Fvv, 3, 3, 3, 3, 0x00);
	AddMxMxMt(pBlock[4], GIKFdata.Fvba_abg, pBlock_1[12], GIKFdata.Fva, 3, 3, 3, 3, 0x00);
	AddMxMxMt(pBlock[4], GIKFdata.Fvba_abg, pBlock_1[13], GIKFdata.Fvba_abg, 3, 3, 3, 3, 0x00);
	//p5 = Faa*conj(p12)*conj(Fabg) + Fabg*p12*conj(Faa) + Fabg*p14*conj(Fabg) + Faa*p5*conj(Faa)
	AddMxMxMt(pBlock[5], GIKFdata.Faa, pBlock_1[5], GIKFdata.Faa, 3, 3, 3, 3, 0x03);
	AddMxMtxMt(pBlock[5], GIKFdata.Faa, pBlock_1[12], GIKFdata.Fvba_abg, 3, 3, 3, 3, 0x02);
	AddMxMxMt(pBlock[5], GIKFdata.Fvba_abg, pBlock_1[12], GIKFdata.Faa, 3, 3, 3, 3, 0x02);
	AddMxMxMt(pBlock[5], GIKFdata.Fvba_abg, pBlock_1[14], GIKFdata.Fvba_abg, 3, 3, 3, 3, 0x03);
	//p6 = Fba*p6*conj(Fpp) + Fba*p7*conj(Fpv)
	AddMxMt(pBlock[6], pBlock_1[6], GIKFdata.Fpp, 3, 3, 3, 0x00);
	AddMxMt(pBlock[6], pBlock_1[7], GIKFdata.Fpv, 3, 3, 3, 0x00);
	//p7 = Fba*p6*conj(Fvp) + Fba*p7*conj(Fvv) + Fba*p8*conj(Fva) + Fba*p9*conj(Fvba)
	AddMxMt(pBlock[7], pBlock_1[6], GIKFdata.Fvp, 3, 3, 3, 0x00);
	AddMxMt(pBlock[7], pBlock_1[7], GIKFdata.Fvv, 3, 3, 3, 0x00);
	AddMxMt(pBlock[7], pBlock_1[8], GIKFdata.Fva, 3, 3, 3, 0x00);
	AddMxMt(pBlock[7], pBlock_1[9], GIKFdata.Fvba_abg, 3, 3, 3, 0x01);
	//p8 = Fba*conj(p13)*conj(Fabg) + Fba*p8*conj(Faa)
	AddMtxMt(pBlock[8], pBlock_1[13], GIKFdata.Fvba_abg, 3, 3, 3, 0x00);
	AddMxMt(pBlock[8], pBlock_1[8], GIKFdata.Faa, 3, 3, 3, 0x00);
	//p9 = Fba*p9*conj(Fba)
	Addequal(pBlock[9], pBlock_1[9], 3, 3, 0x03);
	//p10 = Fbg*p10*conj(Fpp) + Fbg*p11*conj(Fpv)
	AddMxMt(pBlock[10], pBlock_1[10], GIKFdata.Fpp, 3, 3, 3, 0x00);
	AddMxMt(pBlock[10], pBlock_1[11], GIKFdata.Fpv, 3, 3, 3, 0x00);
	//p11 = Fbg*p10*conj(Fvp) + Fbg*p11*conj(Fvv) + Fbg*p12*conj(Fva) + Fbg*p13*conj(Fvba)
	AddMxMt(pBlock[11], pBlock_1[10], GIKFdata.Fvp, 3, 3, 3, 0x00);
	AddMxMt(pBlock[11], pBlock_1[11], GIKFdata.Fvv, 3, 3, 3, 0x00);
	AddMxMt(pBlock[11], pBlock_1[12], GIKFdata.Fva, 3, 3, 3, 0x00);
	AddMxMt(pBlock[11], pBlock_1[13], GIKFdata.Fvba_abg, 3, 3, 3, 0x00);
	//p12 = Fbg*p12*conj(Faa) + Fbg*p14*conj(Fabg)
	AddMxMt(pBlock[12], pBlock_1[12], GIKFdata.Faa, 3, 3, 3, 0x00);
	AddMxMt(pBlock[12], pBlock_1[14], GIKFdata.Fvba_abg, 3, 3, 3, 0x01);
	//p13 = Fbg*p13*conj(Fba)
	Addequal(pBlock[13], pBlock_1[13], 3, 3, 0x00);
	//p14 = Fbg*p14*conj(Fbg)
	Addequal(pBlock[14], pBlock_1[14], 3, 3, 0x03);

	// Add Q
	GetQMatrix();
	for (i = 0; i < 15; i++)
		GIKFdata.P[line_start[i + 1] - 1] += GIKFdata.Q[i] * dt;


}

//Kalman测量更新
//Mask - 更新掩码，用于屏蔽或打开某状态量的更新
void GIKFBatchSolution(U32 Mask, U32 MsrCount, FLOAT64* Msr, FLOAT64* HMatrix, FLOAT64* Sigma)
{
	U32 i, j, k;
	FLOAT64 PHt[STATE_DIM*MAX_MSR_DIM] = { 0.0 }, KMatrix[STATE_DIM*MAX_MSR_DIM] = { 0.0 };
	FLOAT64 temp[STATE_DIM*STATE_DIM];
	FLOAT64 *pHPHt;

	// PHt
	for (i = 0; i < STATE_DIM; i++)
	{
		for (j = 0; j < MsrCount; j++)
		{
			PHt[i*MsrCount + j] = 0.0;
			for (k = 0; k < STATE_DIM; k++)
			{
				if (k <= i)
					PHt[i*MsrCount + j] += GIKFdata.P[line_start[i] + k] * HMatrix[j*STATE_DIM + k];
				else
					PHt[i*MsrCount + j] += GIKFdata.P[line_start[k] + i] * HMatrix[j*STATE_DIM + k];
			}
		}
	}
	// HPHt
	pHPHt = &temp[0];
	for (i = 0; i < MsrCount; i++)
	{
		for (j = line_start[i]; j < line_start[i + 1]; j++)
		{
			pHPHt[j] = 0.0;
			for (k = 0; k < STATE_DIM; k++)
				pHPHt[j] += HMatrix[i*STATE_DIM + k] * PHt[k*MsrCount + j - line_start[i]];
		}
	}
	// HPHt+R
	for (i = 0; i < MsrCount; i++)
		pHPHt[line_start[i + 1] - 1] += Sigma[i];
	// inv(H*P*Ht+R)
	TriangleMatInv(pHPHt, MsrCount);
	// K = P*Ht * inv(H*P*Ht+R)
	for (i = 0; i < STATE_DIM; i++)
	{
		for (j = 0; j < MsrCount; j++)
		{
			KMatrix[i*MsrCount + j] = 0.0;
			for (k = 0; k < MsrCount; k++)
			{
				if ((Mask & (0x01 << i)) > 0)
				{
					if (j <= k)
						KMatrix[i*MsrCount + j] += PHt[i*MsrCount + k] * pHPHt[line_start[k] + j];
					else
						KMatrix[i*MsrCount + j] += PHt[i*MsrCount + k] * pHPHt[line_start[j] + k];
				}
			}
		}
	}
	// state update: dx = K*Z;
	memset(GIKFdata.X, 0, sizeof(GIKFdata.X));
	for (i = 0; i < STATE_DIM; i++)
	{
		for (j = 0; j < MsrCount; j++)
		{
			GIKFdata.X[i] += KMatrix[i*MsrCount + j] * Msr[j];
		}
	}
	// P matrix update: P = (I-K*H)*P*(I-K*H)' + K*R*K'
	// temp = (I-K*H)*P = P-K*H*P = P-K*(PHt)'
	for (i = 0; i < STATE_DIM; i++)
	{
		for (j = 0; j < STATE_DIM; j++)
		{
			if (j <= i)
				temp[i*STATE_DIM + j] = GIKFdata.P[line_start[i] + j];
			else
				temp[i*STATE_DIM + j] = GIKFdata.P[line_start[j] + i];
			for (k = 0; k < MsrCount; k++)
				temp[i*STATE_DIM + j] -= KMatrix[i*MsrCount + k] * PHt[j*MsrCount + k];
		}
	}
	// recalculate PHt
	for (i = 0; i < STATE_DIM; i++)
	{
		for (j = 0; j < MsrCount; j++)
		{
			PHt[i*MsrCount + j] = 0.0;
			for (k = 0; k < STATE_DIM; k++)
				PHt[i*MsrCount + j] += temp[i*STATE_DIM + k] * HMatrix[j*STATE_DIM + k];
		}
	}
	// P = P*(I-K*H)' = P-P*H'*K' = P-PHt*K'
	for (i = 0; i < STATE_DIM; i++)
	{
		for (j = line_start[i]; j < line_start[i + 1]; j++)
		{
			GIKFdata.P[j] = temp[i*STATE_DIM + j - line_start[i]];
			for (k = 0; k < MsrCount; k++)
				GIKFdata.P[j] -= PHt[i*MsrCount + k] * KMatrix[(j - line_start[i])*MsrCount + k];
		}
	}
	// P = P + K*R*K'
	for (i = 0; i<STATE_DIM; i++)
	{
		for (j = line_start[i]; j < line_start[i + 1]; j++)
		{
			for (k = 0; k < MsrCount; k++)
				GIKFdata.P[j] += KMatrix[i*MsrCount + k] * KMatrix[(j - line_start[i])*MsrCount + k] * Sigma[k];
		}
	}
}


//--------------------------------------------
//BOOL SigmaJudge(PGNSS_DATA_T pGnssData)
//--------------------------------------------
BOOL SigmaJudge(PGNSS_DATA_T pGnssData)
{
	BOOL Judge;
	U8   i, Bad_Mum = 0;

	for (i = 0; i<6; i++)
	{
		if ((pGnssData->Sigma[i] * 10000)<1)
		{
			Bad_Mum++;
		}
	}

	if (Bad_Mum >= 2)
	{
		Judge = FALSE;
	}
	else
	{
		Judge = TRUE;
	}

	return Judge;
}

//-------------------------------------------------
//void ChangeGstDeta(PGNSS_DATA_T pGnssData)
//-------------------------------------------------
void ChangeGstDeta(PGNSS_DATA_T pGnssData)
{
	static U8   LowFlag = 0, HighFlag = 0, CarFlag = 0, RunFlag = 0;
	static U16  GstGoodNum = 0,HighGoNum=0;


	if (g_GINavInfo.CarKuFlag)
	{
		CarFlag = 1;
		RunFlag = 0;
		GstGoodNum = 0;
		HighGoNum =  0;
		g_GINavInfo.GstSScale = 1.0;
	}
	else
	{


		if (CarFlag == 1)
		{
		   if ((ABS(g_GINavInfo.delta_RIHeading) < 30) && (g_GINavInfo.GpsHighNum >= pGnssData->Frenqucy * 5) && (ABS(g_GINavInfo.delta_PIHeading) < 30) && (g_GINavInfo.Pos_Diff < 300))
		   {
			  RunFlag = 1;
		   }

		   //-------------------------------------------------
		   if (g_GINavInfo.StaticFlag == 0)
		   {
			   HighGoNum++;
		   }

		   if (HighGoNum > pGnssData->Frenqucy * 20)
		   {
			   HighGoNum = pGnssData->Frenqucy * 20;
			   RunFlag = 1;
		   }

		}
			
		


		if ((g_GINavInfo.Jugde == 1) && (CarFlag == 1) && (g_GINavInfo.GstStatus >= 1) && (g_GINavInfo.SatUseNum >= 8) && (RunFlag == 1))
		{

			if (g_GINavInfo.StaticFlag == 0)
			{

				GstGoodNum++;

				if (GstGoodNum > pGnssData->Frenqucy * 60)
				{
					CarFlag = 0;
					RunFlag = 0;
					GstGoodNum = pGnssData->Frenqucy * 60;
					g_GINavInfo.GstSScale = 1.0;			
				}
				else
				{
					g_GINavInfo.GstSScale = 60 - GstGoodNum / g_GINavInfo.Frenquecy;
					if (g_GINavInfo.GstSScale<1.0)
						g_GINavInfo.GstSScale = 1.0;
				}
			}
		}

	}

	if (g_GINavInfo.GstSScale<1.0)
		g_GINavInfo.GstSScale = 1.0;




}

//GNSS/INS组合导航更新
BOOL GIKFUpdateByGNSS(PGNSS_DATA_T pGnssData, PIMU_DATA_T pImuData)
{
	int MsrCount = 0;
	U32 UpdateMask;

	BOOL retGnssEvaluation;
	FLOAT64 deltaPosVel[6], Sigma[6], HMat[6 * STATE_DIM] = { 0.0 }, SigmaSum[2] = { 0 };

	if (!pGnssData)
		return TRUE;


	if (!pGnssData || pGnssData->NavType == 0 || (!IS_POS_VALID(pGnssData->NavFlag) && !IS_LEVEL_VEL_VALID(pGnssData->NavFlag)))  //最后一个判断的意义是：如果同时没有位置和速度方向，则返回。。
	{
		g_GINavInfo.Jugde = 0;
		return TRUE;
	}


	ChangeGstDeta(pGnssData);

	retGnssEvaluation = GnssEvaluation(pGnssData, pImuData);


	//-------------------------------------
	if (!SigmaJudge(pGnssData))
	{
		retGnssEvaluation = FALSE;
	}
	//-------------------------------------

	g_GINavInfo.GNSSHaltCount = 0;            //gty  清零?...

	if (!retGnssEvaluation)                    //Gps信号太差,直接退出...
	{
		g_GINavInfo.Jugde = 0;

		return TRUE;
	}
	else
	{
		g_GINavInfo.Jugde = 1;
	}



	//填充观测量、H和R阵
	if (IS_POS_VALID(pGnssData->NavFlag))  //gty  经纬高度有效
	{//dPosition
		deltaPosVel[MsrCount] = (g_GINavInfo.Position.Lat - pGnssData->Position.Lat) * (g_GINavInfo.Rm + g_GINavInfo.Position.Alt);
		HMat[MsrCount * STATE_DIM + 0] = 1.0;
		Sigma[MsrCount++] = pGnssData->Sigma[0];
		SigmaSum[0] += pGnssData->Sigma[0];

		deltaPosVel[MsrCount] = (g_GINavInfo.Position.Lon - pGnssData->Position.Lon) * (g_GINavInfo.Rn + g_GINavInfo.Position.Alt) * g_GINavInfo.CM_ne.C31;
		HMat[MsrCount * STATE_DIM + 1] = 1.0;
		Sigma[MsrCount++] = pGnssData->Sigma[1];
		SigmaSum[0] += pGnssData->Sigma[1];

		deltaPosVel[MsrCount] = -(g_GINavInfo.Position.Alt - pGnssData->Position.Alt);
		HMat[MsrCount * STATE_DIM + 2] = 1.0;
		Sigma[MsrCount++] = pGnssData->Sigma[2];


	}
	if (IS_LEVEL_VEL_VALID(pGnssData->NavFlag))//gty 方向速度有效
	{//dVn dVe
		deltaPosVel[MsrCount] = g_GINavInfo.Velocity.Vn - pGnssData->Velocity.Vn;
		HMat[MsrCount * STATE_DIM + 3] = 1.0;
		Sigma[MsrCount++] = pGnssData->Sigma[3];
		SigmaSum[1] += pGnssData->Sigma[3];


		deltaPosVel[MsrCount] = g_GINavInfo.Velocity.Ve - pGnssData->Velocity.Ve;
		HMat[MsrCount * STATE_DIM + 4] = 1.0;
		Sigma[MsrCount++] = pGnssData->Sigma[4];
		SigmaSum[1] += pGnssData->Sigma[4];

	}
	if (IS_UP_VEL_VALID(pGnssData->NavFlag))//gty  向上速度有效，目前这个版本没有这个观测量。。。
	{//dVd
		deltaPosVel[MsrCount] = -(g_GINavInfo.Velocity.Vu - pGnssData->Velocity.Vu);
		HMat[MsrCount * STATE_DIM + 5] = 1.0;
		Sigma[MsrCount++] = pGnssData->Sigma[5];
		SigmaSum[1] += pGnssData->Sigma[5];
	}

	if (MsrCount == 0)
		return TRUE;



	UpdateMask = 0x00000fff;//更新所有状态 默认

	GIKFBatchSolution(UpdateMask, MsrCount, deltaPosVel, HMat, Sigma);

	if (!GIKFCheckPMatrix())
		return FALSE;

	GIKFINSErrorFix(1);     //gty 反馈修正....


	if (!GIKFCheckGIResult())
		return FALSE;


	g_GINavInfo.KFCount++;



	if (SigmaSum[0] < 16.0 && ((SigmaSum[1] * 100) < 36))
		g_GINavInfo.INSAloneMsCount -= g_GINavInfo.INSAloneMsCount * 2 / 3;
	else if (SigmaSum[0] < 36.0 && SigmaSum[1] < 1.0)
		g_GINavInfo.INSAloneMsCount -= g_GINavInfo.INSAloneMsCount / 2;
	else if (SigmaSum[0] < 64.0 && SigmaSum[1] < 2.25)
		g_GINavInfo.INSAloneMsCount -= g_GINavInfo.INSAloneMsCount / 4;
	else if (SigmaSum[0] < 100.0 && SigmaSum[1] < 4.0)
		g_GINavInfo.INSAloneMsCount -= g_GINavInfo.INSAloneMsCount / 6;
	else if (SigmaSum[0] < 200.0 && SigmaSum[1] < 8.0)
		g_GINavInfo.INSAloneMsCount -= g_GINavInfo.INSAloneMsCount / 8;

	return TRUE;
}

//-----------------------------------------------------
//BOOL GIKFUpdateByRMC(PGNSS_DATA_T pGnssData,PIMU_DATA_T pImuData)
//-----------------------------------------------------
BOOL GIKFUpdateByRMC(PGNSS_DATA_T pGnssData, PIMU_DATA_T pImuData)
{
	static	FLOAT64 VLevel, delta_Heading, THeading, Yaw_Scale, HeadingTemp;
	static U8 RmcHeadGetFlag = 0, HighSpeedNum = 0;
	static U16 BigErrorNum = 0;
	U8 RunFlag = 0;

	if (!pGnssData)
		return TRUE;

	if (g_GINavInfo.Jugde == 0)
	{
		g_GINavInfo.Head_ENum = 0;
		BigErrorNum = 0;
		return TRUE;
	}

	//**********************************************************************
	VLevel = sqrt(SQR(pGnssData->Velocity.Ve) + SQR(pGnssData->Velocity.Vn));

	if (VLevel > 0.2)
	{
		HighSpeedNum++;
	}
	else
	{
		HighSpeedNum = 0;
	}


	if (HighSpeedNum > 10)
	{
		HighSpeedNum = 10;
		RunFlag = 1;
	}
	else
	{
		if (g_GINavInfo.StaticCount == 0)
		{
			RunFlag = 1;
		}
	}

	if (HighSpeedNum < 5)
	{
		RunFlag = 0;
	}
	//**********************************************************************

	if (RunFlag && VLevel >0.2 && IS_LEVEL_VEL_VALID(pGnssData->NavFlag))
	{
		HeadingTemp = atan2(pGnssData->Velocity.Ve, pGnssData->Velocity.Vn)*RAD2DEG;

		if (ABS(HeadingTemp) <= 360)   //
		{
			delta_Heading = (atan2(pGnssData->Velocity.Ve, pGnssData->Velocity.Vn) - g_GINavInfo.Euler.Phi)*RAD2DEG;

			if (delta_Heading > 180.0)
				delta_Heading -= 360.0;
			if (delta_Heading < -180.0)
				delta_Heading += 360.0;



			//---------------------------------------------------
			//  如果长时间大角度误差...则有问题...
			//---------------------------------------------------
			if (BigErrorNum >= g_GINavInfo.Frenquecy * 30)
			{
				BigErrorNum = g_GINavInfo.Frenquecy * 30;
			}

			if (BigErrorNum < g_GINavInfo.Frenquecy * 30)
			{
				//------------------------------------------------------
				//     在快速倒车或者大误差不进行.....失效判断+RMC角度
				//-------------------------------------------------------
				if (ABS(delta_Heading) > 120)
				{
					g_GINavInfo.Head_ENum = 0;
					BigErrorNum++;
					return TRUE;
				}
				else
				{
					BigErrorNum = 0;
				}



				//-----------------------------------------------------
				//       在相对空旷的地方才进行.......失效判断+RMC角度
				//-----------------------------------------------------
				if ((g_GINavInfo.GstDeta>g_GINavInfo.GstDetaMin * 3) && (g_GINavInfo.GST_Diff>g_GINavInfo.GstDetaMin / 2))
				{
					g_GINavInfo.Head_ENum = 0;
					return TRUE;
				}
			}
			//-----------------------------------------------------
			//     在直线行驶过程中才可以进行处理.....失效判断+RMC角度
			//-----------------------------------------------------
			if ((ABS(g_GINavInfo.Gyr_Rate)>2) || (g_GINavInfo.LineFlag == 0))
			{
				g_GINavInfo.Head_ENum = 0;
			}

			//-------------------------------------------------------------
			//
			//-------------------------------------------------------------
			if (RmcHeadGetFlag == 0)
			{
				g_GINavInfo.RmcHead = 30 - (FLOAT32)g_GINavInfo.KFCount*1.0f / pGnssData->Frenqucy / 30;

				if (g_GINavInfo.RmcHead<15.0f)
				{
					g_GINavInfo.RmcHead = 15.0f;
					RmcHeadGetFlag = 1;
				}
			}
			else
			{
				g_GINavInfo.RmcHead = 15.0f;
			}

			//-----------------------------------------------------
			//       在相对空旷的地方才进行.......失效判断
			//-----------------------------------------------------
			if (ABS(delta_Heading)>g_GINavInfo.RmcHead)
			{
				g_GINavInfo.Head_ENum++;


				if (g_GINavInfo.Head_ENum>g_GINavInfo.Frenquecy * 10)
				{
					g_GINavInfo.Head_ENum = g_GINavInfo.Frenquecy * 10;
					g_GINavInfo.Head_EENum++;

					if ((g_GINavInfo.Gyr_RateFlag == 0) && (g_GINavInfo.Head_EENum >= 10))
					{
						g_GINavInfo.Head_Flag = 0;
					}

					g_GINavInfo.INSState = INS_ACTIVE;	 //复位		
					g_GINavInfo.SYSFlag = 0;            //标志清零 
					return FALSE;
				}

				return TRUE;
			}
			else
			{
				g_GINavInfo.Head_ENum = 0;
			}



			//---------------------------------------
			// if(g_GINavInfo.delta_Num<5*5)   //5s
			//	  return TRUE;
			//---------------------------------------


			//-----------------------------------------------------
			//       RMC角度处理
			//-----------------------------------------------------

			if (g_GINavInfo.GstStatus == 4)
			{
				Yaw_Scale = 0.02;
			}
			else if (g_GINavInfo.GstStatus == 5)
			{
				Yaw_Scale = 0.01f / g_GINavInfo.GstBScale;
			}
			else if (g_GINavInfo.GstStatus == 2)
			{
				Yaw_Scale = 0.005f / g_GINavInfo.GstBScale;
			}
			else
			{
				Yaw_Scale = 0.001f / g_GINavInfo.GstBScale;
			}

			THeading = g_GINavInfo.Euler.Phi*RAD2DEG + Yaw_Scale*delta_Heading;

			if (THeading > 360.0)
				THeading -= 360.0;
			if (THeading < 0)
				THeading += 360.0;

			if (ABS(THeading) <= 360)   //保护...
			{
				g_GINavInfo.Euler.Phi = THeading*DEG2RAD;
				g_GINavInfo.Euler.Theta = g_GINavInfo.Euler.Theta;
				g_GINavInfo.Euler.Gamma = g_GINavInfo.Euler.Gamma;

				Euler2Quat(&g_GINavInfo.Euler, &g_GINavInfo.Quat_bn);
				Quat2CM(&g_GINavInfo.Quat_bn, &g_GINavInfo.CM_bn);

			}

		}
	}
	else
	{
		g_GINavInfo.Head_ENum = 0;
		BigErrorNum = 0;
	}


	return TRUE;

}



//无侧滑约束，车辆侧向速度为0
BOOL GIKFUpdateByNHC(PIMU_DATA_T pImuData)
{
	U32 UpdateMask;
	FLOAT64 VSide[2], HMat[2 * STATE_DIM] = { 0.0 }, Sigma[2];
	PCOSM_T pCbn = &g_GINavInfo.CM_bn;
	PVEL_T pV = &g_GINavInfo.Velocity;
	FLOAT32 SpeedTemp, UtcTempInt, UtcTempFloat;

	SpeedTemp = sqrt(SQR(pV->Ve) + SQR(pV->Vn));

	UtcTempInt = g_GINavInfo.UtcTime.MillSecond / 100;

	UtcTempFloat = g_GINavInfo.UtcTime.MillSecond - UtcTempInt * 100;



	if (g_GINavInfo.GstStatus >= 1)
	{

	if (g_GINavInfo.UtcTime.MillSecond >= 40)    //每秒一次，动态..如果静态...则最后判据为零，从而Static和NHC相互排斥....
			return TRUE;


		if ((SpeedTemp * 100)< 15)
			return TRUE;
	}
	else
	{
		if (g_GINavInfo.M_City_Flag)
		{
			if (g_GINavInfo.UtcTime.MillSecond >= 40)    //每秒一次，动态..如果静态...则最后判据为零，从而Static和NHC相互排斥....
				return TRUE;
		}

		if (UtcTempFloat >= 40)
			return TRUE;

		if ((UtcTempInt == 1) || (UtcTempInt == 3) || (UtcTempInt == 5) || (UtcTempInt == 7) || (UtcTempInt == 9))
			return TRUE;

		if ((SpeedTemp * 100)< 15)
			return TRUE;
	}


	//INS解算的侧向速度
	VSide[0] = pCbn->C22*pV->Ve + pCbn->C12*pV->Vn - pCbn->C32*pV->Vu;
	//VSide[1] = pCbn->C23*pV->Ve + pCbn->C13*pV->Vn - pCbn->C33*pV->Vu;

	HMat[0 * STATE_DIM + 3] = pCbn->C12;
	HMat[0 * STATE_DIM + 4] = pCbn->C22;
	HMat[0 * STATE_DIM + 5] = pCbn->C32;
	//HMat[1 * STATE_DIM + 3] = pCbn->C13;
	//HMat[1 * STATE_DIM + 4] = pCbn->C23;
	//HMat[1 * STATE_DIM + 5] = pCbn->C33;
	HMat[0 * STATE_DIM + 6] = pCbn->C32*pV->Ve + pCbn->C22*pV->Vu;
	HMat[0 * STATE_DIM + 7] = -pCbn->C32*pV->Vn - pCbn->C12*pV->Vu;
	HMat[0 * STATE_DIM + 8] = -pCbn->C12*pV->Ve + pCbn->C22*pV->Vn;
	//HMat[1 * STATE_DIM + 6] = pCbn->C33*pV->Ve + pCbn->C23*pV->Vu;
	//HMat[1 * STATE_DIM + 7] = -pCbn->C33*pV->Vn - pCbn->C13*pV->Vu;
	//HMat[1 * STATE_DIM + 8] = -pCbn->C13*pV->Ve + pCbn->C23*pV->Vn;
	Sigma[0] = 0.0075*0.0075*(g_GINavInfo.INSAloneMsCount / (INSUPDATE_DATA_INTERVAL)+1);//即随着INS独立工作时间增加，噪声增大..
	Sigma[0] = Sigma[0] > 0.05*0.05 ? 0.05*0.05 : Sigma[0];  //取小的数值...即设定sigma[0]的最大值为0.0025.

	// if(ABS(g_GINavInfo.Gyr_Rate)<4)      //判据1
	//if (ABS(g_GINavInfo.LineFlag == 1))   //判据2
	//if((ABS(g_GINavInfo.LineFlag==1))&&(ABS(g_GINavInfo.Gyr_Diff)<4))  //判据3
	// if (ABS(g_GINavInfo.Gyr_Diff)<10)     //判据4

	if (g_GINavInfo.GstStatus >= 1)
	{
		if ((ABS(g_GINavInfo.LineFlag == 1)) && (ABS(g_GINavInfo.Gyr_Diff)<4))  //判据3
		{
			UpdateMask = 0x00000fff;//1111 1111 1111b, 更新速度、姿态、加计Bias、陀螺仪偏
		}
		else
		{
			UpdateMask = 0x00000ff8;//111 111 110 110b, //1111 1111 0110,更新速度、姿态、加计Bias、陀螺仪偏
		}
	}
	else
	{
		if (g_GINavInfo.M_City_Flag)
		{
			if ((ABS(g_GINavInfo.LineFlag == 1)) && (ABS(g_GINavInfo.Gyr_Diff)<4))  //判据3
			{
				UpdateMask = 0x00000fff;//1111 1111 1111b, 更新速度、姿态、加计Bias、陀螺仪偏
			}
			else
			{
				UpdateMask = 0x00000ff8;//111 111 110 110b, //1111 1111 0110,更新速度、姿态、加计Bias、陀螺仪偏
			}
		}
		else
		{
			UpdateMask = 0x00000fff;//1111 1111 1111b, 更新速度、姿态、加计Bias、陀螺仪偏
		}
		
	}




	GIKFBatchSolution(UpdateMask, 1, VSide, HMat, Sigma);

	if (!GIKFCheckPMatrix())
		return FALSE;

	GIKFINSErrorFix(2);

	if (!GIKFCheckGIResult())
		return FALSE;







	//if ((U32)ABS(VSide[0]) + (U32)ABS(VSide[1]) > 0)
	//g_GINavInfo.INSAloneMsCount += ((U32)ABS(VSide[0]) + (U32)ABS(VSide[1])) * 40000;

	return TRUE;
}

//静态更新
BOOL GIKFUpdateByStatic(PIMU_DATA_T pImuData)
{
	U32 i, UpdateMask;
	FLOAT64 Msrs[6] = { 0 }, HMat[6 * STATE_DIM] = { 0.0 }, Sigma[6], dt = pImuData->MsrInterval / 1000.0;
	PVEL_T pV = &g_GINavInfo.Velocity;


	if (g_GINavInfo.GstStatus >= 1)    //卫星精度可以的情况下
	{
		if (g_GINavInfo.UtcTime.MillSecond >= 50)
		return TRUE;
		
		if ((g_GINavInfo.StaticCount <= 1 + (U32)ABS(pV->Ve) + (U32)ABS(pV->Vn)))
			return TRUE;

		if (g_GINavInfo.GpsSpeed>0.3)   //速度阈值...
			return TRUE;
	}
	else
	{
		if (g_GINavInfo.StaticCount <= 1 + (U32)ABS(pV->Ve) + (U32)ABS(pV->Vn))
			return TRUE;
	}

	Msrs[0] = g_GINavInfo.dVelocity.Vn;
	Msrs[1] = g_GINavInfo.dVelocity.Ve;
	Msrs[2] = -g_GINavInfo.dVelocity.Vu;
	for (i = 0; i < INS_UPDATE_SAMPLE_NUM; i++)
	{
		Msrs[3] += pImuData->Gyro[i][0] / dt;
		Msrs[4] += pImuData->Gyro[i][1] / dt;
		Msrs[5] += pImuData->Gyro[i][2] / dt;
	}
	Msrs[3] /= INS_UPDATE_SAMPLE_NUM;
	Msrs[4] /= INS_UPDATE_SAMPLE_NUM;
	Msrs[5] /= INS_UPDATE_SAMPLE_NUM;

	HMat[3 + STATE_DIM * 0] = HMat[4 + STATE_DIM * 1] = HMat[5 + STATE_DIM * 2] = 1.0;
	HMat[12 + STATE_DIM * 3] = HMat[13 + STATE_DIM * 4] = HMat[14 + STATE_DIM * 5] = 1.0;
	Sigma[0] = Sigma[1] = Sigma[2] = 0.01;
	Sigma[3] = Sigma[4] = Sigma[5] = ANG_RAND_WALK*ANG_RAND_WALK*dt;




	//if((Msrs[0]>0.5)||(Msrs[1]>0.5))
	//{
	//return TRUE;
	//}

	UpdateMask = 0x00007ff8;// 0111 1111 1111 1000b，更新位置、速度、姿态、加计 漂移(No) 默认


	GIKFBatchSolution(UpdateMask, 6, Msrs, HMat, Sigma);

	if (!GIKFCheckPMatrix())
		return FALSE;

	GIKFINSErrorFix(3);

	if (!GIKFCheckGIResult())
		return FALSE;


	MEMSET(&g_GINavInfo.Velocity, 0, SIZEOF(VEL_T)); //gty 既然是静态，则速度清零

	return TRUE;
}


BOOL GIKFUpdateByOli(PGNSS_DATA_T pBGnssData, PIMU_DATA_T pImuData)
{

	return FALSE;

}



//Kalman反馈校正
void GIKFINSErrorFix(U8 RunKind)
{
	FLOAT64 dLat, dLon, DetaSpeed;
	FLOAT64 dTheta[3], temp[3], Phi[3];
	QUAT_T dQuat, tempQ;
	U8  RunFlag, i;

	DetaSpeed = sqrt(GIKFdata.X[3] * GIKFdata.X[3] + GIKFdata.X[4] * GIKFdata.X[4]);

	if ((g_GINavInfo.GstStatus == 4) || (RunKind == 2) || (RunKind == 3))
	{
		RunFlag = 1;
	}
	else
	{
		if (DetaSpeed<5.0)  //20160408
		{
			RunFlag = 1;
		}
		else
		{
			RunFlag = 0;
		}
	}


	if (RunFlag)
	{

		//position feedback
		dLat = GIKFdata.X[0] / (g_GINavInfo.Rm + g_GINavInfo.Position.Alt);
		dLon = GIKFdata.X[1] / ((g_GINavInfo.Rn + g_GINavInfo.Position.Alt)*g_GINavInfo.CM_ne.C31);
		dTheta[0] = -dLon * g_GINavInfo.CM_ne.C31;
		dTheta[1] = dLat;
		dTheta[2] = -dLon * g_GINavInfo.CM_ne.C33;
		RotVec2Quat(dTheta, &dQuat);
		MEMCPY(&tempQ, &g_GINavInfo.Quat_ne, SIZEOF(QUAT_T));
		QuatMulti(&tempQ, &dQuat, &g_GINavInfo.Quat_ne);
		NormQuat(&g_GINavInfo.Quat_ne);
		Quat2CM(&g_GINavInfo.Quat_ne, &g_GINavInfo.CM_ne);
		/*g_GINavInfo.Position.Lat = -asin(g_GINavInfo.CM_ne.C33);
		g_GINavInfo.Position.Lon = atan2(-g_GINavInfo.CM_ne.C23, -g_GINavInfo.CM_ne.C13);*/
		g_GINavInfo.Position.Lat -= dLat;
		g_GINavInfo.Position.Lon -= dLon;
		g_GINavInfo.Position.Alt += GIKFdata.X[2];


		//velocity feedback
		temp[0] = g_GINavInfo.Velocity.Vn - GIKFdata.X[3];
		temp[1] = g_GINavInfo.Velocity.Ve - GIKFdata.X[4];
		temp[2] = -g_GINavInfo.Velocity.Vu - GIKFdata.X[5];
		g_GINavInfo.Velocity.Vn = temp[0] + dTheta[2] * temp[1] - dTheta[1] * temp[2];
		g_GINavInfo.Velocity.Ve = temp[1] + dTheta[0] * temp[2] - dTheta[2] * temp[0];
		g_GINavInfo.Velocity.Vu = -(temp[2] + dTheta[1] * temp[0] - dTheta[0] * temp[1]);

		//attitude correction

		Phi[0] = GIKFdata.X[6] + dTheta[0];
		Phi[1] = GIKFdata.X[7] + dTheta[1];
		Phi[2] = GIKFdata.X[8] + dTheta[2];
		RotVec2Quat(Phi, &dQuat);
		MEMCPY(&tempQ, &g_GINavInfo.Quat_bn, SIZEOF(QUAT_T));
		QuatMulti(&dQuat, &tempQ, &g_GINavInfo.Quat_bn);
		NormQuat(&g_GINavInfo.Quat_bn);
		Quat2CM(&g_GINavInfo.Quat_bn, &g_GINavInfo.CM_bn);
		CM2Euler(&g_GINavInfo.CM_bn, &g_GINavInfo.Euler);

		//IMU bias feedback

		g_GINavInfo.ImuCfg.AccBias[0] += GIKFdata.X[9];
		g_GINavInfo.ImuCfg.AccBias[1] += GIKFdata.X[10];
		g_GINavInfo.ImuCfg.AccBias[2] += GIKFdata.X[11];


		g_GINavInfo.ImuCfg.GyroBias[0] += GIKFdata.X[12];
		g_GINavInfo.ImuCfg.GyroBias[1] += GIKFdata.X[13];
		g_GINavInfo.ImuCfg.GyroBias[2] += GIKFdata.X[14];

	}
}

//P阵奇异检测
BOOL GIKFCheckPMatrix(void)
{
	int i;

	for (i = 0; i < STATE_DIM; i++)
	{
		if (GIKFdata.P[line_start[i + 1] - 1] < 0)
			return FALSE;
	}

	for (i = 0; i<5; i++)
	{
		if (pBlock[line_start[i + 1] - 1][0][0] * pBlock[line_start[i + 1] - 1][1][1] * pBlock[line_start[i + 1] - 1][2][2] +
			pBlock[line_start[i + 1] - 1][1][0] * pBlock[line_start[i + 1] - 1][2][1] * pBlock[line_start[i + 1] - 1][2][0] +
			pBlock[line_start[i + 1] - 1][2][0] * pBlock[line_start[i + 1] - 1][1][0] * pBlock[line_start[i + 1] - 1][2][1] -
			pBlock[line_start[i + 1] - 1][2][0] * pBlock[line_start[i + 1] - 1][1][1] * pBlock[line_start[i + 1] - 1][2][0] -
			pBlock[line_start[i + 1] - 1][2][1] * pBlock[line_start[i + 1] - 1][2][1] * pBlock[line_start[i + 1] - 1][0][0] -
			pBlock[line_start[i + 1] - 1][2][2] * pBlock[line_start[i + 1] - 1][1][0] * pBlock[line_start[i + 1] - 1][1][0] < 0.0)
			return FALSE;
	}
	return TRUE;
}

//导航解正确性检测
BOOL GIKFCheckGIResult()
{
	FLOAT32 VLevel   = sqrt(SQR(g_GINavInfo.Velocity.Ve) + SQR(g_GINavInfo.Velocity.Vn));

	if (g_GINavInfo.Position.Alt > 10000.0 || g_GINavInfo.Position.Alt < -1000.0)
		return FALSE;

	if (ABS(g_GINavInfo.Velocity.Ve) > 1000.0 || ABS(g_GINavInfo.Velocity.Vn) > 1000.0 || ABS(g_GINavInfo.Velocity.Vu) > 1000.0)
		return FALSE;

	if (ABS(g_GINavInfo.dVelocity.Ve) > 100.0 || ABS(g_GINavInfo.dVelocity.Vn) > 100.0 || ABS(g_GINavInfo.dVelocity.Vu) > 100.0)
		return FALSE;

	if (g_GINavInfo.GstStatus == 0)
	{
		if (VLevel >= g_GINavInfo.MaxSpeed*1.0f)
			return FALSE;
	}

	if ((User_Kind == 2) || (User_Kind == 4))
	{
		if (ABS(g_GINavInfo.Euler.Theta) > 60 * DEG2RAD)
			return FALSE;

		if (ABS(g_GINavInfo.Euler.Gamma) > 60 * DEG2RAD)
			return FALSE;
	}
	else
	{
		if (ABS(g_GINavInfo.Euler.Theta) > 30 * DEG2RAD)
			return FALSE;

		if (ABS(g_GINavInfo.Euler.Gamma) > 25 * DEG2RAD)
			return FALSE;

	}

	return TRUE;
}

