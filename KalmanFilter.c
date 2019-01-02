#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include "KalmanFilter.h"

/************************************************************************************************/
/************************************* Function definitions *************************************/
/************************************************************************************************/

/************************************* Main *****************************************************/
int main(void)
{		
	const int numVals = 100;
	float measurementsVel[2][numVals];					/* Artificial Measurement values. Index 1: position in x direction, Index 2: y direction */
	int dt = 1;											/* Timestep in sec */
	
	float meanX = 10;
	float stddevX = 2;
	float meanY = 0;
	float stddevY = 0.05;
	
	srand(time(NULL));									/* Get "real" random numbers */
	
	fillMeasurementMatrix(meanX, stddevX, meanY, stddevY, MATRIX_LEN1(measurementsVel), MATRIX_LEN2(measurementsVel), measurementsVel);
	
	/* Initial matrix states */
	float x[4];										/* state vector: x, y, x', y' */
	float P[4][4];										/* Matrix for covariances betweens variables */
	float A[4][4];										/* Transition Matrix */
	float H[2][4];										/* Measurement Fct (How sensor values are converted) */
	float R[2][2];										/* Measurement noise covariance (low stddev of meas. values: This can be low -> trust measured values MORE). If too high, the filter starts interpreting */	
	float Q[4][4];										/* Process covariance matrix */
	float I[4][4];										/* Identity Matrix */
	
	initMatrices(dt, x, P, A, H, R, Q, I);
		
	/* Intermediate variables, needed for matrix operations */
	float AP[4][4] = {0};
	float At[4][4] = {0};
	float APAt[4][4] = {0};
	float Z[2] = {0};
	float Hx[2] = {0};
	float y[2] = {0};
	float HP[2][4] = {0};
	float Ht[4][2] = {0};
	float HPHt[2][2] = {0};
	float S[2][2] = {0};
	float PHt[4][2] = {0};
	float Si[2][2] = {0};
	float deter = 0;
	float K[4][2] = {0};
	float Ky[4] = {0};
	float KH[4][4] = {0};
	float I_KH[4][4] = {0};
	float P_interm[4][4] = {0};
	
	FILE *f = fopen("out.txt", "w");					/* Define file handle */
	
	FILE *gnuplot = popen("gnuplot -persistent", "w");  /* Define gnuplot handle: https://stackoverflow.com/questions/14311640/how-to-plot-data-by-c-program */
	
	fprintf(gnuplot, "%s \n", "set title \"var\"");	/* Adjust according to the data you're plotting (see end of for loop) */
        fprintf(gnuplot, "plot '-'\n");
	
	for (int iVal = 0; iVal < numVals; iVal++)				/* May be overwritten for debug purposes */
	{
		/* Prediction */
		/* x=A*x */
		multiplyMatrixWithVector_NM_M_N(4, 4, A, x, x);
		
		/* P=A*P*A'+Q */
		multiplyMatrix_NN_NN_NN(4, A, P, AP);
		transposeMatrix_NM_MN(4, 4, A, At);
		multiplyMatrix_NN_NN_NN(4, AP, At, APAt);
		addMatrices(4, 4, APAt, Q, P);
		
		/* Correction */
		/* y=Z-(H*x) */
		Z[0] = measurementsVel[0][iVal];				/* May be overwritten for debug purposes */
		Z[1] = measurementsVel[1][iVal];
		multiplyMatrixWithVector_NM_M_N(2, 4, H, x, Hx);
		subtractVectorFromVector(2, Z, Hx, y);
		
		/* S=(H*P*H'+R) */
		multiplyMatrix_MN_NN_MN(2, 4, H, P, HP);
		transposeMatrix_NM_MN(2, 4, H, Ht);
		multiplyMatrix_NM_MN_NN(2, 4, HP, Ht, HPHt);
		addMatrices(2, 2, HPHt, R, S);
		
		/* K=P*H'*inv(S) */
		multiplyMatrix_NN_NM_NM(4, 2, P, Ht, PHt);
		deter = (float) det(S, 2);
		inverse(S, Si, 2, deter);
		multiplyMatrix_MN_NN_MN(4, 2, PHt, Si, K);
		
		/* x=x+(K*y) */
		multiplyMatrixWithVector_NM_M_N(4, 2, K, y, Ky);
		addVectors(4, x, Ky, x);
		
		/* P=(I-(K*H))*P */
		multiplyMatrix_NM_MN_NN(4, 2, K, H, KH);
		subtractMatrixFromMatrix(4, 4, I, KH, I_KH);
		multiplyMatrix_NN_NN_NN(4, I_KH, P, P_interm);
		memcpy(P, P_interm, sizeof(P));
		
		SHOWFLOATARRAY(x)
		WRITEFLOATARRAYTOFILETAB(f,x)
		WRITEENTERTOFILE(f)
		
		fprintf(gnuplot, "%d %g\n", iVal, x[2]);		/* vEstim */
		// fprintf(gnuplot, "%d %g\n", iVal, Z[0]);		/* vMeas */
		// fprintf(gnuplot, "%d %g\n", iVal, P[2][2]);		/* var */
		
	}
	
	/* Properly finish gnuplot */
	fprintf(gnuplot, "e\n");
	fflush(gnuplot);
		
	/* Properly finish file handlung */
	fclose(f);
	
	return 0;
}

/************************************* Fill Measurement Matrix **********************************/
void fillMeasurementMatrix(float meanX, float stddevX, float meanY, float stddevY, int numVars, int numVals, float measurementsVel[numVars][numVals])
{	
	int iVal;
	for (iVal = 0; iVal < numVals; iVal++)
	{
		measurementsVel[0][iVal] = box_muller(meanX, stddevX);								/* Gaussian distributed random number for x measurements */
		measurementsVel[1][iVal] = box_muller(meanY, stddevY);								/* ... y measurements */
	}	
}

/************************************* Init matrices ********************************************/
void initMatrices(float dt, float x[4], float P[4][4], float A[4][4], float H[2][4], float R[2][2], float Q[4][4], float I[4][4])
{					
	x
