#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "KalmanFilter.h"

int main()
{		
	
	int iVal;
	int iRow;
	int iCol;
	float measurementsVel[2][NUM_VALS];					// Artificial Measurement values. Index 1: x direction, Index 2: y direction
	int dt;												// Timestep
	srand(time(NULL));									// Get "real" random numbers http://users.wpi.edu/~bpwiselybabu/2012/02/07/generating-white-gaussian-noise/
	
	float mean = 10;
	float stddev = mean/2;
	for (iVal = 0; iVal < NUM_VALS; iVal++)
	{
		// measurementsVel[0][iVal] = (((float)rand()) / ((float)RAND_MAX) * 6) + 7;		// NOT GAUSSIAN!! Completely random numbers 7 - 13
		measurementsVel[0][iVal] = box_muller(mean, stddev);	// Gauss distributed random numbers
		measurementsVel[1][iVal] = 0;
	}
	
	dt = 1;
	
	float x[4] = {0, 0, 10, 0};							// state vector: x, y, x', y'
	
	float P[4][4];										// Initial uncertainty
	P[0][0] = 10;	P[0][1] = 0;	P[0][2] = 0;	P[0][3] = 0;	
	P[1][0] = 0;	P[1][1] = 10;	P[1][2] = 0;	P[1][3] = 0;
	P[2][0] = 0;	P[2][1] = 0;	P[2][2] = 10;	P[2][3] = 0;	
	P[3][0] = 0;	P[3][1] = 0;	P[3][2] = 0;	P[3][3] = 10;
	
	float A[4][4];										// Transition Matrix
	A[0][0] = 1;	A[0][1] = 0;	A[0][2] = dt;	A[0][3] = 0;	
	A[1][0] = 0;	A[1][1] = 1;	A[1][2] = 0;	A[1][3] = dt;
	A[2][0] = 0;	A[2][1] = 0;	A[2][2] = 1;	A[2][3] = 0;	
	A[3][0] = 0;	A[3][1] = 0;	A[3][2] = 0;	A[3][3] = 1;
	
	float H[2][4];										// Measurement Fct (How sensor values are converted)
	H[0][0] = 0;	H[0][1] = 0;	H[0][2] = 1;	H[0][3] = 0;	
	H[1][0] = 0;	H[1][1] = 0;	H[1][2] = 0;	H[1][3] = 1;
	
	float R[2][2];										// Measurement noise covariance
	R[0][0] = 10;	R[0][1] = 0;		
	R[1][0] = 0;	R[1][1] = 10;
	
	float Q[4][4];										// Initial uncertainty
	Q[0][0] = 0.25 * powf(dt, 4);	Q[0][1] = 0.25 * powf(dt, 4);	Q[0][2] = 0.5 * powf(dt, 3);	Q[0][3] = 0.5 * powf(dt, 3);	
	Q[1][0] = 0.25 * powf(dt, 4);	Q[1][1] = 0.25 * powf(dt, 4);	Q[1][2] = 0.5 * powf(dt, 3);	Q[1][3] = 0.5 * powf(dt, 3);
	Q[2][0] = 0.5 * powf(dt, 3);	Q[2][1] = 0.5 * powf(dt, 3);	Q[2][2] = powf(dt, 2);	Q[2][3] = powf(dt, 2);
	Q[3][0] = 0.5 * powf(dt, 3);	Q[3][1] = 0.5 * powf(dt, 3);	Q[3][2] = powf(dt, 2);	Q[3][3] = powf(dt, 2);
	
	float I[4][4];										// Identity Matrix
	I[0][0] = 1;	I[0][1] = 0;	I[0][2] = 0;	I[0][3] = 0;	
	I[1][0] = 0;	I[1][1] = 1;	I[1][2] = 0;	I[1][3] = 0;
	I[2][0] = 0;	I[2][1] = 0;	I[2][2] = 1;	I[2][3] = 0;
	I[3][0] = 0;	I[3][1] = 0;	I[3][2] = 0;	I[3][3] = 1;
	
	FILE *f = fopen("out.txt", "w");
	
	FILE *gnuplot = popen("gnuplot", "w");				// https://stackoverflow.com/questions/14311640/how-to-plot-data-by-c-program
	fprintf(gnuplot, "plot '-'\n");
	
	for (iVal = 0; iVal < NUM_VALS; iVal++)
	// DEBUG!!!!!!!!
	// iVal=0;
	{
		// Prediction
		// x = A*x
		multiplyMatrixWithVectorNMMM(4, 4, A, x, x);
		
		// P = A*P*A'+Q
		float AP[4][4] = {0};
		multiplyMatrixNN(4, A, P, AP);
		float At[4][4] = {0};
		transposeMatrix(4,4,A,At);
		float APAt[4][4] = {0};
		multiplyMatrixNN(4, AP, At, APAt);
		addMatrix(4,4,APAt,Q,P);
		
		// y = Z-(H*x)
		float Z[2];
		Z[0] = measurementsVel[0][iVal];
		// DEBUG!!!!!!!!
		// Z[0] = 10.183227263001436;
		Z[1] = measurementsVel[1][iVal];
		float Hx[2] = {0};
		multiplyMatrixWithVectorNMMM(2, 4, H, x, Hx);
		float y[2] = {0};
		subtractVector(2, Z, Hx, y);
		
		// S=(H*P*H'+R)
		float HP[2][4] = {0};
		multiplyMatrixNN(4, H, P, HP);
		float Ht[4][2] = {0};
		transposeMatrix(2, 4, H, Ht);
		float HPHt[2][2] = {0};
		multiplyMatrixNM(2, 4, HP, Ht, HPHt);
		float S[2][2] = {0};
		addMatrix(2,2,HPHt,R,S);
		
		// K=P*H'*inv(S)
		float PHt[4][2] = {0};
		multiplyMatrixNO(4, 2, P, Ht, PHt);
		float Si[2][2] = {0};
		float deter = (float) det(S,2);
		inverse(S,Si,2,deter);
		float K[4][2] = {0};
		multiplyMatrixON(4, 2, PHt, Si, K);
		
		// x=x+(K*y)
		float Ky[4] = {0};
		multiplyMatrixWithVectorNMMN(4, 2, K, y, Ky);
		addVector(4, x, Ky, x);
		
		// P=(I-(K*H))*P
		float KH[4][4] = {0};
		multiplyMatrixNM(4, 2, K, H, KH);
		float I_KH[4][4] = {0};
		subtractMatrix(4, 4, I, KH, I_KH);
		float P_interm[4][4] = {0};
		multiplyMatrixNN(4, I_KH, P, P_interm);
		memcpy(P, P_interm, sizeof(P));
		
		SHOWFLOATARRAY(x)
		WRITEFLOATARRAYTOFILETAB(f,x)
		WRITEENTERTOFILE(f)
		
		
		fprintf(gnuplot, "%d %g\n", iVal, x[2]);
		
	}
	
	fprintf(gnuplot, "e\n");
	fflush(gnuplot);
		
	fclose(f);
	
	system("pause");	
	
	return 0;
}