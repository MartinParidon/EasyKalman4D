#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "KalmanFilterTypes.h"
#include "KalmanFilter.h"

int main()
{	
	int iVal;
	int iRow;
	int iCol;
	float measurementsVel[2][NUM_VALS];					// Artificial Measurement values. Index 1: x direction, Index 2: y direction
	int dt;												// Timestep
	
	for (iVal = 0; iVal < NUM_VALS; iVal++)
	{
		measurementsVel[0][iVal] = (((float)rand()) / ((float)RAND_MAX) * 6) + 7;
		measurementsVel[1][iVal] = 0;
		// fprintf(stderr, "%f\t\t", measurementsVel[0][iVal]);
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
	
	
	// for (iVal = 0; iVal < NUM_VALS; iVal++)
	{
		// Prediction
		// x = A*x
				
		multiplyMatrixWithVector(4, 4, A, x);
		
		// P = A*P*A'+Q
		
		float AP[4][4] = {0};
		multiplyMatrix(4, A, P, AP);
		
		float At[4][4] = {0};
		transposeMatrix(4,4,A,At);
		
		float APAt[4][4] = {0};
		multiplyMatrix(4, AP, At, APAt);
		
		addMatrix(4,4,APAt,Q,P);
		
		SHOWARRAY(P);
		
		// TODO Correction
		iVal=0;
		float Z[2];
		Z[0] = measurementsVel[0][iVal];
		Z[1] = measurementsVel[1][iVal];
		printf("\n");		
		
		// TODO
		
		// y = Z - (H * x);              % Innovation aus Messwertdifferenz
		// S=(H*P*H'+R);           % Innovationskovarianz
		// K=P*H'*inv(S);          % Filter-Matrix (Kalman-Gain)
	  
		// x=x+(K*y);              % aktualisieren des Systemzustands
		// P=(I-(K*H))*P;          % aktualisieren der Kovarianz
		
	}
	
	return 0;
}
void showFloatArray(int max1, int max2, float array[max1][max2])
{
	int i;
	int j;
	for (i = 0; i < max1; i++)
	{
		for (j = 0; j < max2; j++)
			printf("%f\t", array[i][j]);
		printf("\n");
	}
}

void multiplyMatrix(int N, float mat1[][N], float mat2[][N], float res[][N]) 
{ 
    int i, j, k; 
    for (i = 0; i < N; i++) 
    { 
        for (j = 0; j < N; j++) 
        { 
            res[i][j] = 0; 
            for (k = 0; k < N; k++) 
                res[i][j] += mat1[i][k]*mat2[k][j]; 
        } 
    } 
} 

void multiplyMatrixWithVector(int max1, int max2, float matrix[max1][max2],  float vector[max2]) 
{ 
	int iRow, iCol;
	for (iRow = 0; iRow < max1; iRow++)
	{
		float sumProdMV = 0;
		for (iCol = 0; iCol < max2; iCol++)
		{
			sumProdMV += matrix[iRow][iCol] * vector[iCol];
		}
		vector[iRow] = sumProdMV;
	} 
} 

void transposeMatrix(int max1, int max2, float matrix1[max1][max2],  float matrix2[max1][max2])
{
	int i, j;
	for (i = 0; i < max1; i++)
	{
		for (j = 0; j < max2; j++)
			matrix2[i][j] = matrix1[j][i];
	}
}

void addMatrix(int max1, int max2, float matrix1[max1][max2],  float matrix2[max1][max2], float sum[max1][max2])
{
	int i, j;
	for (i = 0; i < max1; i++) 
	{
		for (j = 0 ; j < max2; j++) 
		{
			 sum[i][j] = matrix1[i][j] + matrix2[i][j];
		}
	}
}