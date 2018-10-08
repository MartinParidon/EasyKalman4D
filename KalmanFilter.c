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
	iVal=0;
	{
		// Prediction
		// x = A*x
		multiplyMatrixWithVector(4, 4, A, x, x);
		
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
		Z[1] = measurementsVel[1][iVal];
		float Hx[2] = {0};
		multiplyMatrixWithVector(2, 4, H, x, Hx);
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
		
		// DOES NOT WORK YET
		float PHt[4][2] = {0};
		multiplyMatrixNM(4, 2, P, Ht, PHt);
		SHOWFLOATMATRIX(PHt)
		
		// WORKS
		float Si[2][2] = {0};
		float deter = (float) det(S,2);
		inverse(S,Si,2,deter);
		
		// S=(H*P*H'+R);           % Innovationskovarianz
		// K=P*H'*inv(S);          % Filter-Matrix (Kalman-Gain)
	  
		// x=x+(K*y);              % aktualisieren des Systemzustands
		// P=(I-(K*H))*P;          % aktualisieren der Kovarianz
		
	}
	
	return 0;
}

void showFloatMatrix(int max1, int max2, float array[max1][max2])
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

void showFloatArray(int num, float array[])
{
	int i;
	for (i = 0; i < num; i++)
	{
		printf("%f\t", array[i]);
	}
	printf("\n");
}

void multiplyMatrixNN(int N, float mat1[][N], float mat2[][N], float res[][N]) 
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

void multiplyMatrixNM(int N, int M, float mat1[N][M], float mat2[M][N], float res[N][N]) 
{ 
    int i, j, k; 
    for (i = 0; i < N; i++) 
    { 
        for (j = 0; j < N; j++) 
        { 
            res[i][j] = 0; 
            for (k = 0; k < M; k++) 
                res[i][j] += mat1[i][k]*mat2[k][j]; 
        } 
    } 
} 

void multiplyMatrixWithVector(int max1, int max2, float matrix[max1][max2],  float vectorIn[max2], float vectorOut[max2]) 
{ 
	int iRow, iCol;
	for (iRow = 0; iRow < max1; iRow++)
	{
		float sumProdMV = 0;
		for (iCol = 0; iCol < max2; iCol++)
		{
			sumProdMV += matrix[iRow][iCol] * vectorIn[iCol];
		}
		vectorOut[iRow] = sumProdMV;
	} 
} 

void transposeMatrix(int max1, int max2, float matrix1[max1][max2],  float matrix2[max2][max1])
{
	int i, j;
	for (i = 0; i < max1; i++)
	{
		for (j = 0; j < max2; j++)
			matrix2[j][i] = matrix1[i][j];
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

void subtractVector(int num, float vector1[num], float vector2[num], float erg[num])
{
	int i;
	for (i = 0; i < num; i++) 
	{
		erg[i] = vector1[i] - vector2[i];
	}
}

// NOT TESTED
// void subtractMatrix(int max1, int max2, float matrix1[max1][max2],  float matrix2[max1][max2], float subtr[max1][max2])
// {
	// int i, j;
	// for (i = 0; i < max1; i++) 
	// {
		// for (j = 0 ; j < max2; j++) 
		// {
			 // subtr[i][j] = matrix1[i][j] - matrix2[i][j];
		// }
	// }
// }


// INVERSE FUNCTIONS ONLY FOR 2*2
// https://github.com/md-amanalikhani/Inverse-matrix/blob/master/Inverse-matrix.c

//	calculate cofactor of matrix
void cofactor(float a[2][2],float d[2][2],float n,float determinte){
	float b[2][2],c[2][2];
	int l,h,m,k,i,j;
	for (h=0;h<n;h++)
		for (l=0;l<n;l++){
			m=0;
			k=0;
			for (i=0;i<n;i++)
				for (j=0;j<n;j++)
					if (i != h && j != l){
						b[m][k]=a[i][j];
						if (k<(n-2))
							k++;
						else{
							k=0;
							m++;
						}
					}
			c[h][l] = pow(-1,(h+l))*det(b,(n-1));	// c = cofactor Matrix
		}
	transpose(c,d,n,determinte);	// read function
}// end function

void inverse(float a[2][2],float d[2][2],int n,float det){
	if(det == 0)
		printf("\nInverse of Entered Matrix is not possible\n");
	else if(n == 1)
		d[0][0] = 1;
	else
		cofactor(a,d,n,det);	// read function
}// end function

//	calculate determinte of matrix
float det(float a[2][2],int n){
	int i;
	float b[2][2],sum=0;
	if (n == 1)
return a[0][0];
	else if(n == 2)
return (a[0][0]*a[1][1]-a[0][1]*a[1][0]);
	else
		for(i=0;i<n;i++){
			minor(b,a,i,n);	// read function
			sum = (float) (sum+a[0][i]*pow(-1,i)*det(b,(n-1)));	// read function	// sum = determinte matrix
		}
return sum;
}// end function

//	calculate transpose of matrix
void transpose(float c[2][2],float d[2][2],float n,float det){
	int i,j;
	float b[2][2];
	for (i=0;i<n;i++)
		for (j=0;j<n;j++)
			b[i][j] = c[j][i];
	for (i=0;i<n;i++)
		for (j=0;j<n;j++)
			d[i][j] = b[i][j]/det;	// array d[][] = inverse matrix
}// end function

//	calculate minor of matrix OR build new matrix : k-had = minor
void minor(float b[2][2],float a[2][2],int i,int n){
	int j,l,h=0,k=0;
	for(l=1;l<n;l++)
		for( j=0;j<n;j++){
			if(j == i)
				continue;
			b[h][k] = a[l][j];
			k++;
			if(k == (n-1)){
				h++;
				k=0;
			}
		}
}// end function