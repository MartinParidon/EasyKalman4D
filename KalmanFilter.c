#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include "KalmanFilter.h"

/************************************************************************************************/
/************************************************************************************************/
/************************************* Function definitions *************************************/
/************************************************************************************************/
/************************************************************************************************/

/************************************************************************************************/
/************************************* Main *****************************************************/
/************************************************************************************************/
int main(void)
{		
	int iVal;
	float X, Y; 
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
	float x[4] = {0, 0, 10, 0};							/* state vector: x, y, x', y' */
	
	float P[4][4];										/* Matrix for covariances betweens variables */
	float A[4][4];										/* Transition Matrix */
	float H[2][4];										/* Measurement Fct (How sensor values are converted) */
	float R[2][2];										/* Measurement noise covariance (low stddev of meas. values: This can be low -> trust measured values MORE). If too high, the filter starts interpreting */	
	float Q[4][4];										/* Process covariance matrix */
	float I[4][4];										/* Identity Matrix */
	
	initMatrices(dt, P, A, H, R, Q, I);
	
	Q[0][0] = 0.25 * powf(dt, 4);	Q[0][1] = 0.25 * powf(dt, 4);	Q[0][2] = 0.5 * powf(dt, 3);	Q[0][3] = 0.5 * powf(dt, 3);	
	Q[1][0] = 0.25 * powf(dt, 4);	Q[1][1] = 0.25 * powf(dt, 4);	Q[1][2] = 0.5 * powf(dt, 3);	Q[1][3] = 0.5 * powf(dt, 3);
	Q[2][0] = 0.5 * powf(dt, 3);	Q[2][1] = 0.5 * powf(dt, 3);	Q[2][2] = powf(dt, 2);	Q[2][3] = powf(dt, 2);
	Q[3][0] = 0.5 * powf(dt, 3);	Q[3][1] = 0.5 * powf(dt, 3);	Q[3][2] = powf(dt, 2);	Q[3][3] = powf(dt, 2);
	
	I[0][0] = 1;	I[0][1] = 0;	I[0][2] = 0;	I[0][3] = 0;	
	I[1][0] = 0;	I[1][1] = 1;	I[1][2] = 0;	I[1][3] = 0;
	I[2][0] = 0;	I[2][1] = 0;	I[2][2] = 1;	I[2][3] = 0;
	I[3][0] = 0;	I[3][1] = 0;	I[3][2] = 0;	I[3][3] = 1;
		
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
	
	FILE *gnuplot = popen("gnuplot -persistent", "w");				/* Define gnuplot handle: https://stackoverflow.com/questions/14311640/how-to-plot-data-by-c-program */
	fprintf(gnuplot, "plot '-'\n");
	
	for (iVal = 0; iVal < numVals; iVal++)				/* May be overwritten for debug purposes */

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
		
		fprintf(gnuplot, "%d %g\n", iVal, x[0]);;		/* May be overwritten for debug purposes */
		
	}
	
	/* Properly finish gnuplot */
	fprintf(gnuplot, "e\n");
	fflush(gnuplot);
		
	/* Properly finish file handlung */
	fclose(f);
	
	/* Pause the output so user can see the values and gnuplot won't close*/
	/* Comment this one out if you need to stop after each run. Can't see gnuplot then. */
	// system("pause");	
	
	return 0;
}

/************************************************************************************************/
/************************************* Fill Measurement Matrix **********************************/
/************************************************************************************************/
void fillMeasurementMatrix(float meanX, float stddevX, float meanY, float stddevY, int numVars, int numVals, float measurementsVel[numVars][numVals])
{	
	int iVal;
	for (iVal = 0; iVal < numVals; iVal++)
	{
		// measurementsVel[0][iVal] = (((float)rand()) / ((float)RAND_MAX) * 6) + 7;		// NOT GAUSSIAN!! Completely random number 7 - 13
		// measurementsVel[1][iVal] = 0;													// Just simply 0
		measurementsVel[0][iVal] = box_muller(meanX, stddevX);								// Gaussian distributed random number for x measurements
		measurementsVel[1][iVal] = box_muller(meanY, stddevY);								// ... y measurements
	}	
}

/************************************************************************************************/
/************************************* Init matrices ********************************************/
/************************************************************************************************/
void initMatrices(float dt, float P[4][4], float A[4][4], float H[2][4], float R[2][2], float Q[4][4], float I[4][4])
{					
	P[0][0] = 10;	P[0][1] = 0;	P[0][2] = 0;	P[0][3] = 0;	
	P[1][0] = 0;	P[1][1] = 10;	P[1][2] = 0;	P[1][3] = 0;
	P[2][0] = 0;	P[2][1] = 0;	P[2][2] = 10;	P[2][3] = 0;	
	P[3][0] = 0;	P[3][1] = 0;	P[3][2] = 0;	P[3][3] = 10;
										
	A[0][0] = 1;	A[0][1] = 0;	A[0][2] = dt;	A[0][3] = 0;	
	A[1][0] = 0;	A[1][1] = 1;	A[1][2] = 0;	A[1][3] = dt;
	A[2][0] = 0;	A[2][1] = 0;	A[2][2] = 1;	A[2][3] = 0;	
	A[3][0] = 0;	A[3][1] = 0;	A[3][2] = 0;	A[3][3] = 1;
										
	H[0][0] = 0;	H[0][1] = 0;	H[0][2] = 1;	H[0][3] = 0;	
	H[1][0] = 0;	H[1][1] = 0;	H[1][2] = 0;	H[1][3] = 1;
	
	R[0][0] = 1;	R[0][1] = 0;		
	R[1][0] = 0;	R[1][1] = 1;
}

/************************************************************************************************/
/************************************* Random number generator **********************************/
/************************************************************************************************/
float box_muller(float m, float s)	
{				        			
	float x1, x2, w, y1;
	static float y2;
	static int use_last = 0;

	if (use_last)		        	
	{
		y1 = y2;
		use_last = 0;
	}
	else
	{
		do {
			x1 = 2.0 * (((float)rand()) / ((float)RAND_MAX)) - 1.0;
			x2 = 2.0 * (((float)rand()) / ((float)RAND_MAX)) - 1.0;
			w = x1 * x1 + x2 * x2;
		} while ( w >= 1.0 );

		w = sqrt( (-2.0 * log( w ) ) / w );
		y1 = x1 * w;
		y2 = x2 * w;
		use_last = 1;
	}

	return( m + y1 * s );
}

void polar(float *x1, float *x2)
{
   float u, v, q, p;

   do {
      u = 2.0 * (((float)rand()) / ((float)RAND_MAX)) - 1;
      v = 2.0 * (((float)rand()) / ((float)RAND_MAX)) - 1;
      q  = u * u + v * v;
   } while (q <= 0.0 || q >= 1.0);

   p = sqrt(-2 * log(q) / q);
   *x1 = u * p;
   *x2 = v * p;
}

/* Debug: Could be implemented like this. Could be slightly more efficient, but a bit less easy in implementation
for (iVal = 0; iVal < numVals - 1; iVal+=2)
{
	polar(&X, &Y);
	PRINTFLOAT((X*1+10))
	PRINTFLOAT((Y*1+10))
	measurementsVel[0][iVal] = X*stddevX+meanX;
	measurementsVel[0][iVal+1] = Y*stddevX+meanX;
}

for (iVal = 0; iVal < numVals; iVal++)
{
	PRINTFLOAT(measurementsVel[0][iVal])
} 
*/

/************************************************************************************************/
/************************************* Print functions ******************************************/
/************************************************************************************************/
void showFloatMatrix(int N, int M, float array[N][M])
{
	int n, m;
	for (n = 0; n < N; n++)
	{
		for (m = 0; m < M; m++)
			printf("%f\t", array[n][m]);
		printf("\n");
	}
}

void showFloatArray(int N, float array[])
{
	int n;
	for (n = 0; n < N; n++)
	{
		printf("%f\t", array[n]);
	}
	printf("\n");
}

/************************************************************************************************/
/************************************* Matrix and vector calculations ***************************/
/************************************************************************************************/
void multiplyMatrix_NN_NN_NN(int N, float mat1[][N], float mat2[][N], float res[][N]) 
{ 
    int n1, n2, n3; 
    for (n1 = 0; n1 < N; n1++) 
    { 
        for (n2 = 0; n2 < N; n2++) 
        { 
            res[n1][n2] = 0; 
            for (n3 = 0; n3 < N; n3++) 
                res[n1][n2] += mat1[n1][n3]*mat2[n3][n2]; 
        } 
    } 
} 

void multiplyMatrix_NM_MN_NN(int N, int M, float mat1[N][M], float mat2[M][N], float res[N][N]) 
{ 
    int n1, n2, m; 
    for (n1 = 0; n1 < N; n1++) 
    { 
        for (n2 = 0; n2 < N; n2++) 
        { 
            res[n1][n2] = 0; 
            for (m = 0; m < M; m++) 
                res[n1][n2] += mat1[n1][m]*mat2[m][n2]; 
        } 
    } 
} 

void multiplyMatrix_NN_NM_NM(int N, int M, float mat1[N][N], float mat2[N][M], float res[N][M]) 
{ 
    int n1, m, n2; 
    for (n1 = 0; n1 < N; n1++) 
    { 
        for (m = 0; m < M; m++) 
        { 
            res[n1][m] = 0; 
            for (n2 = 0; n2 < N; n2++) 
                res[n1][m] += mat1[n1][n2]*mat2[n2][m]; 
        } 
    } 
} 

void multiplyMatrix_MN_NN_MN(int M, int N, float mat1[M][N], float mat2[N][N], float res[M][N]) 
{ 
    int m, n1, n2; 
    for (m = 0; m < M; m++) 
    { 
        for (n1 = 0; n1 < N; n1++) 
        { 
            res[m][n1] = 0; 
            for (n2 = 0; n2 < N; n2++) 
                res[m][n1] += mat1[m][n2]*mat2[n2][n1]; 
        } 
    } 
} 

void multiplyMatrixWithVector_NM_M_N(int N, int M, float matrix[N][M],  float vectorIn[M], float vectorOut[N]) 
{ 
	int n, m;
	for (n = 0; n < N; n++)
	{
		float sumProdMV = 0;
		for (m = 0; m < M; m++)
		{
			sumProdMV += matrix[n][m] * vectorIn[m];
		}
		vectorOut[n] = sumProdMV;
	} 
} 

void transposeMatrix_NM_MN(int N, int M, float matrix1[N][M],  float matrix2[M][N])
{
	int n, m;
	for (n = 0; n < N; n++)
	{
		for (m = 0; m < M; m++)
			matrix2[m][n] = matrix1[n][m];
	}
}

void addMatrices(int N, int M, float matrix1[N][M],  float matrix2[N][M], float erg[N][M])
{
	int n, m;
	for (n = 0; n < N; n++) 
	{
		for (m = 0 ; m < M; m++) 
		{
			 erg[n][m] = matrix1[n][m] + matrix2[n][m];
		}
	}
}

void subtractVectorFromVector(int N, float vector1[N], float vector2[N], float erg[N])
{
	int n;
	for (n = 0; n < N; n++) 
	{
		erg[n] = vector1[n] - vector2[n];
	}
}

void addVectors(int N, float vector1[N], float vector2[N], float erg[N])
{
	int n;
	for (n = 0; n < N; n++) 
	{
		erg[n] = vector1[n] + vector2[n];
	}
}

void subtractMatrixFromMatrix(int N, int M, float matrix1[N][M],  float matrix2[N][M], float erg[N][M])
{
	int n, m;
	for (n = 0; n < N; n++) 
	{
		for (m = 0 ; m < M; m++) 
		{
			 erg[n][m] = matrix1[n][m] - matrix2[n][m];
		}
	}
}

/************************************************************************************************/
/************************************* Matrix inverse calculations ******************************/
/************************************************************************************************/
void cofactor(float a[2][2],float d[2][2],float n,float determinate){
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
			c[h][l] = pow(-1,(h+l))*det(b,(n-1));	/* c = cofactor Matrix */
		}
	transpose(c,d,n,determinate);	//* read function */
}

void inverse(float a[2][2],float d[2][2],int n,float det){
	if(det == 0)
		printf("\nInverse of Entered Matrix is not possible\n");
	else if(n == 1)
		d[0][0] = 1;
	else
		cofactor(a,d,n,det);	/* read function */
}

float det(float a[2][2],int n){
	int i;
	float b[2][2],sum=0;
	if (n == 1)
return a[0][0];
	else if(n == 2)
return (a[0][0]*a[1][1]-a[0][1]*a[1][0]);
	else
		for(i=0;i<n;i++){
			minor(b,a,i,n);	/* read function */
			sum = (float) (sum+a[0][i]*pow(-1,i)*det(b,(n-1)));	/* sum = determinate matrix */
		}
return sum;
}

void transpose(float c[2][2],float d[2][2],float n,float det){
	int i,j;
	float b[2][2];
	for (i=0;i<n;i++)
		for (j=0;j<n;j++)
			b[i][j] = c[j][i];
	for (i=0;i<n;i++)
		for (j=0;j<n;j++)
			d[i][j] = b[i][j]/det;	/* array d[][] = inverse matrix */
}

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
}