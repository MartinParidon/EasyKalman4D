int main();

void showFloatArray(int num, float array[]);

void showFloatMatrix(int max1, int max2, float array[max1][max2]);

void multiplyMatrixNN(int N, float mat1[][N], float mat2[][N], float res[][N]);

void multiplyMatrixNM(int N, int M, float mat1[N][M], float mat2[M][N], float res[N][N]);

void multiplyMatrixNO(int N, int M, float mat1[N][M], float mat2[M][N], float res[N][N]);

void multiplyMatrixON(int O, int N, float mat1[O][N], float mat2[N][N], float res[O][N]) ;

void transposeMatrix(int max1, int max2, float matrix1[max1][max2], float matrix2[max1][max2]);

void addMatrix(int max1, int max2, float matrix1[max1][max2], float matrix2[max1][max2], float sum[max1][max2]);

void subtractVector(int num, float vector1[num], float vector2[num], float erg[num]);

void addVector(int num, float vector1[num], float vector2[num], float erg[num]);

void subtractMatrix(int max1, int max2, float matrix1[max1][max2],  float matrix2[max1][max2], float subtr[max1][max2]);

void multiplyMatrixWithVectorNMMM(int max1, int max2, float matrix[max1][max2],  float vectorIn[max2], float vectorOut[max2]);

void multiplyMatrixWithVectorNMMN(int max1, int max2, float matrix[max1][max2],  float vectorIn[max2], float vectorOut[max2]);

// INVERSE FUNCTIONS ONLY FOR 2*2
// https://github.com/md-amanalikhani/Inverse-matrix/blob/master/Inverse-matrix.c
void cofactor(float a[2][2],float d[2][2],float n,float determinte);
void inverse(float a[2][2],float d[2][2],int n,float det);
float det(float a[2][2],int n);
void transpose(float c[2][2],float d[2][2],float n,float det);
void minor(float b[2][2],float a[2][2],int i,int n);

#define SHOWFLOATARRAY(array) showFloatArray(sizeof(array)/sizeof(float), array);

#define SHOWFLOATMATRIX(matrix) showFloatMatrix(sizeof(matrix)/sizeof(matrix[0]), sizeof(matrix[0])/sizeof(typeof(matrix[0][0])), matrix);

#define NUM_VALS 100



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

// Both matrixes have size [N][N], resulting matrix too
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

// Matrix 1: [N][M], Matrix 2: [M][N], Resulting Matrix: [N][N]
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

// Matrix 1: [N][N], Matrix 2: [N][O], Resulting Matrix: [N][O]
void multiplyMatrixNO(int N, int O, float mat1[N][N], float mat2[N][O], float res[N][O]) 
{ 
    int i, j, k; 
    for (i = 0; i < N; i++) 
    { 
        for (j = 0; j < O; j++) 
        { 
            res[i][j] = 0; 
            for (k = 0; k < N; k++) 
                res[i][j] += mat1[i][k]*mat2[k][j]; 
        } 
    } 
} 

// Matrix 1: [O][N], Matrix 2: [N][N], Resulting Matrix: [O][N]
void multiplyMatrixON(int O, int N, float mat1[O][N], float mat2[N][N], float res[O][N]) 
{ 
    int i, j, k; 
    for (i = 0; i < O; i++) 
    { 
        for (j = 0; j < N; j++) 
        { 
            res[i][j] = 0; 
            for (k = 0; k < N; k++) 
                res[i][j] += mat1[i][k]*mat2[k][j]; 
        } 
    } 
} 

// Matrix: [N][M], Vector in: [M], Vector out: [M] OR [N] (only if N<M), N may be = M
void multiplyMatrixWithVectorNMMM(int max1, int max2, float matrix[max1][max2],  float vectorIn[max2], float vectorOut[max2]) 
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

// Matrix: [N][M], Vector in: [M], Vector out: [N]
void multiplyMatrixWithVectorNMMN(int max1, int max2, float matrix[max1][max2],  float vectorIn[max2], float vectorOut[max1]) 
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

void addVector(int num, float vector1[num], float vector2[num], float erg[num])
{
	int i;
	for (i = 0; i < num; i++) 
	{
		erg[i] = vector1[i] + vector2[i];
	}
}

void subtractMatrix(int max1, int max2, float matrix1[max1][max2],  float matrix2[max1][max2], float subtr[max1][max2])
{
	int i, j;
	for (i = 0; i < max1; i++) 
	{
		for (j = 0 ; j < max2; j++) 
		{
			 subtr[i][j] = matrix1[i][j] - matrix2[i][j];
		}
	}
}


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