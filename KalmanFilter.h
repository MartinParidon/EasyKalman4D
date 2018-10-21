#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

/************************************************************************************************/
/************************************* Function declarations ************************************/
/************************************************************************************************/

/************************************* Main *****************************************************/
/* Main function */
int main(void);

/************************************* Fill measurement matrix with random vars *****************/
/* Fills given measurement matrix with gaussian measurement values */
void fillMeasurementMatrix(float meanX, float stddevX, float meanY, float stddevY, int numVars, int numVals, float measurementsVel[numVars][numVals]);

/************************************* Init matrices ********************************************/
/* Initialize matrices for start values */
void initMatrices(float dt, float x[4], float P[4][4], float A[4][4], float H[2][4], float R[2][2], float Q[4][4], float I[4][4]);

/************************************* Random number generator **********************************/
/* Normal random variate generator (box muller method), taken from ftp://ftp.taygeta.com/pub/c/boxmuller.c */
float box_muller(float m, float s);

/* Alternative method that doesn't use trigonometric functions. Might be a bit more performant. Taken from https://de.wikipedia.org/wiki/Polar-Methode#C */
void polar(float *x1, float *x2);

/************************************* Print functions ******************************************/
/* Print function for float array */
void showFloatArray(int size, float array[]);

/* Print function for float matrix */
void showFloatMatrix(int N, int M, float array[N][M]);

/************************************* Matrix and vector calculations ***************************/
/* Multiply matrices. Both matrixes have size [N][N], resulting matrix too */
void multiplyMatrix_NN_NN_NN(int N, float mat1[][N], float mat2[][N], float res[][N]);

/* Multiply matrices. Matrix 1: [N][M], Matrix 2: [M][N], Resulting Matrix: [N][N] */
void multiplyMatrix_NM_MN_NN(int N, int M, float mat1[N][M], float mat2[M][N], float res[N][N]);

/* Multiply matrices. Matrix 1: [N][N], Matrix 2: [N][M], Resulting Matrix: [N][M] */
void multiplyMatrix_NN_NM_NM(int N, int M, float mat1[N][N], float mat2[N][M], float res[N][M]);

/* Multiply matrices. Matrix 1: [M][N], Matrix 2: [N][N], Resulting Matrix: [M][N] */
void multiplyMatrix_MN_NN_MN(int M, int N, float mat1[M][N], float mat2[N][N], float res[M][N]);

/* Multiply Matrix with vector. [N][M], Vector in: [M], Vector out: [N]. [N] may be equal to [M] */
void multiplyMatrixWithVector_NM_M_N(int N, int M, float matrix[N][M],  float vectorIn[M], float vectorOut[N]);

/* Subtracts one vector from another one. Both have same size */
void subtractVectorFromVector(int size, float vector1[size], float vector2[size], float erg[size]);

/* Adds two vector with same sizes */
void addVectors(int size, float vector1[size], float vector2[size], float erg[size]);

/* Adds two matrices with same sizes */
void addMatrices(int N, int M, float matrix1[N][M], float matrix2[N][M], float erg[N][M]);

/* Subtracts same sized matrices from one another */
void subtractMatrixFromMatrix(int N, int M, float matrix1[N][M],  float matrix2[N][M], float erg[N][M]);

/* Transpose Matrix: matrix2[i][j] = matrix1[j][i] [N] may be equal to [M]*/
void transposeMatrix_NM_MN(int N, int M, float matrix1[N][M], float matrix2[M][N]);

/************************************* Matrix inverse calculations ******************************/
/* Functions for inverse matrix calculation, taken from: https://github.com/md-amanalikhani/Inverse-matrix/blob/master/Inverse-matrix.c
   For now, only works for 2*2 matrix! */
   
/*	calculate cofactor of matrix */
void cofactor(float a[2][2], float d[2][2],float n, float determinate);

/*	inverse of matrix */
void inverse(float a[2][2], float d[2][2], int n, float det);

/*	calculate determinate of matrix */
float det(float a[2][2], int n);

/*	calculate transpose of matrix */
void transpose(float c[2][2], float d[2][2], float n, float det);

/*	calculate minor of matrix OR build new matrix : k-had = minor */
void minor(float b[2][2], float a[2][2], int i, int n);

/************************************************************************************************/
/************************************* Useful shortcuts (defines) *******************************/
/************************************************************************************************/
/* Get length of Matrix / Array */
#define MATRIX_LEN1(matrix) (sizeof((matrix))/sizeof((matrix)[0][0]))/(sizeof((matrix)[0])/sizeof((matrix)[0][0]))
#define MATRIX_LEN2(matrix) (sizeof((matrix))/sizeof((matrix)[0][0]))/((sizeof((matrix))/sizeof((matrix)[0][0]))/(sizeof((matrix)[0])/sizeof((matrix)[0][0])))
#define SIZEOFARRAY(arr) sizeof(arr)/sizeof(arr[0])

/*  Easy access to print functions */
#define SHOWFLOATARRAY(array) showFloatArray(sizeof(array)/sizeof(float), array);
#define SHOWFLOATMATRIX(matrix) showFloatMatrix(sizeof(matrix)/sizeof(matrix[0]), sizeof(matrix[0])/sizeof(typeof(matrix[0][0])), matrix);

/* Easy access to generally useful print functions */
#define PRINTFLOAT(val) printf("%f\n", val);
#define PRINTINT(val) printf("%d\n", val);
#define PRINTSTRING(val) printf("%s\n", val);
#define PRINTRETURN printf("\n");
#define PRINTTAB printf("\t");

/* Easy access to file write functions */
#define WRITEFLOATTOFILE(f, val) fprintf(f, "%f\n", val);
#define WRITEFLOATARRAYTOFILE(f, arr) int i; for ( i= 0; i < SIZEOFARRAY(arr); i++) fprintf(f, "%f\n", arr[i]);
#define WRITEFLOATARRAYTOFILETAB(f, arr) int i; for ( i= 0; i < SIZEOFARRAY(arr); i++) fprintf(f, "%f\t", arr[i]);
#define WRITEENTERTOFILE(f) fprintf(f, "\n");

#endif