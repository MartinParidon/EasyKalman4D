int main();

void showFloatArray(int num, float array[]);

void showFloatMatrix(int max1, int max2, float array[max1][max2]);

void multiplyMatrixNN(int N, float mat1[][N], float mat2[][N], float res[][N]);

void multiplyMatrixNM(int N, int M, float mat1[N][M], float mat2[M][N], float res[N][N]);

void transposeMatrix(int max1, int max2, float matrix1[max1][max2], float matrix2[max1][max2]);

void addMatrix(int max1, int max2, float matrix1[max1][max2], float matrix2[max1][max2], float sum[max1][max2]);

void subtractVector(int num, float vector1[num], float vector2[num], float erg[num]);

// NOT TESTED
// void subtractMatrix(int max1, int max2, float matrix1[max1][max2],  float matrix2[max1][max2], float subtr[max1][max2]);

void multiplyMatrixWithVector(int max1, int max2, float matrix[max1][max2],  float vectorIn[max2], float vectorOut[max2]);

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