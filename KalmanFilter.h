int main();

GaussType Predict(GaussType* val1, GaussType* val2);
GaussType Update(GaussType* val1, GaussType* val2);

void showFloatArray(int max1, int max2, float array[max1][max2]);

void multiplyMatrix(int N, float mat1[][N], float mat2[][N], float res[][N]);

void transposeMatrix(int max1, int max2, float matrix1[max1][max2], float matrix2[max1][max2]);

void addMatrix(int max1, int max2, float matrix1[max1][max2], float matrix2[max1][max2], float sum[max1][max2]);

void multiplyMatrixWithVector(int max1, int max2, float matrix[max1][max2],  float vector[max2]);