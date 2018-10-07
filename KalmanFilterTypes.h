typedef struct{
	float mean;
	float variance;
}GaussType;

#define SHOWARRAY(array) showFloatArray(sizeof(array)/sizeof(array[0]), sizeof(array[0])/sizeof(typeof(array[0][0])), array);

#define NUM_VALS 100