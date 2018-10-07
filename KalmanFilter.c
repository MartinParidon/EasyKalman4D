#include <stdio.h>
#include <stdlib.h>
#include "KalmanFilterTypes.h"
#include "KalmanFilter.h"

int main()
{
	int numVal = 5;
	int i;
	
	GaussType currentState;
	GaussType predictionValue[numVal];
	GaussType measurementValue[numVal];
	
	currentState.mean = 0;
	currentState.variance = 10000;
	
	float meanPredValues[] = {1, 1, 2, 1, 1};
	float variancePredValues[] = {2, 2, 2, 2, 2};
	
	float meanMeasValues[] = {5, 6, 7, 9, 10};
	float varianceMeasValues[] = {4, 4, 4, 4, 4};
	
	fprintf(stderr, "Start: %f\t%f\n", currentState.mean, currentState.variance);
	
	for (i = 0; i < numVal; i++)
	{
		predictionValue[i].mean = meanPredValues[i];
		predictionValue[i].variance = variancePredValues[i];
		currentState = Predict(&currentState, &predictionValue[i]);
		fprintf(stderr, "After Pre: %f\t%f\n", currentState.mean, currentState.variance);
		measurementValue[i].mean = meanMeasValues[i];
		measurementValue[i].variance = varianceMeasValues[i];
		currentState = Update(&currentState, &measurementValue[i]);
		fprintf(stderr, "After Mea: %f\t%f\n", currentState.mean, currentState.variance);
	}
	
	fprintf(stderr, "End: %f\t%f\n", currentState.mean, currentState.variance);
	
	return 0;
}

GaussType Predict(GaussType* val1, GaussType* val2)
{
	GaussType valNew;
	
	valNew.mean = val1->mean + val2->mean;
	valNew.variance = val1->variance + val2->variance;
	
	return valNew;				 
}

GaussType Update(GaussType* val1, GaussType* val2)
{
	GaussType valNew;
	
	valNew.mean = ((val2->variance * val1->mean)
				 + (val1->variance * val2->mean))
				 / ((val1->variance + val2->variance));
				 
	valNew.variance = 1 / ((1/val1->variance) + (1/val2->variance));
	
	return valNew;				 
}