@echo off
taskkill /im gnuplot.exe
make clean
make
KalmanFilter
pause
CleanMakeAndRun