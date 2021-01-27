#include "mathlib.h"
#include "dsplib.h"


#define pi 3.14159

void tw_gen(float *w, int n);

void FFT(float *Cw, float *CFFT_In, float *CFFT_Out, int CFTT_N);

int ContinueWave(int sig_n,int fft_n,int f0, int fs, float *s);

void FFT_demo(float *Input, float *Output, float *Cw, int fft_n);

