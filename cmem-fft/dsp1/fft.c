#include "fft.h"


unsigned char brev[64]=
{
	0x0, 0x20, 0x10, 0x30, 0x8, 0x28, 0x18, 0x38,
	0x4, 0x24, 0x14, 0x34, 0xc, 0x2c, 0x1c, 0x3c,
	0x2, 0x22, 0x12, 0x32, 0xa, 0x2a, 0x1a, 0x3a,
	0x6, 0x26, 0x16, 0x36, 0xe, 0x2e, 0x1e, 0x3e,
	0x1, 0x21, 0x11, 0x31, 0x9, 0x29, 0x19, 0x39,
	0x5, 0x25, 0x15, 0x35, 0xd, 0x2d, 0x1d, 0x3d,
	0x3, 0x23, 0x13, 0x33, 0xb, 0x2b, 0x1b, 0x3b,
	0x7, 0x27, 0x17, 0x37, 0xf, 0x2f, 0x1f, 0x3f
};

//FFT旋转因子
void tw_gen(float *w, int n)
{
    int i, j, k;
    const double PI = 3.141592654;
    for (j = 1, k = 0; j <= n >> 2; j = j << 2)
    {
        for (i = 0; i < n >> 2; i += j)
        {
            w[k]     = (float) sinsp (2 * PI * i / n);
            w[k + 1] = (float) cossp (2 * PI * i / n);
            w[k + 2] = (float) sinsp (4 * PI * i / n);
            w[k + 3] = (float) cossp (4 * PI * i / n);
            w[k + 4] = (float) sinsp (6 * PI * i / n);
            w[k + 5] = (float) cossp (6 * PI * i / n);
            k += 6;
        }
    }
}

/*
FFT实现:
CW为旋转因子，CFFT_IN和CFFT_OUT分别为FFT的输入和输出
CW、CFFT_IN和CFFT_OUT均为复数
CFFT_N为FFT的长度
*/
void FFT(float *Cw, float *CFFT_In, float *CFFT_Out, int CFFT_N)
{
    int rad;

	if (CFFT_N == 16 || CFFT_N == 64 || CFFT_N == 256 || CFFT_N == 1024 || CFFT_N == 4096 || CFFT_N == 16384 || CFFT_N == 65536)
	{
		rad = 4;
	}
	else if (CFFT_N == 8 || CFFT_N == 32 || CFFT_N == 128 || CFFT_N == 512 || CFFT_N == 2048 || CFFT_N == 8192 || CFFT_N == 32768 || CFFT_N == 131072)
	{
		rad = 2;
	}
	else
	{
		return;
	}

    DSPF_sp_fftSPxSP(CFFT_N,CFFT_In,Cw,CFFT_Out,brev,rad,0,CFFT_N);

}



int ContinueWave(int sig_n,int fft_n,int f0, int fs, float *s)
{
    int i;
    float t;

    if(sig_n >fft_n)
    {
        return 0;
    }
    for (i = 0; i < sig_n; i++)
    {
        t = (float)1 / fs * i;
	    s[2*i] = cos(2 * pi * f0* t);
        s[2*i+1] = 0;
    }
    for (i = sig_n; i < fft_n; i++)
    {
	    s[2*i] = 0;
        s[2*i+1] = 0;
    }
    return 1;
}

void FFT_demo(float *Input, float *Output, float *Cw, int fft_n)
{
    tw_gen(Cw, fft_n);
    FFT(Cw, Input, Output, fft_n);
}


