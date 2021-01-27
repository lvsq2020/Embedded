#include "dsplib.h"
#include "mathlib.h"                

float CFFT_In[0x40000];
float CFFT_Out[0x40000];
float Cw[0x40000];
float Cmo[0x20000];

void tw_gen(float *w, int n);

void FFT(float *Input, float *Cmo, int Tn);

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

void FFT(float *Input, float *Cmo, int Tn)
{
    int i;
    int rad;

	if (Tn == 16 || Tn == 64 || Tn == 256 || Tn == 1024 || Tn == 4096 || Tn == 16384 || Tn == 65536)
	{
		rad = 4;
	}
	else if (Tn == 8 || Tn == 32 || Tn == 128 || Tn == 512 || Tn == 2048 || Tn == 8192 || Tn == 32768 || Tn == 131072)
	{
		rad = 2;
	}
	else
	{
		return;
	}

	for (i = 0; i < Tn; i++)
	{
		CFFT_In[2 * i] = Input[i];
		CFFT_In[2 * i + 1] = 0;
	}

    tw_gen(Cw,Tn);

    DSPF_sp_fftSPxSP(Tn,CFFT_In,Cw,CFFT_Out,brev,rad,0,Tn);

	for (i = 0; i < Tn; i++)
	{
		if (i == 0)
			Cmo[i] = sqrtsp(CFFT_Out[2 * i] * CFFT_Out[2 * i] + CFFT_Out[2 * i + 1] * CFFT_Out[2 * i + 1]) / Tn;
		else
			Cmo[i] = sqrtsp(CFFT_Out[2 * i] * CFFT_Out[2 * i] + CFFT_Out[2 * i + 1] * CFFT_Out[2 * i + 1]) * 2 / Tn;
	}
}
