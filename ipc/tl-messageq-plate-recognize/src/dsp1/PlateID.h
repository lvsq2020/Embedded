#ifndef _PlateNet_H_
#define _PlateNet_H_

#include "./lib/TH_PlateID.h"

int InitPlateID(char *recBuf, int size, int imgWidth, int imgHeight);
int RecogImage(const unsigned char *pImg, int width, int height, TH_RECT *prcRange);

#endif
