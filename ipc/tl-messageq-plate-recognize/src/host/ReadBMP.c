#include <stdio.h>
#include <stdlib.h>
//#include "..\TH_PlateID_V4\TH_PlateID.h"

#define BFT_BITMAP 0x4d42   /* 'BMP' */
#define BI_RGB        0L
#define BI_RLE8       1L
#define BI_RLE4       2L
#define BI_BITFIELDS  3L

// 读取bmp格式文件，并转换成灰度图，目前只支持彩色和灰度图像
int ReadBmp(char * FileName, unsigned char * pImg, int *width, int *height)
{
	FILE *hFile;
	unsigned short bfType,biBitCount;
	unsigned int biSize,dwCompression, biClrUsed, bfOffBits;
	int biWidth, biHeight;
	unsigned int dwEffWidth;
	int i;

	hFile=fopen(FileName,"rb");
	if (hFile == NULL) return 0;

	fread(&bfType,2,1,hFile);
	if (bfType != BFT_BITMAP) { //do we have a RC HEADER?
        printf("Not bmp File!\n");
		return 0;
    }
	fseek(hFile,10,SEEK_SET);
	fread(&bfOffBits,4,1,hFile);
	fread(&biSize,4,1,hFile);
	if (biSize!=40) {
		printf("Not common BITMAPINFOHEADER, BITMAPCOREHEADER=12!\n");
		return 0;
	}
	fread(&biWidth,4,1,hFile);
	fread(&biHeight,4,1,hFile);
	fseek(hFile,2,SEEK_CUR); // 跳过 biPlanes
	fread(&biBitCount,2,1,hFile);
	fread(&dwCompression,4,1,hFile);
	fseek(hFile,12,SEEK_CUR); // 跳过 biPlanes
	fread(&biClrUsed,4,1,hFile);

	if (dwCompression!=BI_RGB) {
		printf("Not supported Compression!\n");
		fclose(hFile);
		return 0;
	}

	if (biBitCount!=24) {
		printf("only support 24bit color!\n");
		fclose(hFile);
		return 0;
	}

//	if (biClrUsed!=0) {
//		printf("Palette not supported, Colors=%d\n",biClrUsed);
//		return false;
//	}

/*
	pImg= (unsigned char *) malloc (biHeight*biWidth*3);

	if (pImg==NULL) {
		printf("no memory when alloc image\n");
		fclose(hFile);
		return false;
	}
*/
    dwEffWidth = ((((biBitCount * biWidth) + 31) / 32) * 4);

	*width = biWidth;
	*height= biHeight;

	printf("%d,%d\n",biWidth,biHeight);

	// 定位图像数据
	fseek(hFile,bfOffBits,SEEK_SET);
	for(i=0;i<biHeight;i++)
	{
		fread(pImg+(biHeight-1-i)*biWidth*3, dwEffWidth,1,hFile); // read in the pixels
	}

	fclose(hFile);
	return 1;
}
