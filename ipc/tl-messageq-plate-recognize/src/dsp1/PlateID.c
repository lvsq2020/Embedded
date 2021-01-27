#include <stdio.h>
#include <string.h>
#include "PlateID.h"

#include <xdc/std.h>
#include <xdc/runtime/Assert.h>
#include <xdc/runtime/Diags.h>
#include <xdc/runtime/Log.h>
#include <xdc/runtime/Registry.h>

//#pragma DATA_ALIGN(mem1, 128u);
//#pragma DATA_SECTION(mem1, ".L1Buffer" )  //将mem1指定为内部内存，有助于提高运算速度
static char mem1[0x4000];	// 16K

TH_PlateIDCfg plateIDCfg;     //识别库配置信息
TH_PlateIDResult result[6]; //识别结果
const char *version;
int InitPlateID(char *recBuf, int size, int imgWidth, int imgHeight)
{
    int nRet;

	memset(&plateIDCfg, 0, sizeof(TH_PlateIDCfg));

	//检测的最小车牌宽，以像素为单位，最小可设为60，推荐值80
	plateIDCfg.nMinPlateWidth = 60;	

	//检测的最大车牌宽，以像素为单位，最大可设为400
	plateIDCfg.nMaxPlateWidth = 400;  

	//最大图像宽度，设置为实际图像宽度
	plateIDCfg.nMaxImageWidth = imgWidth;     

	//最大图像高度，设置为实际图像高度
	plateIDCfg.nMaxImageHeight = imgHeight;   

	//是否垂直方向压缩1倍后识别   0-否  1-是
	plateIDCfg.bVertCompress = 0;		

	//是否场图像	0-否 1-是
	plateIDCfg.bIsFieldImage = 0;		

	//是否同一辆车的多幅图像只输出一次结果  0-否 1-是
	//视频识别模式有效
	plateIDCfg.bOutputSingleFrame = 1;

	//运动or静止图像  0-静止  1-运动(只对相邻两帧中运动的部分进行识别，且只支持1个车道，速度较快)
	//单帧识别时该值赋值为0
	plateIDCfg.bMovingImage = 0;		

	//是否为夜间图片  0-白天 1-夜间，建议该值固定为0
	plateIDCfg.bIsNight = 0;			

	//图像数据格式:ImageFormatRGB、ImageFormatBGR、ImageFormatYUV422、ImageFormatYUV420COMPASS、ImageFormatYUV420
	//设置为实际的图像格式
	//plateIDCfg.nImageFormat = ImageFormatYUV422; 	
	plateIDCfg.nImageFormat = ImageFormatRGB; 	

	//DSP片内内存
	plateIDCfg.pFastMemory = (unsigned char*)mem1;		

	//DSP片内内存大小
	plateIDCfg.nFastMemorySize = 0x4000;  
	
	//普通内存
	plateIDCfg.pMemory = (unsigned char*)recBuf;			

	//普通内存大小
	plateIDCfg.nMemorySize = size;        

	//保留
	plateIDCfg.DMA_DataCopy = NULL;	   

	//保留
	plateIDCfg.Check_DMA_Finished = NULL;

	//用于传递错误信息:0-无错误 1-未找到车牌 2-车牌评价值0分 3-车牌评价值不及格 4-车牌识别分数0分 5-车怕识别分数不及格
	plateIDCfg.nLastError = 0; 

	//出错的模块编号
	plateIDCfg.nErrorModelSN = 0; 

	//车牌输出顺序选项 0-置信度 1-自上而下 2-自下而上
	plateIDCfg.nOrderOpt = 0;			

	//是否启用车牌旋转功能 0-不启用  1-启用  嵌入式平台暂不支持
	plateIDCfg.bLeanCorrection = 0;		

	//0-内部推送+外部获取 1:外部获取	
	plateIDCfg.bMovingOutputOpt = 0;   	

	//0: 识别率优先 1:识别速度优先
	plateIDCfg.nImproveSpeed = 1;        

	//0: 不检测车标 1: 检测车标
	plateIDCfg.bCarLogo = 0;             

	//0: 不检测车位 1: 检测车位
	plateIDCfg.bLotDetect = 0;			

	//0: 针对无阴影的车牌 1：针对有阴影的车牌
	plateIDCfg.bShadow = 0;              
	
	printf("init SDK\n");

	//初始化车牌识别SDK，在使用该SDK的功能前必需且仅需调用一次该函数。pPlateConfig[in]: 车牌识别SDK的配置
    nRet = TH_InitPlateIDSDK(&plateIDCfg);

	if(nRet != TH_ERR_NONE)
	{
		printf("init ret = %d\n", nRet);
		return nRet;
	}
	//设置默认省份，当车牌省份字符置信度较低时，识别库会参考设置的默认省份，输出一个较相似的字符，最多支持6个默认省份
	nRet = TH_SetProvinceOrder("京津冀\0", &plateIDCfg);

	if(nRet != TH_ERR_NONE)
	{
		printf("Set Porvince ret = %d\n", nRet);
		return nRet;
	}
	
	//设置识别阈值
	//nPlateLocate_Th[in]: 取值范围是0-9，图片默认阈值是5。	用于车牌定位，阈值设置越小，越容易定位出车牌，但准确率会下降。
	//nOCR_Th[in]:         取值范围是0-9，图片默认阈值是1。 用于车牌识别，阈值设置越小，越容易识别车牌，但准确率会下降。
	//pPlateConfig[in]: 车牌识别SDK的配置。
	TH_SetRecogThreshold( 5, 1, &plateIDCfg); 

	//设置对特殊车牌的识别，dFormat[in]:特殊车牌类型，pPlateConfig[in]: 车牌识别SDK的配置。
	//开启双层黄牌，可选
	TH_SetEnabledPlateFormat(PARAM_TWOROWYELLOW_ON, &plateIDCfg);

	//开启单层武警牌，可选
	TH_SetEnabledPlateFormat(PARAM_ARMPOLICE_ON, &plateIDCfg);

	//开启双层军牌，可选
	TH_SetEnabledPlateFormat(PARAM_TWOROWARMY_ON, &plateIDCfg);

	//设置夜间模式，不建议打开夜间模式，导致识别率下降
	//bIsNight[in]: true:晚上; false:白天。默认值为false
	//TH_SetDayNightMode( 1, &plateIDCfg );

	
	//返回车牌识别库版本。格式：主版本号.副版本号.编译号.平台类型
	//version = TH_GetVersion();
    
	return nRet;
}

//#define BMP_TEST
#ifndef BMP_TEST
//TH_RECT *prcRange:识别区域，当为NULL时，整幅图像识别
int RecogImage(const unsigned char *pImg, int width, int height, TH_RECT *prcRange)
{
	int nResultNum;
	int nRet = 0;
	int i;
	//int time1, time2;
	//int sRam, sdRam;

	nResultNum = 6;	// 最大输出车牌个数，每次识别需重新设置
	
	//车牌识别接口
	//pbyBits [in]:    		指向内存图像数据的指针
	//nWidth[in]: 			图像的宽度。										
	//Height[in]: 			图像的高度。										
	//pResult[out]:			车牌识别结果数组, 调用方开辟pResult[nResultNum]内存。
	//nResultNum[in,out]: 	in 最大候选车牌个数，out 识别出的车牌个数。
	//prcRange[in]: 		指定识别范围，为NULL时整幅图像识别
	//，pPlateConfig[in]:   车牌识别SDK的配置
	//time1 = Utils_getCurTimeInMsec();
	//TH_SetImageFormat(ImageFormatRGB, 0, 1, &plateIDCfg);
    //Vps_printf("before TH_RecogImage\n");
    //Log_print0(Diags_INFO, "before TH_RecogImage\n");
	nRet = TH_RecogImage(pImg, width, height, result, &nResultNum, prcRange, &plateIDCfg);
    //Log_print0(Diags_INFO, "after TH_RecogImage\n");
	//time2 = Utils_getCurTimeInMsec();
    //Vps_printf("ret=%d, time=%d\n", nRet, (time2-time1));
   
    //Vps_printf("TH_RecogImage ret:%d\n", nRet);
    if(nRet != TH_ERR_NONE)
		return nRet;

    //检查工作过程中最小的剩余内存，如果出现负数，则需要增加给定的初始内存。
	//pnMinFreeSRAM [out]: DSP分配片内内存是否满足识别库需要
	//pnMinFreeSDRAM [out]: DSP分配片外内存是否满足识别库需要
	//TH_CheckMinFreeMemory( &sRam, &sdRam, &plateIDCfg);
	//if(sRam < 0 || sdRam < 0)
	//	printf("内存不足\n");

    Log_print1(Diags_INFO, "nResultNum: %d\n", nResultNum);
	for(i=0; i<nResultNum; i++)
	{
        //Log_print2(Diags_INFO, "plate: %s, color: %s\n", result[i].license, result[i].color);
		//Vps_printf("plate: %s, color: %s\nleft: %d, top: %d, right: %d, bottom: %d\n",
		//result[i].license, result[i].color, result[i].rcLocation.left, result[i].rcLocation.top,
		//result[i].rcLocation.right, result[i].rcLocation.bottom);
	}
	return nRet;
}

#else //从硬盘读取bmp图片，进行识别测试
#if 0
int readline(FILE* f, char *line)
{
    char c;
    int len = 0;
       
    while ((c=fgetc(f)) != EOF && c != '\n')
    {
        line[len++] = c;
    }
    line[len] = '\0';

    if(len > 0)
        return 1;
    else
        return 0;
}

int ReadBmp(char * FileName, unsigned char * pImg, int *pwidth, int *pheight);
unsigned char ImgBuf[1024*1024*4];
//TH_RECT *prcRange:识别区域，当为NULL时，整幅图像识别
int RecogImage(const unsigned char *pImg, int width, int height, TH_RECT *prcRange)
{
	int nResultNum;
	int nRet = 0;
	int i;

	FILE *file_list = NULL;
	char file_name[64];

	file_list = fopen("pic\\test_list.txt", "r");
    if(file_list == NULL)
    {
        printf("fail to open test file list\n");
        return -1;
    }
	//设置图像格式为BGR
	//cImageFormat [in]:    图像格式，默认值为1
	//bVertFlip [in]:  		是否将图像上下颠倒后进行识别，默认值为0。 
	//bDwordAligned [in]:  	是否对图像进行字节对齐，默认值为0。 
	//pPlateConfig[in]: 	车牌识别SDK的配置
	TH_SetImageFormat(ImageFormatBGR, 0, 1, &plateIDCfg);

    while(readline(file_list, file_name) != 0)
    {	
		ReadBmp(file_name, ImgBuf, &width, &height);
        printf("test %s...\n", file_name);

		plateIDCfg.nMaxImageWidth = width;		// 实际尺寸
		plateIDCfg.nMaxImageHeight = height;
    	
		//time1 = CLK_gethtime();
		nResultNum = 6;		// 最大输出车牌个数，每次识别需重新赋值
		nRet = TH_RecogImage(ImgBuf, width, height,  result, &nResultNum, NULL, &plateIDCfg);
		//time2 = CLK_gethtime();
	    //printf("ret=%d, time = %d\n", nRet, (time2-time1)/CLK_countspms());

		if(nRet != TH_ERR_NONE)
			return nRet;
        
		for(i=0; i<nResultNum; i++)
		{
			printf("plate: %s, color: %s\nleft: %d, top: %d, right: %d, bottom: %d\n",
			result[i].license, result[i].color, result[i].rcLocation.left, result[i].rcLocation.top,
			result[i].rcLocation.right, result[i].rcLocation.bottom);
		}
    }

    fclose(file_list);
    return nRet;
}
#endif
#endif
