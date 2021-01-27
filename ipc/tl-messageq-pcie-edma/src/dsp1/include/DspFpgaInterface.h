/*
 * DspFpgaInterface.h
 *
 *  Created on: 2017年10月13日
 *      Author: Lijy
 */

#ifndef DSP_FPGA_INTERFACE_H
#define DSP_FPGA_INTERFACE_H

#include "Typedef.h"

//------------------------------------PCIe相关信息------------------------------------------------------
//定义使用的lane数
//#define PCIESS1_X2

/* Do gen2 on all devices -- remove or -U for GEN1 */
//PCIe版本，2代=5G
#define GEN2


/* Outbound Base Address for PCIe RC */
#define PCIE_OB_LO_ADDR_RC   0x70000000
//#define PCIE_OB_LO_ADDR_RC   0x10000000
#define PCIE_OB_HI_ADDR_RC   0

/* Inbound  Base Address for PCIe RC */
#define PCIE_IB_LO_ADDR_RC   0x90000000
#define PCIE_IB_HI_ADDR_RC   0

/* Outbound Base Address for PCIe EP */
#define PCIE_OB_LO_ADDR_EP   PCIE_IB_LO_ADDR_RC
#define PCIE_OB_HI_ADDR_EP   0

/* Inbound  Base Address for PCIe EP */
#define PCIE_IB_LO_ADDR_EP   PCIE_OB_LO_ADDR_RC
#define PCIE_IB_HI_ADDR_EP   0

/* Data area offset relative to PCIe base (only used rev 1) */
#define PCIE_WINDOW_MEM_BASE 0x01000000U
#define PCIE_WINDOW_MEM_MASK 0x00FFFFFFU

/* Cfg area offset relative to PCIe base (only used rev 1) */
/* This MUST agree Pciev1_DeviceCfgBaseAddrs.bases! */
#define PCIE_WINDOW_CFG_BASE 0x00001000U
#define PCIE_WINDOW_CFG_MASK 0x00000FFFU

/* MSI address in PCIE data window */
#define PCIE_WINDOW_MSI_ADDR 0x02000000U
#define PCIE_WINDOW_MSI_DATA 0x00000000U

/* Inbound limit (only used rev 1) */
#define PCIE_INBOUND_MASK    0x0FFFFFFFU

/* BAR mask */
#define PCIE_BAR_MASK        0x0FFFFFFF

/* BAR Index PCie*/
#define PCIE_BAR_IDX_RC        1
#define PCIE_BAR_IDX_EP        1


/* PCIe Regions used in the example */
#define PCIE_IB_REGION_RC   0
#define PCIE_OB_REGION_RC   0
#define PCIE_IB_REGION_EP   0
#define PCIE_OB_REGION_EP   0

//pcie出口窗地址
#define ADDR_OUTBOUND_WINDOW 	0x20000000
//发送控制相对于窗的偏移(网口)
#define ADDR_OFFSET_CTRL_SEND		0x100
//接收控制相对于窗的偏移(网口)
#define ADDR_OFFSET_CTRL_RECV		0x200
//发送控制相对于窗的偏移(FT3)
#define ADDR_OFFSET_CTRL_SEND_FT3	0x300
//接收控制相对于窗的偏移(FT3)
#define ADDR_OFFSET_CTRL_RECV_FT3	0x400
//模拟量相对于窗的偏移
#define ADDR_OFFSET_CTRL_ANALOG		0x500
//FPGA状态相对于窗的偏移
#define ADDR_OFFSET_CTRL_FPGA		0x600
//过滤控制相对于窗的偏移
#define ADDR_OFFSET_CTRL_FILTER		0x1000

//------------------------------------报文相关信息------------------------------------------------------

//报文最大长度
#define SIZE_MAX_PKT (1536)
//报文块大小
#define SIZE_PKT_BLOCK (2*1024)

//报文类型定义
#define PKT_TYPE_UNKNOWN	0x0000
#define PKT_TYPE_SV			0xBA88
#define PKT_TYPE_GSE		0xB888
#define PKT_TYPE_FT3		0x6405

//------------------------------------报文发送/接收控制相关------------------------------------------------------
//发送报文的结构信息
typedef struct
{
	union
	{
		struct //st_Pkt_Blk_Infor
		{
			/*
			 1、报文发送按设定时刻发送，若已超时，则立即发送
			 2、当SendTs_sec=SendTs_ns=0，立即发送
			 */
			u32 SendTs_sec;	//发送时刻，秒部分；
			u32 SendTs_ns;	//发送时刻，纳秒部分

			u16 PktType;	//参考PKT_TYPE_xxx定义
			u16 PktSize;//报文实际大小，发送时按此大小发送
			/*
			 1、ptrTS=0，不记录发送时戳
			 2、ptrTS!=0，FPGA将报文实际发送的TS保存到ptrTS指定的地址
			 */
			u32 ptrTS;	//报文实际发送TS存储地址，按Sec、ns顺序12字节
		};
		u32 PktInfor[(SIZE_PKT_BLOCK - SIZE_MAX_PKT)/4]; //占位
	};
	u8 PktData[SIZE_MAX_PKT];//报文数据
} st_Pkt_Blk_Send;

//接收报文的结构信息
typedef struct
{
	union
	{
		struct //st_Pkt_Blk_Infor
		{
			u32 RecvTs_sec;	//接收时刻，秒部分；
			u32 RecvTs_ns;	//接收时刻，纳秒部分

			u16 PktType;	//参考PKT_TYPE_xxx定义
			u16 PktSize;//报文实际大小，发送时按此大小发送
			u16 RecvLanNo;//报文接收网口号，0:网口1,1:网口2....
			u16 rsw;
		};
		u32 PktInfor[(SIZE_PKT_BLOCK - SIZE_MAX_PKT)/4]; //占位
	};
	u8 PktData[SIZE_MAX_PKT];//报文数据
} st_Pkt_Blk_Recv;

//启用标志
#define FLAG_CTRL_REG_EN	1

/*
  发送控制信息
	1、每个网口发送报文对应一个控制信息
 	2、DSP使用ptrWr写数据，FPGA使用ptrRd读取数据
  接收控制信息
	1、n个网口接收报文共用一个控制信息
 	2、DSP使用ptrRd数据，FPGA使用ptrWr写数据
 */
typedef union
{
	struct
	{
		u32 CtrlReg;	//控制寄存器，参考FLAG_CTRL_REG_xxx 定义
		u32 StatReg;	//状态寄存器
		u32 ptrBase;	//循环缓存区基址
		u32 BufSize;	//缓存区大小（按字节，n*BlockSize）
		u32 BlockSize;	//块大小（按字节，SIZE_PKT_BLOCK）
		/*
		 1、 ptrWr=ptrRd	:代表缓存区空
		 2、 ptrWr!=ptrRd	:有数据
		 3、 ptrWr<ptrRd 且 ptrWr+BlockSize>=ptrRd:错误，缓存区满
		 4、ptrWr（或ptrRd）+BlockSize>=ptrBase+BufSize:指针操作翻转为起始ptrBase
		 */
		u32 ptrWr;		//空闲地址(下一次数据写入时地址，写一次+BlockSize)
		u32 ptrRd;		//取值地址(下一次数据读取时地址，读一次+BlockSize)
	};
	u32 CtrlInfo[16];//控制信息，最大16*4
} st_Ctrl_Send,st_Ctrl_Recv;

//------------------------------------报文接收过滤相关------------------------------------------------------
//开关定义
//报文接收过滤	总开关
#define FLAG_ENABLE_CTRL		(1<<0)
//报文接收过滤	报文类型过滤
#define FLAG_ENABLE_FILTER		(1<<1)
//报文接收过滤	Mac地址过滤
#define FLAG_ENABLE_MAC			(1<<2)
//报文接收过滤	透明传输
#define FLAG_ENABLE_TRANSPORT	(1<<3)

//报文类型过滤定义  0:不接收   1：接收
//单个类型定义
//报文类型过滤	接收SV报文
#define MASK_FILTER_SV			(1<0)
//报文类型过滤	接收GSE报文
#define MASK_FILTER_GSE			(1<1)
//报文类型过滤	接收1588报文
#define MASK_FILTER_1588		(1<2)
//报文类型过滤	接收1588报文
#define MASK_FILTER_MMS			(1<3)

//一组类型定义
//报文类型过滤	接收所有报文
#define MASK_FILTER_ALL			(1<24)
//报文类型过滤	接收 单个类型定义之外的报文
#define MASK_FILTER_OTHER		(1<25)

//Mac地址过滤，最大个数
#define MAX_MAC_FILTER_CNT		128

typedef union
{
	struct
	{
		/*
		 1、LanNo=0，表示此Mac地址不判接收网口号
		 2、LanNo!=0，接收网口号必须等于LanNo
		*/
		u8 LanNo;//网口号限制
		u8 rsw;
		u8 MacAddr[6];//接收报文的目标Mac地址
	};
	u16 wVal[4];
	u64 uuVal;
} st_FilterMacSet;

typedef struct
{
	u32 CtrlReg;	//控制寄存器，参考FLAG_ENABLE_xxx 定义
	u32 FilterReg;	//报文类型过滤，参考MASK_FILTER_xxx定义
	union			//透明传输设置
	{
		/*
		 1、每个网口占4bit，共16网口定义
		 2、4bit共16个值，
			 0:代表数据来源DSP
			 1~15：代表当前网口转发设定值代表的网口(1起始)接收的数据，设定值不能为自己
		 */
		struct
		{
			u32 TransLanSet1:4;//网口1
			u32 TransLanSet2:4;//网口2
			u32 TransLanSet3:4;//网口3
			u32 TransLanSet4:4;//网口4
			u32 TransLanSet5:4;//网口5
			u32 TransLanSet6:4;//网口6
			u32 TransLanSet7:4;//网口7
			u32 TransLanSet8:4;//网口8
			u32 TransLanSet9:4;//网口9
			u32 TransLanSet10:4;//网口10
			u32 TransLanSet11:4;//网口11
			u32 TransLanSet12:4;//网口12
			u32 TransLanSet13:4;//网口13
			u32 TransLanSet14:4;//网口14
			u32 TransLanSet15:4;//网口15
			u32 TransLanSet16:4;//网口16
		};
		u64 TransLanSets;//透明传输设置，按4bit代表一个网口使用
	};

	u32 FilterMacCnt;		//Mac地址过滤有效个数
	st_FilterMacSet FilterMacSets[MAX_MAC_FILTER_CNT];
} st_Ctrl_Filter;



//------------------------------------FT3发送、接收相关------------------------------------------------------

#define FT3_BAUDRATE_2_5M	2500000
#define FT3_BAUDRATE_5_0M	5000000
#define FT3_BAUDRATE_10_0M	10000000
#define FT3_BAUDRATE_2_0M	2000000
#define FT3_BAUDRATE_4_0M	4000000
#define FT3_BAUDRATE_6_0M	6000000
#define FT3_BAUDRATE_8_0M	8000000

#define FT3_MODESET_SYNC	0
#define FT3_MODESET_ASYNC	1

#define FT3_INVFLAG_PLUS	0
#define FT3_INVFLAG_INV		1

#define FT3_ASYNCHK_NONE	0
#define FT3_ASYNCHK_ODD		1
#define FT3_ASYNCHK_EVEN	2
#define FT3_ASYNCHK_MARK	3
#define FT3_ASYNCHK_SPACE	4

#define FT3_StopBit(x)		x

#define FT3_AUTOMODE_AUTO	0
#define FT3_AUTOMODE_MANUAL	1

//FT3发送控制
typedef union
{
	struct
	{
		u32 CtrlReg;	//控制寄存器，参考FLAG_CTRL_REG_xxx 定义
		u32 StatReg;	//状态寄存器
		u32 ptrBase;	//循环缓存区基址
		u32 BufSize;	//缓存区大小（按字节，n*BlockSize）
		u32 BlockSize;	//块大小（按字节，SIZE_PKT_BLOCK）
		/*
		 1、 ptrWr=ptrRd	:代表缓存区空
		 2、 ptrWr!=ptrRd	:有数据
		 3、 ptrWr<ptrRd 且 ptrWr+BlockSize>=ptrRd:错误，缓存区满
		 4、ptrWr（或ptrRd）+BlockSize>=ptrBase+BufSize:指针操作翻转为起始ptrBase
		 */
		u32 ptrWr;		//空闲地址(下一次数据写入时地址，写一次+BlockSize)
		u32 ptrRd;		//取值地址(下一次数据读取时地址，读一次+BlockSize)

		u32 BaudRate;	//FT3发送波特率
		union
		{
			struct
			{
				u8 ModeSet:1;//0:同步，1：异步
				u8 InvFlag:1;//0:正码，1：反码
				u8 AsynChk:3;//异步FT3校验方式: 0:none,1:Odd,2:Even,3:Mark,4:Space
				u8 rsw:3;
				u8 StopBit;	//异步FT3停止位个数
				u16 rsw1;
			};
			u32 cfgVal;
		};
	};
	u32 CtrlInfo[16];//控制信息，最大16*4
} st_Ctrl_SendFt3;


//FT3接收控制
typedef union
{
	struct
	{
		u32 CtrlReg;	//控制寄存器，参考FLAG_CTRL_REG_xxx 定义
		u32 StatReg;	//状态寄存器
		u32 ptrBase;	//循环缓存区基址
		u32 BufSize;	//缓存区大小（按字节，n*BlockSize）
		u32 BlockSize;	//块大小（按字节，SIZE_PKT_BLOCK）
		/*
		 1、 ptrWr=ptrRd	:代表缓存区空
		 2、 ptrWr!=ptrRd	:有数据
		 3、 ptrWr<ptrRd 且 ptrWr+BlockSize>=ptrRd:错误，缓存区满
		 4、ptrWr（或ptrRd）+BlockSize>=ptrBase+BufSize:指针操作翻转为起始ptrBase
		 */
		u32 ptrWr;		//空闲地址(下一次数据写入时地址，写一次+BlockSize)
		u32 ptrRd;		//取值地址(下一次数据读取时地址，读一次+BlockSize)

		u32 BaudRate;	//FT3发送波特率
		union
		{
			struct
			{
				u8 ModeSet:1;//0:同步，1：异步
				u8 InvFlag:1;//0:正码，1：反码
				u8 AsynChk:3;//异步FT3校验方式: 0:none,1:Odd,2:Even,3:Mark,4:Space
				u8 rsw:3;
				u8 StopBit;	//异步FT3停止位个数
				u16 rsw1;
			};
			u32 cfgVal;
		};
		u16 AutoMode;//0:自动识别；1：手动指定
		u16 PktType;//0:未指定/未知，其他参考PKT_TYPE_xxx
	};
	u32 CtrlInfo[16];//控制信息，最大16*4
} st_Ctrl_RecvFt3;


//------------------------------------模拟量相关------------------------------------------------------
//ui通道总数
#define TOTAL_CHNUM         32

//模拟量数据定义
typedef struct
{
	u32 daUI[TOTAL_CHNUM];
	u32 output;
} st_AnalogData;

typedef union
{
	struct
	{
		u32 CtrlReg;	//控制寄存器，参考FLAG_CTRL_REG_xxx 定义
		u32 StatReg;	//状态寄存器
		u32 DigSmpRate;	//数字量采用率	商
		u32 AnaSmpRate;	//DSP采用率	商
		u16 DigRemainder;//数字量采用率	余数
		u16 AnaRemainder;//DSP采用率	余数
		u32 AnaMask;	//模拟量通道输出控制，按位使用，1：代表输出该通道
		u32 ptrBase;	//模拟量基址
		u32 BufSize;	//模拟量数据大小
		u32 Amplif;		//功放投切
		u32 Load;		//电源轻重载控制信号
		struct
		{
			u32 Power:26;				//Power控制信号
			u32 DataValid:6;			//波形数据有效位数，0~31
		};

		struct {
			u32 InputBinA :24;	//开入量
			u32 DevError :8;	//故障
		};
		s32 DaDelay;
	};
	u32 CtrlInfo[16];//控制信息，最大16*4
} st_Ctrl_Analog;


//------------------------------------FPGA状态相关------------------------------------------------------
//设置FPGA时间
#define FLAG_CTRL_FPGA_SET_TIME		1
//关闭B码对时   对时守时菜单使用
#define FLAG_CTRL_FPGA_NO_IRGB		2

//发现多个有效的对时信号
#define STAT_CTRL_FPGA_MIX_PPS		1

typedef union
{
	struct
	{
		u32 CtrlReg;	//控制寄存器，参考FLAG_CTRL_FPGA_xxx 定义
		u32 StatReg;	//状态寄存器    参考STAT_CTRL_FPGA_xxx 定义
		u32 UTC_Sec;	//UTC时间，秒
		u32 Cur_Sec;	//FPGA当前时间，秒
		u32 Cur_ns;		//FPGA当前时间，ns
		u32 TimeMode;	//FPGA当前对时模式
		u32 FPGAFre;	//FPGA当前系统时钟频率
		u32 Version;	//FPGA的程序版本
		u32 Function;	//FPGA的功能
	};
	u32 CtrlInfo[16];//控制信息，最大16*4
} st_Ctrl_FPGA;




#endif /* DSP_FPGA_INTERFACE_H */
