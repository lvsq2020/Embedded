/*
 * Typedef.h
 *
 *  Created on: 2016-6-13
 *      Author: ljy
 */

#ifndef TYPEDEF_H_
#define TYPEDEF_H_

#ifndef BYTE
typedef  unsigned  char   BYTE;
#endif

typedef  unsigned  char   u8,U8;//,UINT8;
typedef  signed    char   s8,S8,INT8;
typedef  unsigned  short  u16,U16,UINT16;
typedef  signed    short  s16,S16,INT16;
typedef  unsigned  int    u32,U32;//,UINT32;
typedef  signed    int    s32,S32,INT32;
typedef  long	long	  s64,S64,INT64;
typedef  unsigned long	long	  u64,U64,UINT64;


typedef  unsigned long     ULONG;
typedef  long              LONG;
typedef  long              HRESULT;
//typedef  int               BOOL;
typedef  unsigned short    xchar;
typedef  const char*       PCASTR;
typedef  const xchar*      PCXSTR;
typedef  const xchar*      LPUSTR;
typedef  const char* LPCSTR;


#define ST_MEM_CPY  memcpy
#define ST_STR_CMP  strcmp
#define ST_ASM      asm

//character set to utf-8
#if defined(_MSC_VER) && (_MSC_VER>=1900)
#define _T(x) u8##x
#else
#define _T(x) x
#endif

#define ASTR(x) x


#endif /* TYPEDEF_H_ */
