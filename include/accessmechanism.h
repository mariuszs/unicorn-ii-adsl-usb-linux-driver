#ifndef _ACCESSMECHANISM_H_
#define _ACCESSMECHANISM_H_

#include "Typesdrv.h"

#ifndef __cplusplus
typedef struct BlockAccess
{
      WORD address;
      WORD data;
}BlockAccess;

typedef struct DataBlockIndirect
{
      WORD data0;
      WORD data1;
      WORD data2;
}DataBlockIndirect;
#endif
#endif
