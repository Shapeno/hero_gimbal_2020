#ifndef _SPI_H_
#define _SPI_H_

#include "sys.h"

#define	SPI_CS PFout(6)

void SPI5_Init(void);
void SPI_SetSpeed(u8 SpeedSet);
u8 SPI_ReadWriteByte(u8 TxData);

#endif

