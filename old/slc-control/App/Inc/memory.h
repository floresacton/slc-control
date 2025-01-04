#ifndef INC_MEMORY_H_
#define INC_MEMORY_H_

#include "stdint.h"

/* BEGIN CONFIG */
#define MEMORY_BYTES  4 //including MemHash
#define MEMORY_SHORTS 8
#define MEMORY_INTS   0
#define MEMORY_FLOATS 2

#define MEMORY_HASH 19 //change on upload

//T(name, default value) -> enum Mem+name
#define MEMORY_BYTE_LIST T(Tach1Spokes, 10) T(Tach2Spokes, 10) T(Tach3Spokes, 10)
#define MEMORY_SHORT_LIST T(MinRpm, 20) T(AuxThresh, 1000) T(Tach1Max, 10000) T(Tach2Max, 10000) T(Tach3Max, 10000)\
T(Tach1Thresh, 2048) T(Tach2Thresh, 2048) T(Tach3Thresh, 2048)
#define MEMORY_INT_LIST
#define MEMORY_FLOAT_LIST T(GpsTire, 53.4) T(SlipThresh, 10.0)
/* END CONFIG */

#define MEMORY_SIZE MEMORY_BYTES+2*MEMORY_SHORTS+4*MEMORY_INTS+4*MEMORY_FLOATS

enum MemoryType {
	MemByte,
	MemShort,
	MemInt,
	MemFloat,
};

#define T(name, reset) Mem##name,
enum MemoryByteId {
	MemDummyByte,
	MemHash,
	MEMORY_BYTE_LIST
};
enum MemoryShortId {
	MemDummyShort,
	MEMORY_SHORT_LIST
};
enum MemoryIntId {
	MemDummyInt,
	MEMORY_INT_LIST
};
enum MemoryFloatId {
	MemDummyFloat,
	MEMORY_FLOAT_LIST
};
#undef T

void Memory_Init(void);
void Memory_Reset(void);

uint8_t Memory_ReadByte(enum MemoryByteId loc);
uint16_t Memory_ReadShort(enum MemoryShortId loc);
uint32_t Memory_ReadInt(enum MemoryIntId loc);
float Memory_ReadFloat(enum MemoryFloatId loc);

void Memory_WriteByte(enum MemoryByteId loc, uint8_t val);
void Memory_WriteShort(enum MemoryShortId loc, uint16_t val);
void Memory_WriteInt(enum MemoryIntId loc, uint32_t val);
void Memory_WriteFloat(enum MemoryFloatId loc, float val);

#endif
