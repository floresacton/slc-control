#include <at24c.h>
#include "memory.h"
#include "stm32g4xx_hal.h"
#include "stdlib.h"

static uint8_t mem_buf[MEMORY_SIZE];

void Memory_Init(void) {
	At24c_Read(0, mem_buf, MEMORY_SIZE);
	if (Memory_ReadByte(MemHash) != MEMORY_HASH) {
		Memory_WriteByte(MemHash, MEMORY_HASH);
		Memory_Reset();
	}
}

void Memory_Reset(void) {
	#define T(name, reset) Memory_WriteByte(Mem##name, reset);
	MEMORY_BYTE_LIST
	#undef T
	#define T(name, reset) Memory_WriteShort(Mem##name, reset);
	MEMORY_SHORT_LIST
	#undef T
	#define T(name, reset) Memory_WriteInt(Mem##name, reset);
	MEMORY_INT_LIST
	#undef T
	#define T(name, reset) Memory_WriteFloat(Mem##name, reset);
	MEMORY_FLOAT_LIST
	#undef T
}

uint8_t Memory_ReadByte(uint8_t loc) {
	return mem_buf[loc - 1];
}
uint16_t Memory_ReadShort(uint8_t loc) {
	const uint16_t pos = MEMORY_BYTES + 2 * (loc - 1);
	return *((uint16_t*) (mem_buf + pos));
}
uint32_t Memory_ReadInt(uint8_t loc) {
	const uint16_t pos = MEMORY_BYTES + 2 * MEMORY_SHORTS + 4 * (loc - 1);
	return *((uint32_t*) (mem_buf + pos));
}
float Memory_ReadFloat(uint8_t loc) {
	const uint16_t pos = MEMORY_BYTES + 2 * MEMORY_SHORTS + 4 * MEMORY_INTS + 4 * (loc - 1);
	return *((float*) (mem_buf + pos));
}

void Memory_WriteByte(uint8_t loc, uint8_t val) {
	const uint16_t pos = loc -1;
	mem_buf[pos] = val;
	At24c_Write(pos, &val, 1);
}
void Memory_WriteShort(uint8_t loc, uint16_t val) {
	const uint16_t pos = MEMORY_BYTES + 2 * (loc - 1);
	uint16_t *ptr = (uint16_t*) (mem_buf + pos);
	ptr[0] = val;
	At24c_Write(pos, (uint8_t*) &val, 2);
}
void Memory_WriteInt(uint8_t loc, uint32_t val) {
	const uint16_t pos = MEMORY_BYTES + 2 * MEMORY_SHORTS + 4 * (loc - 1);
	uint32_t *ptr = (uint32_t*) (mem_buf + pos);
	ptr[0] = val;
	At24c_Write(pos, (uint8_t*) &val, 2);
}
void Memory_WriteFloat(uint8_t loc, float val) {
	const uint16_t pos = MEMORY_BYTES + 2 * MEMORY_SHORTS + 4 * MEMORY_INTS + 4 * (loc - 1);
	float *ptr = (float*) (mem_buf + pos);
	ptr[0] = val;
	At24c_Write(pos, (uint8_t*) &val, 4);
}
