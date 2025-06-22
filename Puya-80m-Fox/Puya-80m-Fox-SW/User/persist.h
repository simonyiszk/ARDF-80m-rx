#ifndef _PERSIST_H
#define _PERSIST_H

#include <cmsis_gcc.h>

#define CRITICAL_SECTION(...)           \
do {                                    \
  uint32_t primask = __get_PRIMASK();   \
  __disable_irq();                      \
  {__VA_ARGS__}                         \
  __set_PRIMASK(primask);               \
} while(0);

typedef enum {
  EMPTY = 0, OK=0xA5, FULL = 0xFF
} STORAGE_STATUS;

void Persist_Init(void);
void Persist_Save(volatile uint8_t* content, uint8_t length);
uint8_t Persist_Read(uint8_t* content);
void Persist_DumpPage(uint8_t *content, uint8_t page_idx);

#endif