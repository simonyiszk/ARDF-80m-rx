#include <stdint.h>
#include <string.h>
#include "persist.h"
#include "py32f0xx_ll_flash.h"

/// @brief Base address of .user_data - must be specified in linker script (ld)
extern const uint32_t _USER_DATA;
extern const uint32_t _USER_DATA_SIZE;

/// @brief Pointer to _USER_DATA in flash
static volatile const uint8_t *const USER_DATA = (uint8_t *)&_USER_DATA;

static uint32_t start_pos = 0;
static uint32_t end_pos = 0;
static uint8_t page = 0;

/// @brief Writes a page of data (128 bytes) at the desired address of a flash
/// @param address Page base address in flash
/// @param page_data Data to write (128 bytes = 32 * 32-bit words)
static void Flash_Write(uint32_t address, uint32_t *page_data)
{

  while (__LL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != 0x00)
    ; // wait for flash if busy

  CRITICAL_SECTION(
      (void)LL_FLASH_Unlock();         // unlock flash
      SET_BIT(FLASH->CR, FLASH_CR_PG); // start programming

      uint8_t index = 0;
      uint32_t dest = address;
      uint32_t *src = page_data;

      while (index < 32U) {
        *(uint32_t *)dest = *src;
        src += 1U;
        dest += 4U;
        index++;
        if (index == 31)
        {
          SET_BIT(FLASH->CR, FLASH_CR_PGSTRT);
        }
      }

      while (__LL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != 0x00); // wait for flash if busy

      CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
      CLEAR_BIT(FLASH->SR, FLASH_SR_EOP);

      (void)LL_FLASH_Lock(););
}

//----------------------------------------------------------------------------
//! \brief  Erases one sector in flash
//! \return -
//! \global -
//! \note   Stalls the CPU.
//-----------------------------------------------------------------------------
static void Flash_EraseSector(void)
{
  // Unlock flash
  CRITICAL_SECTION(
      (void)LL_FLASH_Unlock(); // unlock flash

      SET_BIT(FLASH->CR, FLASH_CR_SER); // Trigger sector erase
      *(uint32_t *)USER_DATA = 0xFFFFFFFFu;

      while (FLASH->SR & FLASH_SR_BSY); // Wait for operation to finish

      CLEAR_BIT(FLASH->CR, FLASH_CR_SER); // Clear operation mode bit
      CLEAR_BIT(FLASH->SR, FLASH_SR_EOP);

      (void)LL_FLASH_Lock();)
}

/// @brief Initialize persistent storage in flash
void Persist_Init(void)
{
  start_pos = 0;
  end_pos = 0;

  for (uint32_t i = 0; i < _USER_DATA_SIZE; i++) // find first non-zero byte in memory
  {
    if (USER_DATA[i] != 0x00)
    {
      start_pos = i;
      break;
    }
  }

  for (uint32_t i = start_pos; i < _USER_DATA_SIZE; i++) // find first empty byte in memory
  {
    if (USER_DATA[i] == 0xFF)
    {
      end_pos = i;
      break;
    }
  }

  page = start_pos / 128;
  start_pos -= (uint32_t)page * 128;
  end_pos -= (uint32_t)page * 128;
}

uint8_t Persist_Read(uint8_t *content)
{
  uint8_t length = (uint8_t)end_pos - (uint8_t)start_pos;
  CRITICAL_SECTION(
      (void)memcpy(content, (uint8_t *)&USER_DATA[page * 128 + start_pos], length);)
  return length;
}

void Persist_DumpPage(uint8_t *content, uint8_t page_idx)
{
  CRITICAL_SECTION(
      (void)memcpy(content, (uint8_t *)&USER_DATA[page_idx * 128], 128);)
}

/// @brief Saves data into the next empty slot in flash section `.user_data`
/// @param data_to_save
void Persist_Save(volatile uint8_t *content, volatile uint8_t length)
{

  uint32_t page_content[32] = {0x00000000};

  if (length > 128 - (uint8_t)end_pos)
  {
    // message would overrun, fill current pages with zeros and go to next
    Flash_Write((uint32_t)&USER_DATA[page * 128], page_content);

    page++;
    if (page >= 32)
    {
      Flash_EraseSector();
      page = 0;
    }
    start_pos = 0;
  }
  else
  {
    start_pos = end_pos;
  }
  end_pos = start_pos + length;

  uint8_t *byte_writer = (uint8_t *)page_content;
  for (uint8_t i = 0; i < 128 - (uint8_t) start_pos; i++)
  {
    if (i < length)
    {
      byte_writer[start_pos + i] = content[i];
    }
    else
    {
      byte_writer[start_pos + i] = 0xFF;
    }
  }

  Flash_Write((uint32_t)&USER_DATA[page * 128], page_content);
};