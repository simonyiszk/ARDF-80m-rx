#include "py32f002ax5.h"

#include "py32f0xx_ll_gpio.h"
#include "py32f0xx_ll_tim.h"
#include "py32f0xx_ll_bus.h"
#include "py32f0xx_ll_rcc.h"
#include "py32f0xx_ll_utils.h"
#include "py32f0xx_ll_exti.h"
#include "py32f0xx_ll_pwr.h"
#include "py32f0xx_ll_cortex.h"
#include "py32f0xx_ll_usart.h"

#include <stdint.h>

#include "persist.h"

typedef enum
{
  CHAR_FETCH,
  SYMBOL_FETCH,
  CHAR_END,
  PLAY_MARK,
  PLAY_SPACE,
} MORSE_ENC_SM;

typedef enum
{
  NO,
  YES
} FLAG;

/* Code table starts from asmsg_in_indexi 48 ('0')
 * MSB after consecutive << shifts is the next mark: 1: dash, 0: dot
 * 3 LSB in demsg_in_indexmal indicates the character length
 */

const uint8_t code[] = {
    0b11111101, // 0 '-----'
    0b01111101, // 1 '.----'
    0b00111101, // 2 '..---'
    0b00011101, // 3 '...--'
    0b00001101, // 4 '....-'
    0b00000101, // 5 '.....'
    0b10000101, // 6 '-....'
    0b11000101, // 7 '--...'
    0b11100101, // 8 '---..'
    0b11110101, // 9 '----.'
    0, 0, 0, 0, // invalid characters (6 mark long symbols)
    0, 0, 0,
    0b01000010, // A '.-'
    0b10000100, // B '-...'
    0b10100100, // C '-.-.'
    0b10000011, // D '-..'
    0b00000001, // E '.'
    0b00100100, // F '..-.'
    0b11000011, // G '--.'
    0b00000100, // H '....'
    0b00000010, // I '..'
    0b01110100, // J '.---'
    0b10100011, // K '-.-'
    0b01000100, // L '.-..'
    0b11000010, // M '--'
    0b10000010, // N '-.'
    0b11100011, // O '---'
    0b01100100, // P '.--.'
    0b11010100, // Q '--.-'
    0b01000011, // R '.-.'
    0b00000011, // S '...'
    0b10000001, // T '-'
    0b00100011, // U '..-'
    0b00010100, // V '...-'
    0b01100011, // W '.--'
    0b10010100, // X '-..-'
    0b10110100, // Y '-.--'
    0b11000100  // Z '--..'
};

const char INVALID_CHARACTER = '?'; // has code of 0x00

static int32_t F_CLK = 8000000u;
const uint32_t PSC = 800UL; // Timer: 8MHZ / 800 = 10kHz => 1 tick = 100us
const uint32_t ARR = 800UL; // Overflow (UPDATE event) in every 200ms (PWM frequency is 5Hz)

uint8_t message[128] = {0};
uint32_t RX_idx = 0;
uint32_t TX_idx = 0;

volatile FLAG time_unit_eapsed = NO;
volatile FLAG message_incoming = NO;
volatile FLAG beacon_time = YES;

MORSE_ENC_SM encoder_state = CHAR_FETCH;
uint8_t sym_cnt = 0;
uint8_t play_cnt = 0;
volatile uint8_t chr = INVALID_CHARACTER;

static void APP_SystemClockConfig(void)
{
  LL_RCC_HSI_Enable();
  LL_RCC_HSI_SetCalibFreq(LL_RCC_HSICALIBRATION_8MHz);
  while (!LL_RCC_HSI_IsReady())
    ;

  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSISYS);
  while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSISYS)
    ;

  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_SetSystemCoreClock(F_CLK);
  LL_Init1msTick(F_CLK);
}

static void PeripheralConfig(void)
{
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM1); // Enable clock source for peripherals
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM16);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_USART1);

  LL_GPIO_InitTypeDef KeyerPinInit = {0}, UsartTXInit = {0}, UsartRXInit = {0}; // GPIO
  LL_TIM_InitTypeDef TIM1CountInit = {0};                                       // General TIM

  // Basic TIM setup
  TIM1CountInit.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM1CountInit.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM1CountInit.Prescaler = PSC - 1;
  TIM1CountInit.Autoreload = ARR - 1;
  TIM1CountInit.RepetitionCounter = 0;
  LL_TIM_Init(TIM1, &TIM1CountInit);
  LL_TIM_DisableCounter(TIM1);
  LL_TIM_ClearFlag_UPDATE(TIM1);

  // Tim 16 for 5 sec timing
  TIM1CountInit.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM1CountInit.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM1CountInit.Prescaler = 8000 - 1;
  TIM1CountInit.Autoreload = 5000 - 1;
  TIM1CountInit.RepetitionCounter = 0;
  LL_TIM_Init(TIM16, &TIM1CountInit);
  LL_TIM_DisableCounter(TIM16);
  LL_TIM_ClearFlag_UPDATE(TIM16);

  // GPIO Setup
  KeyerPinInit.Pin = LL_GPIO_PIN_1;
  KeyerPinInit.Mode = LL_GPIO_MODE_OUTPUT;
  KeyerPinInit.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  KeyerPinInit.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  KeyerPinInit.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &KeyerPinInit);
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_1);

  // USART setup
  LL_USART_SetBaudRate(USART1, SystemCoreClock, LL_USART_OVERSAMPLING_16, 9600);
  LL_USART_SetDataWidth(USART1, LL_USART_DATAWIDTH_8B);
  LL_USART_SetStopBitsLength(USART1, LL_USART_STOPBITS_1);
  LL_USART_SetParity(USART1, LL_USART_PARITY_NONE);
  LL_USART_SetHWFlowCtrl(USART1, LL_USART_HWCONTROL_NONE);
  LL_USART_SetTransferDirection(USART1, LL_USART_DIRECTION_TX_RX);
  LL_USART_Enable(USART1);
  LL_USART_ClearFlag_TC(USART1);

  UsartTXInit.Pin = LL_GPIO_PIN_10;
  UsartTXInit.Mode = LL_GPIO_MODE_ALTERNATE;
  UsartTXInit.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  UsartTXInit.Pull = LL_GPIO_PULL_NO;
  UsartTXInit.Alternate = LL_GPIO_AF8_USART1;
  LL_GPIO_Init(GPIOA, &UsartTXInit);

  UsartRXInit.Pin = LL_GPIO_PIN_3;
  UsartRXInit.Mode = LL_GPIO_MODE_ALTERNATE;
  UsartRXInit.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  UsartRXInit.Pull = LL_GPIO_PULL_NO;
  UsartRXInit.Alternate = LL_GPIO_AF1_USART1;
  LL_GPIO_Init(GPIOA, &UsartRXInit);

  // Enable interrupt requests
  LL_TIM_EnableIT_UPDATE(TIM1);
  NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);

  LL_TIM_EnableIT_UPDATE(TIM16);
  NVIC_EnableIRQ(TIM16_IRQn);

  LL_USART_EnableIT_RXNE(USART1);
  NVIC_EnableIRQ(USART1_IRQn);
}

// Timer period elapsed (counter overflow, update event)
void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
  if (LL_TIM_IsActiveFlag_UPDATE(TIM1) != 0)
  {
    time_unit_eapsed = YES;
    LL_TIM_ClearFlag_UPDATE(TIM1);
  }
}

void TIM16_IRQHandler(void)
{
  if (LL_TIM_IsActiveFlag_UPDATE(TIM16) != 0)
  {
    beacon_time = YES;
    LL_TIM_ClearFlag_UPDATE(TIM16);
  }
}

void USART1_IRQHandler(void)
{
  if (LL_USART_IsActiveFlag_RXNE(USART1) != 0)
  {
    message_incoming = YES;
    chr = LL_USART_ReceiveData8(USART1); // this also clears the rx flag
  }
}

void Transmit(volatile uint8_t *data, uint32_t size)
{
  for (uint32_t i = 0; i < size; i++)
  {
    LL_USART_TransmitData8(USART1, data[i]);
    while (!LL_USART_IsActiveFlag_TC(USART1))
      ;
    LL_USART_ClearFlag_TC(USART1);
  }
}

void Load_Message(void)
{
  RX_idx = Persist_Read(message);

  uint8_t content[128];
  for (uint8_t i = 0; i < RX_idx; i++)
  {
    if (message[i] < '0' || (message[i] > '9' && message[i] < 'A') || message[i] > 'Z')
    {
      content[i] = ' ';
    }
    else
    {
      content[i] = message[i];
    }
  }
  Transmit(content, RX_idx);
}

int main(void)
{

  APP_SystemClockConfig();
  PeripheralConfig();
  Persist_Init();

  Transmit((uint8_t*)"Type anything to set beacon message. Type '?' for help.\r\n", 57);
  Transmit((uint8_t*)"'>' symbol indicates ongoing transmission. During that, typing is disabled.\r\n", 77);

  while (1)
  {

    /* TRANSMIT TIME */

    if (beacon_time == YES)
    {
      beacon_time = NO;
      
      LL_USART_DisableIT_RXNE(USART1);

      Transmit((uint8_t *)">", 2);
      Load_Message();
      Transmit((uint8_t[4]){'\r', 0x1B, 0x5B, 0x43}, 4);      

      LL_TIM_SetCounter(TIM1, 0);
      LL_TIM_EnableCounter(TIM1);

      LL_TIM_SetCounter(TIM16, 0);
      LL_TIM_DisableCounter(TIM16);
    }

    /* NEW CHARACTER INCOMING */

    if (message_incoming == YES)
    {
      message_incoming = NO;

      LL_TIM_DisableCounter(TIM16);

      if (chr == '\r') // enter
      {
        Persist_Save(message, RX_idx);
        beacon_time = YES;
      }
      else if (chr == 127) // backspace
      {
        chr = 0;
        message[RX_idx] = 0;
        if (RX_idx > 0)
        {
          RX_idx--;
          Transmit((uint8_t[2]){8, ' '}, 2);
          chr = 8;
        }
      }
      else if ('a' <= chr && chr <= 'z') // to upercase
      {
        chr = chr - 'a' + 'A';
        message[RX_idx] = chr;
        RX_idx++;
      }
      else if (('A' <= chr && chr <= 'Z') || ('0' <= chr && chr <= '9')) // add if valid
      {
        message[RX_idx] = chr;
        RX_idx++;
      }
      else if (chr == '@') // system reset
      {
        NVIC_SystemReset();
      }
      else if (chr == ':') // load message from flash
      {
        Transmit((uint8_t*)"\r\n:", 3);
        Load_Message();
        chr = 0;
      }
      else if (chr == '?') // help
      {
        Transmit((uint8_t*)"****** HELP ******\r\n", 20);
        Transmit((uint8_t*)"? - Show this help\r\n", 20);
        Transmit((uint8_t*)": - Modify message\r\n", 20);
        Transmit((uint8_t*)"@ - System reset\r\n", 18);
        Transmit((uint8_t*)"= - Dump flash\r\n", 16);
        chr = 0;
      }
      else if (chr == '=') // dump flash contents
      {
        LL_USART_DisableIT_RXNE(USART1);

        Transmit((uint8_t *)"\r\n", 2);
        uint8_t content[128];
        for (uint8_t i = 0; i < 32; i++)
        {
          Persist_DumpPage(content, i);
          for (uint8_t j = 0; j < 128; j++)
          {
            if (content[j] == 0x00)
            {
              content[j] = '.';
            }
            else if (content[j] == 0xFF)
            {
              content[j] = '|';
            }
            else if (content[j] < '0' || (content[j] > '9' && content[j] < 'A') || content[j] > 'Z')
            {
              content[j] = ' ';
            }
          }
          Transmit(content, 128);
          Transmit((uint8_t *)"\r\n", 2);
        }

        Transmit(message, RX_idx);
        chr = 0;
        LL_USART_EnableIT_RXNE(USART1);
      }
      else // replace to space if invalid
      {
        chr = ' ';
        message[RX_idx] = INVALID_CHARACTER; // encoded as space
        RX_idx++;
      }

      Transmit(&chr, 1);
    }

    /* TIME UNIT ELAPSED */

    if (time_unit_eapsed == YES)
    {
      time_unit_eapsed = NO;

      if (TX_idx == RX_idx + 1) // all char sent
      {
        LL_TIM_DisableCounter(TIM1);
        while (LL_USART_IsActiveFlag_RXNE(USART1) != 0)
        {
          (void)LL_USART_ReceiveData8(USART1);
        }
        LL_USART_EnableIT_RXNE(USART1);
        
        LL_TIM_SetCounter(TIM16, 0);
        LL_TIM_EnableCounter(TIM16);
        
        LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_1);

        Transmit((uint8_t *)"\n\r", 2);

        encoder_state = CHAR_FETCH;
        play_cnt = 0;
        sym_cnt = 0;
        chr = INVALID_CHARACTER;
        RX_idx = 0;
        TX_idx = 0;
      }
      else // run state machine
      {
        switch (encoder_state)
        {
        case CHAR_FETCH:
          LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_1);

          chr = code[message[TX_idx] - '0'];
          TX_idx++;
          sym_cnt = chr & 0x07;

          if (0 == sym_cnt) // invalid character
          {
            play_cnt = 4;
            encoder_state = PLAY_SPACE;
          }
          else
          {
            encoder_state = SYMBOL_FETCH;
          }
          break;

        case SYMBOL_FETCH:
          LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_1);

          if ((chr & 0x80) != 0) // set dash
          {
            play_cnt = 3;
          }
          else // set dot
          {
            play_cnt = 1;
          }
          chr = chr << 1;
          sym_cnt--;

          encoder_state = PLAY_MARK;
          break;

        case PLAY_MARK:
          LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_1);
          play_cnt--;
          if (play_cnt == 0)
          {
            play_cnt = 1;
            encoder_state = PLAY_SPACE;
          }
          break;

        case PLAY_SPACE:
          LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_1);
          play_cnt--;
          if (0 == play_cnt)
          {
            if (0 == sym_cnt)
            {
              encoder_state = CHAR_END;
            }
            else
            {
              encoder_state = SYMBOL_FETCH;
            }
          }
          break;

        case CHAR_END:
          LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_1);

          Transmit((uint8_t[3]){0x1B, 0x5B, 0x43}, 3); // move cursor right

          encoder_state = CHAR_FETCH;
          break;

        default:
          break;
        }
      }
    }
  }
}