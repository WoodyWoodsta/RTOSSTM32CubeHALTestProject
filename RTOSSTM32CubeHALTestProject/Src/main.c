/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

  /* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "cmsis_os.h"
#include "string.h"
#include <errno.h>
#include "sys/types.h"

  /**"Smart" implementation of the _sbrk() 
    *function for use with the newlib-nano library */
extern caddr_t _sbrk(int incr) {
  register char * stack_ptr asm("sp");
  extern char end asm("end");
  static char * heap_end;
  char * prev_heap_end;

  if (heap_end == NULL)
    heap_end = &end;

  prev_heap_end = heap_end;

  if (heap_end + incr > stack_ptr) {
    errno = ENOMEM;
    return (caddr_t) -1;
  }

  heap_end += incr;

  return (caddr_t) prev_heap_end;
}

/* Private variables ---------------------------------------------------------*/
osThreadId sendTaskHandle;
osThreadId receiveTaskHandle;

/* Flexible message struct implementation for messages */
typedef struct {
  uint32_t stringLength; // Length of the string
  char *string; // Pointer to string
} MSG_STRING_T;

// Define the memory pool and message queue
osPoolDef(msgStringPool, 9, MSG_STRING_T);
osPoolId msgStringPool;
osMessageQDef(msgStringQ, 8, MSG_STRING_T);
osMessageQId msgStringQ;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void sendTaskThread(void const * argument);
void receiveTaskThread(void const * argument);
MSG_STRING_T* msgStringStructAlloc(osPoolId mPool, uint32_t msgStringLength, char *msgString);
void msgStringStructFree(osPoolId mPool, MSG_STRING_T *msgStringStructPtr);

int main(void) {
  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  // Create the memory pool and message queue
  msgStringPool = osPoolCreate(osPool(msgStringPool));
  msgStringQ = osMessageCreate(osMessageQ(msgStringQ), NULL);

  /* Create the thread(s) */
  /* definition and creation of sendTask */
  osThreadDef(sendTask, sendTaskThread, osPriorityNormal, 1, 128);
  sendTaskHandle = osThreadCreate(osThread(sendTask), NULL);

  /* definition and creation of receiveTask */
  osThreadDef(receiveTask, receiveTaskThread, osPriorityNormal, 1, 128); // Seem to need more stack which makes sense
  receiveTaskHandle = osThreadCreate(osThread(receiveTask), NULL);

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  while (1) {
    __asm("nop");
  }

}

/** System Clock Configuration
*/
void SystemClock_Config(void) {

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void) {

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOF_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | 
                        GPIO_PIN_1 |
                        GPIO_PIN_2 |
                        GPIO_PIN_3 |
                        GPIO_PIN_4 |
                        GPIO_PIN_5 |
                        GPIO_PIN_6 |
                        GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* LED0TaskThread function */
void sendTaskThread(void const * argument) {
  MSG_STRING_T *msgSendPtr;
  char msgString[] = "Test string";

  /* Infinite loop */
  for (;;) {
    // Allocate memory for the message struct
    msgSendPtr = msgStringStructAlloc(msgStringPool, sizeof(msgString), msgString);
    osMessagePut(msgStringQ, (uint32_t) msgSendPtr, osWaitForever); // Send the message

    osDelay(500); // Wait for half a second
  }
}

/* LED1TaskThread function */
void receiveTaskThread(void const * argument) {
  MSG_STRING_T *msgReceivePtr;
  osEvent incomingEvent;
  char msgStringTest[] = "Test string";

  /* Infinite loop */
  for (;;) {
    incomingEvent = osMessageGet(msgStringQ, osWaitForever); // Wait for the message

    // If there is a message waiting in the cue
    if (incomingEvent.status == osEventMessage) {
      msgReceivePtr = incomingEvent.value.p; // Grab the sent pointer
      // If the random number is correct, display the message number
      if (strcmp(msgReceivePtr->string, msgStringTest) == 0) {
        GPIOB->ODR = 0b10101010;
      } else {
        GPIOB->ODR = 0b00011000;
      }

      size_t heapMem = xPortGetFreeHeapSize();
      msgStringStructFree(msgStringPool, msgReceivePtr); // Free the string and the pool block

      osDelay(300);
      GPIOB->ODR = 0b00000000;
    }
  }
  /* USER CODE END LED1TaskThread */
}

MSG_STRING_T* msgStringStructAlloc(osPoolId mPool, uint32_t msgStringLength, char *msgString) {
  MSG_STRING_T* msgStringStructPtr; // Create the struct pointer needed for the message

  msgStringStructPtr = osPoolAlloc(mPool);
  
  msgStringStructPtr->stringLength = msgStringLength; // Assign the length of the allocated memory space
  msgStringStructPtr->string = pvPortMalloc(msgStringLength); // Actually allocate the memory space

  strncpy(msgStringStructPtr->string, msgString, msgStringLength); // Copy into the new space, the string contents

  return msgStringStructPtr;
}

void msgStringStructFree(osPoolId mPool, MSG_STRING_T *msgStringStructPtr) {
  vPortFree(msgStringStructPtr->string); // Free the memory

  osPoolFree(mPool, msgStringStructPtr); // Free the message from the memory pool

  // TODO Add some sort of safety here, gee...
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */

}

#endif

/**
  * @}
  */

  /**
    * @}
  */

  /************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
