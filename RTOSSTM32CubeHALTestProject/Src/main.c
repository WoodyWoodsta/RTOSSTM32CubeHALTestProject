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
#include "stdlib.h"

/* Private variables ---------------------------------------------------------*/
osThreadId sendTaskHandle;
osThreadId receiveTaskHandle;
/* Flexible message struct implementation for messages */
//#define msg_st_t struct msg_st*
//#define msg_st_t_Size (sizeof(uint32_t) + sizeof(char))
//struct msg_st {
//  uint32_t msgStringLength; // Length of the string in the message
//  char msgString[]; // Message string
//};

typedef struct {
  uint8_t integer;
  uint8_t integer2;
} T_MEAS;
#define T_MEAS_SIZE sizeof(T_MEAS)

// Define the memory pool and message queue
osPoolDef(msgPool, 16, T_MEAS);
osPoolId msgPool;
osMessageQDef(msgQ, 8, T_MEAS);
osMessageQId msgQ;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void sendTaskThread(void const * argument);
void receiveTaskThread(void const * argument);
struct msg_st* msgStAlloc(uint32_t msgStringLength);

int main(void) {
  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  // Create the memory pool and message queue
  msgPool = osPoolCreate(osPool(msgPool));
  msgQ = osMessageCreate(osMessageQ(msgQ), NULL);

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
  uint32_t msgCount = 1; // Count the number of messages sent

  /* Infinite loop */
  for (;;) {
    T_MEAS *msgPtr;
    // Allocate memory for the message struct
    msgPtr = osPoolAlloc(msgPool);
    msgPtr->integer = msgCount++;
    if (msgCount >= 255) {
      msgCount = 1;
    }

    msgPtr->integer2 = 5; // Random number
    osMessagePut(msgQ, (uint32_t) msgPtr, osWaitForever); // Send the message

    osDelay(100); // Wait for half a second
  }
}

/* LED1TaskThread function */
void receiveTaskThread(void const * argument) {
  T_MEAS *receivePtr;
  osEvent incomingEvent;

  /* Infinite loop */
  for (;;) {
    incomingEvent = osMessageGet(msgQ, osWaitForever); // Wait for the message
    // If there is a message waiting in the cue
    if (incomingEvent.status == osEventMessage) {
      receivePtr = incomingEvent.value.p; // Grab the sent pointer
      // If the random number is correct, display the message number
      if (receivePtr->integer2 == 5) {
        GPIOB->ODR = receivePtr->integer;
      }
      osPoolFree(msgPool, receivePtr); // Free the previously allocated memory
    }
  }
  /* USER CODE END LED1TaskThread */
}

//struct msg_st* msgStructAlloc(uint32_t msgStringLength) {
//  // Allocate the flexible array method
//  struct msg_st* vec = malloc(sizeof(struct msg_st) + (msgStringLength * sizeof(uint32_t)));
//
//  // Handle any errors
//  if (!vec) {
//    __asm("nop"); // TODO Handle some errors
//  } else
//    // Set the length of the string in the new struct
//    vec->msgStringLength = msgStringLength;
//
//  // Zero-fill string array
//  for (uint32_t ix = 0; ix < msgStringLength; ix++)
//    vec->msgString[ix] = 0;
//
//  return vec;
//}


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
