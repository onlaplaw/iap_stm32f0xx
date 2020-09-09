/**
  ******************************************************************************
  * @file    STM32F0xx_IAP/src/main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    29-May-2012
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/** @addtogroup STM32F0xx_IAP
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern pFunction Jump_To_Application;
extern uint32_t JumpAddress;
uint32_t Command_str;

/* Private function prototypes -----------------------------------------------*/
static void SetSysClock(void);
static void Key_Init(void);
static uint8_t Sample_Input(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t bState);
static void IAP_Init(void);

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
    /*!< At this stage the microcontroller clock setting is already configured, 
         this is done through SystemInit() function which is called from startup
         file (startup_stm32f0xx.s) before to branch to application main.
         To reconfigure the default setting of SystemInit() function, refer to
         system_stm32f0xx.c file
       */
    SetSysClock();
    /* Initialize Key Button mounted on STM320518-EVAL board */
    Key_Init();
    FLASH_Unlock();
    Command_str = Read_Flash_Data(0x08002800);
    /* Test if Key push-button on STM320518-EVAL Board is pressed */
    if((Sample_Input(GPIOF, GPIO_Pin_0, 0) == 0) || (Command_str == 0xFFFFFFFF))
    {
        Erase_One_FlashPage(0x08002800);

        /* If Key is pressed, execute the IAP driver in order to re-program the Flash */
        IAP_Init();
        
        /* Display main menu */
        Main_Menu();
    }
    else if(Command_str == 0)
    {/* Keep the user application running */

        /* Test if user code is programmed starting from address "APPLICATION_ADDRESS" */
        if (((*(__IO uint32_t*)APPLICATION_ADDRESS) & 0x2FFE0000 ) == 0x20000000)
        { 
          /* Jump to user application */
          JumpAddress = *(__IO uint32_t*) (APPLICATION_ADDRESS + 4);
          Jump_To_Application = (pFunction) JumpAddress;
          
          /* Initialize user application's Stack Pointer */
          __set_MSP(*(__IO uint32_t*) APPLICATION_ADDRESS);
          
          /* Jump to application */
          Jump_To_Application();
        }
    }
  
    /* Infinite loop */
    while (1)
    {
    }
}

/**
  * @brief  Configures the System clock frequency, AHB/APBx prescalers and Flash
  *         settings.
  * @note   This function should be called only once the RCC clock configuration
  *         is reset to the default reset state (done in SystemInit() function).
  * @param  None
  * @retval None
  */
void SetSysClock(void)
{
    __IO uint32_t StartUpCounter = 0, HSEStatus = 0;
    
    /* SYSCLK, HCLK, PCLK configuration ----------------------------------------*/
    /* Disable HSE */    
    RCC->CR &= ~((uint32_t)RCC_CR_HSEON);

    /* Enable Prefetch Buffer and set Flash Latency */
    FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;
 
    /* HCLK = SYSCLK */
    RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV1;
      
    /* PCLK = HCLK */
    RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE_DIV1;

    /* PLL configuration = HSE * 6 = 48 MHz */
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL));
    RCC->CFGR |= (uint32_t)(/*RCC_CFGR_PLLSRC_PREDIV1 | */RCC_CFGR_PLLXTPRE_PREDIV1 | RCC_CFGR_PLLMULL12);
    RCC_MCOConfig(RCC_MCOSource_HSI);

    /* Enable PLL */
    RCC->CR |= RCC_CR_PLLON;

    /* Wait till PLL is ready */
    while((RCC->CR & RCC_CR_PLLRDY) == 0)
    {
    }

    /* Select PLL as system clock source */
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
    RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;    

    /* Wait till PLL is used as system clock source */
    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)RCC_CFGR_SWS_PLL)
    {
    }
}

/**
  * @brief  Initialize the key.
  * @param  None
  * @retval None
  */
void Key_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable GPIO clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOF, &GPIO_InitStructure);
}

/**
  * @brief  key scan.
  * @param  GPIOx
  * @param  GPIO_Pin
  * @param  bState
  * @retval return 0 if state is true or return 1
  */
uint8_t Sample_Input(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t bState)
{
    uint8_t bRet;
    uint8_t bCount = 0;
    uint8_t bIndex;

    while(GPIO_ReadInputDataBit(GPIOx, GPIO_Pin) == bState)
    {
        bCount++;
        if(bCount > 50)
        {
            break;
        }
        for(bIndex = 0; bIndex < 50; bIndex++);
    }

    if(bCount > 50)
    {
        bRet = 0;
    }
    else
    {
        bRet = 1;
    }

    return bRet;
}

/**
  * @brief  Initialize the IAP.
  * @param  None
  * @retval None
  */
void IAP_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE );
    /* Unlock the Flash Program Erase controller */
    FLASH_If_Init();
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_0);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_0);      
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;                 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; 
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
    GPIO_Init(GPIOB, &GPIO_InitStructure);     
    /* USART resources configuration (Clock, GPIO pins and USART registers) ----*/
    /* USART configured as follow:
          - BaudRate = 115200 baud  
          - Word Length = 8 Bits
          - One Stop Bit
          - No parity
          - Hardware flow control disabled (RTS and CTS signals)
          - Receive and transmit enabled
    */
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
