/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32mp1xx_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
extern DMA_HandleTypeDef hdma_cryp2_out;

extern DMA_HandleTypeDef hdma_cryp2_in;

extern DMA_HandleTypeDef hdma_hash2_in;

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{
  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_HSEM_CLK_ENABLE();

  /* System interrupt init*/
  /* MemoryManagement_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(MemoryManagement_IRQn, 1, 0);
  /* BusFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(BusFault_IRQn, 1, 0);
  /* UsageFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(UsageFault_IRQn, 1, 0);
  /* SVCall_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SVCall_IRQn, 1, 0);
  /* DebugMonitor_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DebugMonitor_IRQn, 1, 0);
  /* PendSV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(PendSV_IRQn, 1, 0);

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

/**
* @brief ADC MSP Initialization
* This function configures the hardware resources used in this example
* @param hadc: ADC handle pointer
* @retval None
*/
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hadc->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_ADC12_CLK_ENABLE();

    __HAL_RCC_GPIOF_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PF12     ------> ADC1_INP6
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    /* ADC1 interrupt Init */
    HAL_NVIC_SetPriority(ADC1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(ADC1_IRQn);
  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }

}

/**
* @brief ADC MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hadc: ADC handle pointer
* @retval None
*/
void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{
  if(hadc->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC12_CLK_DISABLE();

    /**ADC1 GPIO Configuration
    PF12     ------> ADC1_INP6
    */
    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_12);

    /* ADC1 interrupt DeInit */
    HAL_NVIC_DisableIRQ(ADC1_IRQn);
  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }

}

/**
* @brief CRC MSP Initialization
* This function configures the hardware resources used in this example
* @param hcrc: CRC handle pointer
* @retval None
*/
void HAL_CRC_MspInit(CRC_HandleTypeDef* hcrc)
{
  if(hcrc->Instance==CRC2)
  {
  /* USER CODE BEGIN CRC2_MspInit 0 */

  /* USER CODE END CRC2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_CRC2_CLK_ENABLE();
  /* USER CODE BEGIN CRC2_MspInit 1 */

  /* USER CODE END CRC2_MspInit 1 */
  }

}

/**
* @brief CRC MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hcrc: CRC handle pointer
* @retval None
*/
void HAL_CRC_MspDeInit(CRC_HandleTypeDef* hcrc)
{
  if(hcrc->Instance==CRC2)
  {
  /* USER CODE BEGIN CRC2_MspDeInit 0 */

  /* USER CODE END CRC2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CRC2_CLK_DISABLE();
  /* USER CODE BEGIN CRC2_MspDeInit 1 */

  /* USER CODE END CRC2_MspDeInit 1 */
  }

}

/**
* @brief CRYP MSP Initialization
* This function configures the hardware resources used in this example
* @param hcryp: CRYP handle pointer
* @retval None
*/
void HAL_CRYP_MspInit(CRYP_HandleTypeDef* hcryp)
{
  if(hcryp->Instance==CRYP2)
  {
  /* USER CODE BEGIN CRYP2_MspInit 0 */

  /* USER CODE END CRYP2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_CRYP2_CLK_ENABLE();

    /* CRYP2 DMA Init */
    /* CRYP2_OUT Init */
    hdma_cryp2_out.Instance = DMA2_Stream0;
    hdma_cryp2_out.Init.Request = DMA_REQUEST_CRYP2_OUT;
    hdma_cryp2_out.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_cryp2_out.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_cryp2_out.Init.MemInc = DMA_MINC_ENABLE;
    hdma_cryp2_out.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_cryp2_out.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_cryp2_out.Init.Mode = DMA_NORMAL;
    hdma_cryp2_out.Init.Priority = DMA_PRIORITY_LOW;
    hdma_cryp2_out.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_cryp2_out) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hcryp,hdmaout,hdma_cryp2_out);

    /* CRYP2_IN Init */
    hdma_cryp2_in.Instance = DMA2_Stream1;
    hdma_cryp2_in.Init.Request = DMA_REQUEST_CRYP2_IN;
    hdma_cryp2_in.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_cryp2_in.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_cryp2_in.Init.MemInc = DMA_MINC_ENABLE;
    hdma_cryp2_in.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_cryp2_in.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_cryp2_in.Init.Mode = DMA_NORMAL;
    hdma_cryp2_in.Init.Priority = DMA_PRIORITY_LOW;
    hdma_cryp2_in.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_cryp2_in) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hcryp,hdmain,hdma_cryp2_in);

  /* USER CODE BEGIN CRYP2_MspInit 1 */

  /* USER CODE END CRYP2_MspInit 1 */
  }

}

/**
* @brief CRYP MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hcryp: CRYP handle pointer
* @retval None
*/
void HAL_CRYP_MspDeInit(CRYP_HandleTypeDef* hcryp)
{
  if(hcryp->Instance==CRYP2)
  {
  /* USER CODE BEGIN CRYP2_MspDeInit 0 */

  /* USER CODE END CRYP2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CRYP2_CLK_DISABLE();

    /* CRYP2 DMA DeInit */
    HAL_DMA_DeInit(hcryp->hdmaout);
    HAL_DMA_DeInit(hcryp->hdmain);
  /* USER CODE BEGIN CRYP2_MspDeInit 1 */

  /* USER CODE END CRYP2_MspDeInit 1 */
  }

}

/**
* @brief DAC MSP Initialization
* This function configures the hardware resources used in this example
* @param hdac: DAC handle pointer
* @retval None
*/
void HAL_DAC_MspInit(DAC_HandleTypeDef* hdac)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hdac->Instance==DAC1)
  {
  /* USER CODE BEGIN DAC1_MspInit 0 */

  /* USER CODE END DAC1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_DAC12_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**DAC1 GPIO Configuration
    PA4     ------> DAC1_OUT1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN DAC1_MspInit 1 */

  /* USER CODE END DAC1_MspInit 1 */
  }

}

/**
* @brief DAC MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hdac: DAC handle pointer
* @retval None
*/
void HAL_DAC_MspDeInit(DAC_HandleTypeDef* hdac)
{
  if(hdac->Instance==DAC1)
  {
  /* USER CODE BEGIN DAC1_MspDeInit 0 */

  /* USER CODE END DAC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_DAC12_CLK_DISABLE();

    /**DAC1 GPIO Configuration
    PA4     ------> DAC1_OUT1
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4);

  /* USER CODE BEGIN DAC1_MspDeInit 1 */

  /* USER CODE END DAC1_MspDeInit 1 */
  }

}

/**
* @brief HASH MSP Initialization
* This function configures the hardware resources used in this example
* @param hhash: HASH handle pointer
* @retval None
*/
void HAL_HASH_MspInit(HASH_HandleTypeDef* hhash)
{
  /* USER CODE BEGIN HASH2_MspInit 0 */

  /* USER CODE END HASH2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_HASH2_CLK_ENABLE();

    /* HASH2 DMA Init */
    /* HASH2_IN Init */
    hdma_hash2_in.Instance = DMA2_Stream2;
    hdma_hash2_in.Init.Request = DMA_REQUEST_HASH2_IN;
    hdma_hash2_in.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_hash2_in.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_hash2_in.Init.MemInc = DMA_MINC_ENABLE;
    hdma_hash2_in.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_hash2_in.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_hash2_in.Init.Mode = DMA_NORMAL;
    hdma_hash2_in.Init.Priority = DMA_PRIORITY_LOW;
    hdma_hash2_in.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_hash2_in.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
    hdma_hash2_in.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_hash2_in.Init.PeriphBurst = DMA_PBURST_SINGLE;
    // TAKE CARE: not in user code section but required
    if (HAL_DMA_DeInit(&hdma_hash2_in) != HAL_OK)
    {
      Error_Handler();
    }
    // END
    if (HAL_DMA_Init(&hdma_hash2_in) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hhash,hdmain,hdma_hash2_in);

  /* USER CODE BEGIN HASH2_MspInit 1 */

  /* USER CODE END HASH2_MspInit 1 */

}

/**
* @brief HASH MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hhash: HASH handle pointer
* @retval None
*/
void HAL_HASH_MspDeInit(HASH_HandleTypeDef* hhash)
{
  /* USER CODE BEGIN HASH2_MspDeInit 0 */

  /* USER CODE END HASH2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_HASH2_CLK_DISABLE();

    /* HASH2 DMA DeInit */
    HAL_DMA_DeInit(hhash->hdmain);
  /* USER CODE BEGIN HASH2_MspDeInit 1 */

  /* USER CODE END HASH2_MspDeInit 1 */

}

/**
* @brief IPCC MSP Initialization
* This function configures the hardware resources used in this example
* @param hipcc: IPCC handle pointer
* @retval None
*/
void HAL_IPCC_MspInit(IPCC_HandleTypeDef* hipcc)
{
  if(hipcc->Instance==IPCC)
  {
  /* USER CODE BEGIN IPCC_MspInit 0 */

  /* USER CODE END IPCC_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_IPCC_CLK_ENABLE();
    /* IPCC interrupt Init */
    HAL_NVIC_SetPriority(IPCC_RX1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(IPCC_RX1_IRQn);
    HAL_NVIC_SetPriority(IPCC_TX1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(IPCC_TX1_IRQn);
  /* USER CODE BEGIN IPCC_MspInit 1 */

  /* USER CODE END IPCC_MspInit 1 */
  }

}

/**
* @brief IPCC MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hipcc: IPCC handle pointer
* @retval None
*/
void HAL_IPCC_MspDeInit(IPCC_HandleTypeDef* hipcc)
{
  if(hipcc->Instance==IPCC)
  {
  /* USER CODE BEGIN IPCC_MspDeInit 0 */

  /* USER CODE END IPCC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_IPCC_CLK_DISABLE();

    /* IPCC interrupt DeInit */
    HAL_NVIC_DisableIRQ(IPCC_RX1_IRQn);
    HAL_NVIC_DisableIRQ(IPCC_TX1_IRQn);
  /* USER CODE BEGIN IPCC_MspDeInit 1 */

  /* USER CODE END IPCC_MspDeInit 1 */
  }

}

/**
* @brief TIM_Base MSP Initialization
* This function configures the hardware resources used in this example
* @param htim_base: TIM_Base handle pointer
* @retval None
*/
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspInit 0 */

  /* USER CODE END TIM2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();
    /* TIM2 interrupt Init */
    HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* USER CODE BEGIN TIM2_MspInit 1 */

  /* USER CODE END TIM2_MspInit 1 */
  }
  else if(htim_base->Instance==TIM7)
  {
  /* USER CODE BEGIN TIM7_MspInit 0 */

  /* USER CODE END TIM7_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM7_CLK_ENABLE();
    /* TIM7 interrupt Init */
    HAL_NVIC_SetPriority(TIM7_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM7_IRQn);
  /* USER CODE BEGIN TIM7_MspInit 1 */

  /* USER CODE END TIM7_MspInit 1 */
  }

}

/**
* @brief TIM_Base MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param htim_base: TIM_Base handle pointer
* @retval None
*/
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspDeInit 0 */

  /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();

    /* TIM2 interrupt DeInit */
    HAL_NVIC_DisableIRQ(TIM2_IRQn);
  /* USER CODE BEGIN TIM2_MspDeInit 1 */

  /* USER CODE END TIM2_MspDeInit 1 */
  }
  else if(htim_base->Instance==TIM7)
  {
  /* USER CODE BEGIN TIM7_MspDeInit 0 */

  /* USER CODE END TIM7_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM7_CLK_DISABLE();

    /* TIM7 interrupt DeInit */
    HAL_NVIC_DisableIRQ(TIM7_IRQn);
  /* USER CODE BEGIN TIM7_MspDeInit 1 */

  /* USER CODE END TIM7_MspDeInit 1 */
  }

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
