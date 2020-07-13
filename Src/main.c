/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
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
#include "main.h"
#include "stm32f7xx_hal.h"
#include "goose_publisher.h"
#include "sv_subscriber.h"
#include "goose_subscriber.h"
#include "goose_receiver.h"
#include <string.h>
#include <math.h>

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

ETH_HandleTypeDef heth;

TIM_HandleTypeDef htim10;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
__ALIGN_BEGIN ETH_DMADescTypeDef  DMARxDscrTab[ETH_RXBUFNB] __ALIGN_END;/* Ethernet Rx MA Descriptor */
__ALIGN_BEGIN ETH_DMADescTypeDef  DMATxDscrTab[ETH_TXBUFNB] __ALIGN_END;/* Ethernet Tx DMA Descriptor */
__ALIGN_BEGIN uint8_t Rx_Buff[ETH_RXBUFNB][ETH_RX_BUF_SIZE] __ALIGN_END; /* Ethernet Receive Buffer */
__ALIGN_BEGIN uint8_t Tx_Buff[ETH_TXBUFNB][ETH_TX_BUF_SIZE] __ALIGN_END; /* Ethernet Transmit Buffer */
__IO ETH_DMADescTypeDef *dmarxdesc;

uint8_t srcMAC[6] = { 0x01 , 0x0C , 0xCD , 0x04 , 0x00 , 0x01 };

// Variables for receiving frames
uint16_t len;
uint8_t *buffer;

// Bit indicating frame reception to enter SV data unwrap and DFT
char ReceptionFlag = 0;
char Downsampler = 1;

// Value obtained in every SV frame
float v[3];
float i[3];

// Size of a sample buffer
uint8_t sampleBufferSize = 20;

// Sample buffer
float sBufferVA[20];
float sBufferVB[20];
float sBufferVC[20];
float sBufferIA[20];
float sBufferIB[20];
float sBufferIC[20];

// Sine and Cosine DFT filters & filters for LES
float filterCos[20];
float filterSin[20];

//recursive DFT vars
float vPhasor[6];
float iPhasor[6];
uint8_t filtCounter = 0;

//voltage reference phasor with counter to store its value
float vPhasorRef[2];
uint8_t refStoreCnt;

//distance function aux variables
char *distanceElements;

#define DELTA_T 0.001
#define PI 3.1416

//Goose message vars
LinkedList dataSetValues;
GoosePublisher publisher;
_Bool test = true;
int32_t testTest = 1;
char gooseBuf[256];

// Used for measurement and calc output
uint16_t counterStep;
float archive[800];
uint16_t archiveCounter;
uint32_t timCnt;

// Other test variables
float cplxTest[2];
float *cplxTestStart;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM10_Init(void);
static void MX_ETH_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void ETH_SendBuffer_Init(void);
static void svUpdateListener(SVSubscriber subscriber, void* parameter, SVClientASDU asdu);
static void gooseListener(GooseSubscriber subscriber, void* parameter);
void receiveFrame(void);
void dataSaver(void);
void DFT_Recursive(float *v, float *i);
char *distanceFunction(float *vPh, float *iPh);
float *cplxMult(float re1, float im1, float re2, float im2);
float *cplxDiv(float re1, float im1, float re2, float im2);
void updateVoltRef(void);
LinkedList preparePayload(_Bool tripA, _Bool tripB, _Bool tripC);
GoosePublisher preparePublisher(void);
void orthFilterInit(uint8_t SampleBufferSize)
{
  for(uint8_t i = 0; i < SampleBufferSize; i++)
  {
    filterCos[i] = cos(2 * 3.14159 * i / (float)SampleBufferSize);
    filterSin[i] = sin(2 * 3.14159 * i / (float)SampleBufferSize);
  }
}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM10_Init();
  MX_ETH_Init();

  /* USER CODE BEGIN 2 */
  ETH_SendBuffer_Init();

  orthFilterInit(sampleBufferSize);

  dataSetValues = preparePayload(true, true, true);
  publisher = preparePublisher();

  SVReceiver receiverSV = SVReceiver_create();
  SVSubscriber subscriberSV = SVSubscriber_create(srcMAC, 0x4000);

  GooseReceiver receiverGoose = GooseReceiver_create();
  GooseSubscriber subscriberGoose = GooseSubscriber_create("AA1J1Bay1A1LD0/LLN0$GO$gcbTRIP", NULL);

  SVSubscriber_setListener(subscriberSV, svUpdateListener, NULL);
  SVReceiver_addSubscriber(receiverSV, subscriberSV);

  GooseSubscriber_setAppId(subscriberGoose, 0x0001);
  GooseSubscriber_setListener(subscriberGoose, gooseListener, NULL);
  GooseReceiver_addSubscriber(receiverGoose, subscriberGoose);

  HAL_Delay(200);

  // Send a GOOSE message
  GoosePublisher_publish(publisher, heth, dataSetValues);
  // Free the memory from a GOOSE publisher instance
  GoosePublisher_destroy(publisher);

  HAL_GPIO_WritePin(GPIOI, GPIO_PIN_1, GPIO_PIN_SET);
  HAL_TIM_Base_Start(&htim10);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if(HAL_ETH_GetReceivedFrame(&heth) == HAL_OK)
	  {
		receiveFrame();
		/* Function invoking sequence of unpacking received frame */
		SVReceiver_tick(receiverSV, buffer, len);
		GooseReceiver_tick(receiverGoose, buffer, len);
	  }

	  if(ReceptionFlag && Downsampler == 0)
	  {
		__HAL_TIM_SET_COUNTER(&htim10,0);
		/* Estimation algorithms */
		DFT_Recursive(v, i);
		/* Protection logic */
		distanceElements = distanceFunction(vPhasor, iPhasor);
		/* Update reference phasor reference */
		updateVoltRef();
		/* TESTTESTTEST */

		/* Data acquisition */
		counterStep++;
		dataSaver();
		/* Send GOOSE Trip */
		//TripSignal(&heth);
		/* Put the reception flag down */
		ReceptionFlag = 0;
		timCnt = __HAL_TIM_GET_COUNTER(&htim10);
		Downsampler = 4;
	  }


  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ETH init function */
static void MX_ETH_Init(void)
{

   uint8_t MACAddr[6] ;

  heth.Instance = ETH;
  heth.Init.AutoNegotiation = ETH_AUTONEGOTIATION_ENABLE;
  heth.Init.PhyAddress = LAN8742A_PHY_ADDRESS;
  MACAddr[0] = 0x01;
  MACAddr[1] = 0x0C;
  MACAddr[2] = 0xCD;
  MACAddr[3] = 0x04;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x01;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.RxMode = ETH_RXPOLLING_MODE;
  heth.Init.ChecksumMode = ETH_CHECKSUM_BY_HARDWARE;
  heth.Init.MediaInterface = ETH_MEDIA_INTERFACE_RMII;

  /* USER CODE BEGIN MACADDRESS */
  // Mitigate CubeMX bug by setting PHY Init to 0
  heth.Init.PhyAddress = 0;
  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM10 init function */
static void MX_TIM10_Init(void)
{

  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 0;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 65535;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOI, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PI0 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
static void ETH_SendBuffer_Init(void)
{
	  /* Initialize Tx Descriptors list: Chain Mode */
	  HAL_ETH_DMATxDescListInit(&heth, DMATxDscrTab, &Tx_Buff[0][0], ETH_TXBUFNB);
	  /* Initialize Rx Descriptors list: Chain Mode  */
	  HAL_ETH_DMARxDescListInit(&heth, DMARxDscrTab, &Rx_Buff[0][0], ETH_RXBUFNB);

	  /* Start ETH periph */
	  HAL_ETH_Start(&heth);
}
static void svUpdateListener(SVSubscriber subscriber, void* parameter, SVClientASDU asdu)
{

  int32_t value = SVClientASDU_getINT32(asdu, 32);
  v[0] = (float)value/100.0;

  int32_t value1 = SVClientASDU_getINT32(asdu, 40);
  v[1] = (float)value1/100 ;

  int32_t value2 = SVClientASDU_getINT32(asdu, 48);
  v[2] = (float)value2/100;

  int32_t value_3 = SVClientASDU_getINT32(asdu, 0);
  i[0] = (float)value_3/1000.0;

  int32_t value_4 = SVClientASDU_getINT32(asdu, 8);
  i[1] = (float)value_4/1000.0;

  int32_t value_5 = SVClientASDU_getINT32(asdu, 16);
  i[2] = (float)value_5/1000.0;

  ReceptionFlag = 1;
  Downsampler -= 1;

}
static void gooseListener(GooseSubscriber subscriber, void* parameter)
{
	MmsValue* values = GooseSubscriber_getDataSetValues(subscriber);
	MmsValue* intElement = MmsValue_getElement(values, 5);

	//MmsValue_printToBuffer(subscriber, gooseBuf, 256);
	testTest = MmsValue_toInt32(intElement);

	test = false;
}
void receiveFrame(void)
{
  len = heth.RxFrameInfos.length;
  buffer = (uint8_t *)heth.RxFrameInfos.buffer;
  dmarxdesc = heth.RxFrameInfos.FSRxDesc;
  for (uint32_t i=0; i< heth.RxFrameInfos.SegCount; i++)
  {
    dmarxdesc->Status |= ETH_DMARXDESC_OWN;
    dmarxdesc = (ETH_DMADescTypeDef *)(dmarxdesc->Buffer2NextDescAddr);
  }
  /* Clear Segment_Count */
  heth.RxFrameInfos.SegCount =0;

  /* When Rx Buffer unavailable flag is set: clear it and resume reception */
  if ((heth.Instance->DMASR & ETH_DMASR_RBUS) != (uint32_t)RESET)
  {
    /* Clear RBUS ETHERNET DMA flag */
    heth.Instance->DMASR = ETH_DMASR_RBUS;
    /* Resume DMA reception */
    heth.Instance->DMARPDR = 0;
  }
  /*
  if(buffer[0] == 0x01)
  {
    ReceptionFlag = 1;
    Downsampler -= 1;
  }
  */
}
GoosePublisher preparePublisher(void)
{
	// Communication params setup
	CommParameters gooseCommParameters;

	gooseCommParameters.appId = 0x0001;
	gooseCommParameters.dstAddress[0] = 0x01;
	gooseCommParameters.dstAddress[1] = 0x0C;
	gooseCommParameters.dstAddress[2] = 0xCD;
	gooseCommParameters.dstAddress[3] = 0x01;
	gooseCommParameters.dstAddress[4] = 0x28;
	gooseCommParameters.dstAddress[5] = 0x50;
	gooseCommParameters.vlanId = 0x00A;
	gooseCommParameters.vlanPriority = 4;

	// Goose instance creation
	GoosePublisher publisher = GoosePublisher_create(&gooseCommParameters, NULL);

	GoosePublisher_setGoCbRef(publisher, "BOARD/LLN0$GO$Goose_TRIP1");
	GoosePublisher_setConfRev(publisher, 1);
	GoosePublisher_setTimeAllowedToLive(publisher, 500);
	GoosePublisher_setDataSetRef(publisher, "BOARD/LLN0$Goose_TRIP1");
	GoosePublisher_setGoID(publisher, "Goose_TRIP1");

	return publisher;
}
LinkedList preparePayload(_Bool tripA, _Bool tripB, _Bool tripC)
{
  LinkedList dataSetValues = LinkedList_create();

  LinkedList_add(dataSetValues, MmsValue_newBoolean(tripA));
  LinkedList_add(dataSetValues, MmsValue_newBoolean(tripB));
  LinkedList_add(dataSetValues, MmsValue_newBoolean(tripC));

  return dataSetValues;
}
void DFT_Recursive(float *v, float *i)
{

	if(filtCounter > sampleBufferSize-1) filtCounter = 0;

	vPhasor[0] = vPhasor[0] + 2/(float)sampleBufferSize*(v[0]*filterCos[filtCounter] - sBufferVA[0]*filterCos[filtCounter]);
	vPhasor[1] = vPhasor[1] + 2/(float)sampleBufferSize*(v[0]*filterSin[filtCounter] - sBufferVA[0]*filterSin[filtCounter]);

	vPhasor[2] = vPhasor[2] + 2/(float)sampleBufferSize*(v[1]*filterCos[filtCounter] - sBufferVB[0]*filterCos[filtCounter]);
	vPhasor[3] = vPhasor[3] + 2/(float)sampleBufferSize*(v[1]*filterSin[filtCounter] - sBufferVB[0]*filterSin[filtCounter]);

	vPhasor[4] = vPhasor[4] + 2/(float)sampleBufferSize*(v[2]*filterCos[filtCounter] - sBufferVC[0]*filterCos[filtCounter]);
	vPhasor[5] = vPhasor[5] + 2/(float)sampleBufferSize*(v[2]*filterSin[filtCounter] - sBufferVC[0]*filterSin[filtCounter]);

	iPhasor[0] = iPhasor[0] + 2/(float)sampleBufferSize*(i[0]*filterCos[filtCounter] - sBufferIA[0]*filterCos[filtCounter]);
	iPhasor[1] = iPhasor[1] + 2/(float)sampleBufferSize*(i[0]*filterSin[filtCounter] - sBufferIA[0]*filterSin[filtCounter]);

	iPhasor[2] = iPhasor[2] + 2/(float)sampleBufferSize*(i[1]*filterCos[filtCounter] - sBufferIB[0]*filterCos[filtCounter]);
	iPhasor[3] = iPhasor[3] + 2/(float)sampleBufferSize*(i[1]*filterSin[filtCounter] - sBufferIB[0]*filterSin[filtCounter]);

	iPhasor[4] = iPhasor[4] + 2/(float)sampleBufferSize*(i[2]*filterCos[filtCounter] - sBufferIC[0]*filterCos[filtCounter]);
	iPhasor[5] = iPhasor[5] + 2/(float)sampleBufferSize*(i[2]*filterSin[filtCounter] - sBufferIC[0]*filterSin[filtCounter]);

	for(uint8_t i=1;i<sampleBufferSize;i++)
	{
		sBufferVA[i-1] = sBufferVA[i];
		sBufferVB[i-1] = sBufferVB[i];
		sBufferVC[i-1] = sBufferVC[i];

		sBufferIA[i-1] = sBufferIA[i];
		sBufferIB[i-1] = sBufferIB[i];
		sBufferIC[i-1] = sBufferIC[i];
	}

	sBufferVA[sampleBufferSize-1] = v[0];
	sBufferVB[sampleBufferSize-1] = v[1];
	sBufferVC[sampleBufferSize-1] = v[2];

	sBufferIA[sampleBufferSize-1] = i[0];
	sBufferIB[sampleBufferSize-1] = i[1];
	sBufferIC[sampleBufferSize-1] = i[2];

	filtCounter++;

}
char *distanceFunction(float *vPh, float *iPh)
{
	// local vars holding calculated values of phase to ground loops
	float phToGnd[6];
	float phToPh[6];
	float *p;
	static char loops[6] = {0,0,0,0,0,0};

	// phase to ground loops
	p = cplxDiv(vPh[0], vPh[1], iPh[0], iPh[1]);
	phToGnd[0] = *p;
	phToGnd[1] = *(p + 1);

	p = cplxDiv(vPh[2], vPh[3], iPh[2], iPh[3]);
	phToGnd[2] = *p;
	phToGnd[3] = *(p + 1);

	p = cplxDiv(vPh[4], vPh[5], iPh[4], iPh[5]);
	phToGnd[4] = *p;
	phToGnd[5] = *(p + 1);

	if(phToGnd[0]>0 && phToGnd[0]<1 && phToGnd[1]>-0.1 && phToGnd[1]<0.1)
	{
		loops[0] = 1;
	}
	if(phToGnd[2]>0 && phToGnd[2]<1 && phToGnd[3]>-0.1 && phToGnd[3]<0.1)
	{
		loops[1] = 1;
	}
	if(phToGnd[4]>0 && phToGnd[4]<1 && phToGnd[5]>-0.1 && phToGnd[5]<0.1)
	{
		loops[2] = 1;
	}

	// phase to phase loops
	p = cplxDiv(vPh[0]-vPh[2], vPh[1]-vPh[3], iPh[0]-iPh[2], iPh[1]-iPh[3]);
	phToPh[0] = *p;
	phToPh[1] = *(p + 1);

	p = cplxDiv(vPh[2]-iPh[4], vPh[3]-iPh[5], iPh[2]-iPh[4], iPh[3]-iPh[5]);
	phToPh[2] = *p;
	phToPh[3] = *(p + 1);

	p = cplxDiv(vPh[4]-vPh[0], vPh[5]-vPh[1], iPh[4]-iPh[0], iPh[5]-iPh[1]);
	phToPh[4] = *p;
	phToPh[5] = *(p + 1);

	if(phToPh[0]>0 && phToPh[0]<1 && phToPh[1]>-0.1 && phToPh[1]<0.1)
	{
		loops[3] = 1;
	}
	if(phToPh[2]>0 && phToPh[2]<1 && phToPh[3]>-0.1 && phToPh[3]<0.1)
	{
		loops[4] = 1;
	}
	if(phToPh[4]>0 && phToPh[4]<1 && phToPh[5]>-0.1 && phToPh[5]<0.1)
	{
		loops[5] = 1;
	}

	return loops;
}
void updateVoltRef(void)
{
	if(refStoreCnt++ == 40)
	{
		vPhasorRef[0] = vPhasor[0];
		vPhasorRef[1] = vPhasor[1];
		refStoreCnt = 0;
	}

}
float *cplxMult(float re1, float im1, float re2, float im2)
{
	static float res[2];

	res[0] = re1*re2 - im1*im2;
	res[1] = re1*im2 + im1*re2;

	return res;
}
float *cplxDiv(float re1, float im1, float re2, float im2)
{
	static float res[2];
	float denom = (re2*re2 + im2*im2);

	res[0] = (re1*re2 + im1*im2) / denom;
	res[1] = (im1*re2 - re1*im2) / denom;

	return res;
}
void dataSaver(void)
{
    if (counterStep > 1900 && counterStep <= 2900)
    {
      archive[archiveCounter] = sqrt(vPhasor[4]*vPhasor[4]+vPhasor[5]*vPhasor[5]);
      archiveCounter++;
    }
    if (archiveCounter == 800)
    {
      archiveCounter = 0;
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
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
