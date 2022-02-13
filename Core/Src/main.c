/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dfsdm.h"
#include "dma.h"
#include "icache.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "arm_math.h"
#include "sk6812.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define 	DO_FFT

#define	FFT_LENGTH			512
#define HALF_FFT_LENGTH		256
#define	BINSIZE				8
#define DECIMATED_LENGTH	HALF_FFT_LENGTH/BINSIZE

#define HALF_BUFFER_SIZE	FFT_LENGTH
#define BUFFER_SIZE			2*FFT_LENGTH
#define OUTPUT_BUFFER_SIZE	BUFFER_SIZE*8
//#define REMOVE_AVERAGE

// Define this to send the data to the uart
// uncomment the one to send raw/processed data


//#define 	SEND_UART_RAW_DATA
//#define 	SEND_UART_FFT_DATA

#define LED_ROWS			8
#define LED_COLS			16
#define	NUM_LEDS			LED_ROWS*LED_COLS

#ifdef SEND_UART_FFT_DATA
	#define OUTPUT BUFFER_SIZE	HALF_BUFFER_SIZE/2
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define SaturaLH(N, L, H) (((N)<(L))?(L):(((N)>(H))?(H):(N)))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

int32_t 					 RecBuff[BUFFER_SIZE];
__IO uint32_t                DmaRecHalfBuffCplt  = 0;
__IO uint32_t                DmaRecBuffCplt      = 0;
__IO uint32_t				 DmaSentHalfBuffCplt = 1;
__IO uint32_t				 DmaSentBuffCplt     = 1;
__IO uint32_t				 DoSerialOutput 	 = 1;
__IO uint32_t				 debouncing 	 	 = 0;


HAL_StatusTypeDef 			 sts;
char						 commBuff[256];
char						 OutBuff[OUTPUT_BUFFER_SIZE];

#ifdef 	DO_FFT
float32_t					 FFTInBuff[FFT_LENGTH];
float32_t					 FFTOutBuff[FFT_LENGTH];
float32_t					 PowerBuff[HALF_FFT_LENGTH];
uint16_t					 DecimatedBuff[DECIMATED_LENGTH];
arm_rfft_fast_instance_f32	 fft_handler;
#endif
// Neopixel variables
TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim2_ch1;

/* For week 7 homework
int32_t						 DummyGlobal;
char*						 memPtr;
*/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* For homework week 7
void printArgAddress(int32_t arg) {
	int32_t	localVar;
	sprintf(commBuff, "In function printVarAddress arg has value %ld and is stored at 0x%x\r\n"
			          "In function printVarAddress local variable localVar is stored at 0x%x\r\n",
					  arg, (int) &arg, (int) &localVar);
	while(HAL_OK != HAL_UART_Transmit_DMA(&huart3, (uint8_t *) commBuff, strlen(commBuff))) {

	}
}
*/
float complexABS(float real, float compl) {
	return sqrt(real*real+compl*compl);
}


#ifdef DO_FFT
void DoFFT(float32_t*	inBuff, float32_t*	outBuff, float32_t* powerBuff, arm_rfft_fast_instance_f32* handler) {
	int16_t		i;
	float32_t	offset;
	// Do FFT
	arm_rfft_fast_f32(handler, inBuff, outBuff, 0);
	offset = outBuff[0];
	// Compute magnitude of the complex return values
	arm_cmplx_mag_f32(outBuff, powerBuff, HALF_FFT_LENGTH);
	// Convert to DB
	//for (i = 0; i < HALF_FFT_LENGTH/2; i++) {
	//	powerBuff[i] = (20*log10(powerBuff[i]));  // TBD? Subract offset?
	//}
}
#endif

// Uncomment only one of the options below to choose the
// metric (total value or max value) to evaluate the data
// in each bin
// #define DECIMATE_WITH_TOTAL
//   #define DECIMATE_WITH_MAX
 #define DECIMATE_WITH_AVERAGE
uint16_t DecimateFFTData(float32_t *dataBuff, uint16_t* decimatedBuff, char* outBuff, uint32_t npoints, uint32_t binsize) {
	uint16_t 	i, j, nbins;
	float32_t 	metric;	// Either total or max
	char		*outBuffPtr;
	float32_t 	*dataBuffPtr;

	nbins = (uint16_t) npoints/binsize;
	outBuffPtr = outBuff;
	for (i = 0; i < nbins; i++) {
		metric = 0;
		dataBuffPtr = dataBuff + i*binsize;
		for (j =0; j < binsize; j++) {
#if defined DECIMATE_WITH_TOTAL || defined DECIMATE_WITH_AVERAGE
			metric += dataBuffPtr[j];
#endif
#ifdef 	DECIMATE_WITH_MAX
			if (dataBuffPtr[j] > metric) {
				metric = dataBuffPtr[j];
			}
#endif
		}
#ifdef DECIMATE_WITH_AVERAGE
		metric /= binsize;
#endif
		// Format data for UART
		decimatedBuff[i] = (uint16_t) (metric);
		sprintf(outBuffPtr, "%d:%12d\r\n",i,(uint16_t) metric);
		outBuffPtr += strlen(outBuffPtr);
	}
	return outBuffPtr - outBuff; // return #characters in string to send
}

void wheel(uint8_t index, uint8_t *r, uint8_t *g, uint8_t *b) {
	index = 255 - index;
	if (index < 85) {
		*r = 255 - index;
		*g = 0;
		*b = index*3;
	} else if (index < 170) {
		index -= 85;
		*r = 0;
		*g = index *3;
		*b = 255 - index*3;
	} else {
		index -= 170;
		*r = index*3;
		*g = 255 - index*3;
		*b = 0;
	}
}

// Plots data on an 8x8 grid of NeoPixels. Assuming we are sampling at 23.4 kHz with 32 frequency bins,
// Standard audio sampling is 44 kHz, but I want only vocal range frequencies
// Then each bin is 6.5 kHz/256 = 25.4 hZ wide, and the values run from 0->6500 Hz.
// we'll pick values for musical notes for our 8 bins.
/**
 * Note Freq bin
 * A = 440 = 17
 * B = 494 = 19
 * C = 523 = 21
 * D = 587 = 23
 * E = 659 = 25
 * F = 698 = 27
 * G = 784 = 31
 * A = 880 = 35
 */

#define NUM_BINS	16
#define LED_COLUMN_HEIGHT	8
#define MAX_COLUMN_VAL		LED_COLUMN_HEIGHT - 1
// Plots a spectrogram to leds
void plotFFTData(uint16_t *dataBuff, uint16_t numPoints) {

	const  uint8_t	spectrum_colors[LED_COLUMN_HEIGHT][3] = {{51,153,255},{51,255,255},{51,255,153},{51,255,51},{153,255,51},{255,255,51},{255,153,51},{255,51,51}};
	uint8_t			*color_index;
	int16_t 		binsize, max_amplitude = 1024;
	uint8_t			attenuate = 11;
	uint8_t			i, scaled_amplitude;
	uint8_t			r, g, b;

	binsize = numPoints/NUM_BINS;
	led_set_all_RGB(0,0,0);
	// scale to height of 8 and plot
	for (i = 0; i < NUM_BINS; i++) {
		scaled_amplitude = (uint8_t) fmin((dataBuff[i*binsize+1]*LED_COLUMN_HEIGHT) >> attenuate, MAX_COLUMN_VAL);
		color_index = spectrum_colors[scaled_amplitude];
		led_set_RGB(i*8 + scaled_amplitude, color_index[0], color_index[1], color_index[2]);
	}
	led_render();
}

void ledTest() {
	static uint8_t color_index = 0, r, g, b;
	static uint8_t start_i = 0;
	uint8_t			i;

	led_set_all_RGB(0,0,0);
	wheel(color_index, &r, &g, &b);
	color_index = (color_index + 1) % 255;
	for (i = 0; i < NUM_BINS; i++) {
		led_set_RGB(i*LED_COLUMN_HEIGHT + start_i, r, g, b);
	}
	start_i = (start_i + 1) % LED_COLUMN_HEIGHT;
	led_render();
}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	  uint32_t 		i;
	  int32_t		satVal;
	  uint16_t 		outStringLen;
#ifdef SEND_UART_RAW_DATA
	  char* 		outBuffPtr;
#endif
#ifdef DO_FFT
	  float32_t*	fftBuffPtr;
	  float32_t		average;
#endif
	  const uint8_t ATTENUATION = 6;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_DMA_Init();
  MX_DFSDM1_Init();
  MX_ICACHE_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /* Start DFSDM conversions */
  if(HAL_OK != HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, RecBuff, BUFFER_SIZE))
  {
    Error_Handler();
  }
  /*** Homework week 7
  // Homework assignment
  sprintf(commBuff, "***\r\nGlobal var DummyGlobal has address: 0x%x\r\n", (int) &DummyGlobal);
  while (HAL_OK != HAL_UART_Transmit_DMA(&huart3, (uint8_t *) commBuff, strlen(commBuff))) {
  }
  HAL_Delay(200);
  memPtr = (char *) malloc(16);
  sprintf(commBuff, "Allocated 16 bytes on heap at address: 0x%x\r\n", (int) &memPtr);
  while (HAL_OK != HAL_UART_Transmit_DMA(&huart3, (uint8_t *) commBuff, strlen(commBuff))) {
  }
  free(memPtr);
  HAL_Delay(200);
  printArgAddress(42);
  HAL_Delay(200);
  End Homework Week 7***/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
#ifdef DO_FFT
  arm_rfft_fast_init_f32(&fft_handler, FFT_LENGTH);
  led_set_all_RGB(0,0,0);  // If using neopxiels set them all to zero
#endif
  while (1)
  {


	    if(DmaRecHalfBuffCplt == 1)
	    {
	    	//BSP_LED_Toggle(LED_RED);
#ifdef DO_FFT
	    	fftBuffPtr = FFTInBuff;
	    	average = 0;
	    	/* Store values on Play buff */
	    	for(i = 0; i < HALF_BUFFER_SIZE; i++)
	    	{
	    		satVal 			  = SaturaLH((RecBuff[i] >> ATTENUATION), -32768, 32767);
	    		// Copy the value into the FFT Array
	    		*(fftBuffPtr++) = satVal;
#ifdef REMOVE_AVERAGE
	    		average += satVal;
	    	}
	    	// Subract the average to zero the data
	    	average /= HALF_BUFFER_SIZE;
	    	for(i = 0; i < HALF_BUFFER_SIZE; i++) {
	    		FFTInBuff[i] -= average;
	    	}
#else
	    	}
#endif
	    	DoFFT(FFTInBuff, FFTOutBuff, PowerBuff, &fft_handler);

	    	outStringLen = DecimateFFTData(PowerBuff, DecimatedBuff, OutBuff, HALF_FFT_LENGTH, BINSIZE);
	    	// only send data if resource is free - otherwise skip
			if (DoSerialOutput) {
				if (DmaSentBuffCplt) {
					  DmaSentBuffCplt = 0;
					  HAL_UART_Transmit_DMA(&huart3, (uint8_t *) OutBuff, outStringLen);
				  }
			}

			plotFFTData(DecimatedBuff, DECIMATED_LENGTH);
			//ledTest();
#endif
#ifdef SEND_UART_RAW_DATA
	    	outBuffPtr = OutBuff;
	    	for(i = 0; i < HALF_BUFFER_SIZE; i++)
	    	{
	    		satVal 			  = SaturaLH((RecBuff[i] >> ATTENUATION), -32768, 32767);
	    		// Just copy the value over to send it back out
	    		// Format data for UART
	    		sprintf(outBuffPtr, "%ld\r\n", satVal);
	    		outBuffPtr += strlen(outBuffPtr);
	    	}
	    	HAL_UART_Transmit_DMA(&huart3, (uint8_t *) OutBuff, (uint16_t)(outBuffPtr - OutBuff));
#endif
	    	DmaRecHalfBuffCplt  = 0;
  	  }
	  if(DmaRecBuffCplt == 1)
	  {
#ifdef DO_FFT
		  fftBuffPtr = FFTInBuff;
		  /* Store values on Play buff */
		  average = 0;
		  for(i = HALF_BUFFER_SIZE; i < 2*HALF_BUFFER_SIZE; i++)
	      {
			  	satVal 			  = SaturaLH((RecBuff[i] >> ATTENUATION), -32768, 32767);
			  	*(fftBuffPtr++) = satVal;
#ifdef REMOVE_AVERAGE
			  	average += satVal;
	      }
		  // Subract average value
		  average /= HALF_BUFFER_SIZE;
		  for (i = 0; i < HALF_BUFFER_SIZE; i++) {
			  FFTInBuff[i] -= average;
		  }
#else
	  	  }
#endif
		  // only send data if buffer is free, otherwise skip
		  DoFFT(FFTInBuff, FFTOutBuff, PowerBuff, &fft_handler);


		  outStringLen = DecimateFFTData(PowerBuff, DecimatedBuff, OutBuff, HALF_FFT_LENGTH, BINSIZE);
		  if (DoSerialOutput) {
			  if (DmaSentBuffCplt) {
				  DmaSentBuffCplt = 0;
				  HAL_UART_Transmit_DMA(&huart3, (uint8_t *) OutBuff, outStringLen);
			  }
		  }

		  plotFFTData(DecimatedBuff, DECIMATED_LENGTH);
		  //ledTest();
#endif
#ifdef SEND_UART_RAW_DATA
		  outBuffPtr = OutBuff + OUTPUT_BUFFER_SIZE/2;
		  /* Store values on Play buff */
		  for(i = HALF_BUFFER_SIZE; i < 2*HALF_BUFFER_SIZE; i++)
	      {
			  	satVal 			  = SaturaLH((RecBuff[i] >> ATTENUATION), -32768, 32767);
	    		// Format data for UART
			  	sprintf(outBuffPtr, "%ld\r\n", satVal);
	    		outBuffPtr += strlen(outBuffPtr);
	      }
		  //Send data string to UART
	      HAL_UART_Transmit_DMA(&huart3, (uint8_t *) (OutBuff + OUTPUT_BUFFER_SIZE/2), (uint16_t)(outBuffPtr - (OUTPUT_BUFFER_SIZE/2 + OutBuff)));
#endif
	      DmaRecBuffCplt  = 0;
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE0) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 55;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// Fast hsl2rgb algorithm: https://stackoverflow.com/questions/13105185/fast-algorithm-for-rgb-hsl-conversion
uint32_t hsl_to_rgb(uint8_t h, uint8_t s, uint8_t l) {
	if(l == 0) return 0;

	volatile uint8_t  r, g, b, lo, c, x, m;
	volatile uint16_t h1, l1, H;
	l1 = l + 1;
	if (l < 128)    c = ((l1 << 1) * s) >> 8;
	else            c = (512 - (l1 << 1)) * s >> 8;

	H = h * 6;              // 0 to 1535 (actually 1530)
	lo = H & 255;           // Low byte  = primary/secondary color mix
	h1 = lo + 1;

	if ((H & 256) == 0)   x = h1 * c >> 8;          // even sextant, like red to yellow
	else                  x = (256 - h1) * c >> 8;  // odd sextant, like yellow to green

	m = l - (c >> 1);
	switch(H >> 8) {       // High byte = sextant of colorwheel
	 case 0 : r = c; g = x; b = 0; break; // R to Y
	 case 1 : r = x; g = c; b = 0; break; // Y to G
	 case 2 : r = 0; g = c; b = x; break; // G to C
	 case 3 : r = 0; g = x; b = c; break; // C to B
	 case 4 : r = x; g = 0; b = c; break; // B to M
	 default: r = c; g = 0; b = x; break; // M to R
	}

	return (((uint32_t)r + m) << 16) | (((uint32_t)g + m) << 8) | ((uint32_t)b + m);
}

/**
  * @brief  Regular conversion complete callback.
  * @note   In interrupt mode, user has to read conversion value in this function
  *         using HAL_DFSDM_FilterGetRegularValue.
  * @param  hdfsdm_filter DFSDM filter handle.
  * @retval None
  */

void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
	//Override callback from stn32l5xx_hal_dfsdm.c
	  if(hdfsdm_filter == &hdfsdm1_filter0)
	  {
	    DmaRecBuffCplt = 1;
	  }
}

/**
  * @brief  Half regular conversion complete callback.
  * @param  hdfsdm_filter DFSDM filter handle.
  * @retval None
  */

void HAL_DFSDM_FilterRegConvHalfCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
	// Override callback from stn32l5xx_hal_dfsdm.c
	  if(hdfsdm_filter == &hdfsdm1_filter0)
	  {
	    DmaRecHalfBuffCplt = 1;
	  }
}

/**
  * @brief Tx Transfer completed callback.
  * @param huart UART handle.
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart3) {
	  DmaSentBuffCplt = 1;
  }

}

/**
  * @brief  Tx Half Transfer completed callback.
  * @param  huart UART handle.
  * @retval None
  */
void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	  if (huart == &huart3) {
		  DmaSentHalfBuffCplt = 1;
	  }
}

/**
  * @brief  EXTI line rising detection callback.
  * @param  GPIO_Pin Specifies the port pin connected to corresponding EXTI line.
  * @retval None
  */
void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */

  if (!debouncing && GPIO_Pin == PUSH_BUTTON_Pin) {
	  // Implement debouncing by starting timer 3
	  debouncing = 1;
	  HAL_TIM_Base_Start_IT(&htim3);

  }

}

/**
  * @brief  Period elapsed callback in non-blocking mode
  * @param  htim TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// Debounce push button
	if (htim == &htim3) {
		debouncing = 0;
		HAL_TIM_Base_Stop_IT(&htim3);
		// If we're here, the button is debounced and we should toggle the state
		if (HAL_GPIO_ReadPin(GPIOC, 13) == GPIO_PIN_RESET) {	// Button is pushed
			DoSerialOutput = !DoSerialOutput;
		}
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

