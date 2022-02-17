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

#define	FFT_LENGTH				512
#define HALF_FFT_LENGTH			256
#define NUM_DISPLAY_BINS		32  // Width of data display

#define HALF_BUFFER_SIZE		FFT_LENGTH
#define BUFFER_SIZE				2*FFT_LENGTH
#define OUTPUT_BUFFER_SIZE		BUFFER_SIZE*8  // Safety margin probably too big here
//#define REMOVE_AVERAGE

// Define this to send the data to the uart
// uncomment the one to send raw/processed data


//#define 	SEND_UART_RAW_DATA
//#define 	SEND_UART_FFT_DATA

#define LED_ROWS			8
#define LED_COLS			32	// Should be same as NUM_DISPLAY_BINS


#ifdef SEND_UART_FFT_DATA
	#define OUTPUT BUFFER_SIZE	HALF_BUFFER_SIZE/2
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define SaturaLH(N, L, H) (((N)<(L))?(L):(((N)>(H))?(H):(N)))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
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
__IO uint32_t				 RXDataIncoming 	 = 0;
__IO uint32_t				 RXDataComplete   	 = 0;
__IO uint32_t				 rxIndex			 = 0;
__IO char					 RXCmd				 = 0;

volatile 	uint16_t		 DisplayBinWidth 	 = HALF_FFT_LENGTH/LED_COLS;
const  		uint8_t			 RowColors[LED_ROWS][3] =
										{{17,51,81},{17,81,81},{17,81,51},{17,81,17},
										 {51,81,17},{81,81,17},{81,51,17},{81,17,17}};
uint8_t						 ColumnColors[LED_COLS][3];
HAL_StatusTypeDef 			 sts;
// Buffer to receive user commands via USART
char						 rxByte;
char						 rxBuff[64];
char						 commBuff[256];
// Buffer to send FFT Data to computer over serial
char						 OutBuff[OUTPUT_BUFFER_SIZE];

#ifdef 	DO_FFT
float32_t					 FFTInBuff[FFT_LENGTH];
float32_t					 FFTOutBuff[FFT_LENGTH];
float32_t					 PowerBuff[HALF_FFT_LENGTH];
int16_t					 	 DisplayBuff[NUM_DISPLAY_BINS];

arm_rfft_fast_instance_f32	 fft_handler;
#endif


// Neopixel variables
TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim2_ch1;


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

#ifdef DO_FFT
void DoFFT(float32_t*	inBuff, float32_t*	outBuff, float32_t* powerBuff, arm_rfft_fast_instance_f32* handler) {
	// int16_t		i;
	// Do FFT
	arm_rfft_fast_f32(handler, inBuff, outBuff, 0);
	// Compute magnitude of the complex return values. powerBuff has half the number
	//  of elements of inBuff/outBuff
	arm_cmplx_mag_f32(outBuff, powerBuff, HALF_FFT_LENGTH);
	// Convert to DB
	//for (i = 0; i < HALF_FFT_LENGTH/2; i++) {
	//	powerBuff[i] = (20*log10(powerBuff[i]));  // TBD? Subract offset?
	//}
}
#endif

// Parameters for post-processing the FFT data to narrow it to the region of interest
typedef enum {MODE_TOTAL,
			  MODE_MAX,
			  MODE_AVERAGE} ProcessingMode;

// Interpolation FFT comes from: https://www.dsprelated.com/showarticle/1156.php
typedef enum {INTERPOLATION_LINEAR,
			  INTERPOLATION_NEAREST,
			  INTERPOLATION_FFT} InterpolationMode;

// Process the FFT data for display - by puting it in the displayBuff, which has NUM_COLS bins. Must supply resolution +
//  start/stop required frequency resolution



uint16_t ProcessFFTData(float32_t *dataBuff, uint16_t nDataPoints, int16_t *displayBuff, uint8_t dispWidth, uint8_t dispHeight, uint8_t binSize,
		                uint16_t binInitial, char* outBuff, ProcessingMode mode) {
	uint16_t	i, j, binFinal;
	uint16_t	nFilledBins;
	float32_t	metric;
	char		*outPtr;
	float32_t	*dataPtr;


	// We might not have data for all the display columns
	nFilledBins = fmin((int16_t) ((nDataPoints - binInitial)/binSize), dispWidth);
	outPtr = outBuff;

	for (i = 0; i < nFilledBins; i++) {
		metric = 0;
		dataPtr = dataBuff + i*binSize;
		switch(mode) {
			case MODE_TOTAL:
			case MODE_AVERAGE:
				for (j = 0; j < binSize; j++) { metric += dataPtr[j]; }
				if (mode == MODE_AVERAGE) { metric /= binSize; }
				break;
			case MODE_MAX:
				for (j = 0; j < binSize; j++) { metric = fmax(metric, dataPtr[j]); }
				break;

		}
		displayBuff[i] = (uint16_t) metric;
		sprintf(outPtr, "%d:%12d\r\n",i,(uint16_t) metric);
		outPtr += strlen(outPtr);
	}

	// Pack the rest of the display columns with (-1 or 0)
	for (i = nFilledBins; i < dispWidth; i++) {
		DisplayBuff[i] = -1;
		sprintf(outPtr, "%d: N/A",i);
		outPtr += strlen(outPtr);
	}
	return outPtr - outBuff; // return #characters in string to send
}

// Uncomment only one of the options below to choose the
// method used (total, max or average) to use to aggregate the data
// in each bin
// #define DECIMATE_WITH_TOTAL
// #define DECIMATE_WITH_MAX
 #define DECIMATE_WITH_AVERAGE

// Uncomment below if converting to decibels (20log10)
//#define DO_DECIBELS
uint16_t DecimateFFTData(float32_t *dataBuff, uint16_t* DisplayBuff, char* outBuff, uint16_t npoints, uint16_t nbins) {
	uint16_t 	i, j, binsize;
	float32_t 	metric;	// Either total or max
	char		*outBuffPtr;
	float32_t 	*dataBuffPtr;

	outBuffPtr = outBuff;
	binsize = npoints/nbins;
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

		//metric = 20*log10(metric) // If you want decibels

		DisplayBuff[i] = (uint16_t) (metric);
		sprintf(outBuffPtr, "%d:%12d\r\n",i,(uint16_t) metric);
		outBuffPtr += strlen(outBuffPtr);
	}
	return outBuffPtr - outBuff; // return #characters in string to send
}


// Returns the audio sampling frequency by the DFSDM, along with min, max frequencies shown
// and the width of a bin in Hz
float32_t	getSamplingRate(float32_t *fmin, float32_t *fmax, float32_t *fbin) {
	// Should probably be reading these values from various bits in DFSDM configuration registers,
	// but this way is much easier to finish in the limited time available

	float32_t 	clockRate 		=	hdfsdm1_channel1.Init.OutputClock.Selection == DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM ? 110000000: 48000000;	// System or Audio clock rate
	float32_t	foic 			= 	hdfsdm1_filter0.Init.FilterParam.Oversampling;
	float32_t	integ 			= 	hdfsdm1_filter0.Init.FilterParam.IntOversampling;
	float32_t	div				= 	hdfsdm1_channel1.Init.OutputClock.Divider;
	float32_t 	samplingRate 	= 	clockRate/(foic*integ*div);
	*fmin = 0;
	*fmax = samplingRate/2;
	*fbin = (*fmax - *fmin)/NUM_DISPLAY_BINS;
	return samplingRate;
}

// Standard cyclic rainbow spectrum generator function with uint8_t input
// Adapted from Adafruit exanoke
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

#define LED_ROWS			8
#define MAX_PLOT_VAL		LED_ROWS - 1
// Plots a spectrogram to leds
void plotFFTData(int16_t *dataBuff) {


	const 	uint8_t			*color_index;
	int16_t 				index, dataPoint;
	uint8_t					attenuate = 11;
	uint8_t					i, scaled_amplitude;

	led_set_all_RGB(0,0,0);

	// Run through the data points, scaling to a height of 8 and plot.

	// scale to height of 8 and plot - alternating columns run opposite directions
	for (i = 0; i < LED_COLS; i += 2) {

		index = (LED_COLS - i)- 1;
		dataPoint = dataBuff[index];
		if (dataPoint > 0) {
			scaled_amplitude = (uint8_t) fmin((dataPoint*LED_ROWS) >> attenuate, MAX_PLOT_VAL);
			//color_index = RowColors[scaled_amplitude];
			color_index = ColumnColors[i];
			led_set_RGB(i*LED_ROWS + scaled_amplitude, color_index[0], color_index[1], color_index[2]);
		}

		index = (LED_COLS - i) - 2;
		dataPoint = dataBuff[index];
		if (dataPoint > 0) {
			scaled_amplitude = (uint8_t) fmin(dataPoint*LED_ROWS >> attenuate, MAX_PLOT_VAL);
			//color_index = RowColors[scaled_amplitude];
			color_index = ColumnColors[i+1];
			led_set_RGB((i+2)*LED_ROWS - (scaled_amplitude+1), color_index[0], color_index[1], color_index[2]);
		}
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
	for (i = 0; i < LED_COLS; i++) {
		led_set_RGB(i*LED_ROWS + start_i, r, g, b);
	}
	start_i = (start_i + 1) % LED_ROWS;
	led_render();
}

// Ascii codes for arrows:
//  up 	= 72
//  down 	= 80
//  left 	= 75
//  right = 77
// Code to handle incoming data
void handleUARTRequest(char request, char *msgBuff) {
	  char* msgPtr;
	  sprintf(msgBuff, "Process char %c\r\n",request);
	  msgPtr = msgBuff + strlen(msgBuff);
	  switch(request) {
			case 's':
				break;
			case 'd':
				break;
			case 72:	//Up arrow
				break;
			case 80:	//Down arrow
				break;
			case 75:	//Left arrow
				break;
			case 77:	//Right arrow
				break;
			case '+':
				DisplayBinWidth = fmin(DisplayBinWidth + 1, 10);
				initializeColumnColors(0, HALF_FFT_LENGTH, DisplayBinWidth);		// Initialize the color array
				sprintf(msgPtr, "DisplayBinWidth = %d\r\n", DisplayBinWidth);
				break;
			case '-':
				DisplayBinWidth = fmax(DisplayBinWidth - 1, 1);
				initializeColumnColors(0, HALF_FFT_LENGTH, DisplayBinWidth);		// Initialize the color array
				sprintf(msgPtr, "DisplayBinWidth = %d\r\n", DisplayBinWidth);
				break;
	  }
}

// Precompute the base level color for each of the columns
// Because the indixes in the display run from high pixel at left to
// low pixel at right display is reversed for the LED matrix, we
// need to fill the column colors in revere order
void initializeColumnColors(uint16_t firstDataPoint, uint16_t numDataPoints, uint16_t binSize) {
	uint8_t 	r, g, b;
	uint16_t 	i, index;

	for (i = 0; i < LED_COLS; i++) {
		index = (255*(firstDataPoint + i*binSize))/(numDataPoints-1);
		wheel((uint8_t) index, &r, &g, &b);
		ColumnColors[LED_COLS-i-1][0] = r;
		ColumnColors[LED_COLS-i-1][1] = g;
		ColumnColors[LED_COLS-i-1][2] = b;
	}
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
	  float32_t*	fftBuffPtr;
	  float32_t		average, srate, fmin, fmax, fbin;
	  const uint8_t ATTENUATION =8;  // Usually 8

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



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  arm_rfft_fast_init_f32(&fft_handler, FFT_LENGTH);
  led_set_all_RGB(0,0,0);  		// If using neopxiels set them all to zero
  initializeColumnColors(0, HALF_FFT_LENGTH, DisplayBinWidth);		// Initialize the color array

  while (1)
  {

	  	if (RXDataIncoming) {
    		sts = HAL_UART_Receive_IT(&huart3, &rxByte, 1);
    		RXDataIncoming = 0;
	  	} else if  (RXDataComplete) {
	  		srate = getSamplingRate(&fmin, &fmax, &fbin);
	  		sprintf(commBuff, "Sampling rate = %12.2f Hz\r\nMin freq = %12.2f Hz\r\nMax freq = %12.2f Hz\r\nFreq bin size = %12.2f Hz\r\n", srate, fmin, fmax, fbin);
	    	HAL_UART_Transmit_DMA(&huart3, (uint8_t *) commBuff, strlen(commBuff));
	    	RXDataComplete = 0;
	    }
	  	if (RXCmd) {
	  		handleUARTRequest(RXCmd, commBuff);
	  		RXCmd = 0;
	  		HAL_UART_Transmit_DMA(&huart3, (uint8_t *) commBuff, strlen(commBuff));
	  	}

	    if(DmaRecHalfBuffCplt == 1)
	    {
	    	fftBuffPtr = FFTInBuff;
	    	average = 0;
	    	/* Store values on Play buff */
	    	for(i = 0; i < HALF_BUFFER_SIZE; i++)
	    	{
	    		satVal 			  = SaturaLH((RecBuff[i] >> ATTENUATION), -32768, 32767);
	    		// Copy the value into the FFT Array
	    		*(fftBuffPtr++) = satVal;
	    	}

	    	DoFFT(FFTInBuff, FFTOutBuff, PowerBuff, &fft_handler);
	    	outStringLen =
	    			//DecimateFFTData(PowerBuff, DisplayBuff, OutBuff, HALF_FFT_LENGTH, LED_COLS);
	    			ProcessFFTData(PowerBuff, HALF_FFT_LENGTH, DisplayBuff, LED_COLS, LED_ROWS, DisplayBinWidth, 0, OutBuff, MODE_AVERAGE);

	    	// only send data if resource is free - otherwise skip
			if (DoSerialOutput) {
				if (DmaSentHalfBuffCplt) {
					  DmaSentHalfBuffCplt = 0;
 					  HAL_UART_Transmit_DMA(&huart3, (uint8_t *) OutBuff, outStringLen);
				  }
			}

			plotFFTData(DisplayBuff);
			//ledTest();
	    	DmaRecHalfBuffCplt  = 0;
  	  }
	  if(DmaRecBuffCplt == 1)
	  {

		  fftBuffPtr = FFTInBuff;
		  /* Store values on Play buff */
		  average = 0;
		  for(i = HALF_BUFFER_SIZE; i < BUFFER_SIZE; i++)
	      {
			  	satVal 			  = SaturaLH((RecBuff[i] >> ATTENUATION), -32768, 32767);
			  	*(fftBuffPtr++) = satVal;
	  	  }

		  DoFFT(FFTInBuff, FFTOutBuff, PowerBuff, &fft_handler);
	      outStringLen =
	    			//DecimateFFTData(PowerBuff, DisplayBuff, OutBuff, HALF_FFT_LENGTH, LED_COLS);
	    			ProcessFFTData(PowerBuff, HALF_FFT_LENGTH, DisplayBuff, LED_COLS, LED_ROWS, DisplayBinWidth, 0, OutBuff, MODE_AVERAGE);
		  if (DoSerialOutput) {
			  // only send data if buffer is free, otherwise skip
			  if (DmaSentBuffCplt) {
				  DmaSentBuffCplt = 0;
				  HAL_UART_Transmit_DMA(&huart3, (uint8_t *) OutBuff, outStringLen);
			  }
		  }

		  plotFFTData(DisplayBuff);
		  //ledTest();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
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
  * @brief  Rx Transfer completed callback.
  * @param  huart UART handle.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
 	if (huart == &huart3) {
		// If no data is being received, clear the buffer
		// Right now just toggle LED to see if anything is noticed
	    //Toggle LED10
	    HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
	    // Enable receiving of data via UART and interrupt notification

	    if (!DoSerialOutput) {
	    	RXDataIncoming = 1;
	    	if (rxByte == '\r') {
	    		rxBuff[rxIndex] = '\0';
	    		// Now respond to input
	    		RXDataComplete = 1;
	    		rxIndex = 0;
	    	} else {
	    		rxBuff[rxIndex++] = rxByte;
	    		RXCmd = rxByte;
	    	}
	    } else {
	    	RXDataIncoming = 0;
	    }
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
			if (!DoSerialOutput) {
				// Enable receiving of data via UART and interrupt notification
				RXDataIncoming = 1;
			} else {
				RXDataIncoming = 0;
			}
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

