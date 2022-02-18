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


#define MAX_FFT_LENGTH		    2048
#define HALF_MAX_FFT_LENGTH		1024
#define	DEFAULT_FFT_LENGTH		512
/*
#define	FFT_LENGTH				512
#define HALF_FFT_LENGTH			256
#define DOUBLE_FFT_LENGTH		1024
*/

#define LED_ROWS				8
#define LED_COLS				32			// Should be same as NUM_DISPLAY_BINS
#define NUM_DISPLAY_BINS		LED_COLS    // Width of data display


#define OUTPUT_DATA_BUFF_LEN	MAX_FFT_LENGTH*8  // Safety margin probably too big here
//#define REMOVE_AVERAGE

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define SaturaLH(N, L, H) (((N)<(L))?(L):(((N)>(H))?(H):(N)))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int32_t 					 RecBuff[2*MAX_FFT_LENGTH];  // Twice FFT size so can fill other half of circular buffer
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
char						 OutBuff[OUTPUT_DATA_BUFF_LEN];

// FFT Buffers and variables
float32_t					 FFTInBuff[MAX_FFT_LENGTH];
float32_t					 FFTOutBuff[MAX_FFT_LENGTH];
float32_t					 PowerBuff[HALF_MAX_FFT_LENGTH];
arm_rfft_fast_instance_f32	 fftHandler;


// Buffer to hold data values for the display
int16_t					 	 DisplayBuff[NUM_DISPLAY_BINS];



// Parameters for post-processing the FFT data to narrow it to the region of interest
typedef enum {MODE_TOTAL,
			  MODE_MAX,
			  MODE_AVERAGE} ProcessingMode;

			  /*
// Interpolation FFT comes from: https://www.dsprelated.com/showarticle/1156.php
typedef enum {INTERPOLATION_LINEAR,
			  INTERPOLATION_NEAREST,
			  INTERPOLATION_FFT} InterpolationMode;
*/

// Store display parameters
typedef struct DisplayParameters {
	uint16_t			displayWidth;
	uint16_t			displayHeight;
	uint16_t			binWidth;	//# of data points aggregated per display bin
	uint16_t			nFilledBins;
	uint16_t			firstBinPosition;
	int16_t				*displayBuff;
	uint8_t				**colColors;
	ProcessingMode		dataMode;

} DispParamStruct;

DispParamStruct		DisplayParams = {
	LED_COLS,							//displayWidth
	LED_ROWS,							//displayheight
	DEFAULT_FFT_LENGTH/(2*LED_COLS),	//binWidth
	0,
	0,
	DisplayBuff,
	ColumnColors,
	MODE_AVERAGE
};

// Store console write params
typedef struct ConsoleParameters {
	char*				writeBuffer;
	uint16_t			outputColVals;
	uint16_t			outputMaxCol;
	uint16_t			outputFFTTiming;
	uint16_t			outputDisplayRange;
} ConsoleParamStruct;

ConsoleParamStruct	ConsoleParams = {
		&OutBuff,
		0,	// outputColVals
		0,	// outputMaxCol
		1,  // outputFFTTiming;
		1,  // outputDisplayRange
};

// Store FFT Parameters
typedef struct FFTParameters {
	uint16_t					numDataPoints;
	uint16_t					halfNumDataPoints;
	uint16_t					zeroPad; 	// if nonZero, the last "zeroPad" elements of the array are zero-padded
	uint32_t					LastFFTComputationTimeMicros;
	uint16_t					sampleRateHZ;
	float32_t   				*inBuff;
	float32_t					*outBuff;
	float32_t   				*powerBuff;
	uint16_t					maxFreq;
	uint16_t				    minFreq;
	uint16_t					freqResolution; // Hertz per data bin
	uint16_t					doFFT;			// If zero, we're pausing computation to look at existing data
	arm_rfft_fast_instance_f32	*handler;
} FFTParamStruct;


FFTParamStruct		FFTParams = { DEFAULT_FFT_LENGTH,			//numDataPoints
								  DEFAULT_FFT_LENGTH/2,  		//halfNumDataPoints
								  0,							//zeroPad
								  0,							//LastFFTComputationTimeMicros
								  0,    						//SampleRateHz - must be compted in initialization
								  FFTInBuff,
								  FFTOutBuff,
								  PowerBuff, 					//min/max frequency and binsize will be computed
								  0,							//maxFreq
								  0,							//minFreq
								  0,							//binSize
								  1,							//doFFT
								  &fftHandler					// FFT Handler structure
};


// Timers
TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim2_ch1;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

// Returns the audio sampling frequency by the DFSDM, along with min, max frequencies shown
// and the width of a bin in Hz
float32_t	getSamplingRate(uint16_t *fmin, uint16_t *fmax) {
	// Should probably be reading these values from various bits in DFSDM configuration registers,
	// but this way is much easier to finish in the limited time available

	float32_t 	clockRate 		=	hdfsdm1_channel1.Init.OutputClock.Selection == DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM ? 110000000: 48000000;	// System or Audio clock rate
	float32_t	foic 			= 	hdfsdm1_filter0.Init.FilterParam.Oversampling;
	float32_t	integ 			= 	hdfsdm1_filter0.Init.FilterParam.IntOversampling;
	float32_t	div				= 	hdfsdm1_channel1.Init.OutputClock.Divider;
	float32_t 	samplingRate 	= 	clockRate/(foic*integ*div);
	*fmin = 0;
	*fmax = samplingRate/2;
	return samplingRate;
}

void updateFFTParamVals(uint16_t numData) {	// If numData is zero, keep existing dataPointValues

	if (numData) {
		FFTParams.numDataPoints = numData;
		FFTParams.halfNumDataPoints = numData/2;
	}

	FFTParams.sampleRateHZ = getSamplingRate(&(FFTParams.minFreq), &(FFTParams.maxFreq));
	FFTParams.freqResolution = (FFTParams.maxFreq - FFTParams.minFreq)/FFTParams.halfNumDataPoints;
	arm_rfft_fast_init_f32(FFTParams.handler, FFTParams.numDataPoints);
}



void DoFFT(FFTParamStruct *fftParams) {

	// Do FFT
	// Record start time
	int16_t timerVal16 = __HAL_TIM_GET_COUNTER(&htim16);
	arm_rfft_fast_f32(fftParams->handler, fftParams->inBuff, fftParams->outBuff, 0);
	// Compute magnitude of the complex return values. powerBuff has half the number
	fftParams->LastFFTComputationTimeMicros = abs((int)__HAL_TIM_GET_COUNTER(&htim16) - (int) timerVal16);  // Keep timing units in uS for now
	//  of elements of inBuff/outBuff
	arm_cmplx_mag_f32(fftParams->outBuff, fftParams->powerBuff, fftParams->halfNumDataPoints);
	// Convert to DB
	//for (i = 0; i < HALF_FFT_LENGTH/2; i++) {
	//	powerBuff[i] = (20*log10(powerBuff[i]));  // TBD? Subract offset?
	//}
}



ConsoleParamStruct	ConsoleParameters = {
		&OutBuff,
		0,	// outputColVals
		0,	// outputMaxCol
		1,  // outputFFTTiming;
		1,  // outputDisplayRange
};

uint16_t MakeDisplayDescriptorString() {
	char	*outPtr = ConsoleParams.writeBuffer;


}

// Generates the string of data to go to the console output
uint16_t MakeDataDescriptorString() {

	uint16_t	i;
	char		*outPtr = ConsoleParams.writeBuffer;

	if (ConsoleParams.outputColVals) {
		for (i = 0; i < NUM_DISPLAY_BINS; i++) {
			sprintf(outPtr, "%d:%12d\r\n",i,DisplayBuff[i]);
			outPtr += strlen(outPtr);
		}
	}
	if (ConsoleParams.outputMaxCol) {
		// TBD
	}
	if (ConsoleParams.outputFFTTiming) {
		sprintf(outPtr, "# samples:    %12d\r\n"
				        "FFT duration: %12.2f ms\r\n"
				        "Sample rate:  %12d Hz\r\n"
				        "Sample period:%12.2f ms\r\n\r\n",
					     FFTParams.numDataPoints,
						 FFTParams.LastFFTComputationTimeMicros/1000.0,
						 FFTParams.sampleRateHZ,
						 FFTParams.numDataPoints*1000.0/FFTParams.sampleRateHZ);
		outPtr += strlen(outPtr);
	}
	if (ConsoleParams.outputDisplayRange) {
		// TBD
	}

	return outPtr - ConsoleParams.writeBuffer;
}



// Process the FFT data for display - by puting it in the displayBuff, which has NUM_COLS bins. Must supply resolution +
//  start/stop required frequency resolution
void ProcessFFTData(FFTParamStruct *fftParams, DispParamStruct *dispParams) {
	uint16_t	i, j, dataBinSize;
	uint16_t	nFilledBins;
	float32_t	metric;
	float32_t	*dataPtr;


	// We might not have data for all the display columns
	dataBinSize = dispParams->binWidth;
	nFilledBins = fmin((int16_t) ((fftParams->halfNumDataPoints - dispParams->firstBinPosition)/dataBinSize), LED_COLS);


	for (i = 0; i < nFilledBins; i++) {
		metric = 0;
		dataPtr = fftParams->powerBuff + i*dataBinSize;
		switch(dispParams->dataMode) {
			case MODE_TOTAL:
			case MODE_AVERAGE:
				for (j = 0; j < dataBinSize; j++) { metric += dataPtr[j]; }
				if (dispParams->dataMode == MODE_AVERAGE) { metric /= dataBinSize; }
				break;
			case MODE_MAX:
				for (j = 0; j < dataBinSize; j++) { metric = fmax(metric, dataPtr[j]); }
				break;

		}
		dispParams-> displayBuff[i] = (uint16_t) metric;
	}

	// Pack the rest of the empty display columns with (-1 or 0)
	for (i = nFilledBins; i < dispParams->displayWidth; i++) {
		dispParams->displayBuff[i] = -1;
	}

}



// Standard cyclic rainbow spectrum generator function with uint8_t input
// Adapted from Adafruit example
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


#define MAX_PLOT_VAL		LED_ROWS - 1
// Computes the actual pixel value for our snake-layout 8x32 LED matrix which currently
// has zero valued pixel on the left
#define pixel_pos_even_col(X,Y)   (X+2)*LED_ROWS - (Y + 1)
#define pixel_pos_odd_col(X,Y)  X*LED_ROWS + Y
// Plots a spectrogram to leds
void plotFFTData(int16_t *dataBuff) {


	const 	uint8_t			*color_index;
	int16_t 				dataPoint;
	uint8_t					attenuate = 11;
	uint8_t					i, scaled_amplitude;

	// Clear old leds
	led_set_all_RGB(0,0,0);

	// scale to height of 8 and plot - alternating columns run opposite directions
	for (i = 0; i < LED_COLS; i += 2) {
		// Data order reversed from display order
		dataPoint = dataBuff[(LED_COLS - i)- 1];
		if (dataPoint > 0) {
			scaled_amplitude = (uint8_t) fmin((dataPoint*LED_ROWS) >> attenuate, MAX_PLOT_VAL);
			color_index = ColumnColors[i];
			led_set_RGB(pixel_pos_odd_col(i,scaled_amplitude), color_index[0], color_index[1], color_index[2]);
		}
		// Data order reversed from display order
		dataPoint = dataBuff[(LED_COLS - i) - 2];
		if (dataPoint > 0) {
			scaled_amplitude = (uint8_t) fmin(dataPoint*LED_ROWS >> attenuate, MAX_PLOT_VAL);
			color_index = ColumnColors[i+1];
			led_set_RGB(pixel_pos_even_col(i,scaled_amplitude), color_index[0], color_index[1], color_index[2]);
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
void handleUARTRequest(char request, char *msgBuff, DispParamStruct *dispParams, FFTParamStruct *fftParams) {
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
				dispParams->binWidth = fmin(dispParams->binWidth + 1,10);
				initializeColumnColors(&FFTParams, &DisplayParams);		// Initialize the color array
				sprintf(msgPtr, "Display bin size: %12d Hz\r\n", fftParams->freqResolution*dispParams->binWidth);
				break;
			case '-':
				dispParams->binWidth = fmax(dispParams->binWidth - 1, 1);
				initializeColumnColors(&FFTParams, &DisplayParams);		// Initialize the color array
				sprintf(msgPtr, "Display bin size: %12d Hz\r\n", fftParams->freqResolution*dispParams->binWidth);
				break;
	  }
}

// Precompute the base level color for each of the columns
// Because the indixes in the display run from high pixel at left to
// low pixel at right display is reversed for the LED matrix, we
// need to fill the column colors in revere order
void initializeColumnColors(FFTParamStruct *fftParams, DispParamStruct *dispParams) {
//uint16_t firstDataPoint, uint16_t numDataPoints, uint16_t binSize) {
	uint8_t 	r, g, b;
	uint16_t 	i, index;

	for (i = 0; i < LED_COLS; i++) {
		index = (255*(dispParams->firstBinPosition + i*dispParams->binWidth))/(fftParams->halfNumDataPoints-1);
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
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */

  // Initialize FFT information
  updateFFTParamVals(DEFAULT_FFT_LENGTH);
  // Start our timekeeping timer
  HAL_TIM_Base_Start(&htim16);
  /* Start DFSDM conversions */
  if(HAL_OK != HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, RecBuff, 2*FFTParams.numDataPoints))
  {
    Error_Handler();
  }



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  led_set_all_RGB(0,0,0);  								// Start all Neopixels Off
  initializeColumnColors(&FFTParams, &DisplayParams);		// Initialize the color array

  while (1)
  {
	  	// TBD - fix up the handling of RX transmission data
	  	if (RXDataIncoming) {
    		sts = HAL_UART_Receive_IT(&huart3, &rxByte, 1);
    		RXDataIncoming = 0;
	  	} else if  (RXDataComplete) {
//	  		srate = getSamplingRate(&fmin, &fmax);
//	  		sprintf(commBuff, "Sampling rate = %12.2f Hz\r\nMin freq = %12.2f Hz\r\nMax freq = %12.2f Hz\r\nFreq bin size = %12.2f Hz\r\n", srate, fmin, fmax, fbin);
//	    	HAL_UART_Transmit_DMA(&huart3, (uint8_t *) commBuff, strlen(commBuff));
	    	RXDataComplete = 0;
	    }
	  	if (RXCmd) {
	  		handleUARTRequest(RXCmd, commBuff, &DisplayParams, &FFTParams);
	  		RXCmd = 0;
	  		HAL_UART_Transmit_DMA(&huart3, (uint8_t *) commBuff, strlen(commBuff));
	  	}

	    if(DmaRecHalfBuffCplt == 1)
	    {
	    	fftBuffPtr = FFTInBuff;
	    	average = 0;
	    	/* Store values on Play buff */
	    	for(i = 0; i < FFTParams.numDataPoints; i++) {
	    		// Saturate data values and copy them into the array
	    		satVal 			  = SaturaLH((RecBuff[i] >> ATTENUATION), -32768, 32767);
	    		*(fftBuffPtr++) = satVal;
	    	}

	    	DoFFT(&FFTParams);
	    	ProcessFFTData(&FFTParams, &DisplayParams);

	    	// only send data if resource is free - otherwise skip
			if (DoSerialOutput) {
				outStringLen = MakeDataDescriptorString();
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
		  for(i = FFTParams.numDataPoints; i < 2*FFTParams.numDataPoints; i++) {
			  	satVal 			  = SaturaLH((RecBuff[i] >> ATTENUATION), -32768, 32767);
			  	*(fftBuffPtr++) = satVal;
	  	  }

		  DoFFT(&FFTParams);
	      ProcessFFTData(&FFTParams, &DisplayParams);

	      if (DoSerialOutput) {
	    	  outStringLen = MakeDataDescriptorString();
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
/*
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
*/
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

