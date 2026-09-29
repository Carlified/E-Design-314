/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "app_fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../../Drivers/OLED/ssd1306.h"
#include "../../Drivers/OLED/fonts.h"
#include "../../Drivers/MFC522/RC522.h"
#include <stdio.h>
#include <math.h>
#include <ctype.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_tx;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
void dump_file(TCHAR *filename);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char studentNum[] = "*27005348#\n";
uint8_t rx;					//recieve a single byte here
uint8_t input_buf_len = 0;  //current length of input buffer
uint8_t input_buffer[16];	//input buffer

uint16_t waves[400];		//buffer for ADC readings

float currentReal = 0; //most recent converted voltage (V)
float voltageReal = 0; //most recent converted current (mA)
float phaseReal = 0;   //most recent phase difference (thetaV-thetaA) (rad)

float currentAvg = 0;
float voltageAvg = 0;
float powAvg = 0;
float phaseAvg = 0;
uint8_t n = 0;

volatile uint8_t stat_ready = 0;          //flag for stat command to be sent
volatile uint8_t dump_ready = 0;
volatile uint8_t logging = 0;
volatile uint8_t count_units = 1;

volatile uint8_t push_button_pressed = 0; //flag to check if push button has been pressed
volatile uint32_t button_time = 0;        //time since last push button reading

volatile uint8_t debug_button_pressed = 0;
volatile uint32_t debug_button_time = 0;
uint8_t debug_screen = 0;

volatile uint32_t last_keypad_poll = 0;   //time since last polled keypad
uint16_t keypad_mask = 0; //12 bits representing which keys are currently pressed - not used right now
volatile uint8_t num_keys = 0;			 //number of key presses left to process
volatile char key_buffer[10];	//buffer of all keys that need to be processed

enum menu {
	topmenu, lvl1, lvl2, lvl3,
};
enum menu menupos = topmenu;			//current position in menu tree
uint8_t menu_states[4] = { 0, 0, 0, 0 }; //current and previous states in the menu tree

uint32_t last_screen_update = 0;      //time since last frame

uint32_t newunits;
float units_left = 10.0099;
float prevUnits = 10.0099;

uint8_t status, rfid_data[16];
uint8_t valid_keycard[5] = { 0xf6, 0xf1, 0x4d, 0x05, 0x4f };
uint8_t valid_keyfob[5] = { 0xc5, 0x2a, 0xb1, 0x01, 0x5f };
uint8_t valid_studentcard[5] = { 73, 81, 94, 124, 58 };
int32_t rfid_timeout = -5000;
uint32_t rfid_evt = 0;
enum rfid_states {
	poll, anti_collision, target,
};
enum rfid_states rfid_state = poll;

static FATFS FatFs;
FIL fil;
FRESULT fres;
UINT bw = 0;
uint8_t log_len = 0;
char logbuff[300] = { 0 };

uint32_t ADC_back_on = -1000;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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
  MX_USART2_UART_Init();
  MX_RTC_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  if (MX_FATFS_Init() != APP_OK) {
    Error_Handler();
  }
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
	//reset RFID reader
	MFRC522_Init();
	deactivateAntenna();

	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) waves, 400);
	HAL_TIM_Base_Start(&htim2);

	while (HAL_GetTick() <= 100)
		;      //make sure its been 100ms minimum

	ssd1306_Init();
	ssd1306_FlipScreenVertically();
	ssd1306_SetColor(White);

	HAL_UART_Transmit_IT(&huart2, (uint8_t*) studentNum,
			sizeof(studentNum) - 1);      //transmit student num
	HAL_UART_Receive_IT(&huart2, &rx, 1);      //get ready to recieve a byte

	// Try mounting the filesystem
	fres = f_mount(&FatFs, "", 1);
	if (fres != FR_OK) {
		//all hope is lost
		//HAL_GPIO_TogglePin(D4_GPIO_Port, D4_Pin); // Signal error
	}

	fres = f_open(&fil, "log.csv", FA_READ);

	UINT bytesRead;
	char last_units[7] = { 0 };
	if (fres == FR_OK) {//try open, and only work if the file has data in it.
		if (f_size(&fil) > 8) {
			f_lseek(&fil, f_size(&fil) - 8);
			fres = f_read(&fil, last_units, 7, &bytesRead);
		}
		f_close(&fil);

		units_left = atof(last_units);
		if (units_left == 0 && last_units[3] != '.') {//if atof failed fr and not because units ran out (Should never happen again)
			units_left = 10.0099;
		}
		prevUnits = units_left;

	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if (log_len > 0) {
			//Try sneak in a log if possible
			UINT bytesWritten;
			HAL_TIM_Base_Stop(&htim2);
			fres = f_open(&fil, "log.csv", FA_WRITE | FA_OPEN_APPEND);
			if (fres != FR_OK) {
				break;
			}
			fres = f_write(&fil, logbuff, log_len, &bytesWritten);
			if (fres == FR_OK) {
				log_len = 0;
			}
			f_close(&fil);
			ADC_back_on = HAL_GetTick();

		}

		if (HAL_GetTick() - ADC_back_on > 220
				&& HAL_GetTick() - ADC_back_on < 1220) {
			ADC_back_on -= 1000;
			HAL_TIM_Base_Start_IT(&htim2);
		}

		if (round(prevUnits * 1000.0) - round(units_left * 1000) >= 0.001) { //TODO CHECK IF ROUND INSTEAD OF FLOOR
			prevUnits = units_left;
			HAL_GPIO_TogglePin(D4_GPIO_Port, D4_Pin);
		}

		if (HAL_GetTick() - rfid_timeout < 5000
				&& HAL_GetTick() - rfid_evt > 50) {
			rfid_evt = HAL_GetTick();
			if (rfid_state == poll) {
				if (status == MI_OK) {
					status = MFRC522_Anticoll(rfid_data);
					rfid_state++;

				} else {
					status = MFRC522_Request(PICC_REQIDL, rfid_data);
				}
			} else if (rfid_state == anti_collision) {
				if (status == MI_OK) {
					status = MFRC522_SelectTag(rfid_data);
					rfid_state++;

				} else {
					status = MFRC522_Request(PICC_REQIDL, rfid_data);
				}
			} else {
				if (status != MI_OK) {
					if (strncmp((char*) rfid_data, (char*) valid_keycard, 5)
							== 0
							|| strncmp((char*) rfid_data, (char*) valid_keyfob,
									5) == 0
							|| strncmp((char*) rfid_data,
									(char*) valid_studentcard, 5) == 0) {
						deactivateAntenna();
						if (menu_states[lvl3] == 1) {
							count_units = 1;
						} else if (menu_states[lvl3] == 2) {
							count_units = 0;
						} else if (menu_states[lvl3] == 3) {
							add_units(newunits);
						}
						newunits = 0;
						HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, GPIO_PIN_SET);
						rfid_timeout -= 5000;
					} else {
						status = MFRC522_Request(PICC_REQIDL, rfid_data);
					}
					rfid_state = poll;

				} else {
					status = MFRC522_Request(PICC_REQIDL, rfid_data);
				}

			}

		} else if (HAL_GetTick() - rfid_timeout > 5000
				&& HAL_GetTick() - rfid_timeout < 5100) {
			deactivateAntenna();
			rfid_timeout -= 5000;
		}     //TODO maybe use this to get a seperate page for RFID verification

		if (stat_ready) {
			stats();
			stat_ready = 0;
		}

		if (dump_ready) {
			dump_file("log.csv");
		}
		if (num_keys > 0) {
			if (menupos == topmenu) {
				switch (key_buffer[num_keys]) {
				case '0':
				case '1':
				case '2':
				case '3':
					menu_states[topmenu] =
							(uint8_t) (key_buffer[num_keys] - '0');
					break;
				case '*':
					menupos = lvl1;
					menu_states[menupos] = 1;
				}
			} else if (menupos == lvl1) {
				switch (key_buffer[num_keys]) {
				case '#':
					menupos = topmenu;
					break;
				case '1':
					menupos = lvl2;
					menu_states[menupos] = 1;
					break;
				case '2':
					menupos = lvl2;
					menu_states[menupos] = 4;
					break;
				}
			} else if (menupos == lvl2) {
				switch (key_buffer[num_keys]) {
				case '#':
					menupos = lvl1;
					break;
				case '1':
				case '2':
					if (menu_states[menupos] < 4) {
						menu_states[menupos] = key_buffer[num_keys] - '0' + 1;
					} else if (menu_states[menupos] == 4) {
						menupos = lvl3;
						newunits = 0;
						menu_states[menupos] = (key_buffer[num_keys] - '1') * 3;
					}
					break;
				case '*':
					if (menu_states[menupos] == 2) {
						toggle_load(on);

						menupos = topmenu;
						menu_states[menupos] = 0;
					} else if (menu_states[menupos] == 3) {
						toggle_load(off);

						menupos = topmenu;
						menu_states[menupos] = 0;
					}
					break;
				}

			} else if (menupos == lvl3) {
				switch (key_buffer[num_keys]) {
				case '#':
					menupos = lvl2;
					newunits = 0;
					break;
				case '*':
					if (menu_states[menupos] > 0 && menu_states[menupos] <= 3
							&& !(menu_states[menupos] == 3 && newunits == 0)) {
						rfid_timeout = HAL_GetTick();
						activateAntenna();
						rfid_state = poll;
						status = MFRC522_Request(PICC_REQIDL, rfid_data);
					} else {      //don't returrn to top if nothing is selected
						break;
					}
					menupos = topmenu;
					menu_states[menupos] = 0;
					break;
				case '1':
				case '2':
					if (menu_states[menupos] < 3) {
						menu_states[menupos] = key_buffer[num_keys] - '0';
						break;
					}
				default:
					add_unit_digit(key_buffer[num_keys] - '0');
					break;
				}

			}
			num_keys--;
		}

		if (push_button_pressed) {
			if (HAL_GetTick() - button_time > 25) {
				if (!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5))
					toggle_load(toggle);
				push_button_pressed = 0;
			}
		}

		if (debug_button_pressed) {
			if (HAL_GetTick() - debug_button_time > 25) {
				if (!HAL_GPIO_ReadPin(DEBUG_BTN_GPIO_Port, DEBUG_BTN_Pin))
					//f_mount(NULL, "", 0);
					debug_screen = (debug_screen + 1) % 12;
				debug_button_pressed = 0;
			}
		}

		if (HAL_GetTick() - last_keypad_poll > 25) {
			last_keypad_poll = HAL_GetTick();
			poll_keypad();
		}

		if (HAL_GetTick() - last_screen_update > 100) {
			last_screen_update = HAL_GetTick();

			char temp_buff[22];
			if (debug_screen) {
				ssd1306_Clear();
				switch (debug_screen) {
				case 1:
					for (uint8_t row = 0; row < 32; row++) {
						ssd1306_DrawPixel(waves[row] / 32, row);
					}
					break;
				case 2:
					for (uint8_t col = 0; col < 120; col++) {
						ssd1306_DrawPixel(col, 32 - waves[col] / 128);
					}
					break;

				default:
					for (uint8_t col = 0; col < 120; col++) {
						ssd1306_DrawPixel(col,
								(waves[0] - waves[col])
										/ (1 << (debug_screen - 3)) + 16);
					}
					ssd1306_SetCursor(0, 0);
					ssd1306_SetColor(Black);
					ssd1306_DrawRect(0, 0, 28, 10);
					ssd1306_SetColor(White);
					sprintf(studentNum, "%4d", waves[0]);
					ssd1306_WriteString(studentNum, Font_7x10);

					ssd1306_SetCursor(0, 20);
					ssd1306_SetColor(Black);
					ssd1306_DrawRect(0, 20, 14, 10);
					ssd1306_SetColor(White);
					sprintf(studentNum, "%2d", 1 << (debug_screen - 3));
					ssd1306_WriteString(studentNum, Font_7x10);
					break;

				}

			} else if (HAL_GetTick() - rfid_timeout < 5000) {
				ssd1306_Clear();
				ssd1306_SetCursor(0, 0);
				ssd1306_WriteString("Present Tag/Card  ", Font_7x10);
				uint32_t length = (5000 - HAL_GetTick() + rfid_timeout) * (128)
						/ 5000;
				ssd1306_DrawHorizontalLine(0, 15, (int16_t) length);
			} else if (menupos == topmenu && menu_states[menupos] == 0) {
				ssd1306_SetCursor(0, 0);
				sprintf(temp_buff, "Apparent:  %04.0fVA ",
						voltageReal * currentReal / 1000.0);
				ssd1306_WriteString(temp_buff, Font_7x10);

				ssd1306_SetCursor(0, 10);
				sprintf(temp_buff, "Energy/d: 00000Wh ");
				ssd1306_WriteString(temp_buff, Font_7x10);

				ssd1306_SetCursor(0, 20);
				sprintf(temp_buff, "Units: %07.3fkWh ", units_left);
				ssd1306_WriteString(temp_buff, Font_7x10);

			} else if (menupos == topmenu && menu_states[menupos] == 1) {
				ssd1306_SetCursor(0, 0);
				sprintf(temp_buff, "Voltage:   %05.1fV ", voltageReal);
				ssd1306_WriteString(temp_buff, Font_7x10);

				ssd1306_SetCursor(0, 10);
				sprintf(temp_buff, "Current:  %05.0fmA ", currentReal);
				ssd1306_WriteString(temp_buff, Font_7x10);

				ssd1306_SetCursor(0, 20);
				sprintf(temp_buff, "Phase:  % 06.3fRad ", phaseReal);
				ssd1306_WriteString(temp_buff, Font_7x10);
			} else if (menupos == topmenu && menu_states[menupos] == 2) {
				float power_fact = fabs(cos(phaseReal));
				float apparent = voltageReal * currentReal / 1000.0;
				ssd1306_SetCursor(0, 0);
				sprintf(temp_buff, "Real:       %04.0fW ",
						apparent * power_fact);
				ssd1306_WriteString(temp_buff, Font_7x10);

				ssd1306_SetCursor(0, 10);
				sprintf(temp_buff, "Reactive:% 05.0fVAR ",
						apparent * sin(phaseReal));
				ssd1306_WriteString(temp_buff, Font_7x10);

				ssd1306_SetCursor(0, 20);
				sprintf(temp_buff, "PowerFact:  %05.3f ", power_fact);
				ssd1306_WriteString(temp_buff, Font_7x10);
			} else if (menupos == topmenu && menu_states[menupos] == 3) {
				RTC_DateTypeDef currDate;
				RTC_TimeTypeDef currTime;
				HAL_RTC_GetTime(&hrtc, &currTime, RTC_FORMAT_BIN);
				HAL_RTC_GetDate(&hrtc, &currDate, RTC_FORMAT_BIN);

				ssd1306_SetCursor(0, 0);
				sprintf(temp_buff, "%02u:%02u:%02u %02u/%02u/%02u ",
						currTime.Hours, currTime.Minutes, currTime.Seconds,
						currDate.Year, currDate.Month, currDate.Date);
				ssd1306_WriteString(temp_buff, Font_7x10);

				ssd1306_SetCursor(0, 10);
				sprintf(temp_buff, "Max power:  3000W ");
				ssd1306_WriteString(temp_buff, Font_7x10);

				ssd1306_SetCursor(0, 20);
				sprintf(temp_buff, "Min Units: 010kWh ");
				ssd1306_WriteString(temp_buff, Font_7x10);
			} else if (menupos == lvl1 && menu_states[menupos] == 1) {
				ssd1306_SetCursor(0, 0);
				ssd1306_WriteString("Menu(Lv1 1):      ", Font_7x10);

				ssd1306_SetCursor(0, 10);
				ssd1306_WriteString("1: Load On/Off    ", Font_7x10);

				ssd1306_SetCursor(0, 20);
				ssd1306_WriteString("2: manage Units   ", Font_7x10);
			} else if (menupos == lvl2 && menu_states[menupos] < 4) {
				ssd1306_SetCursor(0, 0);
				ssd1306_WriteString("Load on/off       ", Font_7x10);

				ssd1306_SetCursor(0, 10);
				switch (menu_states[menupos]) {
				case 1:
					ssd1306_WriteString("1=ON     2=OFF    ", Font_7x10);
					break;
				case 2:
					ssd1306_WriteString("1=ON#    2=OFF    ", Font_7x10);
					break;
				case 3:
					ssd1306_WriteString("1=ON     2=OFF#   ", Font_7x10);
					break;
				}
				ssd1306_SetCursor(0, 20);
				ssd1306_WriteString("'*' to confirm    ", Font_7x10);
			} else if (menupos == lvl2 && menu_states[menupos] == 4) {
				ssd1306_SetCursor(0, 0);
				ssd1306_WriteString("Menu(Lv1 2):      ", Font_7x10);

				ssd1306_SetCursor(0, 10);
				ssd1306_WriteString("1: Count On/Off   ", Font_7x10);

				ssd1306_SetCursor(0, 20);
				ssd1306_WriteString("2: Add Units      ", Font_7x10);
			} else if (menupos == lvl3 && menu_states[menupos] < 3) {
				ssd1306_SetCursor(0, 0);
				ssd1306_WriteString("Unit Count on/off ", Font_7x10);

				ssd1306_SetCursor(0, 10);
				switch (menu_states[menupos]) {
				case 0:
					ssd1306_WriteString("1=ON     2=OFF    ", Font_7x10);
					break;
				case 1:
					ssd1306_WriteString("1=ON#    2=OFF    ", Font_7x10);
					break;
				case 2:
					ssd1306_WriteString("1=ON     2=OFF#   ", Font_7x10);
					break;
				}
				ssd1306_SetCursor(0, 20);
				ssd1306_WriteString("'*' to confirm    ", Font_7x10);
			} else if (menupos == lvl3 && menu_states[menupos] == 3) {
				ssd1306_SetCursor(0, 0);
				ssd1306_WriteString("Units to add:     ", Font_7x10);

				ssd1306_SetCursor(0, 10);
				sprintf(temp_buff, "%-19lu", newunits);
				ssd1306_WriteString(temp_buff, Font_7x10);

				ssd1306_SetCursor(0, 20);
				ssd1306_WriteString("Press '*' to add  ", Font_7x10);
			} else { //Debug step - if state machine has impossible states, screen goes black
				ssd1306_Clear();
			}
			ssd1306_UpdateScreen();
			last_screen_update = HAL_GetTick();
		}
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV32;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T2_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_12CYCLES_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_12CYCLES_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00C12166;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 1023, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
	RTC_DateTypeDef startDate = { 0 };
	startDate.Date = 26;
	startDate.Month = 2;
	startDate.Year = 25;

	RTC_TimeTypeDef startTime = { 0 };
	startTime.Hours = 22;
	startTime.Minutes = 12;
	startTime.Seconds = 42;

	if (HAL_RTC_SetDate(&hrtc, &startDate, RTC_FORMAT_BIN) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_RTC_SetTime(&hrtc, &startTime, RTC_FORMAT_BIN) != HAL_OK) {
		Error_Handler();
	}

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 15;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 799;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 57600;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_EVEN;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX1_OVR_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX1_OVR_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX1_OVR_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RFID_CS_GPIO_Port, RFID_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, ROW4_Pin|D4_Pin|CS__SD_Pin|ROW1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, D2_Pin|D3_Pin|ROW3_Pin|ROW2_Pin
                          |RFID_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DEBUG_BTN_Pin */
  GPIO_InitStruct.Pin = DEBUG_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DEBUG_BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RFID_CS_Pin */
  GPIO_InitStruct.Pin = RFID_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RFID_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ROW4_Pin D4_Pin ROW1_Pin */
  GPIO_InitStruct.Pin = ROW4_Pin|D4_Pin|ROW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : COL3_Pin */
  GPIO_InitStruct.Pin = COL3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(COL3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PUSHBTN_Pin */
  GPIO_InitStruct.Pin = PUSHBTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(PUSHBTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : COL2_Pin */
  GPIO_InitStruct.Pin = COL2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(COL2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : D2_Pin D3_Pin ROW3_Pin ROW2_Pin
                           RFID_RST_Pin */
  GPIO_InitStruct.Pin = D2_Pin|D3_Pin|ROW3_Pin|ROW2_Pin
                          |RFID_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : COL1_Pin */
  GPIO_InitStruct.Pin = COL1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(COL1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS__SD_Pin */
  GPIO_InitStruct.Pin = CS__SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS__SD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : D5_Pin */
  GPIO_InitStruct.Pin = D5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(D5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RFID_IRQ_Pin */
  GPIO_InitStruct.Pin = RFID_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(RFID_IRQ_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void add_units(uint32_t units) {
	float new_amt = units_left + units / 1000.0;
	if (floor(new_amt) <= 999.0) {
		units_left = new_amt;
	}
}

void add_unit_digit(int8_t digit) {
	if (!(newunits >= 10000)) {
		newunits = newunits * 10 + digit;
	}
}

void add_keypress(char key) {
	if (num_keys < 9) {
		num_keys++;
		key_buffer[num_keys] = key;
	}
}

void toggle_load(enum load_options option) {
	if (option == toggle) {
		option = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
	}

	if (option == off) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	} else if (units_left > 0.0) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	}

}

void poll_keypad(void) {
	uint16_t new_mask = 0;

	GPIOC->BSRR = (uint32_t) GPIO_PIN_7;
	if (GPIOB->IDR & GPIO_PIN_0) {
		new_mask |= 1 << 2;
	}
	if (GPIOA->IDR & GPIO_PIN_9) {
		new_mask |= 1 << 1;
	}
	if (GPIOC->IDR & GPIO_PIN_1) {
		new_mask |= 1 << 3;
	}
	GPIOC->BRR = (uint32_t) GPIO_PIN_7;

	GPIOA->BSRR = (uint32_t) GPIO_PIN_12;
	if (GPIOB->IDR & GPIO_PIN_0) {
		new_mask |= 1 << 5;
	}
	if (GPIOA->IDR & GPIO_PIN_9) {
		new_mask |= 1 << 4;
	}
	if (GPIOC->IDR & GPIO_PIN_1) {
		new_mask |= 1 << 6;
	}
	GPIOA->BRR = (uint32_t) GPIO_PIN_12;

	GPIOA->BSRR = (uint32_t) GPIO_PIN_11;
	if (GPIOB->IDR & GPIO_PIN_0) {
		new_mask |= 1 << 8;
	}
	if (GPIOA->IDR & GPIO_PIN_9) {
		new_mask |= 1 << 7;
	}
	if (GPIOC->IDR & GPIO_PIN_1) {
		new_mask |= 1 << 9;
	}
	GPIOA->BRR = (uint32_t) GPIO_PIN_11;

	GPIOC->BSRR = (uint32_t) GPIO_PIN_0;
	if (GPIOB->IDR & GPIO_PIN_0) {
		new_mask |= 1 << 0;
	}
	if (GPIOA->IDR & GPIO_PIN_9) {
		new_mask |= 1 << 10;
	}
	if (GPIOC->IDR & GPIO_PIN_1) {
		new_mask |= 1 << 11;
	}
	GPIOC->BRR = (uint32_t) GPIO_PIN_0;

	uint16_t new_activations = new_mask & (~keypad_mask);
	char keys[] = "0123456789*#";
	if (new_activations) {
		for (int i = 0; i < 12; i++) {
			if (new_activations & 1 << i) {
				add_keypress(keys[i]);
			}
		}
	}
	keypad_mask = new_mask;
}

void stats() {
	RTC_DateTypeDef currDate;
	RTC_TimeTypeDef currTime;
	HAL_RTC_GetTime(&hrtc, &currTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &currDate, RTC_FORMAT_BIN);

	float apparent = voltageReal * currentReal / 1000.0;
	float powerFact = fabs(cos(phaseReal));
	float real = apparent * powerFact;
	float reactive = apparent * sin(phaseReal);
	//REACTIVE CAN BE NEGATIVE - SHOWS LEADING OR LAGGING

	char message[221];

	sprintf(message, "20%02u/%02u/%02u %02u:%02u:%02u\n"
			"Voltage:     %05.1fV\n"
			"Current:    %05.0fmA\n"
			"Phase:    % 06.3frad\n"
			"Apparent:    %04.0fVA\n"
			"Real:         %04.0fW\n"
			"Reactive:  % 05.0fVAR\n"
			"PowerFact:    %05.3f\n"
			"Energy/d:   00000Wh\n"
			"MaxPower:     3000W\n"
			"Units:   %07.3fkWh\n", currDate.Year, currDate.Month,
			currDate.Date, currTime.Hours, currTime.Minutes, currTime.Seconds,
			voltageReal, currentReal, phaseReal, apparent, real, reactive,
			powerFact, units_left);

	HAL_UART_Transmit(&huart2, (uint8_t*) message, sizeof(message) - 1, 100);
}

void clear_file(void) {
	fres = f_unlink("log.csv");
	if (fres != FR_OK) {
		//THIS ONLY HAPPENS IF FILE IS ALREADY GONE
		f_close(&fil);
		f_unlink("log.csv");
		//toggle_load(toggle);
	}
}

void handle_inputs(uint8_t *data, uint8_t len) {
	const char *commands[] = { "*CLF#", "*Log#", "*Stat#", "*Load#", "*Dump#",
			"*Units" }; //TODO: Units command support
	char *cast_data = (char*) data;
	if (len == 5) { //CLF, Log
		if (!strncmp(cast_data, commands[0], 5)) {
			if (!logging) {
				clear_file();
			}
		} else if (!strncmp(cast_data, commands[1], 5)) {
			logging = !logging;
			FILINFO fno;
			fres = f_stat("log.csv", &fno);
			uint8_t file_exists = (fres == FR_OK);

			fres = f_open(&fil, "log.csv", FA_OPEN_APPEND | FA_WRITE);
			if (!file_exists && fres == FR_OK) {

//				const char header[] =
//						"Date and time,Voltage (V),Current (mA),Phase (rad),Units left (kWh)\n";
//				f_write(&fil, header, sizeof(header) - 1, &bw);
			}
			f_close(&fil);
		}

	} else if (len == 6) { //Stat, Load, Dump
		if (!strncmp(cast_data, commands[2], 6)) {
			stat_ready = 1;
		} else if (!strncmp(cast_data, commands[3], 6)) {
			if (HAL_GPIO_ReadPin(PUSHBTN_GPIO_Port, PUSHBTN_Pin)) {
				toggle_load(toggle);
			}
		} else if (!strncmp(cast_data, commands[4], 6)) {
			dump_ready = 1;
		}
	} else if (len == 12) { //Units
		if (!strncmp(cast_data, commands[5], 6) && cast_data[11] == '#') {
			cast_data[11] = '\0';
			uint32_t units_to_add = atoi(cast_data + 6);
			if (units_to_add != 0) {
				for (uint32_t i = 6; i < 11; i++) {
					if (!isdigit((unsigned char )cast_data[i])) {
						return;
					}
				}
				newunits = units_to_add;
				menu_states[lvl3] = 3; //emulate having just entered the units
				rfid_timeout = HAL_GetTick();
				activateAntenna();
				rfid_state = poll;
				status = MI_ERR; //trick the loop into doing the work for us
			}
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	switch (rx) { //fill buffer based on simple state machine - commands should start at '*'
	case '*': //reset input buffer when a star is seen
		input_buf_len = 1;
		input_buffer[0] = '*';
		break;
	case '\n': //handle input on '\n'
		handle_inputs(input_buffer, input_buf_len);
		input_buf_len = 0;
		break;
	default:
		input_buffer[input_buf_len] = rx;
		input_buf_len = (input_buf_len + 1) % 16;
		//reset if more chars than 16
		//star case guarantees real commands won't be lost anyways
		break;
	}

	HAL_UART_Receive_IT(&huart2, &rx, 1); //You need to toggle a breakpoint on this line!
}

void process(uint16_t *startPos) { //assume 100 measurements
	//TODO maybe split the conversion up again - zero wasn't needed
	//SIGNAL GENERATOR CALIBRATED
	const int maxInC = 3973;
	const int minInC = 342;

	const int maxInV = 4000;
	const int minInV = 363;

	const float tau = 2 * 3.1415926535;

	float interpolated_pos[2];
	uint32_t peak_peak[2];

	for (uint16_t wave = 0; wave < 2; wave++) {
		//---- MAX AND MIN ----------------------
		startPos += wave;
		uint32_t max = 0;
		uint32_t min = 4095;
		uint32_t pos_max = 0;    //position of max value
		uint32_t pos_min = 0;
		for (uint16_t i = 0; i < 200; i += 2) {
			if (*(startPos + i) > max) {
				max = *(startPos + i);
				pos_max = i;
			}
			if (*(startPos + i) < min) {
				min = *(startPos + i);
				pos_min = i;
			}

		}
//		if(pos_min==0&&startPos==(waves+1)){
//			pos_max+=1;
//			HAL_TIM_Base_Stop(&htim2);
//			uint16_t* testpt =waves;
//			HAL_TIM_Base_Start(&htim2);
//		}
		peak_peak[wave] = max - min;
		//--- PHASE ------------------------------
		uint16_t zero_pos;
		uint32_t after_zero, before_zero;
		float phase_shift = 0.0;

		pos_max /= 2;
		pos_min /= 2;
		if (pos_max > pos_min) {
			zero_pos = 2 * (pos_min + (((pos_max - pos_min) * 7) >> 4)); //start 87.5% the to the zero
			while (zero_pos < 200 && *(startPos + zero_pos) < ((max + min) / 2)) {
				zero_pos += 2;
			}
			phase_shift = 50;

		} else {
			zero_pos = 2 * (pos_max + (((pos_min - pos_max) * 7) >> 4)); //start 87.5% the to the zero
			while (zero_pos < 200 && *(startPos + zero_pos) > ((max + min) / 2)) {
				zero_pos += 2;
			}
		}

		after_zero = *(startPos + zero_pos);
		before_zero = *(startPos + zero_pos - 2);
		interpolated_pos[wave] = zero_pos / 2.0 + phase_shift
				+ ((float) ((max + min) / 2) - after_zero)
						/ ((float) after_zero - before_zero);
	}
	//phase calc
	float phase = interpolated_pos[0] - interpolated_pos[1];
	phase = phase * (tau / 100.0);

	// -- CURRENT CALC --------------
	const float maxOutC = 15000.0;

	currentReal = ((maxOutC) / (maxInC - minInC) * ((float) peak_peak[0]));

	if (currentReal <= 250.0) {
		currentReal = 0.0;
		phase = 0.0;
	}

	// -- VOLTAGE CALC -------------
	const float maxOutV = 253.0;

	voltageReal = ((maxOutV) / (maxInV - minInV) * ((float) peak_peak[1]));

	if (voltageReal <= 2.3) {
		voltageReal = 0.0;
		phase = 0.0;
	}

	if (isnan(phase) || isinf(phase)) {    //cheeky nan check
		phase = 0.0;
	}
	phase = fmod(phase, tau);
	if (phase > tau / 2.0) {
		phase -= tau;
	} else if (phase < -tau / 2.0) {
		phase += tau;
	}

	phaseReal = phase;    //*(9.0/(tau*25.0));// <- convert to degrees

	voltageAvg += voltageReal;
	currentAvg += currentReal;
	powAvg += voltageReal * currentReal / 1000.0 * cos(phaseReal);
	phaseAvg += phase;
	n++;
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == PUSHBTN_Pin) {
		push_button_pressed = 1;
		button_time = HAL_GetTick();
	} else if (GPIO_Pin == DEBUG_BTN_Pin) {
		debug_button_pressed = 1;
		debug_button_time = HAL_GetTick();
	}

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	process(&waves[200]);
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {
	process(&waves[0]);
}

void activateAntenna() {    //TODO test here?
	AntennaOn();
	//HAL_TIM_Base_Stop(&htim2);
}

void deactivateAntenna() {
	AntennaOff();
	//HAL_TIM_Base_Start(&htim2);
}

uint8_t stopped_logging = 0;

void dump_file(TCHAR *filename) {
#define CHUNK_SIZE 512
	//pause logging if possible.
	if (logging) {
		logging = 0;
		stopped_logging = 1;
	}
	uint8_t buffer[CHUNK_SIZE];
	DWORD read_offset = 0;
	UINT bytesRead, bytesWritten;
	fres = f_open(&fil, filename, FA_READ | FA_WRITE);
	if (fres == FR_NO_FILE)
		dump_ready = 0;
	if (fres != FR_OK)
		return;

	do {
		fres = f_lseek(&fil, read_offset); // Seek to the current offset
		if (fres != FR_OK) {
			f_close(&fil);
			break;
		}

		fres = f_read(&fil, buffer, CHUNK_SIZE, &bytesRead);

		if (fres != FR_OK || bytesRead == 0)
			break;

		HAL_UART_Transmit(&huart2, buffer, bytesRead, HAL_MAX_DELAY);

		read_offset += bytesRead;

		//Try sneak in a log if possible
		if (log_len > 0) {
			//HAL_TIM_Base_Stop(&htim2);
			fres = f_lseek(&fil, f_size(&fil));
			fres = f_write(&fil, logbuff, log_len, &bytesWritten);
			if (fres == FR_OK) {
				log_len = 0;
			}
			//ADC_back_on = HAL_GetTick();//DEAL WITH THIS PROPERLY LOL TODO
		}

	} while (bytesRead == CHUNK_SIZE);
	f_close(&fil);
	dump_ready = 0;

	if (stopped_logging) {
		logging = 1;
		stopped_logging = 0;
	}
}

uint8_t chirality = 0;

void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc) { //happens every 500ms. Chirality dictates when 1s passes
	if (hrtc != 0) { //if this was a valid call
		chirality = ~chirality;

		if (units_left < 10.0) {
			HAL_GPIO_TogglePin(D3_GPIO_Port, D3_Pin);
		} else {
			HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, GPIO_PIN_RESET);
		}
		if (fabs(voltageReal * currentReal / 1000.0 * cos(phaseReal))
				> 3000.0) {
			HAL_GPIO_TogglePin(D5_GPIO_Port, D5_Pin);
		} else {
			HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, GPIO_PIN_RESET);
		}

		if (n > 0) {
			voltageAvg /= (float) n;
			currentAvg /= (float) n;
			powAvg /= (float) n;
			phaseAvg /= (float) n;

			n = 0;
			if (count_units) {
				units_left -= powAvg / (7200000.0); //taking no chances - 60 sec * 60 min * 1000(kilo) * 2(HZ)

				if (units_left <= 0.0) {
					units_left = 0.0;
					toggle_load(off);
				}
			}

		}
	}

	if (!logging || chirality) {
		voltageAvg = 0.0;
		currentAvg = 0.0;
		phaseAvg = 0.0;
		powAvg = 0.0;
		return;
	}

	char line[51];
	//"YYYY/mm/dd HH:mm:ss xxx.x xxxxx x.xxx xxx.xxx"
	RTC_DateTypeDef currDate;
	RTC_TimeTypeDef currTime;
	HAL_RTC_GetTime(hrtc, &currTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(hrtc, &currDate, RTC_FORMAT_BIN);
	snprintf(line, sizeof(line),
			"20%02u/%02u/%02u %02u:%02u:%02u,%05.1f,%05.0f,%05.3f,%07.3f\n",
			currDate.Year, currDate.Month, currDate.Date, currTime.Hours,
			currTime.Minutes, currTime.Seconds, voltageAvg, currentAvg,
			phaseAvg, units_left);
	size_t linelen = strlen(line); //TODO HARDCODE THIS
	memcpy((logbuff + log_len), line, linelen);
	log_len += linelen;

	voltageAvg = 0.0;
	currentAvg = 0.0;
	powAvg = 0.0;
	phaseAvg = 0.0;
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
	while (1) {
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
