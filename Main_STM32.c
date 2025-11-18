/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Proyecto 3 Digital II | Sophia Franke y Dulce Ovando
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fatfs_sd.h"
#include "string.h"
#include "stdio.h"
#include "ili9341.h"
#include "fonts.h"
#include "math.h"
#include "stdbool.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// PROTOCOLO DE COMANDOS UART
#define CMD_START_MEASURE    0x01  // Iniciar medición
#define CMD_INHALE           0x02  // Fase de inhalación
#define CMD_EXHALE           0x03  // Fase de exhalación
#define CMD_STOP_MEASURE     0x04  // Detener medición
#define CMD_GET_DATA         0x05  // Solicitar datos finales

// RESPUESTAS DEL ESP32
#define RESP_READY           0xAA  // ESP32 listo
#define RESP_MEASURING       0xBB  // Midiendo
#define RESP_DATA_READY      0xCC  // Datos listos
#define RESP_ERROR           0xEE  // Error

// Estados de sincronización
typedef enum {
    SYNC_IDLE = 0,
    SYNC_STARTING,
    SYNC_BREATHING,
    SYNC_GETTING_DATA,
    SYNC_COMPLETE
} SyncState;

SyncState syncState = SYNC_IDLE;

typedef enum {
    SYS_IDLE = 0,
    SYS_MEASURING,
    SYS_ANALYZING,
    SYS_SAVING,
    SYS_ERROR
} SystemState;

typedef struct {
    float temps[100];       // Historial de temperaturas
    float humidities[100];  // Historial de humedad
    float pressures[100];   // Historial de presión
    uint32_t timestamps[100]; // Tiempos de cada medición
    uint8_t count;          // Número de mediciones en la sesión
    uint32_t start_time;    // Tiempo de inicio
    uint32_t end_time;      // Tiempo de fin
} BreathSession;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
FATFS fs;          // Sistema de archivos (montaje SD)
FIL fil;           // Archivo
FRESULT fres;      // Resultado de funciones FatFs
char buffer[100];  // Buffer general para mensajes

// Variables para la interfaz
uint8_t menu_state = 0;  // 0=menu principal, 1=leyendo, 2=mostrando datos, 3=guardando
uint8_t measurements_count = 0;

//Datos del sensor de respiración
float last_temp = 0;
float last_hum = 0;
float last_pressure = 0;
float temp_change = 0;
float breath_rate = 0;
bool dataReady = false;

// datos de las typedef
BreathSession currentSession;
bool sessionActive = false;
SystemState currentSystemState = SYS_IDLE;

int8_t contador1 = 0;
int8_t contador2 = 0;

uint32_t last_btn1_time = 0;
uint32_t last_btn2_time = 0;
const uint32_t DEBOUNCE_DELAY = 300;

uint8_t datoTEMP = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
//Transmisió
void transmit_uart(char *string);
void transmit_pc(char *string);
void transmit_esp(char command);

//Pantalla
void LCD_DrawMenu(void);
void LCD_DrawHeader(void);
void LCD_DrawStatusCard(const char *status, uint16_t color);
void LCD_DrawBreathData(void);
void LCD_DrawBreathGuide(void);
void SaveSessionToSD(void);
void ReadBreathData(void);
void StartBreathSession(void);
void AddMeasurementToSession(void);
uint8_t IsButtonPressed(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint32_t* last_press_time);
void UpdateSystemState(SystemState newState);

//Botones
void ProcessButton1(void);
void ProcessButton2(void);
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
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_FATFS_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  transmit_pc("Iniciando Sistema...");
  HAL_Delay(200);

  // Inicializar LCD
  transmit_pc("Iniciando LCD...\r\n");
  ILI9341_Init();
  HAL_Delay(100);

  //Pantalla de Inicio
  ILI9341_FillScreen(ROSE_GOLD);
  ILI9341_WriteString(10, 50, "PROYECTO 3:", &Font_16x26, BLACK, ROSE_GOLD);
  ILI9341_WriteString(10, 75, "Terapia", &Font_16x26, BLACK, ROSE_GOLD);
  ILI9341_WriteString(10, 95, "Respiratoria", &Font_16x26, BLACK, ROSE_GOLD);
  ILI9341_WriteString(10, 115, "Sophia Franke", &Font_16x26, BLACK, ROSE_GOLD);
  ILI9341_WriteString(10, 135, "Dulce Ovando", &Font_16x26, BLACK, ROSE_GOLD);

  transmit_pc("LCD inicializada.\r\n");
  HAL_Delay(200);

  // Montar SD. Al final no agarro :((( creo es problema de la manera en la que se guarda
  fres = f_mount(&fs, "/", 1);
  if (fres == FR_OK) {
      transmit_pc("SD: MONTADA \r\n");
      ILI9341_FillRectangle(230, 215, 80, 20, MINT_GREEN);
      ILI9341_WriteString(240, 218, "SD OK", &Font_7x10, WHITE, MINT_GREEN);
  } else {
      transmit_pc("SD: ERROR \r\n");
      ILI9341_FillRectangle(230, 215, 80, 20, PINK);
      ILI9341_WriteString(235, 218, "SD ERR", &Font_7x10, WHITE, PINK);
  }
  LCD_DrawMenu();
  transmit_pc("\r\nSistema Listo\r\n");
  UpdateSystemState(SYS_IDLE);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//Botones y cada uno lleva a una función diferente para procesarlos para hacer el loop lo más corto posible y no sobresaturar el micro.
	      if (IsButtonPressed(BTN1_GPIO_Port, BTN1_Pin, &last_btn1_time)) {
	          ProcessButton1();
	          }

	          if (IsButtonPressed(BTN2_GPIO_Port, BTN2_Pin, &last_btn2_time)) {
		          ProcessButton2();
	          }

	          HAL_Delay(10);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_DC_Pin|LCD_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_SS_GPIO_Port, SD_SS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : BTN1_Pin */
  GPIO_InitStruct.Pin = BTN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BTN1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN2_Pin */
  GPIO_InitStruct.Pin = BTN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BTN2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_DC_Pin */
  GPIO_InitStruct.Pin = LCD_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LCD_DC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_RESET_Pin */
  GPIO_InitStruct.Pin = LCD_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_SS_Pin */
  GPIO_InitStruct.Pin = SD_SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(SD_SS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//Aquí empiezan todas las funciones, hice bastantes.

//funciones de transmisión
void transmit_uart(char *string) {
    HAL_UART_Transmit(&huart2, (uint8_t*) string, strlen(string), 1000);
}

void transmit_pc(char *string) {
    HAL_UART_Transmit(&huart2, (uint8_t*)string, strlen(string), HAL_MAX_DELAY);
}

void transmit_esp(char command) {
    HAL_UART_Transmit(&huart3, (uint8_t*)&command, 1, HAL_MAX_DELAY);
}

//Función para ver que el botón esté presionado y así seleccionarlo (DEBOUNCE)
uint8_t IsButtonPressed(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint32_t* last_press_time) {
    uint32_t current_time = HAL_GetTick();

    // Con PULLUP, el botón presionado es GND
    if (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_RESET) {
        if ((current_time - *last_press_time) > DEBOUNCE_DELAY) {
            // Debounce: esperar y verificar nuevamente
            HAL_Delay(50);
            if (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_RESET) {
                *last_press_time = current_time;
                return 1;
            }
        }
    }
    return 0;
}


//Se puso el procesamiento de los botones en funciones para no sobresaturar el loop :)
void ProcessButton1(void) {
    transmit_pc("\r\n>>> Botón 1: Iniciando medicion de respiracion...\r\n");
    UpdateSystemState(SYS_MEASURING); //establecer en que parte del sistema se encuentra para poder simplificar en que función debe de etsar y poder mandar indicaciones de estado meidnate el serial.

    // Iniciar sesión si es la primera medición
    if (!sessionActive) {
        StartBreathSession();
    }

    ReadBreathData();

    if (dataReady) {
        LCD_DrawBreathData();
        HAL_Delay(3000);
    }
    // Actualizar menú principal
    LCD_DrawMenu();
}

void ProcessButton2(void) {
    transmit_pc("\r\n>>> Botón 2: Guardando sesión en SD...\r\n");
    UpdateSystemState(SYS_SAVING);

    SaveSessionToSD();

    // Actualizar menú principal
    LCD_DrawMenu();
}

//Dibujar header de la TFT usando la librería que cree
void LCD_DrawHeader(void) {
    for (uint16_t y = 0; y < 50; y++) {
        uint16_t color = BABY_PINK - (y * 8);
        ILI9341_DrawLine(0, y, 320, y, color);
    }
    ILI9341_WriteString(5, 12, "Terapia Respirativa", &Font_16x26, BLACK, CREAM);
}

//Dibujar Menú Principal
void LCD_DrawMenu(void) {
    ILI9341_FillScreen(CREAM);
    LCD_DrawHeader();

    // Indicador de estado
    ILI9341_FillRectangle(15, 50, 290, 45, WHITE);

    if (sessionActive) {
        sprintf(buffer, "Sesion: %d med", currentSession.count);
        LCD_DrawStatusCard(buffer, SKY_BLUE);
    } else if (dataReady) {
        LCD_DrawStatusCard("Datos listos", MINT_GREEN);
    } else {
        LCD_DrawStatusCard("Presiona BTN1", LAVENDER);
    }

    // Rectángulo 1: Tomar medición
    ILI9341_FillRectangle(15, 105, 290, 45, LAVENDER);
    ILI9341_DrawRectangle(15, 105, 290, 45, WHITE);
    ILI9341_WriteString(25, 115, "1:Tomar Medicion", &Font_16x26, BLACK, LAVENDER);

    // Rectángulo 2: Guardar sesión
    ILI9341_FillRectangle(15, 155, 290, 45, PEACH);
    ILI9341_DrawRectangle(15, 155, 290, 45, WHITE);
    ILI9341_WriteString(25, 165, "2:Guardar Sesion", &Font_16x26, BLACK, PEACH);

    // Rectángulo 3: Info
    ILI9341_FillRectangle(15, 205, 200, 30, MINT_GREEN);
    ILI9341_DrawRectangle(15, 205, 200, 30, WHITE);
    sprintf(buffer, "Total: %d", measurements_count);
    ILI9341_WriteString(25, 21, buffer, &Font_16x26, BLACK, MINT_GREEN);
}

//Dibujar tarjeta de estado
void LCD_DrawStatusCard(const char *status, uint16_t color) {
    ILI9341_FillRectangle(20, 55, 280, 35, color);
    sprintf(buffer, "%s", status);
    ILI9341_WriteString(35, 65, buffer, &Font_16x26, WHITE, color);
}

//Mostrar la información de la respiración
void LCD_DrawBreathData(void) {
    ILI9341_FillScreen(CREAM);
    ILI9341_WriteString(60, 60, "MEDICION", &Font_16x26, PURPLE, CREAM);

    // Temperatura (indicador principal de exhalación)
    ILI9341_FillRectangle(20, 95, 280, 40, SKY_BLUE);
    ILI9341_DrawRectangle(20, 95, 280, 40, PURPLE);
    sprintf(buffer, "Temp: %.2f C", last_temp);
    ILI9341_WriteString(30, 105, buffer, &Font_16x26, WHITE, SKY_BLUE);

    // Humedad
    ILI9341_FillRectangle(20, 145, 135, 35, PEACH);
    ILI9341_DrawRectangle(20, 145, 135, 35, PURPLE);
    sprintf(buffer, "H:%.1f%%", last_hum);
    ILI9341_WriteString(30, 152, buffer, &Font_16x26, WHITE, PEACH);

    // Presión
    ILI9341_FillRectangle(165, 145, 135, 35, MINT_GREEN);
    ILI9341_DrawRectangle(165, 145, 135, 35, PURPLE);
    sprintf(buffer, "P:%.0f", last_pressure);
    ILI9341_WriteString(170, 152, buffer, &Font_16x26, WHITE, MINT_GREEN);

    // Contador de sesión
    if (sessionActive) {
        ILI9341_FillRectangle(60, 195, 200, 30, LAVENDER);
        sprintf(buffer, "Sesion: %d/%d", currentSession.count, 100);
        ILI9341_WriteString(65, 202, buffer, &Font_16x26, PURPLE, LAVENDER);
    }
}

//Dibujar guía de respiración para que el usuario sepa que colores se van a encender cuando inhalen y exhalen
void LCD_DrawBreathGuide(void) {
    ILI9341_FillScreen(CREAM);
    ILI9341_WriteString(10, 40, "GUIA DE RESPIRACION", &Font_16x26, PURPLE, CREAM);

    // Animación de guía (círculo que crece y se contrae)
    uint16_t center_x = 160;
    uint16_t center_y = 150;

    // Inhala (círculo crece)
    ILI9341_WriteString(50, 70, "INHALA", &Font_16x26, SKY_BLUE, CREAM);
    for (uint8_t r = 10; r < 50; r += 5) {
        ILI9341_DrawCircle(center_x, center_y, r, SKY_BLUE);
        HAL_Delay(10);
    }

    ILI9341_FillRectangle(0, 90, 320, 150, CREAM);

    // Exhala (círculo se contrae)
    ILI9341_WriteString(50, 70, "EXHALA", &Font_16x26, PEACH, CREAM);
    for (uint8_t r = 50; r > 10; r -= 5) {
        ILI9341_DrawCircle(center_x, center_y, r, PEACH);
        HAL_Delay(10);
    }
}

//Actualización del Estado del Sistema
void UpdateSystemState(SystemState newState) {
    currentSystemState = newState;

    switch(newState) {
        case SYS_IDLE:
            transmit_pc("[SISTEMA] Estado: IDLE - Listo para medir\r\n");
            break;
        case SYS_MEASURING:
            transmit_pc("[SISTEMA] Estado: MIDIENDO RESPIRACION\r\n");
            break;
        case SYS_ANALYZING:
            transmit_pc("[SISTEMA] Estado: ANALIZANDO PATRON\r\n");
            break;
        case SYS_SAVING:
            transmit_pc("[SISTEMA] Estado: GUARDANDO SESION\r\n");
            break;
        case SYS_ERROR:
            transmit_pc("[SISTEMA] Estado: ERROR\r\n");
            break;
    }
}

//Iniciar nueva sesión de respiración
void StartBreathSession(void) {
    memset(&currentSession, 0, sizeof(BreathSession));
    currentSession.start_time = HAL_GetTick();
    sessionActive = true;
    UpdateSystemState(SYS_MEASURING);
    transmit_pc(">>> SESION DE RESPIRACION INICIADA\r\n");

}

//Agregar medición a la sesión
void AddMeasurementToSession(void) {
    if (sessionActive && currentSession.count < 100) {
        uint8_t idx = currentSession.count;
        currentSession.temps[idx] = last_temp;
        currentSession.humidities[idx] = last_hum;
        currentSession.pressures[idx] = last_pressure;
        currentSession.timestamps[idx] = HAL_GetTick() - currentSession.start_time;
        currentSession.count++;
        UpdateSystemState(SYS_ANALYZING);
        sprintf(buffer, ">>> Medicion %d agregada a la sesion\r\n", currentSession.count);
        transmit_pc(buffer);
    }
}

//Esta es la función más complicada que hice, pero es la encargada de realizar las mediciones y obtener los datos de la STM
void ReadBreathData(void) {

    uint8_t cmd = 1;                     // Comando para solicitar medición al ESP32
    uint8_t packet[6];                  // Buffer para almacenar el frame recibido
    uint8_t ack = 0;                    // Byte donde se recibirá el ACK
    HAL_StatusTypeDef status;           // Estado de las operaciones UART

    UpdateSystemState(SYS_ANALYZING);   // Cambiar estado del sistema a "analizando"
    transmit_pc("\r\n>>> INICIANDO MEDICIoN...\r\n"); // Notificar inicio por UART PC

    LCD_DrawBreathGuide();              // Dibujar guía visual de respiración
    ILI9341_FillScreen(CREAM);          // Limpiar pantalla con color crema
    HAL_Delay(1);

    LCD_DrawStatusCard("Enviando cmd...", SKY_BLUE); // Mostrar estado en pantalla

    uint8_t dummy;                      // Byte temporal para limpiar el buffer UART
    int cleared = 0;                    // Contador de bytes limpiados

    // Limpiar cualquier dato residual pendiente en UART3
    while(HAL_UART_Receive(&huart3, &dummy, 1, 10) == HAL_OK) {
        cleared++;                      // Contar cuántos bytes habían pendientes
    }

    sprintf(buffer, ">>> Buffer limpiado: %d bytes\r\n", cleared);
    transmit_pc(buffer);                // Reportar bytes limpiados

    // Enviar comando al ESP32
    status = HAL_UART_Transmit(&huart3, &cmd, 1, 1000);
    if (status != HAL_OK) {             // Si falla transmitir
        sprintf(buffer, ">>> Error TX UART3: %d\r\n", status);
        transmit_pc(buffer);
        ILI9341_FillScreen(CREAM);          // Limpiar pantalla con color crema
        LCD_DrawStatusCard("Error TX", RED); // Mostrar error en pantalla
        return;                          // Salir de la función
    }

    transmit_pc(">>> Comando enviado, esperando ACK...\r\n");

    // Esperar ACK = 0xAA del ESP32
    status = HAL_UART_Receive(&huart3, &ack, 1, 2000);
    if (status == HAL_OK && ack == 0xAA) {
        transmit_pc(">>> ACK recibido del ESP32!\r\n");
    } else {
        sprintf(buffer, ">>> No se recibió ACK (status=%d, valor=0x%02X)\r\n", status, ack);
        transmit_pc(buffer);             // Reportar problema, pero continuar
    }
    ILI9341_FillScreen(CREAM);          // Limpiar pantalla con color crema
    LCD_DrawStatusCard("Esperando datos...", LAVENDER); // UI de espera

    bool dataReceived = false;          // Bandera para saber si ya llegó un frame correcto

    // Intentar recibir datos hasta 5 veces
    for (int attempt = 0; attempt < 5; attempt++) {

        sprintf(buffer, ">>> Intento %d/5 de recepción...\r\n", attempt + 1);
        transmit_pc(buffer);

        // Intentar recibir los 6 bytes del frame
        status = HAL_UART_Receive(&huart3, packet, 6, 3000); // Timeout 3s

        if (status == HAL_OK) {

            // Validar inicio y fin de frame
            if (packet[0] == 0xAA && packet[5] == 0x55) {
                transmit_pc(">>> Frame válido recibido!\r\n");

                // Mostrar los bytes recibidos
                sprintf(buffer,
                    ">>> Bytes: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\r\n",
                    packet[0], packet[1], packet[2], packet[3], packet[4], packet[5]
                );
                transmit_pc(buffer);

                dataReceived = true;     // Marcar éxito
                break;                   // Salir del ciclo
            }
            else {
                // Frame recibido pero no cumple formato
                sprintf(buffer,
                    ">>> Frame inválido: inicio=0x%02X, fin=0x%02X\r\n",
                    packet[0], packet[5]
                );
                transmit_pc(buffer);
            }
        }
        else {
            // Timeout o error en la recepción
            sprintf(buffer, ">>> Timeout/Error en intento %d (status=%d)\r\n",
                    attempt + 1, status);
            transmit_pc(buffer);
        }

        HAL_Delay(200); // Pausa entre intentos
    }

    // Si después de 5 intentos no llega un frame válido
    if (!dataReceived) {
        transmit_pc(">>> ERROR: No se recibieron datos válidos del ESP32\r\n");
        ILI9341_FillScreen(CREAM);          // Limpiar pantalla con color crema
        LCD_DrawStatusCard("Error: Sin datos", RED); // Mostrar error
        UpdateSystemState(SYS_ERROR);                // Cambiar estado
        return;
    }

    // Decodificar datos recibidos
    float temp = (float)packet[1] / 10.0f;           // Temperatura real
    float hum  = (float)packet[2];                   // Humedad %
    uint16_t presRaw = ((uint16_t)packet[3] << 8) | packet[4]; // Unir bytes de presión
    float pres = (float)presRaw / 10.0f;             // Ajustar escala

    // Guardar valores globales
    last_temp = temp;
    last_hum = hum;
    last_pressure = pres;
    dataReady = true;               // Indicar que ya hay medición lista

    // Reportar por UART PC
    sprintf(buffer, ">>> DATOS RECIBIDOS: T=%.1fC, H=%.1f%%, P=%.1fhPa\r\n",
            temp, hum, pres);
    transmit_pc(buffer);
    AddMeasurementToSession();      // Guardar medición en sesión
    measurements_count++;           // Incrementar conteo

    LCD_DrawStatusCard("Medicion OK!", MINT_GREEN); // UI éxito
    UpdateSystemState(SYS_IDLE);   // Volver a estado IDLE

    transmit_pc(">>> Medicion completada exitosamente!\r\n");
}


//Guardar a la SD. Al final no funcionó la función
void SaveSessionToSD(void) {
    if (!sessionActive || currentSession.count == 0) {
        transmit_pc(">>> No hay sesion activa para guardar\r\n");
        LCD_DrawStatusCard("Sin sesion!", PEACH);
        HAL_Delay(1500);
        return;
    }

    menu_state = 3;
    LCD_DrawStatusCard("Guardando...", PEACH);
    UpdateSystemState(SYS_SAVING);

    // Finalizar sesión
    currentSession.end_time = HAL_GetTick();
    sessionActive = false;

    // Crear nombre de archivo único
    char filename[32];
    sprintf(filename, "SESSION_%03u.TXT", measurements_count);

    transmit_pc(">>> Guardando sesion en SD...\r\n");

    fres = f_open(&fil, filename, FA_CREATE_ALWAYS | FA_WRITE); //Escribir en un archivo
    if (fres != FR_OK) {
        sprintf(buffer, ">>> Error abriendo archivo: %d\r\n", fres);
        transmit_pc(buffer);
        UpdateSystemState(SYS_ERROR);
        return;
    }

    // Escribir encabezado
    char header[200];
    uint32_t duration_sec = (currentSession.end_time - currentSession.start_time) / 1000;
    sprintf(header,
        "=== SESION DE RESPIRACION #%d ===\r\n"
        "Duracion: %lu segundos\r\n"
        "Mediciones: %d\r\n"
        "================================\r\n\r\n",
        measurements_count, duration_sec, currentSession.count);

    UINT bytesWritten;
    f_write(&fil, header, strlen(header), &bytesWritten);

    // Escribir cada medición
    for (uint8_t i = 0; i < currentSession.count; i++) {
        char line[150];
        sprintf(line,
            "[%lu ms] Temp: %.2f C | Hum: %.2f %% | Pres: %.2f hPa\r\n",
            currentSession.timestamps[i],
            currentSession.temps[i],
            currentSession.humidities[i],
            currentSession.pressures[i]);

        f_write(&fil, line, strlen(line), &bytesWritten);
    }

    // Escribir resumen
    float avg_temp = 0, avg_hum = 0, avg_pres = 0;
    for (uint8_t i = 0; i < currentSession.count; i++) {
        avg_temp += currentSession.temps[i];
        avg_hum += currentSession.humidities[i];
        avg_pres += currentSession.pressures[i];
    }
    avg_temp /= currentSession.count;
    avg_hum /= currentSession.count;
    avg_pres /= currentSession.count;

   f_close(&fil); //cerrar para no corromper datos de la SD

    sprintf(buffer, ">>> Sesion guardada: %s (%u bytes)\r\n", filename, bytesWritten);
    transmit_pc(buffer);

    LCD_DrawStatusCard("Guardado!", MINT_GREEN);
    UpdateSystemState(SYS_IDLE);
    HAL_Delay(2000);

    menu_state = 0;
    LCD_DrawMenu();
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
#ifdef USE_FULL_ASSERT
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
