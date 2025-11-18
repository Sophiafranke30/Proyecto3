/**
 ******************************************************************************
 * @file    ili9341.c
 * @brief   ILI9341 TFT LCD | Libreria hecha por mí :))
 ******************************************************************************
 */

#include "ili9341.h"
#include <string.h>

/* Private variables */
static uint16_t lcd_width = ILI9341_WIDTH;
static uint16_t lcd_height = ILI9341_HEIGHT;

/**
 * @brief  Inicializar los pines GPIO para la comunicación con la LCD
 */
static void ILI9341_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Enable GPIO Clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* Configure Control Pins: RD, WR, RS, CS, RST */
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    // LCD_RD (PA0)
    GPIO_InitStruct.Pin = LCD_RD_PIN;
    HAL_GPIO_Init(LCD_RD_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(LCD_RD_PORT, LCD_RD_PIN, GPIO_PIN_SET);

    // LCD_WR (PA1)
    GPIO_InitStruct.Pin = LCD_WR_PIN;
    HAL_GPIO_Init(LCD_WR_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(LCD_WR_PORT, LCD_WR_PIN, GPIO_PIN_SET);

    // LCD_RS (PA4)
    GPIO_InitStruct.Pin = LCD_RS_PIN;
    HAL_GPIO_Init(LCD_RS_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(LCD_RS_PORT, LCD_RS_PIN, GPIO_PIN_SET);

    // LCD_CS (PB0)
    GPIO_InitStruct.Pin = LCD_CS_PIN;
    HAL_GPIO_Init(LCD_CS_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(LCD_CS_PORT, LCD_CS_PIN, GPIO_PIN_SET);

    // LCD_RST (PC1)
    GPIO_InitStruct.Pin = LCD_RST_PIN;
    HAL_GPIO_Init(LCD_RST_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(LCD_RST_PORT, LCD_RST_PIN, GPIO_PIN_SET);

    /* Configure Data Bus Pins (D0-D7) */
    // D0 (PA9) - Arduino D8
    GPIO_InitStruct.Pin = LCD_D0_PIN;
    HAL_GPIO_Init(LCD_D0_PORT, &GPIO_InitStruct);

    // D1 (PC7) - Arduino D9
    GPIO_InitStruct.Pin = LCD_D1_PIN;
    HAL_GPIO_Init(LCD_D1_PORT, &GPIO_InitStruct);

    // D2 (PA10) - Arduino D2
    GPIO_InitStruct.Pin = LCD_D2_PIN;
    HAL_GPIO_Init(LCD_D2_PORT, &GPIO_InitStruct);

    // D3 (PB3) - Arduino D3
    GPIO_InitStruct.Pin = LCD_D3_PIN;
    HAL_GPIO_Init(LCD_D3_PORT, &GPIO_InitStruct);

    // D4 (PB5) - Arduino D4
    GPIO_InitStruct.Pin = LCD_D4_PIN;
    HAL_GPIO_Init(LCD_D4_PORT, &GPIO_InitStruct);

    // D5 (PB4) - Arduino D5
    GPIO_InitStruct.Pin = LCD_D5_PIN;
    HAL_GPIO_Init(LCD_D5_PORT, &GPIO_InitStruct);

    // D6 (PB10) - Arduino D6
    GPIO_InitStruct.Pin = LCD_D6_PIN;
    HAL_GPIO_Init(LCD_D6_PORT, &GPIO_InitStruct);

    // D7 (PA8) - Arduino D7
    GPIO_InitStruct.Pin = LCD_D7_PIN;
    HAL_GPIO_Init(LCD_D7_PORT, &GPIO_InitStruct);
}

/**
 * @brief  Escribir datos de 8bits para la LCD
 * @note   D9=D1, D8=D0, D7=D7, D6=D6, D5=D5, D4=D4, D3=D3, D2=D2
 */
void ILI9341_Write8(uint8_t data) {

    // Port A: D0(PA9), D2(PA10), D7(PA8)
    uint32_t porta_set = 0;
    uint32_t porta_reset = (LCD_D0_PIN | LCD_D2_PIN | LCD_D7_PIN) << 16;
    if (data & 0x01) porta_set |= LCD_D0_PIN;  // D0 = bit 0
    if (data & 0x04) porta_set |= LCD_D2_PIN;  // D2 = bit 2
    if (data & 0x80) porta_set |= LCD_D7_PIN;  // D7 = bit 7
    GPIOA->BSRR = porta_reset | porta_set;

    // Port B: D3(PB3), D4(PB5), D5(PB4), D6(PB10)
    uint32_t portb_set = 0;
    uint32_t portb_reset = (LCD_D3_PIN | LCD_D4_PIN | LCD_D5_PIN | LCD_D6_PIN) << 16;
    if (data & 0x08) portb_set |= LCD_D3_PIN;  // D3 = bit 3
    if (data & 0x10) portb_set |= LCD_D4_PIN;  // D4 = bit 4
    if (data & 0x20) portb_set |= LCD_D5_PIN;  // D5 = bit 5
    if (data & 0x40) portb_set |= LCD_D6_PIN;  // D6 = bit 6
    GPIOB->BSRR = portb_reset | portb_set;

    // Port C: D1(PC7)
    uint32_t portc_set = 0;
    uint32_t portc_reset = LCD_D1_PIN << 16;
    if (data & 0x02) portc_set |= LCD_D1_PIN;  // D1 = bit 1
    GPIOC->BSRR = portc_reset | portc_set;

    //  "Strobe" de la LCD en LOW y HIGH para indicar que hay datos disponibles
    LCD_WR_LOW();
    __NOP(); __NOP(); 
    LCD_WR_HIGH();
}


void ILI9341_WriteCommand(uint8_t cmd) {
    LCD_CS_LOW();
    LCD_RS_LOW();
    ILI9341_Write8(cmd);
    LCD_CS_HIGH();
}

void ILI9341_WriteData(uint8_t data) {
    LCD_CS_LOW();
    LCD_RS_HIGH();  // Data mode
    ILI9341_Write8(data);
    LCD_CS_HIGH();
}


void ILI9341_WriteData16(uint16_t data) {
    LCD_CS_LOW();
    LCD_RS_HIGH();
    ILI9341_Write8(data >> 8);    // High byte
    ILI9341_Write8(data & 0xFF);  // Low byte
    LCD_CS_HIGH();
}


void ILI9341_Reset(void) {
    LCD_RST_HIGH();
    HAL_Delay(10);
    LCD_RST_LOW();
    HAL_Delay(20);
    LCD_RST_HIGH();
    HAL_Delay(150);
}


void ILI9341_Init(void) {
    // Inicializar GPIOs
    ILI9341_GPIO_Init();

    // Hardware reset para la LCD
    ILI9341_Reset();

    // Software reset y delay para que se estabilice
    ILI9341_WriteCommand(0x01);
    HAL_Delay(150);

    // Power Control A
    // Este comando configura los parámetros de voltaje de la pantalla ILI9341.
    ILI9341_WriteCommand(0xCF);
    ILI9341_WriteData(0x00);
    ILI9341_WriteData(0x83);
    ILI9341_WriteData(0x30);

    // Power Control B
    // Configura más parámetros de voltaje para la pantalla para un rendimiento óptimo.
    ILI9341_WriteCommand(0xED);
    ILI9341_WriteData(0x64);
    ILI9341_WriteData(0x03);
    ILI9341_WriteData(0x12);
    ILI9341_WriteData(0x81);

    // Driver timing control A
    // Ajusta los tiempos de conducción para la pantalla así como la frecuencia de operación.
    ILI9341_WriteCommand(0xE8);
    ILI9341_WriteData(0x85);
    ILI9341_WriteData(0x01);
    ILI9341_WriteData(0x79);

    // Power on sequence control
    // Ajusta la secuencia de encendido de la pantalla para asegurar un arranque correcto.
    ILI9341_WriteCommand(0xCB);
    ILI9341_WriteData(0x39);
    ILI9341_WriteData(0x2C);
    ILI9341_WriteData(0x00);
    ILI9341_WriteData(0x34);
    ILI9341_WriteData(0x02);

    // Pump ratio control
    // Configura la relación de la bomba de carga interna para optimizar el rendimiento de voltaje.
    ILI9341_WriteCommand(0xF7);
    ILI9341_WriteData(0x20);

    // Driver timing control B
    // Ajusta los tiempos de conducción adicionales para la pantalla.
    ILI9341_WriteCommand(0xEA);
    ILI9341_WriteData(0x00);
    ILI9341_WriteData(0x00);

    // Power Control 1
    // Configura el control de potencia principal para la pantalla.
    ILI9341_WriteCommand(0xC0);
    ILI9341_WriteData(0x26);

    // Power Control 2
    // Configura el control de potencia secundario para la pantalla.
    ILI9341_WriteCommand(0xC1);
    ILI9341_WriteData(0x11);

    // VCOM Control 1
    // Ajusta el voltaje de referencia VCOM para la pantalla. VCOM es crucial para la calidad de la imagen y signifca Voltage Common.
    ILI9341_WriteCommand(0xC5);
    ILI9341_WriteData(0x35);
    ILI9341_WriteData(0x3E);

    // VCOM Control 2
    // Lo mismo que el anterior pero para otro parámetro de VCOM.
    ILI9341_WriteCommand(0xC7);
    ILI9341_WriteData(0xBE);

    // Memory Access Control
    // Configura la orientación de la pantalla y el orden de los bytes.
    ILI9341_WriteCommand(0x36);
    ILI9341_WriteData(0x48);

    // Pixel Format
    // Configura el formato de color de los píxeles.
    ILI9341_WriteCommand(0x3A);
    ILI9341_WriteData(0x55);  // 16-bit color

    // Frame Rate Control
    // Configura el frame rate de la pantalla.
    ILI9341_WriteCommand(0xB1);
    ILI9341_WriteData(0x00);
    ILI9341_WriteData(0x1B);

    // Display Function Control
    // Configura funciones específicas de la pantalla.
    ILI9341_WriteCommand(0xB6);
    ILI9341_WriteData(0x0A);
    ILI9341_WriteData(0x82);
    ILI9341_WriteData(0x27);
    ILI9341_WriteData(0x00);

    // Gamma Set
    // Configura la curva gamma para la pantalla. Básicamente ajusta cómo se representan los colores.
    ILI9341_WriteCommand(0x26);
    ILI9341_WriteData(0x01);

    // Positive Gamma Correction
    //Esta es la configuración de la corrección gamma positiva, que afecta cómo se muestran los colores claros en la pantalla.
    ILI9341_WriteCommand(0xE0);
    ILI9341_WriteData(0x1F);
    ILI9341_WriteData(0x1A);
    ILI9341_WriteData(0x18);
    ILI9341_WriteData(0x0A);
    ILI9341_WriteData(0x0F);
    ILI9341_WriteData(0x06);
    ILI9341_WriteData(0x45);
    ILI9341_WriteData(0x87);
    ILI9341_WriteData(0x32);
    ILI9341_WriteData(0x0A);
    ILI9341_WriteData(0x07);
    ILI9341_WriteData(0x02);
    ILI9341_WriteData(0x07);
    ILI9341_WriteData(0x05);
    ILI9341_WriteData(0x00);

    // Negative Gamma Correction
    //Lo mismo que el anterior pero para la corrección gamma negativa, que afecta cómo se muestran los colores oscuros.
    ILI9341_WriteCommand(0xE1);
    ILI9341_WriteData(0x00);
    ILI9341_WriteData(0x25);
    ILI9341_WriteData(0x27);
    ILI9341_WriteData(0x05);
    ILI9341_WriteData(0x10);
    ILI9341_WriteData(0x09);
    ILI9341_WriteData(0x3A);
    ILI9341_WriteData(0x78);
    ILI9341_WriteData(0x4D);
    ILI9341_WriteData(0x05);
    ILI9341_WriteData(0x18);
    ILI9341_WriteData(0x0D);
    ILI9341_WriteData(0x38);
    ILI9341_WriteData(0x3A);
    ILI9341_WriteData(0x1F);

    // Sleep Out
    // Salir del modo de suspensión o sleep mode que es un estado de bajo consumo de energía.
    ILI9341_WriteCommand(0x11);
    HAL_Delay(120);

    // Display ON
    // Enciende la pantalla (como dice el nombre del comando).
    ILI9341_WriteCommand(0x29);
    HAL_Delay(50);

    // Set default rotation
    // La manera en la que se muestra la pantalla (horizontal/vertical).
    ILI9341_SetRotation(SCREEN_HORIZONTAL_1);

    // Clear screen
    ILI9341_FillScreen(BLACK);
}


//Establecer los casos de rotación de la pantalla para que se pueda ver d cualquier manera
void ILI9341_SetRotation(LCD_Orientation orientation) {
    ILI9341_WriteCommand(0x36);

    switch (orientation) {
        case SCREEN_VERTICAL_1:
            ILI9341_WriteData(0x40 | 0x08);
            lcd_width = 240;
            lcd_height = 320;
            break;
        case SCREEN_VERTICAL_2:
            ILI9341_WriteData(0x80 | 0x08);
            lcd_width = 240;
            lcd_height = 320;
            break;
        case SCREEN_HORIZONTAL_1:
            ILI9341_WriteData(0x20 | 0x08);
            lcd_width = 320;
            lcd_height = 240;
            break;
        case SCREEN_HORIZONTAL_2:
            ILI9341_WriteData(0xE0 | 0x08);
            lcd_width = 320;
            lcd_height = 240;
            break;
    }
}

//Establecer la ventana de dibujo en la pantalla para los gráficos
void ILI9341_SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    // Column Address Set
    ILI9341_WriteCommand(0x2A);
    ILI9341_WriteData(x0 >> 8);
    ILI9341_WriteData(x0 & 0xFF);
    ILI9341_WriteData(x1 >> 8);
    ILI9341_WriteData(x1 & 0xFF);

    // Page Address Set
    ILI9341_WriteCommand(0x2B);
    ILI9341_WriteData(y0 >> 8);
    ILI9341_WriteData(y0 & 0xFF);
    ILI9341_WriteData(y1 >> 8);
    ILI9341_WriteData(y1 & 0xFF);

    // Memory Write
    ILI9341_WriteCommand(0x2C);
}

//Dibujar un pixel en la pantalla
void ILI9341_DrawPixel(uint16_t x, uint16_t y, uint16_t color) {
    if (x >= lcd_width || y >= lcd_height) return;

    ILI9341_SetWindow(x, y, x, y);
    ILI9341_WriteData16(color);
}

//Rellenar toda la pantalla con un color
void ILI9341_FillScreen(uint16_t color) {
    ILI9341_FillRectangle(0, 0, lcd_width, lcd_height, color);
}

//Rellenar un rectángulo con un color
void ILI9341_FillRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    if (x >= lcd_width || y >= lcd_height) return;
    if (x + w > lcd_width) w = lcd_width - x;
    if (y + h > lcd_height) h = lcd_height - y;

    ILI9341_SetWindow(x, y, x + w - 1, y + h - 1);

    LCD_CS_LOW();
    LCD_RS_HIGH();

    uint32_t pixels = (uint32_t)w * h;
    for (uint32_t i = 0; i < pixels; i++) {
        ILI9341_Write8(color >> 8);
        ILI9341_Write8(color & 0xFF);
    }

    LCD_CS_HIGH();
}

//Dibujar la orilla de un rectángulo
void ILI9341_DrawRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    ILI9341_DrawLine(x, y, x + w, y, color);           // Top
    ILI9341_DrawLine(x, y + h, x + w, y + h, color);   // Bottom
    ILI9341_DrawLine(x, y, x, y + h, color);           // Left
    ILI9341_DrawLine(x + w, y, x + w, y + h, color);   // Right
}

//Dibujar una línea entre dos puntos usando el algoritmo de Bresenham (fue una sesión larga de youtube jsjs)
void ILI9341_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color) {
    int16_t dx = abs(x1 - x0);
    int16_t dy = abs(y1 - y0);
    int16_t sx = (x0 < x1) ? 1 : -1;
    int16_t sy = (y0 < y1) ? 1 : -1;
    int16_t err = dx - dy;

    while (1) {
        ILI9341_DrawPixel(x0, y0, color);

        if (x0 == x1 && y0 == y1) break;

        int16_t e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }
}

//Dibujar el contorno de un círculo. Las ecuaciones sí me las saqué de internet jsjs
void ILI9341_DrawCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color) {
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;

    ILI9341_DrawPixel(x0, y0 + r, color);
    ILI9341_DrawPixel(x0, y0 - r, color);
    ILI9341_DrawPixel(x0 + r, y0, color);
    ILI9341_DrawPixel(x0 - r, y0, color);

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        ILI9341_DrawPixel(x0 + x, y0 + y, color);
        ILI9341_DrawPixel(x0 - x, y0 + y, color);
        ILI9341_DrawPixel(x0 + x, y0 - y, color);
        ILI9341_DrawPixel(x0 - x, y0 - y, color);
        ILI9341_DrawPixel(x0 + y, y0 + x, color);
        ILI9341_DrawPixel(x0 - y, y0 + x, color);
        ILI9341_DrawPixel(x0 + y, y0 - x, color);
        ILI9341_DrawPixel(x0 - y, y0 - x, color);
    }
}

//Dibujar un círculo relleno
void ILI9341_FillCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color) {
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;

    ILI9341_DrawLine(x0, y0 - r, x0, y0 + r, color);

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        ILI9341_DrawLine(x0 + x, y0 - y, x0 + x, y0 + y, color);
        ILI9341_DrawLine(x0 - x, y0 - y, x0 - x, y0 + y, color);
        ILI9341_DrawLine(x0 + y, y0 - x, x0 + y, y0 + x, color);
        ILI9341_DrawLine(x0 - y, y0 - x, x0 - y, y0 + x, color);
    }
}

//Dibujar el contorno de un triángulo (no lo use pero por sí las dudas)
void ILI9341_DrawTriangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1,
                          uint16_t x2, uint16_t y2, uint16_t color) {
    ILI9341_DrawLine(x0, y0, x1, y1, color);
    ILI9341_DrawLine(x1, y1, x2, y2, color);
    ILI9341_DrawLine(x2, y2, x0, y0, color);
}

//Rellenar un triángulo
void ILI9341_FillTriangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1,
                          uint16_t x2, uint16_t y2, uint16_t color) {
    int16_t a, b, y, last;

    // Sort vertices by Y
    if (y0 > y1) { SWAP(y0, y1); SWAP(x0, x1); }
    if (y1 > y2) { SWAP(y2, y1); SWAP(x2, x1); }
    if (y0 > y1) { SWAP(y0, y1); SWAP(x0, x1); }

    if (y0 == y2) {
        a = b = x0;
        if (x1 < a) a = x1;
        else if (x1 > b) b = x1;
        if (x2 < a) a = x2;
        else if (x2 > b) b = x2;
        ILI9341_DrawLine(a, y0, b, y0, color);
        return;
    }

    int16_t dx01 = x1 - x0;
    int16_t dy01 = y1 - y0;
    int16_t dx02 = x2 - x0;
    int16_t dy02 = y2 - y0;
    int16_t dx12 = x2 - x1;
    int16_t dy12 = y2 - y1;
    int32_t sa = 0, sb = 0;

    last = (y1 == y2) ? y1 : y1 - 1;

    for (y = y0; y <= last; y++) {
        a = x0 + sa / dy01;
        b = x0 + sb / dy02;
        sa += dx01;
        sb += dx02;
        if (a > b) SWAP(a, b);
        ILI9341_DrawLine(a, y, b, y, color);
    }

    sa = dx12 * (y - y1);
    sb = dx02 * (y - y0);
    for (; y <= y2; y++) {
        a = x1 + sa / dy12;
        b = x0 + sb / dy02;
        sa += dx12;
        sb += dx02;
        if (a > b) SWAP(a, b);
        ILI9341_DrawLine(a, y, b, y, color);
    }
}

// Escribir un carácter en la pantalla
void ILI9341_WriteChar(uint16_t x, uint16_t y, char ch, const FontDef* font, uint16_t color, uint16_t bgcolor){
    if (ch < ' ' || ch > '~') return;  // solo caracteres imprimibles ASCII

    uint16_t char_index = ch - ' ';      // índice en la tabla
    uint16_t offset = char_index * font->height; // cada carácter tiene "height" filas de uint16_t
    const uint16_t *char_data = &font->data[offset];

    for (uint8_t row = 0; row < font->height; row++) {
        uint16_t row_data = char_data[row];

        for (uint8_t col = 0; col < font->width; col++) {
            // bit más significativo = píxel más a la izquierda
            if (row_data & (1 << (font->width - 1 - col))) {
                ILI9341_DrawPixel(x + col, y + row, color);
            } else {
                ILI9341_DrawPixel(x + col, y + row, bgcolor);
            }
        }
    }
}



//Escribe una cadena de texto en la pantalla 
void ILI9341_WriteString(uint16_t x, uint16_t y, const char *str, const FontDef *font,
                         uint16_t color, uint16_t bgcolor) {
    if (font == NULL) return;

    while (*str) {
        if (x + font->width >= lcd_width) { //Lo que hace es que si se pasa del ancho de la pantalla, baja una línea para que siempre quede bien puesto el texto
            x = 0; 
            y += font->height;
            if (y + font->height >= lcd_height) break;
        }

        if (*str == '\n') { //Nueva línea
            x = 0;
            y += font->height;
            str++;
            continue;
        }

        ILI9341_WriteChar(x, y, *str, font, color, bgcolor); //forma el carácter
        x += font->width;
        str++;
    }
}
