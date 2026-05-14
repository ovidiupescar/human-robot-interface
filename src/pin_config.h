#pragma once

// Waveshare ESP32-S3 Touch AMOLED 1.75" pin definitions
// Source: https://github.com/adamcooks/ESP32-S3-1.75inch-AMOLED-Round-Touch

// AMOLED / CO5300 QSPI
#define LCD_SDIO0   4
#define LCD_SDIO1   5
#define LCD_SDIO2   6
#define LCD_SDIO3   7
#define LCD_SCLK   38
#define LCD_CS     12
#define LCD_RESET  39
#define LCD_WIDTH  466
#define LCD_HEIGHT 466

// Touch / CST9217
#define IIC_SDA    15
#define IIC_SCL    14
#define TP_INT     11
#define TP_RESET   40

// NeoPixel ring (12 LEDs)
#define NEOPIXEL_PIN   18
#define NEOPIXEL_COUNT 12
