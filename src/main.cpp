// Robot face — Waveshare ESP32-S3 Touch AMOLED 1.75"
// CO5300 AMOLED 466x466, QSPI, native USB for programming

#include <Arduino.h>
#include <Wire.h>
#include <Arduino_GFX_Library.h>
#include <Adafruit_NeoPixel.h>

#include "pin_config.h"
#include "face/face_renderer.h"
#include "face/face_states.h"

// NeoPixel ring
static Adafruit_NeoPixel pixels(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

static void neopixel_test() {
    Serial.println("[neo] starting test");
    pixels.begin();
    pixels.setBrightness(40);  // keep current draw modest
    pixels.clear();
    pixels.show();
    delay(200);

    // Test 1: each LED red one at a time
    Serial.println("[neo] phase 1: chase red");
    for (int i = 0; i < NEOPIXEL_COUNT; i++) {
        pixels.clear();
        pixels.setPixelColor(i, pixels.Color(255, 0, 0));
        pixels.show();
        delay(80);
    }

    // Test 2: all green
    Serial.println("[neo] phase 2: all green");
    for (int i = 0; i < NEOPIXEL_COUNT; i++) {
        pixels.setPixelColor(i, pixels.Color(0, 255, 0));
    }
    pixels.show();
    delay(500);

    // Test 3: all blue
    Serial.println("[neo] phase 3: all blue");
    for (int i = 0; i < NEOPIXEL_COUNT; i++) {
        pixels.setPixelColor(i, pixels.Color(0, 0, 255));
    }
    pixels.show();
    delay(500);

    // Test 4: rainbow wheel
    Serial.println("[neo] phase 4: rainbow");
    for (int frame = 0; frame < 256; frame += 4) {
        for (int i = 0; i < NEOPIXEL_COUNT; i++) {
            int hue = (frame + i * (256 / NEOPIXEL_COUNT)) & 0xFF;
            // Simple HSV-ish wheel
            uint8_t r, g, b;
            if (hue < 85) { r = hue * 3; g = 255 - hue * 3; b = 0; }
            else if (hue < 170) { uint8_t h = hue - 85; r = 255 - h * 3; g = 0; b = h * 3; }
            else { uint8_t h = hue - 170; r = 0; g = h * 3; b = 255 - h * 3; }
            pixels.setPixelColor(i, pixels.Color(r, g, b));
        }
        pixels.show();
        delay(20);
    }

    pixels.clear();
    pixels.show();
    Serial.println("[neo] test done");
}

// AXP2101 PMIC — must enable LCD power rails before display init
#define AXP2101_ADDR 0x34

// Display bus + panel — CO5300 driver (not RM67162)
static Arduino_DataBus *bus = new Arduino_ESP32QSPI(
    LCD_CS, LCD_SCLK, LCD_SDIO0, LCD_SDIO1, LCD_SDIO2, LCD_SDIO3);

static Arduino_GFX *gfx = new Arduino_CO5300(
    bus, LCD_RESET, 0 /*rotation*/, LCD_WIDTH, LCD_HEIGHT,
    6, 0, 6, 0);

static FaceRenderer face;
static uint32_t last_ms = 0;
static FaceState current_state = FaceState::STANDBY;

// Main color per state (full brightness, ring scales via setBrightness)
static uint32_t state_color(FaceState s) {
    switch (s) {
        case FaceState::STANDBY:    return pixels.Color(0, 220, 230);   // cyan
        case FaceState::PROCESSING: return pixels.Color(0, 180, 255);   // blue-cyan
        case FaceState::SPEAKING:   return pixels.Color(255, 140, 20);  // orange
        case FaceState::AGGRESSIVE: return pixels.Color(255, 30, 10);   // red
    }
    return 0;
}

static void leds_set_state(FaceState s) {
    uint32_t c = state_color(s);
    for (int i = 0; i < NEOPIXEL_COUNT; i++) pixels.setPixelColor(i, c);
    pixels.show();
}

// Update LED brightness based on face energy (0..1)
static void leds_update(FaceState s, float energy) {
    // Map 0..1 -> 25%..100% of base brightness so LEDs never fully die
    float scale = 0.25f + 0.75f * energy;
    uint32_t base = state_color(s);
    uint8_t r = (uint8_t)(((base >> 16) & 0xFF) * scale);
    uint8_t g = (uint8_t)(((base >> 8)  & 0xFF) * scale);
    uint8_t b = (uint8_t)(( base        & 0xFF) * scale);
    uint32_t c = pixels.Color(r, g, b);
    for (int i = 0; i < NEOPIXEL_COUNT; i++) pixels.setPixelColor(i, c);
    pixels.show();
}

// Minimal JSON parser: {"state":0..3,"amp":0.0..1.0}
static bool parseCmd(const String &s, int &state, float &amp) {
    int si = s.indexOf("\"state\"");
    if (si < 0) return false;
    int colon = s.indexOf(':', si);
    if (colon < 0) return false;
    state = s.substring(colon + 1).toInt();
    if (state < 0 || state > 3) return false;

    amp = 0.0f;
    int ai = s.indexOf("\"amp\"");
    if (ai >= 0) {
        int ac = s.indexOf(':', ai);
        if (ac >= 0) amp = s.substring(ac + 1).toFloat();
    }
    amp = constrain(amp, 0.0f, 1.0f);
    return true;
}

static void axp_write(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(AXP2101_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

static uint8_t axp_read(uint8_t reg) {
    Wire.beginTransmission(AXP2101_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((int)AXP2101_ADDR, 1);
    return Wire.available() ? Wire.read() : 0;
}

static void axp_init() {
    Wire.begin(IIC_SDA, IIC_SCL, 400000);
    delay(10);

    // Check AXP2101 presence
    Wire.beginTransmission(AXP2101_ADDR);
    if (Wire.endTransmission() != 0) {
        Serial.println("[face] AXP2101 NOT found on I2C!");
        return;
    }
    Serial.printf("[face] AXP2101 chip ID: 0x%02X\n", axp_read(0x03));

    // Enable ALDO1 (LCD digital, 1.8V) — reg 0x92, 1.8V = 0x0C
    axp_write(0x92, 0x0C);
    // Enable ALDO2 (LCD analog, 2.8V) — reg 0x93, 2.8V = 0x18
    axp_write(0x93, 0x18);
    // Enable ALDO3 (3.3V) — reg 0x94, 3.3V = 0x1C
    axp_write(0x94, 0x1C);
    // Enable ALDO4 (3.3V) — reg 0x95, 3.3V = 0x1C
    axp_write(0x95, 0x1C);
    // Enable BLDO1 (1.8V) — reg 0x96, 1.8V = 0x0C
    axp_write(0x96, 0x0C);
    // Enable BLDO2 (3.3V) — reg 0x97, 3.3V = 0x1C
    axp_write(0x97, 0x1C);

    // Turn on ALDO1-4 + BLDO1-2 (reg 0x90 bits)
    uint8_t ldo_en = axp_read(0x90);
    axp_write(0x90, ldo_en | 0x3F);  // enable all 6 LDOs

    delay(100);
    Serial.println("[face] AXP2101 LDOs enabled");
}

void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println("\n[face] boot");

    // Power up display via PMIC before init
    axp_init();

    // NeoPixel ring test
    neopixel_test();

    if (!gfx->begin()) {
        Serial.println("[face] gfx->begin() FAILED");
        while (1) delay(500);
    }
    // Clear display RAM
    gfx->fillScreen(0x0000);
    delay(50);
    Serial.println("[face] display up");

    face.init(gfx, LCD_WIDTH, LCD_HEIGHT);
    face.setState(FaceState::STANDBY);

    // LEDs follow face at 50% brightness
    pixels.setBrightness(50);
    leds_set_state(FaceState::STANDBY);

    Serial.println("[face] ready. send: {\"state\":0-3,\"amp\":0.0-1.0}");
    Serial.println("[face] auto-transitioning to PROCESSING in 3s for demo");
    last_ms = millis();
}

static int demo_fired = 0;

void loop() {
    if (Serial.available()) {
        String line = Serial.readStringUntil('\n');
        line.trim();
        if (line.length() > 0) {
            int st = 0;
            float amp = 0.0f;
            if (parseCmd(line, st, amp)) {
                FaceState new_state = static_cast<FaceState>(st);
                face.setState(new_state, amp);
                if (new_state != current_state) {
                    current_state = new_state;
                    leds_set_state(new_state);
                }
                Serial.printf("[face] state=%d amp=%.2f\n", st, amp);
            } else {
                Serial.printf("[face] bad cmd: %s\n", line.c_str());
            }
        }
    }

    // No demo — controlled via serial commands

    uint32_t now = millis();
    uint32_t delta = now - last_ms;
    last_ms = now;

    face.tick(delta);

    // Sync LEDs to face energy
    leds_update(current_state, face.getEnergy());

    // ~15fps target
    uint32_t frame = millis() - now;
    if (frame < 66) delay(66 - frame);
}
