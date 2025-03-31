#include "../include/NeoPixel.h"

Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

void setupNeoPixel() {
    pixels.begin();
}

void setPixelColor(uint8_t pixel, uint8_t r, uint8_t g, uint8_t b) {
    pixels.setPixelColor(pixel, pixels.Color(r, g, b));
}

void showPixels() {
    pixels.show();
}

void clearPixels() {
    pixels.clear();
    pixels.show();
} 