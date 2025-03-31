#ifndef NEOPIXEL_H
#define NEOPIXEL_H

#include <Adafruit_NeoPixel.h>

#define NEOPIXEL_PIN          5         // NeoPixel LED control pin
#define NUMPIXELS             4         // Number of NeoPixel LEDs

void setupNeoPixel();
void setPixelColor(uint8_t pixel, uint8_t r, uint8_t g, uint8_t b);
void showPixels();
void clearPixels();

#endif 