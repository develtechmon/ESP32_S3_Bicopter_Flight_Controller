#include <Adafruit_NeoPixel.h>

// Define the pin for the built-in LED. Change this if your board uses a different pin.
#define LED_PIN 48  
#define NUM_LEDS 1

Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  strip.begin();       // Initialize the NeoPixel strip
  strip.show();        // Turn all pixels off as an initial state
}

void loop() {
  // Cycle through a few colors

  // Red
  strip.setPixelColor(0, strip.Color(255, 0, 0));  
  strip.show();
  delay(1000);
  
  // Green
  strip.setPixelColor(0, strip.Color(0, 255, 0));
  strip.show();
  delay(1000);
  
  // Blue
  strip.setPixelColor(0, strip.Color(0, 0, 255));
  strip.show();
  delay(1000);
  
  // Yellow (Red + Green)
  strip.setPixelColor(0, strip.Color(255, 255, 0));
  strip.show();
  delay(1000);
  
  // Cyan (Green + Blue)
  strip.setPixelColor(0, strip.Color(0, 255, 255));
  strip.show();
  delay(1000);
  
  // Magenta (Red + Blue)
  strip.setPixelColor(0, strip.Color(255, 0, 255));
  strip.show();
  delay(1000);
  
  // White (all colors)
  strip.setPixelColor(0, strip.Color(255, 255, 255));
  strip.show();
  delay(1000);
  
  // Turn off the LED
  strip.setPixelColor(0, strip.Color(0, 0, 0));
  strip.show();
  delay(1000);
}
