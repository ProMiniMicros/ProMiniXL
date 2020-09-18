#include <Arduino.h>

//#define MY_DISABLED_SERIAL // Disable Serial Monitor (for Lower Power)
#define MY_BAUD_RATE 115200  // Must match Optiboot Compilation
#define LED_ON_MS    1000    // Time LED is ON (in ms)
#define LED_OFF_MS   1000    // Time LED is OFF (in ms)

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
#ifndef MY_DISABLED_SERIAL
  Serial.begin(MY_BAUD_RATE);
  while (!Serial);
  Serial.println("Pro Mini XL - v2 - Blink & UART0 Test");
#endif
}

void loop() {
  // put your main code here, to run repeatedly:
#ifndef MY_DISABLED_SERIAL
  Serial.println("LED ON");
#endif
  digitalWrite(LED_BUILTIN, HIGH);
  delay(LED_ON_MS);
#ifndef MY_DISABLED_SERIAL
  Serial.println("LED OFF");
#endif
  digitalWrite(LED_BUILTIN, LOW);
  delay(LED_OFF_MS);
}
