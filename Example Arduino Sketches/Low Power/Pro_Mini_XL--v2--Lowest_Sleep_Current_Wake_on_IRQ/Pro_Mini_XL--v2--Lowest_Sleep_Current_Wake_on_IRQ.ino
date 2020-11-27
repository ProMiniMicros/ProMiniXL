#include <avr/power.h>
#include <avr/sleep.h>

#define LED_PIN 22 // PC3 on Pro Mini XL v2 or Arduino pin 15 (PD7) on Pro Mini XL v1.
#define IRQ_PIN 10 // PD2 on Pro Mini XL v2 and v1.
#define IRQ_NUM  0 // PD2 is INT0 on Pro Mini XL v2 and v1.

void awake() {
  sleep_disable();
  detachInterrupt(IRQ_NUM);
}

void setup() {
  // Configure all I/O pins as inputs and with internal pull-ups disabled.
  for (byte pin = 0; pin < NUM_DIGITAL_PINS; pin++) {
    pinMode(pin, INPUT);
    digitalWrite(pin, LOW);
  }
  pinMode(LED_PIN, OUTPUT);       // Use above define for LED.
  pinMode(IRQ_PIN, INPUT_PULLUP); // Ground this pin to wake.
}

void loop() {
  digitalWrite(LED_PIN, HIGH); // Light board LED as proof of life.
  delay(3000);                 // Behold! (in ms)
  digitalWrite(LED_PIN, LOW);  // Turn it off...

  ADCSRA = 0;          // Disable ADC.
  power_all_disable(); // Turn everything else off - must come after disabling ADC.

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // Sleep Mode: POWER-DOWN.
  noInterrupts();                       // Must not interrupt next sequence.

  attachInterrupt(IRQ_NUM, awake, LOW); // Ground IRQ_PIN to wake up.
  EIFR = bit(INTF0);                    // Clear flag for INT0 - just in case.

  sleep_enable();                       // Enable Sleep.
  MCUCR = bit(BODS) | bit(BODSE);       // Software disable Brown-out Detection (BoD).
  MCUCR = bit(BODS); 
  interrupts();                         // Now we can be interrupted again.
  sleep_cpu();                          // Sleep - must have been within 3 cycles of enabling.
  power_timer0_enable();                // delay() needs Timer0 enabled!
}
