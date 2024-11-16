#include <LiquidCrystal.h>

LiquidCrystal lcd(2, 3, 4, 5, 6, 7);
const int buzzer_Pin = 11;  // I/O pin of the 3-pin buzzer
const int led_Pin = 8;      // LED pin
char sleep_status = 0;
unsigned long previousMillis = 0;  // Store last time buzzer or LED was activated
const long interval = 1000;         // Interval for LED and buzzer

void setup() {
  Serial.begin(9600);
  pinMode(buzzer_Pin, OUTPUT);  // Buzzer pin as output
  pinMode(led_Pin, OUTPUT);     // LED pin as output
  lcd.begin(16, 2);
  lcd.print("Driver Sleep ");
  lcd.setCursor(0, 1);
  lcd.print("Detection SYSTEM");
  digitalWrite(buzzer_Pin, HIGH); // Make sure buzzer is OFF initially
  digitalWrite(led_Pin, LOW);    // Make sure LED is OFF initially
}

void loop() {
  if (Serial.available() > 0) {
    sleep_status = Serial.read();

    if (sleep_status == 'a') {
      lcd.clear();
      lcd.print("Please wake up Sir..");
      digitalWrite(buzzer_Pin, HIGH);
      digitalWrite(led_Pin, HIGH);
      previousMillis = millis();  // Reset the timer
    } 
    else if (sleep_status == 'b') {
      lcd.clear();
      lcd.print("All Ok");
      lcd.setCursor(0, 1);
      lcd.print("Drive Safe..");
      digitalWrite(buzzer_Pin, HIGH);   // Ensure buzzer is OFF     // Ensure LED is OFF
      previousMillis = millis();  // Reset the timer
    }
  }

  // Check if buzzer/LED should be turned off after the interval
  if (digitalRead(buzzer_Pin) == HIGH && millis() - previousMillis >= interval) {
    digitalWrite(buzzer_Pin, LOW);
    digitalWrite(led_Pin, LOW);
  }
}
