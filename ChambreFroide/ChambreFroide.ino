#include <LiquidCrystal_I2C.h> // Library for LCD
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHT_PIN 7
#define DHTTYPE DHT11

#define BUZZER_PIN 9
uint32_t MAIN_LOOP_DELAY = 10000;
const float COLD_TEMP = 5;
const float WARN_TEMP = 2.5;
const uint16_t BUZZ_TIME = 5000;

// Wiring: SDA pin is connected to A4 and SCL pin to A5.
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 16, 2); // Change to (0x27,20,4) for 20x4 LCD.

DHT dht = DHT(DHT_PIN, DHTTYPE);

void setup() {
  Serial.begin(9600);
  initDhtSensor(dht);
  initLCD();
  //init buzzer
  pinMode(BUZZER_PIN, OUTPUT);
}

void initLCD() {
  Serial.println(F("INIT LCD"));
  lcd.init();
  lcd.noBacklight();
}

void initDhtSensor(DHT &dht)
{
  Serial.println(F("INIT DHT"));
  dht.begin();
  getTemperature(dht);
}

void loop() {
  float temperature = getTemperature(dht);
  printLCD(String(temperature, 2) + " C", 0);
  if (temperature <= WARN_TEMP)
  {
    printLCD("TOO COLD", 1);
    lcd.backlight();
    buzz();
  }
  else if (temperature <= COLD_TEMP)
  {
    printLCD("So cold", 1);
    lcd.backlight();
  }
  else
  {
    lcd.noBacklight();
  }
  delay(MAIN_LOOP_DELAY);
}

float getTemperature(DHT &dht)
{
  float temp = dht.readTemperature();
  Serial.println(F("------------------------------------"));
  Serial.print  (F("Temperature : ")); Serial.println(temp);
  Serial.println(F("------------------------------------"));
  return temp;
}

void printLCD(String message, uint8_t row) {
  lcd.setCursor(0, row); // Set the cursor on the third column and first row.
  lcd.print("                ");
  lcd.setCursor(0, row); // Set the cursor on the third column and first row.
  lcd.print(message);
}

void buzz()
{
  tone(BUZZER_PIN,3000,BUZZ_TIME);
}