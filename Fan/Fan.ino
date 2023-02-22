#include <LiquidCrystal_I2C.h> // Library for LCD
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define FAN_PIN 5
#define DOWNSTAIR_DHT_PIN 7
#define UPSTAIR_DHT_PIN 8
#define DHTTYPE DHT11

#define BUZZER_PIN 9
uint32_t MAIN_LOOP_DELAY = 1000;
const float DESIRED_TEMPERATURE = 20.00;
const uint8_t MAX_FAN_SPEED = 125;
const uint8_t MIN_FAN_SPEED = 40;

// Wiring: SDA pin is connected to A4 and SCL pin to A5.
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 16, 2); // Change to (0x27,20,4) for 20x4 LCD.
DHT dhtDownstair = DHT(DOWNSTAIR_DHT_PIN, DHTTYPE);
DHT dhtUpstair = DHT(UPSTAIR_DHT_PIN, DHTTYPE);

void setup() {
  Serial.begin(9600);
  initDhtSensor(dhtDownstair);
  initDhtSensor(dhtUpstair);
  initLCD();
  //init fan
  pinMode(FAN_PIN, OUTPUT);
}

void initLCD() {
  Serial.println(F("INIT LCD"));
  lcd.init();
  lcd.backlight();
}

void initDhtSensor(DHT &dht)
{
  Serial.println(F("INIT DHT"));
  dht.begin();
  getTemperature(dht);
}
void loop() {
  float temperatureUpstair = getTemperature(dhtUpstair);
  float temperatureDownstair = getTemperature(dhtDownstair);

  uint16_t speed = 0;
  if (temperatureDownstair > temperatureUpstair && temperatureUpstair < DESIRED_TEMPERATURE)
  {
    speed = (int)(temperatureDownstair-temperatureUpstair)*40;
    speed = min(MAX_FAN_SPEED, max(MIN_FAN_SPEED, speed));
  }
  
  analogWrite(FAN_PIN, speed);

  printLCD("UPS " + String(temperatureUpstair, 2) + " | " + String(speed), 0);
  printLCD("DOWNSTAIR " + String(temperatureDownstair, 2), 1);

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