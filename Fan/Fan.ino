#include <LiquidCrystal_I2C.h> // Library for LCD
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <FanController.h>

#define FAN_O_PIN 5
#define FAN_I_PIN 2
#define DOWNSTAIR_DHT_PIN 7
#define UPSTAIR_DHT_PIN 8
#define DHTTYPE DHT11

#define BUZZER_PIN 9
uint32_t MAIN_LOOP_DELAY = 1000;
const float DESIRED_TEMPERATURE = 20.00;
bool fanIsOn = false;

// Wiring: SDA pin is connected to A4 and SCL pin to A5.
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 16, 2); // Change to (0x27,20,4) for 20x4 LCD.
DHT dhtDownstair = DHT(DOWNSTAIR_DHT_PIN, DHTTYPE);
DHT dhtUpstair = DHT(UPSTAIR_DHT_PIN, DHTTYPE);
FanController fan(FAN_I_PIN, 1000, FAN_O_PIN);

void setup() {
  Serial.begin(9600);
  initDhtSensor(dhtDownstair);
  initDhtSensor(dhtUpstair);
  initLCD();
  //init fan
  fan.begin();
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
  unsigned int rpms = fan.getSpeed(); // Send the command to get RPM
  Serial.print(rpms);
  Serial.println("RPM");
  analogWrite(FAN_O_PIN, 2.25*25);
  Serial.println("speed is at 25");
  delay(20000);
  analogWrite(FAN_O_PIN, 2.25*100);
  Serial.println("speed is at 100");
  delay(20000);
  analogWrite(FAN_O_PIN, 2.25*50);
  Serial.println("speed is at 50");
  delay(20000);
  analogWrite(FAN_O_PIN, 2.25*0);
  Serial.println("speed is at 0");
  delay(20000);
}

// void loop() {
//   float temperatureUpstair = getTemperature(dhtUpstair);
//   float temperatureDownstair = getTemperature(dhtDownstair);
//   printLCD("UPSTAIR " + String(temperatureUpstair, 2) + " C", 0);
//   printLCD("DOWNSTAIR " + String(temperatureDownstair, 2) + " C", 1);

//   if (fanIsOn)
//   {
//     bool downstairIsTooCold = temperatureDownstair <= DESIRED_TEMPERATURE - 0.25;
//     bool temperatureUpstairIsConfortable = temperatureUpstair >= DESIRED_TEMPERATURE + 0.25;
//     if (temperatureUpstairIsConfortable && downstairIsTooCold)
//     {
//       turnOffFan();
//     }
//   }
//   else
//   {
//     bool downstairIsHotEnough = temperatureDownstair > DESIRED_TEMPERATURE;
//     bool upstairIsCold = temperatureUpstair < DESIRED_TEMPERATURE;
//     if (downstairIsHotEnough && upstairIsCold)
//     {
//       turnOnFan(1);
//     }
//   }
//   delay(MAIN_LOOP_DELAY);
// }

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