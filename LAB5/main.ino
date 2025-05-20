#include <DHT.h>
#include <Fuzzy.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define DHTPIN       10      
#define DHTTYPE      DHT22
#define BUTTON_UP    11       
#define BUTTON_DOWN  12       
#define FAN_PIN      6        
const int ledBarPins[5] = {2, 3, 4, 5, 6};

DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2);

Fuzzy fuzzy;
FuzzyInput* TempInput;
FuzzyOutput* fanOutput;


FuzzySet *temp_LN, *temp_SN, *temp_ZE, *temp_SP, *temp_LP;

FuzzySet *out_OFF, *out_LOW, *out_MED, *out_HIGH, *out_MAX;


float desiredTemp = 26.0;              // nhiệt độ mong muốn
unsigned long lastBtnTime = 0;
const unsigned long debounce = 200;    // chống rung

void setupFuzzy() {

  TempInput = new FuzzyInput(1);
  temp_LN = new FuzzySet(-12, -9.5, -7, -4.5);
  temp_SN = new FuzzySet(-6, -3.5, -2.5, -1);
  temp_ZE = new FuzzySet(-1, 0, 0, 1);
  temp_SP = new FuzzySet(1, 2.5, 3.5, 5);
  temp_LP = new FuzzySet(3.5, 5.5, 6.5, 7.5);
  TempInput->addFuzzySet(temp_LN);
  TempInput->addFuzzySet(temp_SN);
  TempInput->addFuzzySet(temp_ZE);
  TempInput->addFuzzySet(temp_SP);
  TempInput->addFuzzySet(temp_LP);
  fuzzy.addFuzzyInput(TempInput);


  fanOutput = new FuzzyOutput(1);
  out_OFF  = new FuzzySet(0, 0, 0, 0);
  out_LOW  = new FuzzySet(0, 14, 50, 64);
  out_MED  = new FuzzySet(46, 60, 114, 128);
  out_HIGH = new FuzzySet(110, 140, 162, 192);
  out_MAX  = new FuzzySet(200,A 220, 235, 255);
  fanOutput->addFuzzySet(out_OFF);
  fanOutput->addFuzzySet(out_LOW);
  fanOutput->addFuzzySet(out_MED);
  fanOutput->addFuzzySet(out_HIGH);
  fanOutput->addFuzzySet(out_MAX);
  fuzzy.addFuzzyOutput(fanOutput);

  // Luật: LN→MAX, SN→HIGH, ZE→MED, SP→LOW, LP→OFF
  auto addRule = [&](int i, FuzzySet* inS, FuzzySet* outS) {
    FuzzyRuleAntecedent* a = new FuzzyRuleAntecedent();
    a->joinSingle(inS);
    FuzzyRuleConsequent* c = new FuzzyRuleConsequent();
    c->addOutput(outS);
    fuzzy.addFuzzyRule(new FuzzyRule(i, a, c));
  };
  addRule(1, temp_LN, out_MAX);
  addRule(2, temp_SN, out_HIGH);
  addRule(3, temp_ZE, out_MED);
  addRule(4, temp_SP, out_LOW);
  addRule(5, temp_LP, out_OFF);
}

void setup() {
  Serial.begin(9600);
  dht.begin();
  lcd.init(); lcd.backlight();

  pinMode(BUTTON_UP,   INPUT);
  pinMode(BUTTON_DOWN, INPUT);
  pinMode(FAN_PIN,     OUTPUT);
  for (int i = 0; i < 5; i++) pinMode(ledBarPins[i], OUTPUT);

  setupFuzzy();
}

void loop() {
  // Debounce nút nhấn
  if (millis() - lastBtnTime > debounce) {
    if (digitalRead(BUTTON_UP) == HIGH) { desiredTemp += 0.5; lastBtnTime = millis(); }
    if (digitalRead(BUTTON_DOWN) == HIGH) { desiredTemp -= 0.5; lastBtnTime = millis(); }
  }
  desiredTemp = constrain(desiredTemp, 16.0, 39.0);

  // Đọc nhiệt độ
  float actualTemp = dht.readTemperature();
  if (isnan(actualTemp)) {
    lcd.clear(); lcd.setCursor(0,0); lcd.print("DHT22 tempor"); delay(1000);
    return;
  }

  // Tính tempor rồi fuzzify
  float temp = desiredTemp - actualTemp;
  fuzzy.setInput(1, temp);
  fuzzy.fuzzify();
  float pwmVal = fuzzy.defuzzify(1);

  // Xuất PWM
  analogWrite(FAN_PIN, (int)pwmVal);

  // LED bar hiển thị mức công suất (0-5)
  int level = round(pwmVal / 255.0 * 5);
  for (int i = 0; i < 5; i++) digitalWrite(ledBarPins[i], i < level);

  // Phần trăm công suất
  float pct = pwmVal / 255.0 * 100.0;

  // LCD hiển thị
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Act:"); lcd.print(actualTemp,1);
  lcd.print((char)223); lcd.print("C");
  lcd.setCursor(0,1);
  lcd.print("Des:"); lcd.print(desiredTemp,1);
  lcd.print((char)223); lcd.print("C");

  // Serial debug
  Serial.print("Act="); Serial.print(actualTemp,1);
  Serial.print("C Des="); Serial.print(desiredTemp,1);
  Serial.print("C temp="); Serial.print(temp,1);
  Serial.print(" PWM="); Serial.print(pwmVal,0);
  Serial.print(" ( "); Serial.print(pct,1); Serial.println("% )");

  delay(500);
}
