#include <DHT.h>
#include "Fuzzy.h"

// Định nghĩa các chân
#define DHTPIN 4
#define DHTTYPE DHT22
#define LED_LOW 18
#define LED_MEDIUM 19
#define LED_HIGH 21

// Khởi tạo cảm biến DHT
DHT dht(DHTPIN, DHTTYPE);

// Khởi tạo đối tượng logic mờ
Fuzzy* fuzzy = new Fuzzy();

// Khởi tạo các tập mờ cho nhiệt độ (cold, warm, hot)
FuzzySet* lowTemp = new FuzzySet(0, 0, 10, 20);
FuzzySet* mediumTemp = new FuzzySet(15, 20, 25, 35);
FuzzySet* highTemp = new FuzzySet(30, 40, 50, 50);

// Khởi tạo các tập mờ cho độ ẩm (dry, normal, humid)
FuzzySet* lowHumidity = new FuzzySet(0, 0, 20, 40);
FuzzySet* mediumHumidity = new FuzzySet(30, 40, 60, 70);
FuzzySet* highHumidity = new FuzzySet(60, 80, 100, 100);

// Khởi tạo các tập mờ cho công suất
FuzzySet* lowPower = new FuzzySet(0, 0, 20, 40);
FuzzySet* mediumPower = new FuzzySet(30, 40, 60, 70);
FuzzySet* highPower = new FuzzySet(60, 80, 100, 100);

// Định nghĩa các đối tượng mờ cho đầu vào và đầu ra
FuzzyInput* temp = new FuzzyInput(1);
FuzzyInput* humidity = new FuzzyInput(2);
FuzzyOutput* power = new FuzzyOutput(1);

// Định nghĩa các luật mờ antecedents
FuzzyRuleAntecedent *ifColdAndDry   = new FuzzyRuleAntecedent();
FuzzyRuleAntecedent *ifColdAndNormal = new FuzzyRuleAntecedent();
FuzzyRuleAntecedent *ifColdAndHumid  = new FuzzyRuleAntecedent();
FuzzyRuleAntecedent *ifWarmAndDry    = new FuzzyRuleAntecedent();
FuzzyRuleAntecedent *ifWarmAndNormal = new FuzzyRuleAntecedent();
FuzzyRuleAntecedent *ifWarmAndHumid  = new FuzzyRuleAntecedent();
FuzzyRuleAntecedent *ifHotAndDry     = new FuzzyRuleAntecedent();
FuzzyRuleAntecedent *ifHotAndNormal  = new FuzzyRuleAntecedent();
FuzzyRuleAntecedent *ifHotAndHumid   = new FuzzyRuleAntecedent();

// Định nghĩa các luật mờ consequents (chúng được tạo ở đây, nhưng sẽ được setup trong setup())
FuzzyRuleConsequent *thenLow    = new FuzzyRuleConsequent();
FuzzyRuleConsequent *thenMedium = new FuzzyRuleConsequent();
FuzzyRuleConsequent *thenHigh   = new FuzzyRuleConsequent();

void setup() {
  Serial.begin(115200);

  // Khởi động cảm biến DHT
  dht.begin();
  delay(2000); // Chờ cảm biến ổn định
  float testT = dht.readTemperature();
  float testH = dht.readHumidity();
  if (isnan(testT) || isnan(testH)) {
    Serial.println("DHT sensor error! Check wiring.");
    while (1); // Dừng chương trình nếu cảm biến lỗi
  }

  // Cấu hình chân LED
  pinMode(LED_LOW, OUTPUT);
  pinMode(LED_MEDIUM, OUTPUT);
  pinMode(LED_HIGH, OUTPUT);

  // Thêm các tập mờ vào biến mờ của nhiệt độ
  temp->addFuzzySet(lowTemp);
  temp->addFuzzySet(mediumTemp);
  temp->addFuzzySet(highTemp);
  
  // Thêm các tập mờ vào biến mờ của độ ẩm
  humidity->addFuzzySet(lowHumidity);
  humidity->addFuzzySet(mediumHumidity);
  humidity->addFuzzySet(highHumidity);
  
  // Thêm các tập mờ vào biến mờ của công suất
  power->addFuzzySet(lowPower);
  power->addFuzzySet(mediumPower);
  power->addFuzzySet(highPower);
  
  // Thêm các biến mờ vào hệ thống mờ
  fuzzy->addFuzzyInput(temp);
  fuzzy->addFuzzyInput(humidity);
  fuzzy->addFuzzyOutput(power);

  // Cấu hình các consequents
  thenLow->addOutput(lowPower);
  thenMedium->addOutput(mediumPower);
  thenHigh->addOutput(highPower);

  // Định nghĩa luật mờ cho từng trường hợp

  // Rule 1: If (Cold and Dry) then Low Power
  ifColdAndDry->joinWithAND(lowTemp, lowHumidity);
  fuzzy->addFuzzyRule(new FuzzyRule(1, ifColdAndDry, thenLow));

  // Rule 2: If (Cold and Normal) then Low Power
  ifColdAndNormal->joinWithAND(lowTemp, mediumHumidity);
  fuzzy->addFuzzyRule(new FuzzyRule(2, ifColdAndNormal, thenLow));

  // Rule 3: If (Cold and Humid) then Low Power
  ifColdAndHumid->joinWithAND(lowTemp, highHumidity);
  fuzzy->addFuzzyRule(new FuzzyRule(3, ifColdAndHumid, thenLow));

  // Rule 4: If (Warm and Dry) then Medium Power
  ifWarmAndDry->joinWithAND(mediumTemp, lowHumidity);
  fuzzy->addFuzzyRule(new FuzzyRule(4, ifWarmAndDry, thenMedium));

  // Rule 5: If (Warm and Normal) then High Power
  ifWarmAndNormal->joinWithAND(mediumTemp, mediumHumidity);
  fuzzy->addFuzzyRule(new FuzzyRule(5, ifWarmAndNormal, thenHigh));

  // Rule 6: If (Warm and Humid) then Low Power
  ifWarmAndHumid->joinWithAND(mediumTemp, highHumidity);
  fuzzy->addFuzzyRule(new FuzzyRule(6, ifWarmAndHumid, thenLow));

  // Rule 7: If (Hot and Dry) then Medium Power
  ifHotAndDry->joinWithAND(highTemp, lowHumidity);
  fuzzy->addFuzzyRule(new FuzzyRule(7, ifHotAndDry, thenMedium));

  // Rule 8: If (Hot and Normal) then High Power
  ifHotAndNormal->joinWithAND(highTemp, mediumHumidity);
  fuzzy->addFuzzyRule(new FuzzyRule(8, ifHotAndNormal, thenHigh));

  // Rule 9: If (Hot and Humid) then High Power
  ifHotAndHumid->joinWithAND(highTemp, highHumidity);
  fuzzy->addFuzzyRule(new FuzzyRule(9, ifHotAndHumid, thenHigh));
}

void loop() {
  // Đọc dữ liệu từ cảm biến DHT22
  float t = dht.readTemperature();
  float h = dht.readHumidity();

  // Kiểm tra nếu dữ liệu đọc hợp lệ
  if (isnan(t) || isnan(h)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  // Đặt giá trị đầu vào cho hệ thống mờ
  fuzzy->setInput(1, t);
  fuzzy->setInput(2, h);

  // Bắt đầu quá trình fuzzification
  fuzzy->fuzzify();

  // Lấy giá trị đầu ra
  float powerOutput = fuzzy->defuzzify(1);

  // Điều khiển đèn LED dựa vào giá trị powerOutput
  digitalWrite(LED_LOW, powerOutput > 0);
  digitalWrite(LED_MEDIUM, powerOutput >= 33);
  digitalWrite(LED_HIGH, powerOutput >= 66);

  // In ra giá trị để debug
  Serial.print("Temp: "); Serial.print(t);
  Serial.print("°C, Humidity: "); Serial.print(h);
  Serial.print("%, Power Output: "); Serial.println(powerOutput);

  delay(1000);
}
