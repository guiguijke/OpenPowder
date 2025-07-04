#include <Arduino.h>
#include <LiquidCrystal.h>
#include <HX711_ADC.h>
#include <AccelStepper.h>
#include <Ewma.h>

void Update();
void filters();
void Process();
void dataReadyISR();

AccelStepper stepperX(1, 3, 4);
long max_speed = 1500;
bool runallowed = false;

HX711_ADC LoadCell(11, 12);
LiquidCrystal lcd(46, 47, 48, 49, 50, 51);

Ewma adcFilterA0(0.1);
Ewma adcFilterA1(0.1);
Ewma loadcellfilter(0.01);

int step = 0;
float consigne = 0.0;
float i = 0.0; // Poids filtré
unsigned long timer = 0;
float low_speed = max_speed / 10;
float very_low_speed = max_speed / 20;

const int buttonPin = 9;   // Départ cycle
const int buttonPin1 = 10; // TARE
const int modeautopin = 8; // AUTO

boolean auto_mode = false;
byte buttonState1 = 0;
byte lastReading = 0;
unsigned long t, z;

volatile boolean newDataReady = false;

void setup() {
  Serial.begin(115200);
  pinMode(buttonPin, INPUT);
  pinMode(buttonPin1, INPUT);
  pinMode(modeautopin, INPUT);

  lcd.begin(20, 4);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Connex / Stab / Tare");

  float calValue = 36160;
  LoadCell.begin();
  long stabilisingtime = 2000;
  LoadCell.start(stabilisingtime);
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("Tare timeout");
  } else {
    LoadCell.setCalFactor(calValue);
  }
  while (!LoadCell.update());

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Step = ");
  lcd.setCursor(0, 1);
  lcd.print("Target g = ");
  lcd.setCursor(0, 2);
  lcd.print("Weight g = ");
  lcd.setCursor(0, 3);
  lcd.print("Spd = ");

  stepperX.setMaxSpeed(max_speed);
  stepperX.setSpeed(max_speed);
  stepperX.setAcceleration(100.0);
  stepperX.setEnablePin(2);
  stepperX.setPinsInverted(false, false, true);
  stepperX.enableOutputs();

  attachInterrupt(digitalPinToInterrupt(11), dataReadyISR, FALLING);
}

void dataReadyISR() {
  if (LoadCell.update()) {
    newDataReady = 1;
  }
}

void filters() {
  float a0 = adcFilterA0.filter(analogRead(A0));
  float a1 = adcFilterA1.filter(analogRead(A1));

  if (newDataReady) {
    if (millis() > t) {
      float loadcelldata = LoadCell.getData();
      i = loadcellfilter.filter(loadcelldata);
      newDataReady = 0;
      t = millis();
    }
  }

  float entierconsignebrut = map(a0, 4, 1020, 30, 0) / 10.0;
  float decimalesconsignebrut = map(a1, 4, 1020, 100, 0) / 1000.0;
  consigne = entierconsignebrut + decimalesconsignebrut;

  auto_mode = digitalRead(modeautopin);
  
  buttonState1 = digitalRead(buttonPin1);

  if ((buttonState1 != lastReading) && (buttonState1 == HIGH)) {
    LoadCell.tare();
  }
  lastReading = buttonState1;
}

void Process() {
  switch (step) {
    case 0: // Attente
      lcd.setCursor(8, 0);
      lcd.print("            ");
      lcd.setCursor(8, 0);
      lcd.print(auto_mode ? "Waiting A" : "Waiting");
      if ((digitalRead(buttonPin) == HIGH) || (auto_mode && i > -0.5)) {
        step = 1;
      }
      break;

    case 1: // Tarage
      lcd.setCursor(8, 0);
      lcd.print("            ");
      lcd.setCursor(8, 0);
      lcd.print("Taring");
      LoadCell.tare();
      timer = millis() + 2000;
      step = 2;
      break;

    case 2: // Attente fin tarage
      if (LoadCell.getTareStatus() || millis() > timer) {
        step = 3;
      }
      break;

    case 3: // Dosage rapide
      lcd.setCursor(8, 0);
      lcd.print("            ");
      lcd.setCursor(8, 0);
      lcd.print("Fast Dispense");
      stepperX.setSpeed(max_speed);
      runallowed = true;
      if (i >= consigne * 0.9) {
        step = 4;
      }
      break;

    case 4: // Dosage lent
      lcd.setCursor(8, 0);
      lcd.print("            ");
      lcd.setCursor(8, 0);
      lcd.print("Slow Dispense");
      stepperX.setSpeed(low_speed);
      if (i >= consigne - 0.01) {
        step = 5;
      }
      break;

    case 5: // Début stabilisation
      runallowed = false;
      timer = millis() + 500;
      step = 6;
      break;

    case 6: // Attente stabilisation
      if (millis() > timer) {
        step = 7;
      }
      break;

    case 7: // Vérification poids
      if (i >= consigne - 0.005) {
        step = 0;
      } else {
        step = 8;
      }
      break;

    case 8: // Début dernier ajout
      stepperX.setSpeed(very_low_speed);
      runallowed = true;
      timer = millis() + 20;
      step = 9;
      break;

    case 9: // Fin dernier ajout
      if (millis() > timer) {
        runallowed = false;
        step = 5;
      }
      break;
  }
}

void loop() {
  filters();
  Process();
  if (runallowed) {
    stepperX.runSpeed();
  }
  Update();
}

void Update() {
  if (millis() > z + 200) {
    lcd.setCursor(11, 2);
    lcd.print("       ");
    lcd.setCursor(11, 2);
    lcd.print(i, 3);

    lcd.setCursor(11, 1);
    lcd.print("    ");
    lcd.setCursor(11, 1);
    lcd.print(consigne, 3);

    lcd.setCursor(6, 3);
    lcd.print("    ");
    lcd.setCursor(6, 3);
    lcd.print((float(stepperX.speed()) / max_speed * 100), 0);
    lcd.print("%");

    z = millis();
  }
}
