#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <HX711_ADC.h>
#include <AccelStepper.h>

void Update();
void filters();
void Process();
void dataReadyISR();

AccelStepper stepperX(1, 3, 4); // Retour à l'original : Driver mode, STEP pin 3, DIR pin 4
long max_speed = 1500;
bool runallowed = false;

HX711_ADC LoadCell(11, 12);
LiquidCrystal_I2C lcd(0x27, 16, 2);  // Adresse I2C courante 0x27, ajustez si nécessaire

int step = 0;
float consigne = 0.0;
float i = 0.0; // Poids brut
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

String current_status = "Init";

void setup() {
  Serial.begin(115200);
  Serial.println("Setup started");

  pinMode(buttonPin, INPUT);
  pinMode(buttonPin1, INPUT);
  pinMode(modeautopin, INPUT);

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Init / Tare");
  Serial.println("LCD: Init / Tare displayed");

  float calValue = 36160;
  LoadCell.begin();
  Serial.println("LoadCell.begin() done");

  long stabilisingtime = 2000;
  LoadCell.start(stabilisingtime);
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("Tare timeout");
    lcd.setCursor(0, 1);
    lcd.print("Tare Timeout");
  } else {
    LoadCell.setCalFactor(calValue);
    Serial.println("Calibration set");
  }

  unsigned long startTime = millis();
  while (!LoadCell.update() && millis() - startTime < 5000);  // Timeout après 5 secondes
  if (millis() - startTime >= 5000) {
    Serial.println("Timeout waiting for LoadCell update");
    lcd.setCursor(0, 1);
    lcd.print("LC Update TO");
  } else {
    Serial.println("LoadCell.update() successful");
  }

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Setup Done");
  Serial.println("LCD cleared and Setup Done displayed");

  stepperX.setMaxSpeed(max_speed);
  stepperX.setSpeed(max_speed);
  stepperX.setAcceleration(100.0);
  stepperX.setEnablePin(2);
  stepperX.setPinsInverted(false, false, true);
  stepperX.enableOutputs();
  Serial.println("AccelStepper initialized");

  attachInterrupt(digitalPinToInterrupt(11), dataReadyISR, FALLING);
  Serial.println("Interrupt attached");

  Serial.println("Setup completed");
  z = millis();  // Initialiser z pour que Update commence immédiatement
}

void dataReadyISR() {
  if (LoadCell.update()) {
    newDataReady = 1;
  }
}

void filters() {
  // Pas de filtrage Ewma pour la consigne, on utilise brut
  float a0 = analogRead(A0);
  float a1 = analogRead(A1);

  if (newDataReady) {
    if (millis() > t) {
      float loadcelldata = LoadCell.getData();
      i = loadcelldata;  // Valeur brute sans filtre
      newDataReady = 0;
      t = millis();
      Serial.print("Poids brut mesure: ");
      Serial.println(i, 3);
    }
  }

  float entierconsignebrut = map(a0, 4, 1020, 30, 0) / 10.0;
  float decimalesconsignebrut = map(a1, 4, 1020, 100, 0) / 1000.0;
  consigne = entierconsignebrut + decimalesconsignebrut;

  auto_mode = digitalRead(modeautopin);
  int buttonstate = digitalRead(buttonPin);
  buttonState1 = digitalRead(buttonPin1);

  if ((buttonState1 != lastReading) && (buttonState1 == HIGH)) {
    Serial.println("Bouton TARE presse: Effectue tare, arret et reset");
    LoadCell.tare();
    runallowed = false;
    step = 0;
    current_status = "Tare/Reset";
  }
  lastReading = buttonState1;
}

void Process() {
  Serial.print("Etat actuel: ");
  Serial.print(step);
  Serial.print(" - ");
  Serial.println(current_status);

  switch (step) {
    case 0: // Attente
      current_status = auto_mode ? "WaitA" : "Wait";
      if ((digitalRead(buttonPin) == HIGH) || (auto_mode && i > -0.5)) {
        step = 1;
        Serial.println("Demarrage du processus");
      }
      break;

    case 1: // Tarage
      current_status = "Tare";
      LoadCell.tare();
      timer = millis() + 2000;
      step = 2;
      Serial.println("Tarage en cours");
      break;

    case 2: // Attente fin tarage
      if (LoadCell.getTareStatus() || millis() > timer) {
        step = 3;
        Serial.println("Tarage termine");
      }
      break;

    case 3: // Dosage rapide
      current_status = "Fast";
      stepperX.setSpeed(max_speed);
      runallowed = true;
      if (i >= consigne * 0.9) {
        step = 4;
      }
      break;

    case 4: // Dosage lent
      current_status = "Slow";
      stepperX.setSpeed(low_speed);
      if (i >= consigne - 0.01) {
        step = 5;
      }
      break;

    case 5: // Début stabilisation
      current_status = "Stab";
      runallowed = false;
      timer = millis() + 500;
      step = 6;
      Serial.println("Stabilisation commencee");
      break;

    case 6: // Attente stabilisation
      if (millis() > timer) {
        step = 7;
        Serial.println("Stabilisation terminee, verification poids");
      }
      break;

    case 7: // Vérification poids
      if (i >= consigne - 0.005) {
        step = 0;
        Serial.println("Poids atteint, retour a attente");
      } else {
        step = 8;
        Serial.println("Ajout final necessaire");
      }
      break;

    case 8: // Début dernier ajout
      current_status = "Drop";
      stepperX.setSpeed(very_low_speed);
      runallowed = true;
      timer = millis() + 20;
      step = 9;
      break;

    case 9: // Fin dernier ajout
      if (millis() > timer) {
        runallowed = false;
        step = 5;
        Serial.println("Dernier ajout termine, retour a stabilisation");
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
  // Log pour confirmer que la loop tourne
  static unsigned long lastLoopLog = 0;
  if (millis() - lastLoopLog > 1000) {
    Serial.println("Loop is running");
    lastLoopLog = millis();
  }
}

void Update() {
  if (millis() > z + 200) {
    lcd.setCursor(0, 0);
    lcd.print("                ");  // Efface la ligne 0
    lcd.setCursor(0, 0);
    String stp_label = auto_mode ? "StpA:" : "Stp:";
    lcd.print(stp_label + current_status + " Sp:" + String((int)(stepperX.speed() / max_speed * 100)) + "%");

    lcd.setCursor(0, 1);
    lcd.print("                ");  // Efface la ligne 1
    lcd.setCursor(0, 1);
    lcd.print("T:" + String(consigne, 3) + " W:" + String(i, 3));

    // Log pour affichage
    Serial.print("Affichage mis a jour - Consigne: ");
    Serial.print(consigne, 3);
    Serial.print(" Poids: ");
    Serial.println(i, 3);

    z = millis();
  }
}