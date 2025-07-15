#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <HX711_ADC.h>
#include <FastAccelStepper.h>

void Update();
void filters();
void Process();
void dataReadyISR();

FastAccelStepperEngine engine;
FastAccelStepper *stepperX;
long max_speed = 1500;
bool runallowed = false;

HX711_ADC LoadCell(11, 12);
LiquidCrystal_I2C lcd(0x27, 16, 2);  // Adresse I2C courante 0x27, ajustez si nécessaire

int step = 0;
float consigne = 0.0;
float i = 0.0; // Poids brut
unsigned long timer = 0;
float med_speed = max_speed / 3;
float low_speed = max_speed / 10;
float very_low_speed = max_speed / 20;
float ultra_low_speed = max_speed / 30;

const int buttonPin = 9;   // Départ cycle
const int buttonPin1 = 10; // TARE
const int modeautopin = 8; // AUTO

boolean auto_mode = false;
byte buttonState1 = 0;
byte lastReading = 0;
unsigned long t, z;
unsigned long lastProcessLog = 0;
unsigned long lastWeightLog = 0;
unsigned long lastLoopLog = 0;

volatile boolean newDataReady = false;

String current_status = "Init";

// Nouvelles variables pour calibration
float grams_per_step = 0.0001; // Valeur initiale par défaut (ajustez après tests)
const int calib_steps = 200; // Nombre de steps pour calibration
float initial_weight = 0.0;
bool calibration_done = false;

// Pour détection long press sur TARE
boolean buttonActive = false;
boolean longPressActive = false;
unsigned long buttonTimer = 0;
const unsigned long longPressTime = 1000; // 1 seconde pour long press

// Pour affichage optimisé
static String last_current_status = "";
static bool last_auto_mode = false;
static int last_speed_pc = -1;
static float last_consigne = -1.0;
static float last_i = -1.0;

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

  // Initialiser affichage fixe
  lcd.setCursor(0, 1);
  lcd.print("T:      W:      ");

  engine = FastAccelStepperEngine();
  engine.init();
  stepperX = engine.stepperConnectToPin(6);  // Pin STEP sur D6
  if (stepperX == NULL) {
    Serial.println("Error connecting stepper");
    lcd.setCursor(0, 1);
    lcd.print("Stepper Err");
    while (1);
  }
  stepperX->setDirectionPin(4);
  stepperX->setEnablePin(2, true); // low active
  stepperX->setAutoEnable(true);
  stepperX->setSpeedInHz(max_speed);
  stepperX->setAcceleration(150000);  // Réduite à 500 pour plus de précision et moins d'overshoot
  Serial.println("FastAccelStepper initialized on pin 6");

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
      if (millis() > lastWeightLog + 1000) {  // Log poids toutes les 1s
        Serial.print("Poids brut mesure: ");
        Serial.println(i, 3);
        lastWeightLog = millis();
      }
    }
  }

  float entierconsignebrut = map(a0, 4, 1020, 30, 0) / 10.0;
  float decimalesconsignebrut = map(a1, 4, 1020, 100, 0) / 1000.0;
  consigne = entierconsignebrut + decimalesconsignebrut;

  auto_mode = digitalRead(modeautopin);
  buttonState1 = digitalRead(buttonPin1);

  // Détection long press pour TARE
  if (buttonState1 == HIGH) {
    if (buttonActive == false) {
      buttonActive = true;
      buttonTimer = millis();
    }

    if ((millis() - buttonTimer > longPressTime) && (longPressActive == false)) {
      longPressActive = true;
      Serial.println("Long press TARE détecté: Lance calibration standalone");
      LoadCell.tare();  // Tare d'abord
      step = 100;  // Nouvel état pour calibration standalone
      current_status = "Calib";
    }
  } else {
    if (buttonActive == true) {
      if (longPressActive == true) {
        longPressActive = false;
      } else {
        // Short press: Tare normal
        Serial.println("Short press TARE: Tare, arrêt et reset");
        LoadCell.tare();
        runallowed = false;
        stepperX->stopMove();
        step = 0;
        current_status = "Tare/Reset";
        calibration_done = false;
      }
      buttonActive = false;
    }
  }

  lastReading = buttonState1;
}

void Process() {
  if (millis() > lastProcessLog + 500) {  // Log état toutes les 500ms
    Serial.print("Etat actuel: ");
    Serial.print(step);
    Serial.print(" - ");
    Serial.println(current_status);
    lastProcessLog = millis();
  }

  float fast_pct = min(0.95f, max(0.80f, 0.80f + (consigne / 3.0f) * 0.15f));
  float med_pct = 0.97f;
  float slow_pct = 0.995f;
  float tolerance = 0.005f;

  switch (step) {
    case 0: // Attente
      current_status = "Wait" + String(auto_mode ? "A" : "");
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
        step = 4;  // Calibration est maintenant séparée, passer directement au dosage
        Serial.println("Tarage termine, passage au dosage (calib séparée)");
      }
      break;

    case 4: { // Dosage rapide estimé jusqu'à fast_pct
      current_status = "Fast";
      float target_fast = consigne * fast_pct;
      float remaining_grams = target_fast - i;
      long estimated_steps = (long)(remaining_grams / grams_per_step);
      long fast_speed = (consigne < 0.5) ? max_speed / 2 : max_speed;
      if (i >= target_fast) {
        stepperX->stopMove();
        runallowed = false;
        timer = millis() + 500;
        step = 41;
      } else if (estimated_steps > 0) {
        stepperX->setSpeedInHz(fast_speed);
        stepperX->move(estimated_steps);
        runallowed = true;
        Serial.println("Estimated steps to fast_pct: " + String(estimated_steps));
      }
      if (!stepperX->isRunning() || i >= target_fast) {
        stepperX->stopMove();
        runallowed = false;
        timer = millis() + 500;  // Stabilisation plus longue
        step = 41;
      }
      break;
    }

    case 41: // Stabilisation après fast
      if (millis() > timer) {
        step = 50;
        Serial.println("Stabilisation après fast terminée, passage à med slow");
      }
      break;

    case 50: { // Med slow burst
      current_status = "MedSlw";
      if (i >= consigne * med_pct) {
        step = 60;
        break;
      }
      float remaining = consigne * med_pct - i;
      if (remaining <= 0) {
        step = 60;
        break;
      }
      long burst_steps = (long)(remaining / grams_per_step * 0.8);
      burst_steps = max(5L, min(20L, burst_steps));
      stepperX->setSpeedInHz(med_speed);
      stepperX->move(burst_steps);
      runallowed = true;
      step = 51;
      break;
    }

    case 51: // Wait after med burst
      current_status = "MedSlw";
      if (stepperX->isRunning()) {
        break;
      }
      runallowed = false;
      timer = millis() + 300;
      step = 52;
      break;

    case 52: // Stab after med
      current_status = "MedSlw";
      if (millis() > timer) {
        step = 50;
      }
      break;

    case 60: { // Slow burst
      current_status = "Slow";
      if (i >= consigne * slow_pct) {
        step = 70;
        break;
      }
      float remaining_slow = consigne * slow_pct - i;
      if (remaining_slow <= 0) {
        step = 70;
        break;
      }
      long burst_steps_slow = (long)(remaining_slow / grams_per_step * 0.8);
      burst_steps_slow = max(3L, min(10L, burst_steps_slow));
      stepperX->setSpeedInHz(low_speed);
      stepperX->move(burst_steps_slow);
      runallowed = true;
      step = 61;
      break;
    }

    case 61: // Wait after slow burst
      current_status = "Slow";
      if (stepperX->isRunning()) {
        break;
      }
      runallowed = false;
      timer = millis() + 400;
      step = 62;
      break;

    case 62: // Stab after slow
      current_status = "Slow";
      if (millis() > timer) {
        step = 60;
      }
      break;

    case 70: { // Very slow burst
      current_status = "VSlow";
      if (i >= consigne - tolerance) {
        step = 0;
        Serial.println("Poids atteint, retour a attente");
        break;
      }
      float remaining_vslow = consigne - tolerance - i;
      if (remaining_vslow <= 0) {
        step = 0;
        break;
      }
      long burst_steps_vslow = (long)(remaining_vslow / grams_per_step * 0.8);
      burst_steps_vslow = max(1L, min(5L, burst_steps_vslow));
      stepperX->setSpeedInHz(very_low_speed);
      stepperX->move(burst_steps_vslow);
      runallowed = true;
      step = 71;
      break;
    }

    case 71: // Wait after vslow burst
      current_status = "VSlow";
      if (stepperX->isRunning()) {
        break;
      }
      runallowed = false;
      timer = millis() + 500;
      step = 72;
      break;

    case 72: // Stab after vslow
      current_status = "VSlow";
      if (millis() > timer) {
        step = 70;
      }
      break;

    // Calibration standalone (séparée, via long press TARE)
    case 100: // Tarage pour calib standalone
      current_status = "Tare Calib";
      LoadCell.tare();
      timer = millis() + 2000;
      step = 101;
      Serial.println("Tarage pour calibration standalone");
      break;

    case 101: // Attente fin tarage calib standalone
      if (LoadCell.getTareStatus() || millis() > timer) {
        step = 102;
        Serial.println("Tarage calib termine");
      }
      break;

    case 102: // Exécution calibration standalone
      initial_weight = i;
      stepperX->setSpeedInHz(max_speed);
      stepperX->move(calib_steps);
      runallowed = true;
      timer = millis() + 5000;
      step = 103;
      Serial.println("Calibration standalone started: " + String(calib_steps) + " steps");
      break;

    case 103: // Attente fin move + stabilisation
      if (!stepperX->isRunning() || millis() > timer) {
        runallowed = false;
        timer = millis() + 500;  // Attente stabilisation
        step = 104;
      }
      break;

    case 104: // Mesure après stabilisation, update et reset
      if (millis() > timer) {
        float delta_weight = i - initial_weight;
        if (delta_weight > 0) {
          grams_per_step = delta_weight / calib_steps;
          Serial.print("Calibration standalone done: grams/step = ");
          Serial.println(grams_per_step, 6);
        } else {
          Serial.println("Calibration standalone failed: no weight change, using default");
        }
        calibration_done = true;
        step = 0;  // Reset à attente
        current_status = "Wait";
        Serial.println("Calibration terminée, retour à attente");
      }
      break;
  }
}

void loop() {
  filters();
  Process();
  Update();
  // Log pour confirmer que la loop tourne, toutes les 5s pour réduire
  if (millis() > lastLoopLog + 5000) {
    Serial.println("Loop is running");
    lastLoopLog = millis();
  }
}

void Update() {
  if (millis() > z + 200) {
    String stp_label = auto_mode ? "StpA:" : "Stp:";
    String status_full = stp_label + current_status;
    while (status_full.length() < 10) status_full += " ";
    if (status_full.length() > 10) status_full = status_full.substring(0, 10);

    int speed_pc = (int)(stepperX->getCurrentSpeedInMilliHz() / 1000.0 / max_speed * 100.0);

    // Mise à jour status si changé
    if (current_status != last_current_status || auto_mode != last_auto_mode) {
      lcd.setCursor(0, 0);
      lcd.print(status_full);
      last_current_status = current_status;
      last_auto_mode = auto_mode;
    }

    // Mise à jour speed %
    if (speed_pc != last_speed_pc) {
      lcd.setCursor(11, 0);
      lcd.print("Sp:");
      lcd.print(speed_pc);
      lcd.print("% ");
      last_speed_pc = speed_pc;
    }

    // Mise à jour consigne
    if (abs(consigne - last_consigne) > 0.001) {
      lcd.setCursor(2, 1);
      lcd.print(String(consigne, 3));
      lcd.print(" ");
      last_consigne = consigne;
    }

    // Mise à jour poids
    if (abs(i - last_i) > 0.001) {
      lcd.setCursor(10, 1);
      lcd.print(String(i, 3));
      lcd.print(" ");
      last_i = i;
    }

    // Log pour affichage, toutes les 1s au lieu de 200ms
    static unsigned long lastUpdateLog = 0;
    if (millis() > lastUpdateLog + 1000) {
      Serial.print("Affichage mis a jour - Consigne: ");
      Serial.print(consigne, 3);
      Serial.print(" Poids: ");
      Serial.println(i, 3);
      lastUpdateLog = millis();
    }

    z = millis();
  }
}