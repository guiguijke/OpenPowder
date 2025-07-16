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
LiquidCrystal_I2C lcd(0x27, 16, 2);  // Common I2C address 0x27, adjust if necessary

int state = 0;
float setpoint = 0.0;
float current_weight = 0.0; // Gross weight
unsigned long timer = 0;
float med_speed = max_speed / 3;
float low_speed = max_speed / 10;
float very_low_speed = max_speed / 20;
float ultra_low_speed = max_speed / 30;

const int buttonPin = 9;   // Start cycle
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

// New variables for calibration
float grams_per_step = 0.0001; // Default initial value (adjust after tests)
const int calib_steps = 400; // Number of steps for calibration
float initial_weight = 0.0;
bool calibration_done = false;
int calib_count = 0;
float calib_sums = 0.0;

// For long press detection on TARE
boolean buttonActive = false;
boolean longPressActive = false;
unsigned long buttonTimer = 0;
const unsigned long longPressTime = 1000; // 1 second for long press

// For optimized display
static String last_current_status = "";
static bool last_auto_mode = false;
static int last_speed_pc = -1;
static float last_setpoint = -1.0;
static float last_current_weight = -1.0;

// Rotary encoder
const int encoderClk = 3;
const int encoderDt = 5;
const int encoderSw = 7;
int last_clk;
unsigned long last_sw_press = 0;
unsigned long last_encoder_time = 0;
const unsigned long button_debounce = 50; // ms for button
const unsigned long encoder_debounce = 20; // Increased for better debouncing
int edit_digit = -1;
int digits[4] = {0, 0, 0, 0}; // units, tenths, hundredths, thousandths
unsigned long validation_time = 0; // For displaying "Valid"

void setup() {
  Serial.begin(115200);
  Serial.println("Setup started");

  pinMode(buttonPin, INPUT);
  pinMode(buttonPin1, INPUT);
  pinMode(modeautopin, INPUT);

  pinMode(encoderClk, INPUT);
  pinMode(encoderDt, INPUT);
  pinMode(encoderSw, INPUT_PULLUP); // Active low

  last_clk = digitalRead(encoderClk);

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
  while (!LoadCell.update() && millis() - startTime < 5000);  // Timeout after 5 seconds
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

  // Initialize fixed display
  lcd.setCursor(0, 1);
  lcd.print("T:      W:      ");

  engine = FastAccelStepperEngine();
  engine.init();
  stepperX = engine.stepperConnectToPin(6);  // STEP pin on D6
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
  stepperX->setAcceleration(150000);  // Reduced to 150000 for more precision and less overshoot
  Serial.println("FastAccelStepper initialized on pin 6");

  attachInterrupt(digitalPinToInterrupt(11), dataReadyISR, FALLING);
  Serial.println("Interrupt attached");

  Serial.println("Setup completed");
  z = millis();  // Initialize z so that Update starts immediately
}

void dataReadyISR() {
  if (LoadCell.update()) {
    newDataReady = 1;
  }
}

void filters() {
  if (newDataReady) {
    if (millis() > t) {
      float loadcelldata = LoadCell.getData();
      current_weight = loadcelldata;  // Raw value without filter
      newDataReady = 0;
      t = millis();
      if (millis() > lastWeightLog + 1000) {  // Log weight every 1s
        Serial.print("Measured gross weight: ");
        Serial.println(current_weight, 3);
        lastWeightLog = millis();
      }
    }
  }

  auto_mode = digitalRead(modeautopin);
  buttonState1 = digitalRead(buttonPin1);

  // Long press detection for TARE
  if (buttonState1 == HIGH) {
    if (buttonActive == false) {
      buttonActive = true;
      buttonTimer = millis();
    }

    if ((millis() - buttonTimer > longPressTime) && (longPressActive == false)) {
      longPressActive = true;
      Serial.println("Long press TARE detected: Launch standalone calibration");
      LoadCell.tare();  // Tare first
      state = 100;  // New state for standalone calibration
      current_status = "Calib";
    }
  } else {
    if (buttonActive == true) {
      if (longPressActive == true) {
        longPressActive = false;
      } else {
        // Short press: Normal tare
        Serial.println("Short press TARE: Tare, stop and reset");
        LoadCell.tare();
        runallowed = false;
        stepperX->stopMove();
        state = 0;
        current_status = "Tare/Reset";
        calibration_done = false;
      }
      buttonActive = false;
    }
  }

  lastReading = buttonState1;

  // Encoder button (SW)
  bool sw_state = digitalRead(encoderSw);
  if (sw_state == LOW && millis() - last_sw_press > button_debounce) {
    last_sw_press = millis();
    if (edit_digit == -1) {
      // Enter edit mode, initialize digits from current setpoint
      int sp_int = (int)(setpoint * 1000 + 0.5);
      digits[3] = sp_int % 10; // thousandths
      sp_int /= 10;
      digits[2] = sp_int % 10; // hundredths
      sp_int /= 10;
      digits[1] = sp_int % 10; // tenths
      sp_int /= 10;
      digits[0] = sp_int % 10; // units
      edit_digit = 0; // Start with units
    } else {
      edit_digit++;
      if (edit_digit > 3) {
        edit_digit = -1;
        validation_time = millis() + 1000; // Show "Valid" for 1s
        // Setpoint already updated
      }
    }
  }

  // Encoder rotation - improved to trigger only on falling edge of CLK
  int current_clk = digitalRead(encoderClk);
  if (last_clk == HIGH && current_clk == LOW && millis() - last_encoder_time > encoder_debounce) { // Falling edge
    if (edit_digit >= 0) {
      int dt_state = digitalRead(encoderDt);
      if (dt_state == LOW) {
        // CW: increase
        if (digits[edit_digit] < 9) digits[edit_digit]++;
      } else {
        // CCW: decrease
        if (digits[edit_digit] > 0) digits[edit_digit]--;
      }
      // Update setpoint immediately for display
      setpoint = digits[0] + digits[1] / 10.0 + digits[2] / 100.0 + digits[3] / 1000.0;
      last_encoder_time = millis();
    }
  }
  last_clk = current_clk;
}

void Process() {
  if (millis() > lastProcessLog + 500) {  // Log state every 500ms
    Serial.print("Current state: ");
    Serial.print(state);
    Serial.print(" - ");
    Serial.println(current_status);
    lastProcessLog = millis();
  }

  float fast_pct = min(0.95f, max(0.80f, 0.80f + (setpoint / 3.0f) * 0.15f));
  float med_pct = 0.97f;
  float slow_pct = 0.995f;
  float tolerance = 0.005f;

  switch (state) {
    case 0: // Waiting
      current_status = "Wait" + String(auto_mode ? "A" : "");
      if (((digitalRead(buttonPin) == HIGH) || (auto_mode && current_weight > -0.5)) && edit_digit == -1) {
        state = 1;
        Serial.println("Process starting");
      }
      break;

    case 1: // Taring
      current_status = "Tare";
      LoadCell.tare();
      timer = millis() + 2000;
      state = 2;
      Serial.println("Taring in progress");
      break;

    case 2: // Waiting for taring to finish
      if (LoadCell.getTareStatus() || millis() > timer) {
        state = 4;  // Calibration is now separate, go directly to dosing
        Serial.println("Taring finished, proceeding to dosing (calib separate)");
      }
      break;

    case 4: { // Fast dosing estimated up to fast_pct
      current_status = "Fast";
      float target_fast = setpoint * fast_pct;
      float remaining_grams = target_fast - current_weight;
      long estimated_steps = (long)(remaining_grams / grams_per_step);
      long fast_speed = (setpoint < 0.5) ? max_speed / 2 : max_speed;
      if (current_weight >= target_fast) {
        stepperX->stopMove();
        runallowed = false;
        timer = millis() + 500;
        state = 41;
      } else if (estimated_steps > 0) {
        stepperX->setSpeedInHz(fast_speed);
        stepperX->move(estimated_steps);
        runallowed = true;
        Serial.println("Estimated steps to fast_pct: " + String(estimated_steps));
      }
      if (!stepperX->isRunning() || current_weight >= target_fast) {
        stepperX->stopMove();
        runallowed = false;
        timer = millis() + 500;  // Longer stabilization
        state = 41;
      }
      break;
    }

    case 41: // Stabilization after fast
      if (millis() > timer) {
        state = 50;
        Serial.println("Stabilization after fast finished, proceeding to med slow");
      }
      break;

    case 50: { // Med slow burst
      current_status = "MedSlw";
      if (current_weight >= setpoint * med_pct) {
        state = 60;
        break;
      }
      float remaining = setpoint * med_pct - current_weight;
      if (remaining <= 0) {
        state = 60;
        break;
      }
      long burst_steps = (long)(remaining / grams_per_step * 0.8);
      burst_steps = max(5L, min(20L, burst_steps));
      stepperX->setSpeedInHz(med_speed);
      stepperX->move(burst_steps);
      runallowed = true;
      state = 51;
      break;
    }

    case 51: // Wait after med burst
      current_status = "MedSlw";
      if (stepperX->isRunning()) {
        break;
      }
      runallowed = false;
      timer = millis() + 300;
      state = 52;
      break;

    case 52: // Stab after med
      current_status = "MedSlw";
      if (millis() > timer) {
        state = 50;
      }
      break;

    case 60: { // Slow burst
      current_status = "Slow";
      if (current_weight >= setpoint * slow_pct) {
        state = 70;
        break;
      }
      float remaining_slow = setpoint * slow_pct - current_weight;
      if (remaining_slow <= 0) {
        state = 70;
        break;
      }
      long burst_steps_slow = (long)(remaining_slow / grams_per_step * 0.8);
      burst_steps_slow = max(3L, min(10L, burst_steps_slow));
      stepperX->setSpeedInHz(low_speed);
      stepperX->move(burst_steps_slow);
      runallowed = true;
      state = 61;
      break;
    }

    case 61: // Wait after slow burst
      current_status = "Slow";
      if (stepperX->isRunning()) {
        break;
      }
      runallowed = false;
      timer = millis() + 400;
      state = 62;
      break;

    case 62: // Stab after slow
      current_status = "Slow";
      if (millis() > timer) {
        state = 60;
      }
      break;

    case 70: { // Very slow burst
      current_status = "VSlow";
      if (current_weight >= setpoint - tolerance) {
        state = 0;
        Serial.println("Weight reached, back to waiting");
        break;
      }
      float remaining_vslow = setpoint - tolerance - current_weight;
      if (remaining_vslow <= 0) {
        state = 0;
        break;
      }
      long burst_steps_vslow = (long)(remaining_vslow / grams_per_step * 0.8);
      burst_steps_vslow = max(1L, min(5L, burst_steps_vslow));
      stepperX->setSpeedInHz(very_low_speed);
      stepperX->move(burst_steps_vslow);
      runallowed = true;
      state = 71;
      break;
    }

    case 71: // Wait after vslow burst
      current_status = "VSlow";
      if (stepperX->isRunning()) {
        break;
      }
      runallowed = false;
      timer = millis() + 500;
      state = 72;
      break;

    case 72: // Stab after vslow
      current_status = "VSlow";
      if (millis() > timer) {
        state = 70;
      }
      break;

    // Standalone calibration (separate, via long press TARE)
    case 100: // Taring for standalone calib
      current_status = "Tare Calib";
      LoadCell.tare();
      timer = millis() + 2000;
      calib_count = 0;
      calib_sums = 0.0;
      state = 101;
      Serial.println("Taring for standalone calibration");
      break;

    case 101: // Waiting for calib taring to finish
      if (LoadCell.getTareStatus() || millis() > timer) {
        state = 102;
        Serial.println("Calib taring finished");
      }
      break;

    case 102: // Executing standalone calibration burst
      initial_weight = current_weight;
      stepperX->setSpeedInHz(max_speed);
      stepperX->move(calib_steps);
      runallowed = true;
      timer = millis() + 5000;
      state = 103;
      Serial.println("Calibration burst started: " + String(calib_steps) + " steps, count: " + String(calib_count + 1));
      break;

    case 103: // Waiting for move end + stabilization
      if (!stepperX->isRunning() || millis() > timer) {
        runallowed = false;
        timer = millis() + 500;  // Waiting for stabilization
        state = 104;
      }
      break;

    case 104: // Measurement after stabilization
      if (millis() > timer) {
        float delta_weight = current_weight - initial_weight;
        if (delta_weight > 0) {
          calib_sums += delta_weight / calib_steps;
          Serial.print("Burst done, grams/step = ");
          Serial.println(delta_weight / calib_steps, 6);
        } else {
          Serial.println("Burst failed: no weight change");
        }
        calib_count++;
        if (calib_count < 3) {
          timer = millis() + 500;  // Extra stabilization before next burst
          state = 106;
        } else {
          grams_per_step = calib_sums / 3.0;
          Serial.print("Calibration average grams/step = ");
          Serial.println(grams_per_step, 6);
          calibration_done = true;
          state = 0;  // Reset to waiting
          current_status = "Wait";
          Serial.println("Calibration finished, back to waiting");
        }
      }
      break;

    case 106: // Extra stabilization before next burst
      if (millis() > timer) {
        state = 102;
      }
      break;
  }
}

void loop() {
  filters();
  Process();
  Update();
  // Log to confirm the loop is running, every 5s to reduce
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
    if (status_full.length() > 10) status_full = status_full.substring(0,10);

    static bool flash_on = false;
    flash_on = !flash_on;  // Toggle every 200ms

    bool show_calib_message = !calibration_done && current_status.startsWith("Wait");
    if (show_calib_message) {
      if (flash_on) {
        status_full = "Calib Need    ";  // Adjust to fit 10 chars + Sp:XX
      }
    }

    // Override with "Valid" if in validation period
    if (millis() < validation_time) {
      status_full = "Valid      ";
    }

    int speed_pc = (int)(stepperX->getCurrentSpeedInMilliHz() / 1000.0 / max_speed * 100.0);
    speed_pc = min(99, max(0, speed_pc));

    // Update status if changed or flashing
    if (current_status != last_current_status || auto_mode != last_auto_mode || show_calib_message || millis() < validation_time) {
      lcd.setCursor(0, 0);
      lcd.print(status_full);
      last_current_status = current_status;
      last_auto_mode = auto_mode;
    }

    // Update speed %
    if (speed_pc != last_speed_pc) {
      lcd.setCursor(11, 0);
      lcd.print("Sp:");
      String speed_str = speed_pc < 10 ? "0" + String(speed_pc) : String(speed_pc);
      lcd.print(speed_str);
      last_speed_pc = speed_pc;
    }

    // Update setpoint (force if editing for blink)
    if (abs(setpoint - last_setpoint) > 0.001 || edit_digit >= 0) {
      lcd.setCursor(2, 1);
      if (edit_digit >= 0) {
        // Build the string
        String sp_str = String(digits[0]) + "." + String(digits[1]) + String(digits[2]) + String(digits[3]);
        if (!flash_on) {
          int char_pos;
          if (edit_digit == 0) char_pos = 0; // units
          else if (edit_digit == 1) char_pos = 2; // tenths
          else if (edit_digit == 2) char_pos = 3; // hundredths
          else char_pos = 4; // thousandths
          sp_str.setCharAt(char_pos, ' ');
        }
        lcd.print(sp_str);
        lcd.print(" ");
      } else {
        lcd.print(String(setpoint, 3));
        lcd.print(" ");
      }
      last_setpoint = setpoint;
    }

    // Update weight
    if (abs(current_weight - last_current_weight) > 0.001) {
      lcd.setCursor(10, 1);
      lcd.print(String(current_weight, 3));
      lcd.print(" ");
      last_current_weight = current_weight;
    }

    // Log for display, every 1s instead of 200ms
    static unsigned long lastUpdateLog = 0;
    if (millis() > lastUpdateLog + 1000) {
      Serial.print("Display updated - Setpoint: ");
      Serial.print(setpoint, 3);
      Serial.print(" Weight: ");
      Serial.println(current_weight, 3);
      lastUpdateLog = millis();
    }

    z = millis();
  }
}