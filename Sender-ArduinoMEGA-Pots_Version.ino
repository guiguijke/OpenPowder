/*
  AnalogReadSerial

  Reads an analog input on pin 0, prints the result to the Serial Monitor.
  Graphical representation is available using Serial Plotter (Tools > Serial Plotter menu).
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/AnalogReadSerial
*/
#include <LiquidCrystal.h>
#include <Ewma.h>
Ewma adcFilter1(0.1);
Ewma adcFilterA2(0.1);
Ewma adcFilterA3(0.1);
LiquidCrystal lcd(46, 47, 48, 49, 50, 51);

#include <HX711_ADC.h>


HX711_ADC LoadCell(2, 3);


int valkey[3];
int j = 0;
int cup = 0;
int motor = 0;
int num = 0;
float entierconsignebrut, decimalesconsignebrut, GTVbrut, CTbrut;
float decimalesconsigne;
float GVtoPV = 0.80;
float AjustChute = 0.95;
int buttonstate, buttonmem, sensorA0, etape;
long t, y, z;
float i, i2;
float consigne ;
const int buttonPin = 31; //départ cycle
const int buttonPin1 = 30; //TARE
const int modeautopin = 4; //AUTO

boolean auto_mode = false;
boolean buttonActive = false;
boolean longPressActive = false;
int firsttime = 1;

byte buttonState1 = 0;
byte  lastReading = 0;
unsigned long startTime = 0;
unsigned long pressTime = 0;

long buttonTimer;
long longPressTime = 1000;
// the setup routine runs once when you press reset:
//SMOOTHING WEight
const int numReadingsWE = 32;
float readingsWE[numReadingsWE];      // the readings from the analog input
int readIndexWE = 0;              // the index of the current reading
float totalWE = 0;                  // the running total
float averageWE = 0;                // the average
//int inputPinWE = A0;

//SMOOTHING A0
const int numReadingsA0 = 32;
int readingsA0[numReadingsA0];      // the readings from the analog input
int readIndexA0 = 0;              // the index of the current reading
int totalA0 = 0;                  // the running total
int averageA0 = 0;                // the average
int inputPinA0 = A0;
//SMOOTHING A1
const int numReadingsA1 = 32;
int readingsA1[numReadingsA1];      // the readings from the analog input
int readIndexA1 = 0;              // the index of the current reading
int totalA1 = 0;                  // the running total
int averageA1 = 0;                // the average
int inputPinA1 = A1;



void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial1.begin(9600);
  Serial.begin(9600);

  pinMode(buttonPin, INPUT);
  pinMode(buttonPin1, INPUT);
  pinMode(modeautopin, INPUT);

  lcd.begin(20, 4);
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("Connex / Stab / Tare");

  float calValue; // calibration value
  calValue = 11493.0; // uncomment this if you want to set this value in the sketch
  //Serial.println("Starting...");
  LoadCell.begin();
  long stabilisingtime = 2000; // tare preciscion can be improved by adding a few seconds of stabilising time
  LoadCell.start(stabilisingtime);
  if (LoadCell.getTareTimeoutFlag()) {
    //Serial.println("Tare timeout, check MCU>HX711 wiring and pin designations");
  }
  else {
    LoadCell.setCalFactor(calValue); // set calibration factor (float)
    //Serial.println("Startup + tare is complete");
  }
  while (!LoadCell.update());
  //Serial.print("Calibration factor: ");
  //Serial.println(LoadCell.getCalFactor());
  //Serial.print("HX711 measured conversion time ms: ");
  //Serial.println(LoadCell.getConversionTime());
  //Serial.print("HX711 measured sampling rate HZ: ");
  // Serial.println(LoadCell.getSPS());
  // Serial.print("HX711 measured settlingtime ms: ");
  //Serial.println(LoadCell.getSettlingTime());
  //Serial.println("Note that the settling time may increase significantly if you use delay() in your sketch!");
  if (LoadCell.getSPS() < 7) {
    //Serial.println("!!Sampling rate is lower than specification, check MCU>HX711 wiring and pin designations");
  }
  else if (LoadCell.getSPS() > 100) {
    //Serial.println("!!Sampling rate is higher than specification, check MCU>HX711 wiring and pin designations");
  }

  etape = 0;

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Step = ");
  lcd.setCursor(0, 1);
  lcd.print("Target = ");
  lcd.setCursor(0, 2);
  lcd.print("Weight g = ");
  lcd.setCursor(0, 3);
  lcd.print("GtV = ");
  lcd.setCursor(11, 3);
  lcd.print("CT = ");

  buttonmem = LOW;
  Serial1.write(0);

  //SMOOTHING A0
  for (int thisReadingA0 = 0; thisReadingA0 < numReadingsA0; thisReadingA0++) {
    readingsA0[thisReadingA0] = 0;
  }
  //SMOOTHING A1
  for (int thisReadingA1 = 0; thisReadingA1 < numReadingsA1; thisReadingA1++) {
    readingsA1[thisReadingA1] = 0;
  }
  //SMOOTHING WEight
  for (int thisReadingWE = 0; thisReadingWE < numReadingsWE; thisReadingWE++) {
    readingsWE[thisReadingWE] = 0;
  }
}

// the loop routine runs over and over again forever:
void loop() {

  auto_mode = digitalRead(modeautopin);
  buttonstate = digitalRead(buttonPin);
  buttonState1 = digitalRead(buttonPin1);

  //ANALOG A0 INPUT SMOOTHED
  totalA0 = totalA0 - readingsA0[readIndexA0];
  readingsA0[readIndexA0] = analogRead(inputPinA0);
  totalA0 = totalA0 + readingsA0[readIndexA0];
  readIndexA0 = readIndexA0 + 1;
  if (readIndexA0 >= numReadingsA0) {
    readIndexA0 = 0;
  }
  averageA0 = totalA0 / numReadingsA0;
  entierconsignebrut = map(averageA0, 4, 1020, 30, -5);

  //ANALOG A1 INPUT SMOOTHED
  totalA1 = totalA1 - readingsA1[readIndexA1];
  readingsA1[readIndexA1] = analogRead(inputPinA1);
  totalA1 = totalA1 + readingsA1[readIndexA1];
  readIndexA1 = readIndexA1 + 1;
  if (readIndexA1 >= numReadingsA1) {
    readIndexA1 = 0;
  }
  averageA1 = totalA1 / numReadingsA1;
  decimalesconsignebrut = map(averageA1, 4, 1020, 50, -5);


  float GTVf = adcFilterA2.filter(analogRead(A2));
  float CTf = adcFilterA3.filter(analogRead(A3));

  GTVbrut = map(GTVf, 4, 1020, 50, 99);
  CTbrut = map(CTf, 4, 1020, 80, 100);

  consigne = ( entierconsignebrut / 10 ) + (decimalesconsignebrut / 100) ;
  GVtoPV = GTVbrut / 100;
  AjustChute = CTbrut / 100;



  LoadCell.update();




  //  i = LoadCell.getData();





  if ((buttonState1 != buttonmem) && (buttonState1 = HIGH))
  {
    LoadCell.tare();
  }



  switch (etape) {

    case 0:
      if ( (buttonstate != buttonmem) && (buttonstate = HIGH) ) {
        etape = 1;
      }
      if (auto_mode == true ) {
        etape = 4;
      }

      if (millis() > y + 300) {
        lcd.setCursor(8, 0);
        lcd.print("            ");
        lcd.setCursor(8, 0);
        lcd.print("Waiting");
        y = millis();
      }
      Update();
      break;

    case 4: //Attente Auto

      if (auto_mode == false ) {
        etape = 0;
      }

      if ( i < (-5) ) {
        if (firsttime == 1) {
          startTime = millis();
          firsttime = 0;
        }
        pressTime = millis() - startTime;

      }
      else if (firsttime == 0) {
        firsttime = 1;
      }

      if ( pressTime>3500 && i > (-0.5) ) {
        etape = 5;
        pressTime=0;
      }
      if ( (buttonstate != buttonmem) && (buttonstate = HIGH) ) {
        etape = 1;
      }

      if (millis() > y + 300) {
        lcd.setCursor(8, 0);
        lcd.print("            ");
        lcd.setCursor(8, 0);
        lcd.print("Waiting A");
        y = millis();
      }
      Update();
      break;

    case 5:
      if (millis() > y + 300) {
        lcd.setCursor(8, 0);
        lcd.print("            ");
        lcd.setCursor(8, 0);
        lcd.print("Tare Auto");
        y = millis();
      }
      LoadCell.tare();
      t = millis();
      while ( millis() < t + 2000) {
        Update();
      }
      etape = 1;
      Update();
      break;

    case 1:
      lcd.setCursor(8, 0);
      lcd.print("          ");
      lcd.setCursor(8, 0);
      lcd.print("High Speed");
      if (consigne < 0.5) {
        motor = 30;
      }
      else {
        motor = 20;
      }
      Serial1.write (motor);
      while (i < (consigne * GVtoPV)) {
        Update();
      }
      etape = 2;
      break;

    case 2:
      lcd.setCursor(8, 0);
      lcd.print("          ");
      lcd.setCursor(8, 0);
      lcd.print("Small Speed");
      Serial1.write(200);
      while (i + 0.0025 < (consigne * AjustChute)) {
        Update();
      }
      etape = 3;
      break;

    case 3:

      Serial1.write(0);

      if (AjustChute <= 0.99) {
        //Serial.println("AJUSTEMENT ON");
        lcd.setCursor(8, 0);
        lcd.print("          ");
        lcd.setCursor(8, 0);
        lcd.print("ADJUSTING...");

        Update();
        while ( i + 0.0025 < consigne ) {

          //          Serial.print("VALEUR PESEE = ");
          //          Serial.println(i);
          //          Serial.print("VALEUR CONSIGNE = " );
          //          Serial.println(consigne);
          t = millis();
          while ( millis() < t + 400) {
            Update();
            if (i + 0.0025 >= consigne) {
              etape = 0;
              Serial1.write(0);
              break;
            }
          }
          Serial1.write(100);
          while ( millis() < t + 400 + 150) {
            Update();
            if (i + 0.0025 >= consigne) {
              etape = 0;
              Serial1.write(0);
              break;
            }
          }

          Serial1.write(0);
          Update();

        }
      }
      etape = 0;

      lcd.setCursor(8, 0);
      lcd.print("          ");
      lcd.setCursor(8, 0);
      lcd.print("END");
      Update();
      break;
  }
  Update();



}

void Update() {
  //ANALOG WEight INPUT SMOOTHED
  LoadCell.update();
  totalWE = totalWE - readingsWE[readIndexWE];
  readingsWE[readIndexWE] = LoadCell.getData();
  totalWE = totalWE + readingsWE[readIndexWE];
  readIndexWE = readIndexWE + 1;
  if (readIndexWE >= numReadingsWE) {
    readIndexWE = 0;
  }
  float i3 = totalWE / numReadingsWE;

  float i1 = adcFilter1.filter(LoadCell.getData());
  i = (i1 + i3) / 2;
  //  LoadCell.update();
  i2 = LoadCell.getData();
  //Serial.print("VALEUR UPDATE = ");
  Serial.print(i, 3);
  Serial.print(", ");
  Serial.print(i2, 3);
  Serial.print(", ");
  Serial.print(i3, 3);
  Serial.println();


  if (millis() > z + 200) {
    lcd.setCursor(11, 2);
    lcd.print("       ");
    lcd.setCursor(11, 2);
    lcd.print(i, 3);


    if (auto_mode == HIGH)
    {
      lcd.setCursor(0, 0);
      lcd.print("       ");
      lcd.setCursor(0, 0);
      lcd.print("StepA= ");
    }
    if (auto_mode == LOW) {
      lcd.setCursor(0, 0);
      lcd.print("       ");
      lcd.setCursor(0, 0);
      lcd.print("Step = ");
    }

    //    Serial.println(analogRead(A1));
    //    Serial.println(decimalesconsignebrut);

    lcd.setCursor(11, 1);
    lcd.print("    ");
    lcd.setCursor(11, 1);
    lcd.print(consigne, 3);

    lcd.setCursor(6, 3);
    lcd.print("    ");
    lcd.setCursor(6, 3);
    lcd.print(GVtoPV);

    lcd.setCursor(16, 3);
    lcd.print("    ");
    lcd.setCursor(16, 3);
    lcd.print(AjustChute);


    z = millis();
  }

}
