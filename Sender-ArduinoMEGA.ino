/*
  AnalogReadSerial

  Reads an analog input on pin 0, prints the result to the Serial Monitor.
  Graphical representation is available using Serial Plotter (Tools > Serial Plotter menu).
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/AnalogReadSerial
*/
#include <LiquidCrystal.h>
LiquidCrystal lcd(46, 47, 48, 49, 50, 51);

#include <HX711_ADC.h>
HX711_ADC LoadCell(2, 3);

#include <Keypad.h>
const byte ROWS = 4; //four rows
const byte COLS = 4; //four columns
//define the cymbols on the buttons of the keypads
char hexaKeys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};
byte rowPins[ROWS] = {36, 37, 38, 39}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {40, 41, 42, 43}; //connect to the column pinouts of the keypad

//initialize an instance of class NewKeypad
Keypad keypad = Keypad( makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);



int valkey[3];
int j = 0;
int motor = 0;
int num = 0;
float GVtoPV = 0.80;
float AjustChute = 0.95;
int buttonstate, buttonstate1, buttonmem, sensorA0, etape;
long t, y, z;
float i;
float consigne = 1.0;
const int buttonPin = 31; //départ cycle
const int buttonPin1 = 30; //TARE
// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial1.begin(9600);
  Serial.begin(9600);

  pinMode(buttonPin, INPUT);
  pinMode(buttonPin1, INPUT);

  lcd.begin(20, 4);
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("Connex / Stab / Tare");

  float calValue; // calibration value
  calValue = 11493.0; // uncomment this if you want to set this value in the sketch
  Serial.println("Starting...");
  LoadCell.begin();
  long stabilisingtime = 2000; // tare preciscion can be improved by adding a few seconds of stabilising time
  LoadCell.start(stabilisingtime);
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("Tare timeout, check MCU>HX711 wiring and pin designations");
  }
  else {
    LoadCell.setCalFactor(calValue); // set calibration factor (float)
    Serial.println("Startup + tare is complete");
  }
  while (!LoadCell.update());
  Serial.print("Calibration factor: ");
  Serial.println(LoadCell.getCalFactor());
  Serial.print("HX711 measured conversion time ms: ");
  Serial.println(LoadCell.getConversionTime());
  Serial.print("HX711 measured sampling rate HZ: ");
  Serial.println(LoadCell.getSPS());
  Serial.print("HX711 measured settlingtime ms: ");
  Serial.println(LoadCell.getSettlingTime());
  Serial.println("Note that the settling time may increase significantly if you use delay() in your sketch!");
  if (LoadCell.getSPS() < 7) {
    Serial.println("!!Sampling rate is lower than specification, check MCU>HX711 wiring and pin designations");
  }
  else if (LoadCell.getSPS() > 100) {
    Serial.println("!!Sampling rate is higher than specification, check MCU>HX711 wiring and pin designations");
  }

  etape = 0;

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Etape = ");
  lcd.setCursor(0, 1);
  lcd.print("Consigne = ");
  lcd.setCursor(0, 2);
  lcd.print("Mesure g = ");
  lcd.setCursor(0, 3);
  lcd.print("GtV = ");
  lcd.setCursor(11, 3);
  lcd.print("CT = ");

  buttonmem = LOW;
  Serial1.write(0);
}

// the loop routine runs over and over again forever:
void loop() {

  buttonstate = digitalRead(buttonPin);
  buttonstate1 = digitalRead(buttonPin1);
  LoadCell.update();
  i = LoadCell.getData();

  if ( (buttonstate1 != buttonmem) && (buttonstate1 = HIGH) ) {
    LoadCell.tareNoDelay();
  }

  char key = keypad.getKey();
  if (key == 'A') {
    startTypeConsigne();
  }

  if (key == 'B') {
    startTypeGVtoPV();
  }

  if (key == 'C') {
    startTypeAjustChute();
  }

  if (key == 'D' ) {
    Serial1.write(25);
  }

  if (key == '#'  ) {
    Serial1.write(0);
  }




  switch (etape) {

    case 0:
      if ( (buttonstate != buttonmem) && (buttonstate = HIGH) ) {
        etape = 1;
      }
      if (millis() > y + 300) {
        lcd.setCursor(8, 0);
        lcd.print("            ");
        lcd.setCursor(8, 0);
        lcd.print("En Attente");
        y = millis();
      }
      Update();
      break;

    case 1:
      lcd.setCursor(8, 0);
      lcd.print("          ");
      lcd.setCursor(8, 0);
      lcd.print("Grande V");
      if (consigne < 0.5) {
        motor = 50;
      }
      else {
        motor = 25;
      }
      Serial1.write(motor);
      while (i < (consigne * GVtoPV)) {
        Update();
      }
      etape = 2;
      break;

    case 2:
      lcd.setCursor(8, 0);
      lcd.print("          ");
      lcd.setCursor(8, 0);
      lcd.print("Petite V");
      Serial1.write(200);
      while (i < (consigne * AjustChute)) {
        Update();
      }
      etape = 3;
      break;

    case 3:

      Serial1.write(0);

      if (AjustChute <= 0, 97) {
        Serial.println("AJUSTEMENT ON");
        lcd.setCursor(8, 0);
        lcd.print("          ");
        lcd.setCursor(8, 0);
        lcd.print("AJUST");

        Update();
        while ( i + 0.01 <= consigne ) {

          Serial.print("VALEUR PESEE = ");
          Serial.println(i);
          Serial.print("VALEUR CONSIGNE = " );
          Serial.println(consigne);
          t = millis();
          while ( millis() < t + 600) {
            Update();
            if (i - 0.01 >= consigne) {
              break;
            }
          }
          Serial1.write(100);
          while ( millis() < t + 600 + 100) {
            Update();
            if (i + 0.01 >= consigne) {
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
      lcd.print("FIN");
      Update();
      break;
  }
  Update();

  /*

  */


}

void  startTypeConsigne() {
  lcd.setCursor(8, 0);
  lcd.print("            ");
  lcd.setCursor(8, 0);
  lcd.print("Saisie Cons.");
  j = 0;
  while ( j < 3) {
    int key = keypad.getKey();
    switch (key)
    {
      case NO_KEY: break;
      case '0': case '1': case '2': case '3': case '4':
      case '5': case '6': case '7': case '8': case '9':
        {
          num = 0 + (key - '0');
          valkey[j] = num;
          break;
        }
      case '*':
        {
          j++;
          break;
        }
    }
  }
  consigne = valkey[0] + valkey[1] * 0.1 + valkey[2] * 0.01;
}

void  startTypeGVtoPV() {
  lcd.setCursor(8, 0);
  lcd.print("            ");
  lcd.setCursor(8, 0);
  lcd.print("Saisie GtV");
  j = 0;
  while ( j < 3) {
    int key = keypad.getKey();
    switch (key)
    {
      case NO_KEY: break;
      case '0': case '1': case '2': case '3': case '4':
      case '5': case '6': case '7': case '8': case '9':
        {
          num = 0 + (key - '0');
          valkey[j] = num;
          break;
        }
      case '*':
        {
          j++;
          break;
        }
    }
  }
  GVtoPV = valkey[0] + valkey[1] * 0.1 + valkey[2] * 0.01;
}

void  startTypeAjustChute() {
  lcd.setCursor(8, 0);
  lcd.print("            ");
  lcd.setCursor(8, 0);
  lcd.print("Saisie CT");
  j = 0;
  while ( j < 3) {
    int key = keypad.getKey();
    switch (key)
    {
      case NO_KEY: break;
      case '0': case '1': case '2': case '3': case '4':
      case '5': case '6': case '7': case '8': case '9':
        {
          num = 0 + (key - '0');
          valkey[j] = num;
          break;
        }
      case '*':
        {
          j++;
          break;
        }
    }
  }
  AjustChute = valkey[0] + valkey[1] * 0.1 + valkey[2] * 0.01;
}

void Update() {
  LoadCell.update();
  i = LoadCell.getData();
  Serial.print("VALEUR UPDATE = ");
  Serial.println(i);
  //  if (millis() > t + 100) {
  //
  //    t = millis();
  //  }

  if (millis() > z + 200) {
    lcd.setCursor(11, 2);
    lcd.print("      ");
    lcd.setCursor(11, 2);
    lcd.print(i);

    lcd.setCursor(11, 1);
    lcd.print("    ");
    lcd.setCursor(11, 1);
    lcd.print(consigne);

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

void UpdateFast() {
  LoadCell.update();
  i = LoadCell.getData();
}
