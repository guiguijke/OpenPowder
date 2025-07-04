#include <Arduino.h>
#include <LiquidCrystal.h>
#include <HX711_ADC.h>
#include <AccelStepper.h>
#include <Ewma.h>
#include <PID_v1.h>

void Update();
void UpdateFast();
void continuousRun();
void filters();
void Process();
void dataReadyISR();

AccelStepper stepperX(1, 3, 4);
long max_speed = 1500;
bool runallowed;

HX711_ADC LoadCell(11, 12);
LiquidCrystal lcd(46, 47, 48, 49, 50, 51);

Ewma adcFilterA0(0.1);
Ewma adcFilterA1(0.1);
Ewma adcFilterA2(0.1);
Ewma adcFilterA3(0.1);
Ewma loadcellfilter(0.01);

//PID
double Setpoint, Input, Output;
double Kp=2.5*1500, Ki=10, Kd=1;
PID myPID(&Input, &Output, &Setpoint,Kp,Ki,Kd,P_ON_E, DIRECT); //P_ON_M specifies that Proportional on Measurement be used
                                                            //P_ON_E (Proportional on Error) is the default behavior


int valkey[3];
int j = 0;
int cup = 0;
int motor = 0;
int num = 0;
float entierconsignebrut, decimalesconsignebrut, GTVbrut, CTbrut;
float decimalesconsigne;
float GVtoPV = 0.80;
float AjustChute = 0.95;
int buttonstate, buttonmem, sensorA0, step;

float a0, a1, a2, a3, i,i2, i3, p, pa, pb ,loadcelldata;
float consigne;
const int buttonPin = 9;   //départ cycle
const int buttonPin1 = 10; //TARE
const int modeautopin = 8; //AUTO

boolean auto_mode = false;
boolean buttonActive = false;
boolean longPressActive = false;

byte buttonState1 = 0;
byte lastReading = 0;
unsigned long s2 = 2000;
unsigned long t, z, y,r;

long buttonTimer;
long longPressTime = 1000;

volatile boolean newDataReady;

void setup()
{
  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);
  p = max_speed;
  pinMode(buttonPin, INPUT);
  pinMode(buttonPin1, INPUT);
  pinMode(modeautopin, INPUT);

  lcd.begin(20, 4);
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("Connex / Stab / Tare");

  float calValue;     // calibration value
  calValue = 36160; //11486.0; // uncomment this if you want to set this value in the sketch
  //Serial.println("Starting...");
  LoadCell.begin();
  long stabilisingtime = 2000; // tare preciscion can be improved by adding a few seconds of stabilising time
  LoadCell.start(stabilisingtime);
  if (LoadCell.getTareTimeoutFlag())
  {
    //Serial.println("Tare timeout, check MCU>HX711 wiring and pin designations");
  }
  else
  {
    LoadCell.setCalFactor(calValue); // set calibration factor (float)
    //Serial.println("Startup + tare is complete");
  }
  while (!LoadCell.update())
    ;
  /*Serial.print("Calibration factor: ");
  Serial.println(LoadCell.getCalFactor());
  Serial.print("HX711 measured conversion time ms: ");
  Serial.println(LoadCell.getConversionTime());
  Serial.print("HX711 measured sampling rate HZ: ");
  Serial.println(LoadCell.getSPS());
  Serial.print("HX711 measured settlingtime ms: ");
  Serial.println(LoadCell.getSettlingTime());
  Serial.println("Note that the settling time may increase significantly if you use delay() in your sketch!");
  */
  if (LoadCell.getSPS() < 7)
  {
    //Serial.println("!!Sampling rate is lower than specification, check MCU>HX711 wiring and pin designations");
  }
  else if (LoadCell.getSPS() > 100)
  {
    //Serial.println("!!Sampling rate is higher than specification, check MCU>HX711 wiring and pin designations");
  }

  step = 0;

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Step = ");
  lcd.setCursor(0, 1);
  lcd.print("Target g = ");
  lcd.setCursor(0, 2);
  lcd.print("Weight g = ");
  lcd.setCursor(0, 3);
  lcd.print("Spd = ");
  //lcd.setCursor(11, 3);
  //lcd.print("CT = ");
  lcd.setCursor(10, 3);
  lcd.print("I=");
  lcd.setCursor(15, 3);
  lcd.print("D=");

  buttonmem = LOW;

  // Configure stepper
  stepperX.setMaxSpeed(max_speed);
  stepperX.setSpeed(max_speed);
  stepperX.setAcceleration(100.0);
  stepperX.setEnablePin(2); //58
  stepperX.setPinsInverted(false, false, true);
  stepperX.enableOutputs(); //enable pins
  runallowed = false;

  attachInterrupt(digitalPinToInterrupt(11), dataReadyISR, FALLING);

  //PID
  //Input = loadcellfilter.filter(LoadCell.getData());
  //Setpoint = 1.00;

  //turn the PID on
  
  myPID.SetOutputLimits(0,2500);
  myPID.SetControllerDirection(DIRECT);
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(100);

}

void dataReadyISR()
{
  if (LoadCell.update())
  {
    newDataReady = 1;
  }
}

void filters()
{

  /*Ewma adcFilterA0(0.1);
  Ewma adcFilterA1(0.1);
  Ewma adcFilterA2(0.1);
  Ewma adcFilterA3(0.1);*/

  a0 = adcFilterA0.filter(analogRead(A0));
  a1 = adcFilterA1.filter(analogRead(A1));
  a2 = adcFilterA2.filter(analogRead(A2));
  a3 = adcFilterA3.filter(analogRead(A3));

  if (newDataReady)
  {
    if (millis() > t)
    {
      loadcelldata=LoadCell.getData();
      i = (loadcellfilter.filter(loadcelldata))/1;
      i2= (loadcellfilter.filter(loadcelldata)+loadcelldata)/2;
      i3=loadcelldata;
      newDataReady = 0;

      t = millis();
    }
  }

  entierconsignebrut = map(a0, 4, 1020, 30, 0) / 10.0;
  decimalesconsignebrut = map(a1, 4, 1020, 100, 0) / 1000.0;
  //GTVbrut = map(a2, 4, 1020, 99, 50);
  //CTbrut = map(a3, 4, 1020, 99, 0);
  Ki=map(a2, 4, 1020, 100, 0);
  Kd=map(a3, 4, 1020, 100, 0);


  consigne = entierconsignebrut + decimalesconsignebrut;
  
  //double Kp=5*1500, Ki=10, Kd=1;

  //GVtoPV = GTVbrut / 100;
  //AjustChute = CTbrut / 100;

  auto_mode = digitalRead(modeautopin);
  buttonstate = digitalRead(buttonPin);
  buttonState1 = digitalRead(buttonPin1);

  if ((buttonState1 != buttonmem) && (buttonState1 = HIGH))
  {
    LoadCell.tare();
  }
}

void Process()
{
  switch (step)
  {

  case 0:

    lcd.setCursor(8, 0);
    lcd.print("            ");
    lcd.setCursor(8, 0);
    lcd.print("Waiting");
    runallowed=false;
    step = 1;
    break;

  case 1:

    if ((buttonstate != buttonmem) && (buttonstate == HIGH))
    {
      step = 5;
      myPID.SetTunings(Kp, Ki, Kd);
    }
    if (auto_mode == true)
    {
      step = 2;
    }

    break;

  case 2:

    lcd.setCursor(8, 0);
    lcd.print("            ");
    lcd.setCursor(8, 0);
    lcd.print("Waiting A");
    step = 21;

    if (auto_mode == false)
    {
      step = 0;
    }

    break;

  case 21:

    if (i > -0.5)
    {
      step = 3;
    }

    break;

  case 3:
    lcd.setCursor(8, 0);
    lcd.print("            ");
    lcd.setCursor(8, 0);
    lcd.print("Tare Auto");
    LoadCell.tare();
    y = millis();
    step = 4;
    break;

  case 4:

    if (LoadCell.getTareStatus() == true || millis() >= y + 2000)
    {
      step = 5;
    }

    break;

  case 5: //START DOSING
    lcd.setCursor(8, 0);
    lcd.print("          ");
    lcd.setCursor(8, 0);
    lcd.print("High Speed");
    step = 6;
    break;

  case 6:
    /*if (consigne < 0.5)
    {
      max_speed = max_speed / 2;
    }
    */

    stepperX.setSpeed(max_speed);
    runallowed = true;
    step = 71;

    if (step == 71){
      Setpoint = consigne; //SETPOINT PID
    }

    break;

case 71: //RUN PID

    Input = i;
    myPID.Compute();
    max_speed = Output;
    stepperX.setSpeed(max_speed);
    
    if (i >= consigne)
    {
      step = 0;
    }

    break;
  
  
  
  
  
  case 7:

    if (i >= consigne * GVtoPV)
    {
      step = 8;
    }

    break;

  case 8:
    lcd.setCursor(8, 0);
    lcd.print("          ");
    lcd.setCursor(8, 0);
    lcd.print("Finalizing");
    step = 9;
    //stepperX.setSpeed(max_speed/2);
    pb = max_speed / log((consigne - i) * 100 + 1);
    break;

  case 9: // boucle P

    pa = log((consigne - i) * 100 + 1);
    p = pa * pb - (50 / consigne - i);
    stepperX.setSpeed(p);

    if (i >= consigne - AjustChute || p < 0)
    {
      runallowed = false;
      step = 10;
    }

    break;

  case 10:

    lcd.setCursor(8, 0);
    lcd.print("           ");
    lcd.setCursor(8, 0);
    lcd.print("Stabilizing");
    y = millis();
    step = 11;
    break;

  case 11:

    if (millis() > y + 2000)
    {

      Serial.print("consigne - i = ");
      Serial.println(consigne - i);

      if (consigne - i >= 0.005)
      {

        step = 12;
      }
      if (consigne - i < 0.005)
      {
        step = 0;
      }
    }
    break;

  case 12:

    lcd.setCursor(8, 0);
    lcd.print("           ");
    lcd.setCursor(8, 0);
    lcd.print("Last Drop");
    step = 13;
    stepperX.setSpeed(100);
    break;

    case 13:
    if (consigne - i < 0.005)
      {
        step = 0;
        runallowed = false;
      }
    if (millis() > y + 50)
    {
      
      runallowed = !runallowed;
      if( runallowed==false){
      y = millis()+200;
      break;  
      }


      y = millis();
    }
    
    break;







  }
}

// the loop routine runs over and over again forever:
void loop()
{

  continuousRun();

  filters();

  Process();

  Update();
}

void Update()
{
  //  Serial.print("VALEUR UPDATE = ");
  //  Serial.println(i);
if (millis() > r + 50)
  {
    //Serial.print("VALEUR UPDATE = ");
    Serial.print(i*2 ,3);
    Serial.print(",");
    Serial.print(i2*2, 3);
    Serial.print(",");
    Serial.print(i3*2, 3);
    Serial.print(",");
    //Serial.print("VALEUR CONSIGNE = ");
    /*Serial.print(consigne*2000, 3);
    Serial.print(",");
    //Serial.print("Speed = ");
    Serial.print(max_speed);*/
    Serial.println();
    r = millis();

  }
  if (millis() > z + 200)
  {
    


    /*Serial.print("INPUT= ");
    Serial.println(Input);
    Serial.print("OUTPUT= ");
    Serial.println(Output);
    Serial.print("SETPOINT= ");
    Serial.println(Setpoint/100);*/

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
    else if (auto_mode == LOW)
    {
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
    lcd.print((float(max_speed)/20),0);
    lcd.print("%");

    lcd.setCursor(12, 3);
    lcd.print("   ");
    lcd.setCursor(12, 3);
    lcd.print(Ki,0);

    lcd.setCursor(17, 3);
    lcd.print("   ");
    lcd.setCursor(17, 3);
    lcd.print(Kd,0);

    z = millis();
  }
}

void continuousRun() //method for the motor
{
  if (runallowed == true)
  {

    stepperX.run();

    /*if (stepperX.run() == false)
    {
      stepperX.setCurrentPosition(0);
      runallowed = false;
      Serial.println("FINISHED continuousRun");
    }*/
  }
  else //program enters this part if the runallowed is FALSE, we do not do anything
  {
    return;
  }
}