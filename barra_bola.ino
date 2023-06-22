/* PID balance code with ping pong ball and distance sensor sharp 2y0a21
 *  by ELECTRONOOBS: https://www.youtube.com/channel/UCjiVhIvGmRZixSzupD0sS9Q
 *  Tutorial: http://electronoobs.com/eng_arduino_tut100.php
 *  Code: http://electronoobs.com/eng_arduino_tut100_code1.php
 *  Scheamtic: http://electronoobs.com/eng_arduino_tut100_sch1.php
 *  3D parts: http://electronoobs.com/eng_arduino_tut100_stl1.php
 */
#include <Wire.h>
#include <Servo.h>
#include <NewPing.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>

#define TRIGGER_PIN 12
#define ECHO_PIN 11
#define MAX_DISTANCE 43
#define PIN_SERVO 9

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
LiquidCrystal_I2C lcd(0x27, 16, 2);

int readPosition(int previo);
float leerPuerto(float dato);
float get_dist(int n);
void display();

///////////////////////Inputs/outputs///////////////////////
int Analog_in = A0;
Servo myservo; // create servo object to control a servo, later attatched to D9
///////////////////////////////////////////////////////

////////////////////////Variables///////////////////////
int Read = 0;
int distance = 0;
unsigned long elapsedTime, time, timePrev,timeI2C; // Variables for time control
float distance_previous_error, distance_error;
int period = 50; // Refresh rate period of the loop is 50ms
///////////////////////////////////////////////////////

///////////////////PID constants///////////////////////
float kp = 2.0;                 // Mine was 8
float ki = 0.0;                 // Mine was 0.2
float kd = 0.0;                 // Mine was 3100
float distance_setpoint = 21.0; // Should be the distance from sensor to the middle of the bar in mm
float PID_p, PID_i, PID_d, PID_total;
///////////////////////////////////////////////////////

void setup()
{
  // analogReference(EXTERNAL);
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  digitalWrite(13, 1);
  myservo.attach(PIN_SERVO); // attaches the servo on pin 9 to the servo object
  myservo.write(90);        // Put the servco at angle 125, so the balance is in the middle
  pinMode(Analog_in, INPUT);

  time = millis();
  timeI2C = millis();

  lcd.begin(16,2);
  lcd.backlight();
  
  delay(5000);
  digitalWrite(13, 0);
}

void loop()
{
  if (millis() - time >= period)
  {
    time = millis();
    // distance = get_dist(5);
    distance = readPosition(distance);
    distance_error = distance_setpoint - float(distance);
    PID_p = kp * distance_error;
    float dist_diference = distance_error - distance_previous_error;
    PID_d = kd * ((distance_error - distance_previous_error) / period);

    if (-3 < distance_error && distance_error < 3)
    {
      PID_i = PID_i + (ki * distance_error);
    }
    else
    {
      PID_i = 0;
    }

    PID_total = PID_p + PID_i + PID_d;
    PID_total = map(PID_total, -150, 150, 0, 150);
    PID_total = max(20, min(PID_total, 150));

    myservo.write(PID_total + 30);
    distance_previous_error = distance_error;
  }
  if (millis()-timeI2C>=2000)
  {
    display();
    timeI2C=millis();
  }
  
  // distance_setpoint = leerPuerto(distance_setpoint);
  // kp = leerPuerto(kp);
  // kd = leerPuerto(kd);
  // ki = leerPuerto(ki);
}

/*float get_dist(int n)
{
  long sum = 0;
  for (int i = 0; i < n; i++)
  {
    // sum=sum+analogRead(Analog_in);
    sum += readPosition();
  }
  float adc = sum / n;
  // float volts = analogRead(adc)*0.0048828125;  // value from sensor * (5/1024)
  // float volts = sum*0.003222656;  // value from sensor * (3.3/1024) EXTERNAL analog refference

  float distance_cm = 17569.7 * pow(adc, -1.2062);
  // float distance_cm = 13*pow(volts, -1);
  return (distance_cm);
}*/

int readPosition(int previo)
{
  delay(40);
  long cm;
  cm = sonar.convert_cm(sonar.ping_median(5));

  if (cm > 41||cm==0)
  {
    cm=previo;
  }
  Serial.println(cm);
  return cm;
}

float leerPuerto(float dato)
{
  if (Serial.available())
  {
    dato = Serial.parseInt();
    String descarte = Serial.readString();
    Serial.print("Nuevo valor: ");
    Serial.println(dato);
  }
  return dato;
}
/*
#include <Servo.h>
#include <PID_v1.h>

const int servoPin = 9;

float kp = 1.0;
float ki = 0.7;
float kd = 0.7;
double SetPoint, Input, Output, ServoOutput;

PID myPID(&Input, &Output, &Setpoint, kp, ai, kd, DIRECT);

Servo myServo;

void setup()
{
  Serial.begin(9600);
  myServo.attach(servoPin);

  Input = readPosition();
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-90, 90);
}

void loop()
{
  Setpoint = 19;

  myPID.Compute();

  ServoOuput = 80 + Ouput;
  myServo.write(ServoOutput);
}


*/

void display(){
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("KP:");
  lcd.setCursor(3, 0);
  lcd.print(kp);
  lcd.setCursor(8, 0);
  lcd.print("KD:");
  lcd.setCursor(11, 0);
  lcd.print(kd);
  lcd.setCursor(0, 1);
  lcd.print("KI:");
  lcd.setCursor(3, 0);
  lcd.print(ki);
  lcd.setCursor(8, 1);
  lcd.print("Set:");
  lcd.setCursor(13, 0);
  lcd.print(distance_setpoint);
}
