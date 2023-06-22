#include <Servo.h>
#include <PID_v1.h>
#include <NewPing.h>

#define SERVO_PIN 9
#define TRIGGER_PIN 12
#define ECHO_PIN 11
#define MAX_DISTANCE 44

float modificar(float valor);
float readPosition(float previo);
void menu();
int tendencia(float valor1, float valor2);

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

float kp = 1.0;
float kd = 0.0;
float ki = 0.0;
double Setpoint = 23.0, Input, Output, ServoOutput;

String variables[4] = {"Kp", "Kd", "Ki", "Setpoint"};

PID myPID(&Input, &Output, &Setpoint, kp, ki, kd, DIRECT);

Servo myServo;

void setup()
{
  Serial.begin(9600);
  myServo.attach(SERVO_PIN);

  Input = readPosition(Input);
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-90, 90);

  pinMode(13, OUTPUT);
  digitalWrite(13, 1);
  while (!Serial.available())
  {
  }
  Serial.readString();
  digitalWrite(13, 0);
}

void loop()
{
  Input = readPosition(Input);

  myPID.Compute();

  ServoOutput = 80 + Output;
  myServo.write(ServoOutput);

  menu();
}

float readPosition(float previo)
{
  static float historico[2] = {0.0, 0.0};
  int tend = tendencia(historico[1], historico[0]);
  int promedio = (historico[1] - historico[0]) / 2;
  delay(20);
  double cm;
  cm = sonar.convert_cm(sonar.ping_median(5));

  if (cm >= 41 || cm == 0)
  {
    cm = previo + promedio * tend;
  }
  historico[0] = historico[1];
  historico[1] = previo;
  return cm;
}

float modificar(float valor)
{
  float nuevo = valor;
  if (Serial.available())
  {
    nuevo = Serial.parseFloat();
    Serial.print("Nuevo valor: ");
    Serial.println(nuevo);
  }
  return nuevo;
}

void menu()
{
  static bool menuVisualizado = false;
  static bool opcionElegida = false;
  static int opcion = 0;
  float valores[4] = {kp, kd, ki, Setpoint};
  if (!menuVisualizado)
  {
    String visualizar = "Ingrese la opcion:";
    for (int i = 0; i < 4; i++)
    {
      visualizar += "\nOpcion ";
      visualizar += i + 1;
      visualizar += ": ";
      visualizar += variables[i];
      visualizar += " - ";
      visualizar += valores[i];
    }

    Serial.println(visualizar);
    menuVisualizado = true;
  }

  if (!opcionElegida)
  {
    if (Serial.available())
    {
      opcion = Serial.parseInt();
      Serial.readString();
      if (opcion >= 1 && opcion <= 4)
      {
        Serial.print("Eligio la opcion ");
        Serial.println(opcion);
        opcionElegida = true;
      }
    }
  }

  if (opcionElegida)
  {
    switch (opcion)
    {
    case 1:
      kp = modificar(kp);
      break;
    case 2:
      kd = modificar(kd);
      break;
    case 3:
      ki = modificar(ki);
      break;
    case 4:
      Setpoint = modificar(Setpoint);
      break;

    default:
      break;
    }

    if (Serial.available())
    {
      String resto = Serial.readString();
      Serial.println(resto);
      if (resto.startsWith("fin"))
      {
        opcionElegida = false;
        menuVisualizado = false;
      }
    }
  }
}

int tendencia(float valor1, float valor2)
{
  float creciente = valor1 - valor2;
  if (creciente <= 0)
  {
    return -1;
  }
  return 1;
}
