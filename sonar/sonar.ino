#include <NewPing.h>

#define TRIGGER_PIN 12
#define ECHO_PIN 11
#define MAX_DISTANCE 44

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

int anterior=0;

void setup()
{
  Serial.begin(9600);
  
}

void loop()
{
anterior = readPosition(anterior);
}
int readPosition(int previo)
{
  delay(40);
  long cm;
  cm = sonar.convert_cm(sonar.ping_median(5));
  if(cm>=41||cm==0){
    cm=previo;
  }
  Serial.println(cm);
  return cm;
}
