int Sensor1 = 14;
int Sensor2 = 15;

int val1;
int val2;


void setup ()
{
  pinMode (Sensor1, INPUT);
  pinMode (Sensor2, INPUT);
  Serial.begin(115200);
}


void loop ()
{
  if(digitalRead(Sensor1) == HIGH) {val1 = 1;}
  else {val1 = 0;}

  if (digitalRead(Sensor2) == HIGH) {val2 = 1;}
  else {val2 = 0;}

  Serial.print(val1);
  Serial.print(' ');
  Serial.println(val2);
  
  delay(100);
}
