#include <Servo.h>

Servo servo;

int t1, t2, t3, vout;
uint8_t dummy_load_current;
int dummy_load_value;
int servo_value;


int servo_dir = 1;

void read_analog_in()
{
  vout = analogRead(A0);
  t1 = analogRead(A1);
  t2 = analogRead(A2);
  t3 = analogRead(A3);
}

void dummy_load_write()
{
  uint8_t temp_value = dummy_load_value / 100;
  digitalWrite(9, dummy_load_current > temp_value/16 ? LOW : HIGH);
  ++dummy_load_current;
  if(dummy_load_current > 15) dummy_load_current = 0;
}

void print_all()
{
  Serial.print(vout);
  Serial.print(" ");
  Serial.print(dummy_load_current*16);
  Serial.print(" ");
  Serial.print(digitalRead(9)*255);
  Serial.println(" ");
}

void setup() {
  Serial.begin(9600);
  //while (!Serial);
  pinMode(9, OUTPUT);
  servo.attach(6);
}

void loop() {
  read_analog_in();

  ++dummy_load_value;
  if(dummy_load_value > 25500) dummy_load_value = 0;
  
  servo_value += servo_dir;
  if(servo_value > 160) servo_dir = -1;
  if(servo_value < 30) servo_dir = 1;
  
  delay(10);

  servo.write(servo_value % 180);
  dummy_load_write();

  print_all();
}
