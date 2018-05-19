#include <Servo.h>

Servo servo;

int t1, t2, t3, vout;
uint8_t dummy_load_current;
int dummy_load_value;
uint32_t debug_servo_value;

uint32_t vout_iir_coeff = 210;
uint32_t vout_prev = 0, vout_current = 0;

int servo_dir = 1;

void read_analog_in()
{
  vout = analogRead(A0);
  t1 = analogRead(A1);
  t2 = analogRead(A2);
  t3 = analogRead(A3);
}

void apply_vout_filter()
{
  vout_current = (vout_prev * vout_iir_coeff) + (vout * (256 - vout_iir_coeff));
  vout_current /= 256;
  vout_prev = vout_current;
}

uint32_t vout_get()
{
  return vout_current;
}

void output_set(uint32_t value)
{
  debug_servo_value = (value * 180) / 1024;
  servo.write(debug_servo_value);
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
  Serial.print(vout_get());
  Serial.print(" ");
  Serial.print(debug_servo_value);
  Serial.print(" ");
  Serial.println(" ");
}

void setup() 
{
  Serial.begin(9600);
  //while (!Serial);
  pinMode(9, OUTPUT);
  servo.attach(6);
}

void loop() {
  read_analog_in();

  apply_vout_filter();

  ++dummy_load_value;
  if(dummy_load_value > 25500) dummy_load_value = 0;
  
  delay(10);

  output_set(vout_get());
  dummy_load_write();

  print_all();
}
