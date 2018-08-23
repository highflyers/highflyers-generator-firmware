#include <Servo.h>
#include "pid.h"
#include "iir.h"
#include "thermistor.h"

#define RELAY_POS_RECTIFIER   0
#define RELAY_POS_ESC         1

Servo servo;
Servo servo_esc;
Servo servo_suction;
uint32_t servo_esc_startup_value = 100;
uint32_t servo_esc_idle_value = 25;
uint32_t servo_esc_delay_max = 350;
uint32_t servo_esc_delay;
uint32_t servo_esc_pulse_suction_max = 5;
uint32_t servo_esc_pulse_delay_max = 40;
uint32_t servo_esc_pulse_delay;
uint32_t servo_suction_active_vaule = 93;
uint32_t servo_suction_inactive_value = 35;

int vout;
uint8_t dummy_load_current;
int dummy_load_value;
uint32_t debug_servo_value;
uint8_t irq_count = 0, irq_flag = 0;
uint8_t irq_compare = 20;   // <-- config loop frequency

//uint32_t vout_iir_coeff = 240;    // <-- config filter
//uint32_t vout_prev = 0, vout_current = 0;
uint32_t output_minimum_value = 230;    // <-- config
uint32_t output_maximum_value = 470;    // <-- config
uint32_t output_idle_value = 217;
uint32_t output_startup_value = 280;    // <-- config

uint8_t digital_in_starter, digital_in_enable;

uint32_t powering_up_delay = 250;    // <-- config
uint32_t vout_scale_factor = 51;    // <-- config
uint32_t powering_up_counter = 0;

uint32_t setpoint = 10000;
uint32_t motor_dead_threshold = 100;
uint32_t motor_dead_count = 0;
uint32_t motor_dead_count_max = 100;

int current_state = 0;

Pid pid(0.75, 1, 1, output_minimum_value, 1000);
IIR voutIir(240);
IIR t1iir(120), t2iir(120);
Thermistor therm1(11, 10, 1.0);
Thermistor therm2(9.52, 10, 1.0);

enum state_machine_states
{
  STATE_IDLE = 0, STATE_STARTING, STATE_POWERING_UP, STATE_RUNNING, STATE_RE_IGNITION
};

void read_analog_in()
{
  vout = analogRead(A0);
  vout = vout > 600 ? 600 : vout;
  vout *= 51;
  t1iir.newSample(therm1.calculate(analogRead(A3)));
  t2iir.newSample(therm2.calculate(analogRead(A2)));
}

void digital_in_read()
{
  digital_in_starter = !digitalRead(15);
  digital_in_enable = !digitalRead(4); 
}

void relay_set(uint8_t value)
{
  digitalWrite(8, value);
}

void esc_enable(uint8_t value)
{
  digitalWrite(3, !value);
}

void digital_in_setup()
{
  pinMode(15, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(8, OUTPUT);
  pinMode(3, OUTPUT);
}

void apply_vout_filter()
{
  voutIir.newSample(vout);
}

uint32_t vout_get()
{
  return voutIir.getValue();
}

void output_set(uint32_t value)
{
  value = value < output_minimum_value ? output_minimum_value : value;
  value = value > output_maximum_value ? output_maximum_value : value;
  debug_servo_value = (value * 180) / 1024;
  servo.write(debug_servo_value);
}

void dummy_load_set(uint8_t value)
{
  digitalWrite(7, value);
}

void print_all()
{
    Serial.print(current_state);
    Serial.print(" ");
    Serial.print(vout_get());
    Serial.print(" ");
//    Serial.print(debug_servo_value);
//    Serial.print(" ");
//    Serial.print(pid.aggE);
//    Serial.print(" ");
//
//    Serial.print(t1iir.getValue());
//    Serial.print(" ");
//    Serial.print(t2iir.getValue());
//    Serial.print(" ");

//  Serial.print(servo_esc_delay);

  Serial.print(motor_dead_count);

  Serial.println(" ");
}

SIGNAL(TIMER0_COMPA_vect)
{
  ++irq_count;
  if (irq_count > irq_compare)
  {
    irq_count = 0;
    irq_flag = 1;
  }
}

void setup()
{
  Serial.begin(9600);
  //while (!Serial);
  pinMode(9, OUTPUT);
  servo.attach(6);
  servo_esc.attach(10);
  servo_suction.attach(9);

  digital_in_setup();

  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);
}

void state_machine_update()
{
  digitalWrite(8,1);
  if (digital_in_starter)
  {
    if(current_state != STATE_STARTING)
    {
      servo_esc_delay = servo_esc_delay_max;
      servo_esc_pulse_delay = servo_esc_pulse_delay_max;
    }
    current_state = STATE_STARTING;
  }
  else
  {
    if (digital_in_enable)
    {  
      if (current_state != STATE_RE_IGNITION)
      {
        if (current_state != STATE_RUNNING)
        {
          if (current_state != STATE_POWERING_UP)
          {
            powering_up_counter = 0;
          }
          current_state = STATE_POWERING_UP;
        }
      }
    }
    else
    {
      if (current_state != STATE_RE_IGNITION)
      {
        current_state = STATE_IDLE;       
      }
    }
  }
}

void loop_idle()
{
  dummy_load_set(0);
  pid.reset();
  output_set(output_idle_value);
  relay_set(RELAY_POS_RECTIFIER);
  esc_enable(0);
  servo_esc.write(servo_esc_idle_value);
  
  if(vout_get() < motor_dead_threshold)
  {
    ++motor_dead_count;
    if(motor_dead_count > motor_dead_count_max)
    {
      servo_esc_delay = servo_esc_delay_max;
      servo_esc_pulse_delay = servo_esc_pulse_delay_max;
      current_state = STATE_RE_IGNITION;
    }
  }
  else
  {
    motor_dead_count = 0;
  }
}

void loop_starting()
{
  dummy_load_set(0);
  relay_set(RELAY_POS_ESC);
  esc_enable(1);
  motor_dead_count = 0;
  if(servo_esc_delay > 0)
  {
    --servo_esc_delay;
    servo_esc.write(servo_esc_idle_value);
    output_set(output_minimum_value);
  }
  else
  {
    servo_esc_delay = 0;
    output_set(output_startup_value);
    if(servo_esc_pulse_delay > 0)
    {
      --servo_esc_pulse_delay;
      servo_esc.write(servo_esc_startup_value);
      servo_suction.write(servo_esc_pulse_delay < servo_esc_pulse_suction_max ? servo_suction_active_vaule : servo_suction_inactive_value);
    }
    else
    {
      servo_esc.write(servo_esc_idle_value);
      current_state = STATE_POWERING_UP;
    }
  }
}

void loop_powering_up()
{
  dummy_load_set(1);
  ++powering_up_counter;
  if (powering_up_counter >= powering_up_delay)
  {
    dummy_load_set(0);
    current_state = STATE_RUNNING;
  }
  loop_running();
}

void loop_running()
{
  double e = ((double)setpoint - (double)vout_get()) / 1000.0;
  double u = pid.loop(e);
  u = u > 1000 ? 1000 : u;
  u = u < output_minimum_value ? output_minimum_value : u;
  uint32_t uU = u;
  output_set(uU);
  relay_set(RELAY_POS_RECTIFIER);
  esc_enable(0);
  servo_esc.write(servo_esc_idle_value);

  if(vout_get() < motor_dead_threshold)
  {
    ++motor_dead_count;
    if(motor_dead_count > motor_dead_count_max)
    {
      servo_esc_delay = servo_esc_delay_max;
      servo_esc_pulse_delay = servo_esc_pulse_delay_max;
      current_state = STATE_RE_IGNITION;
    }
  }
  else
  {
    motor_dead_count = 0;
  }
  // set Q and P_GOOD
}

void loop_re_ignition()
{
  loop_starting();
}

void loop() {
  if (irq_flag)
  {
    read_analog_in();
    digital_in_read();
    apply_vout_filter();

    state_machine_update();
      switch(current_state)
      {
      case STATE_IDLE:
        loop_idle();
        break;
      case STATE_STARTING:
        loop_starting();
        break;
      case STATE_POWERING_UP:
        loop_powering_up();
        break;
      case STATE_RUNNING:
        loop_running();
        break;
      case STATE_RE_IGNITION:
        loop_re_ignition();
        break;
      default:
        loop_idle();
        break;
      }

    print_all();
    irq_flag = 0;
  }
}


