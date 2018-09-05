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
uint32_t servo_esc_delay_max = 200;
uint32_t servo_esc_delay;
uint32_t servo_esc_pulse_suction_max = 5;
uint32_t servo_esc_pulse_delay_max = 40;
uint32_t servo_esc_pulse_delay;
uint32_t servo_suction_active_vaule = 4;
uint32_t servo_suction_inactive_value = 36;

int vout;
uint8_t dummy_load_current;
int dummy_load_value;
uint32_t debug_servo_value;
uint8_t irq_count = 0, irq_flag = 0;
uint8_t irq_compare = 20;   // <-- config loop frequency

//uint32_t vout_iir_coeff = 240;    // <-- config filter
//uint32_t vout_prev = 0, vout_current = 0;
uint32_t output_minimum_value = 260;    // <-- config
//uint32_t output_maximum_value = 470;    // <-- config
uint32_t output_maximum_value = 350;    // <-- config

uint32_t output_idle_value = 217;
uint32_t output_startup_value = 310;    // <-- config

uint8_t digital_in_starter, digital_in_enable;

uint32_t powering_up_delay = 250;    // <-- config
uint32_t vout_scale_factor = 51;    // <-- config
uint32_t powering_up_counter = 0;

uint32_t setpoint = 12600;
uint32_t motor_dead_threshold = 50;
uint32_t motor_dead_count = 0;
uint32_t motor_dead_count_max = 100;

int current_state = 0;

Pid pid(1, 0.1, 1, output_minimum_value, 50);
IIR voutIir(250);
IIR t1iir(120), t2iir(120);
IIR m_v(120), m_a(120), o_v(120), o_a(120);
Thermistor therm1(11, 10, 1.0);
Thermistor therm2(9.52, 10, 1.0);

enum state_machine_states
{
  STATE_IDLE = 0, STATE_STARTING, STATE_POWERING_UP, STATE_RUNNING, STATE_RE_IGNITION
};

void read_analog_in()
{
  vout = analogRead(A7);
  vout = vout > 600 ? 600 : vout;
  vout *= 51;

  m_v.newSample(analogRead(A0));
  m_a.newSample(analogRead(A1));
  o_v.newSample(analogRead(A3));
  o_a.newSample(analogRead(A2));
  
  t1iir.newSample(therm1.calculate(analogRead(A6)));
  t2iir.newSample(therm2.calculate(analogRead(A8)));
}

void digital_in_read()
{
  digital_in_starter = !digitalRead(16);
  digital_in_enable = !digitalRead(7); 
}

void relay_set(uint8_t value)
{
  digitalWrite(15, value);
}

void digital_in_setup()
{
  pinMode(16, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);
  pinMode(15, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(5, OUTPUT);
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
  debug_servo_value = value;
  servo.write((value*180)/1024);
}

void dummy_load_set(uint8_t value)
{
  digitalWrite(7, value);
}

void print_all()
{
//    Serial11.print(current_state*5000);
//    Serial11.print(" ");
    Serial1.print(vout_get());
    Serial1.print(" ");
//    Serial1.print(m_a.getValue());
//    Serial1.print(" ");
//    Serial1.print(m_v.getValue());
//    Serial1.print(" ");
//    Serial1.print(o_a.getValue());
//    Serial1.print(" ");
//    Serial1.print(o_v.getValue());
//    Serial1.print(" ");
    Serial1.print(pid.aggE*10);
    Serial1.print(" ");
    Serial1.print(pid.ret*100);
    Serial1.print(" ");
//
//    Serial1.print(t1iir.getValue());
//    Serial1.print(" ");
//    Serial1.print(t2iir.getValue());
//    Serial1.print(" ");

//  Serial1.print(servo_esc_delay);

  Serial1.print(motor_dead_count*1000);
  
    Serial1.print(" ");
    Serial1.print((output_maximum_value-output_minimum_value)*100);
    Serial1.print(" ");
    Serial1.print((debug_servo_value-output_minimum_value)*100);
//    Serial1.print(digital_in_enable*10000);

  Serial1.println(" ");
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
  Serial1.begin(9600);
  //while (!Serial1);
  pinMode(9, OUTPUT);
  servo.attach(9);
  servo_esc.attach(10);
  servo_suction.attach(5);

  digital_in_setup();

  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);
}

void state_machine_update()
{
  digitalWrite(8,1);
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

void loop_idle()
{
  dummy_load_set(0);
  pid.reset();
  output_set(output_idle_value);
  relay_set(RELAY_POS_RECTIFIER);

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
  double u = pid.loop(e) + output_minimum_value;
  u = u > 1000 ? 1000 : u;
  u = u < output_minimum_value ? output_minimum_value : u;
  uint32_t uU = u;
  output_set(uU);
  relay_set(RELAY_POS_RECTIFIER);

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

    
    if (digital_in_starter)
    {
      servo_suction.write(servo_suction_active_vaule);
    }
    else
    {
      servo_suction.write(servo_suction_inactive_value);
    }

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


