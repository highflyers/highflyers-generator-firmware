#include <Servo.h>

Servo servo;

int t1, t2, t3, vout;
uint8_t dummy_load_current;
int dummy_load_value;
uint32_t debug_servo_value;
uint8_t irq_count = 0, irq_flag = 0;
uint8_t irq_compare = 20;   // <-- config loop frequency

uint32_t vout_iir_coeff = 210;    // <-- config filter
uint32_t vout_prev = 0, vout_current = 0;
uint32_t output_minimum_value = 256;    // <-- config
uint32_t output_startup_value = 512;    // <-- config

uint8_t digital_in_starter, digital_in_enable;

uint32_t powering_up_delay = 250;    // <-- config
uint32_t powering_up_counter = 0;

int current_state = 0;

enum state_machine_states
{
  STATE_IDLE = 0, STATE_STARTING, STATE_POWERING_UP, STATE_RUNNING
};

void read_analog_in()
{
  vout = analogRead(A0);
  t1 = analogRead(A1);
  t2 = analogRead(A2);
  t3 = analogRead(A3);
}

void digital_in_read()
{
  digital_in_starter = !digitalRead(15);
  digital_in_enable = !digitalRead(4);
}

void digital_in_setup()
{
  pinMode(15, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
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

void dummy_load_set(uint8_t value)
{
  digitalWrite(9, value);
}

void print_all()
{
  Serial.print(current_state);
  Serial.print(" ");
  Serial.print(vout_get());
  Serial.print(" ");
  Serial.print(debug_servo_value);
  Serial.print(" ");
  Serial.print(powering_up_counter);
  Serial.print(" ");
  Serial.println(" ");
}

SIGNAL(TIMER0_COMPA_vect) 
{
  ++irq_count;
  if(irq_count > irq_compare)
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

  digital_in_setup();

  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);
}

void state_machine_update()
{
  if(digital_in_starter)
    {
      current_state = STATE_STARTING;
    }
    else
    {
      if(digital_in_enable)
      {
        if(current_state != STATE_RUNNING)
        {
          if(current_state != STATE_POWERING_UP)
          {
            powering_up_counter = 0;
          }
          current_state = STATE_POWERING_UP;
        }
      }
      else
      {
        current_state = STATE_IDLE;
      }
    }
}

void loop_idle()
{
  dummy_load_set(0);
  output_set(output_minimum_value);
}

void loop_starting()
{
  dummy_load_set(0);
  output_set(output_startup_value);
}

void loop_powering_up()
{
  dummy_load_set(1);
  ++powering_up_counter;
  if(powering_up_counter >= powering_up_delay)
  {
    dummy_load_set(0);
    current_state = STATE_RUNNING;
  }
  loop_running();
}

void loop_running()
{
    output_set(vout_get());
    // set Q and P_GOOD
}

void loop() {
  if(irq_flag)
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
      default:
        loop_idle();
        break;
    }
      
    print_all();
    irq_flag = 0;
  }
}


