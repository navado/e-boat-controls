#include <Arduino.h>
#include <TimeInterrupt.h>

#include <pinout.h>
#include <ui.h>
#include <buttons.h>
#include <models.h>
#include <comms.h>


bool serial_sent = false;

void setup_rpm_counter(){
  pinMode(RPM_PIN, INPUT_PULLUP);
  TCCR1A=0; // setup timer-1
  TCCR1C=0;
  TIMSK1=0;
  GTCCR=0;
  TCCR1B=0b00000110; // falling edge
}

void run_every_1s(){
  engine_state.rpm = TCNT1; // TODO: Need to divide by 6 (6 pulses per revolution) or find correct prescaler
  TCNT1=0;
  serial_sent = false;
}

void run_every_100ms(){
  engine_state.throttle_val = analogRead(THROTTLE_IN);
  engine_state.vcc48v = analogRead(VCC_SENS_IN);
  digitalWrite(LED_BRD, !digitalRead(LED_BRD));
}

void setup_timing_functions(){
  TimeInterrupt.begin(PRECISION);
  TimeInterrupt.addInterrupt(run_every_100ms,100);
  TimeInterrupt.addInterrupt(run_every_1s,1000);
}

void set_state(){
  digitalWrite(RELAY_LOCK, engine_state.power);
  digitalWrite(RELAY_REVERSE, engine_state.reverse);
  digitalWrite(RELAY_REGEN, engine_state.regen);
  digitalWrite(RELAY_SPARE, engine_state.spare);
  analogWrite(THROTTLE_OUT, engine_state.throttle);
}

void setup() {
  Serial.begin(115200);
  analogReference(DEFAULT);
  // Throttle
  pinMode(THROTTLE_IN, INPUT);
  pinMode(THROTTLE_OUT, OUTPUT);
  // VCC
  pinMode(VCC_SENS_IN, INPUT);
  // Timers/counters
  setup_rpm_counter();
  setup_timing_functions();
  // Relays
  pinMode(RELAY_LOCK, OUTPUT);
  pinMode(RELAY_REVERSE, OUTPUT);
  pinMode(RELAY_REGEN, OUTPUT);
  pinMode(RELAY_SPARE, OUTPUT);

  set_state();
}

void loop() {
  if(serial_available()){
    read_serial_commands();
    set_state();
  }
  if (serial_sent) return;
  serial_sent = true;
  // send_serial_field(&Serial, "t", String(millis()));
  // send_serial_field(&Serial, "rpm", String(engine_state.rpm));
  // send_serial_field(&Serial, "pow", engine_state.power?"on":"off");
  // send_serial_field(&Serial, "rev", engine_state.reverse?"on":"off");
  // send_serial_field(&Serial, "reg", engine_state.regen?"on":"off");
  // send_serial_field(&Serial, "thr", String(engine_state.throttle));
  // send_serial_field(&Serial, "vth", String(map(engine_state.throttle_val,0,1023,0,5000)));
  // send_serial_field(&Serial, "vcc", String(map(engine_state.vcc48v,0,1023,0,100000)), true);
  model_foo();
  }
