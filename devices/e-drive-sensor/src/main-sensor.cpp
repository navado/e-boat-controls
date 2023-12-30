#include <Arduino.h>
#include <TimeInterrupt.h>

#include <pinout.h>
#include <buttons.h>
#include <models.h>
#include <comms.h>


bool serial_sent = false;
void setup_rpm_counter(){
  pinMode(RPM_PIN, INPUT);
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
  update_buttons();
  digitalWrite(LED_BRD, !digitalRead(LED_BRD));
}

void setup_timing_functions(){
  TimeInterrupt.begin(PRECISION);
  TimeInterrupt.addInterrupt(run_every_100ms,100);
  TimeInterrupt.addInterrupt(run_every_1s,1000);
}

void set_state(){
  // TODO: implement guards for reverse and regen, not to switch them while throttle is actually powered

  digitalWrite(RELAY_LOCK, engine_state.power);
  digitalWrite(RELAY_REVERSE, engine_state.reverse);
  digitalWrite(RELAY_REGEN, engine_state.regen);
  digitalWrite(RELAY_SPARE, engine_state.spare);
  analogWrite(THROTTLE_OUT, engine_state.throttle);
}

void setup() {
  Serial.begin(115200);
  analogReference(DEFAULT);
  pinMode(LED_BRD, OUTPUT);
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
  // Buttons
  setup_buttons();
  // LCD
  // setup_screen();
  set_state();
}

void loop() {
  // digitalWrite(LED_BRD, !digitalRead(LED_BRD));
  if(serial_available()){
    read_serial_commands();
  }
  set_state();
  if (serial_sent) return;
  serial_sent = true;
  send_serial_field(&Serial, "t", String(millis()));
  send_serial_field(&Serial, "rpm", String(engine_state.rpm));
  send_serial_field(&Serial, "pow", bool_to_on_of(engine_state.power));
  send_serial_field(&Serial, "rev", bool_to_on_of(engine_state.reverse));
  send_serial_field(&Serial, "reg", bool_to_on_of(engine_state.regen));
  send_serial_field(&Serial, "thr", String(engine_state.throttle));
  send_serial_field(&Serial, "vth", String(map(engine_state.throttle_val,0,1023,0,5000)));
  send_serial_field(&Serial, "vcc", String(map(engine_state.vcc48v,0,1023,0,100000)));
  for (int i=0; i<4; i++){
    send_serial_field(&Serial, String("btn")+String(i), String(bool_to_on_of(buttons_state[i].state)), i==3);
  }

  // draw_screen();
  }
