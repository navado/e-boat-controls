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
  char msg[] = "ENINF,st:STARTED";
  send_msg(&Serial, msg);
}

void loop() {
  // digitalWrite(LED_BRD, !digitalRead(LED_BRD));
  if(serial_available()){
    read_serial_commands();
  }
  set_state();
  if (serial_sent) return;
  serial_sent = true;
  char msg[128];
  sprintf(msg, "ENINF,T:%lu,rpm:%d,pow:%s,rev:%s,reg:%s,thr:%d,vth:%lu,vcc:%lu",
    millis(),
    engine_state.rpm,
    bool_to_on_of(engine_state.power).c_str(),
    bool_to_on_of(engine_state.reverse).c_str(),
    bool_to_on_of(engine_state.regen).c_str(),
    engine_state.throttle,
    map(engine_state.throttle_val,0,1023,0,5000),
    map(engine_state.vcc48v,0,1023,0,100000)
  );
  send_msg(&Serial, msg);
}

