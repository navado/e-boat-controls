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
}

void run_every_100ms(){
  engine_state.throttle_val = analogRead(THROTTLE_IN);
  engine_state.vcc48v = analogRead(VCC_SENS_IN);
  update_buttons();
  digitalWrite(LED_BRD, !digitalRead(LED_BRD));
}

void run_every_10s(){
    serial_sent = false;
}

void setup_timing_functions(){
  TimeInterrupt.begin(PRECISION);
  TimeInterrupt.addInterrupt(run_every_100ms,100);
  TimeInterrupt.addInterrupt(run_every_1s,1000);
  TimeInterrupt.addInterrupt(run_every_10s,10*1000);
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


bool handle_command(String token){
  String tok[4];
  uint8_t num_tokens = tokenize(token, ':', tok, 4);
  if(num_tokens != 2){
    return false;
  }
  cmd_t cmd = parse_cmd(tok[0]);
  on_off_t val = parse_on_off(tok[1]);
  int tv = 0;
  switch(cmd){
    case CMD_POWER:
      if(engine_state.power && engine_state.throttle > 1){
       send_serial_dbg("Cannot force power off when engine is running", WARN);
       break;
      }
      UPDATE_ON_OFF_FIELD(power, val);
      break;
    case CMD_REVERSE:
      if(engine_state.power==0) break;
      if(engine_state.throttle > 1){
        send_serial_dbg("Cannot force reverse when engine is running", WARN);
        break;
      } else if(engine_state.regen){
        send_serial_dbg("Cannot force reverse when regen is on",  WARN);
        break;
      }
      UPDATE_ON_OFF_FIELD(reverse, val);
      break;
    case CMD_REGEN:
      if(engine_state.power==0) break;
      if(engine_state.throttle > 1){
        send_serial_dbg("Cannot force regen when engine is running", WARN);
        break;
      } else if(engine_state.reverse){
        send_serial_dbg("Cannot force regen when reverse is on", WARN);
        break;
      }
      UPDATE_ON_OFF_FIELD(regen, val);
      break;
    case CMD_THROTTLE:
      if(engine_state.power==0){
        send_serial_dbg("Cannot set throttle when engine is off", WARN);
        break;
      }
      if(engine_state.regen){
        send_serial_dbg("Cannot set throttle when regen is on", WARN);
        break;
      }
      tv = tok[1].toInt();
      update_throttle_value(tv);
      break;
    case CMD_RESET:
      engine_state.power = 1;
      engine_state.reverse = 0;
      engine_state.regen = 0;
      engine_state.throttle = 0;
      break;
    case CMD_UNKNOWN:
    default:
  #if defined(DEBUG)
      Serial.print("WARNING: Command not supported: ");
      Serial.println(tokens[0]);
  #endif
    return false;
  }
  return true;
}

void loop() {
  if(serial_available()){
    read_engine_commands(handle_command);
  }
  set_state();
  if (serial_sent) return;

  
  serial_sent = true;
  char msg[128];
  // ENINF,T,POW,REV,REG,THR,VTH,VCC,RESERVED
  sprintf(msg, "ENINF,%lu,%d,%u,%u,%u,%d,%d,%ld,0",
    millis(),
    engine_state.rpm,
    engine_state.power,
    engine_state.reverse,
    engine_state.regen,
    engine_state.throttle,
    map(engine_state.throttle_val,0,1023,0,5000),
    map(engine_state.vcc48v,0,1023,0,100000)
  );
    while (Serial.available())
  {
    Serial.read();
  }
  send_msg(&Serial, msg);
}

