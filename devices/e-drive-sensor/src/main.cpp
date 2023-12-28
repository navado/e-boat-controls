#include <Arduino.h>
#include <TimeInterrupt.h>

#define LED_BRD 13      // LED on board
#define RPM_PIN 5       // T1
#define THROTTLE_IN A0  // 0.01/1 voltage divider
#define THROTTLE_OUT 6  // PWM
#define VCC_SENS_IN A1  // 470k/20k voltage divider

#define RELAY_LOCK PB0
#define RELAY_REVERSE PD7
#define RELAY_REGEN PD4
#define RELAY_SPARE PD2

typedef struct {
  uint8_t power:1;          // 0 - OFF, 1 - ON
  uint8_t reverse:1;        // 0 - OFF, 1 - ON
  uint8_t regen: 1;         // 0 - OFF, 1 - ON
  uint8_t spare: 6;
  uint8_t throttle: 8;      // For analogWrite
  uint16_t throttle_val;    // As read from ADC
  uint16_t vcc48v;          // Batery voltage
  uint16_t rpm;             // RPM
} engine_state_t;

volatile engine_state_t engine_state = {0};

typedef enum {
  off = 0,
  on = 1,
  error = 2
} on_off_t;

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
  engine_state.rpm = TCNT1;
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

String tokens[16];
uint8_t num_tokens = 0;
void msgToTokens(String msg){
  uint8_t i = 0;
  uint8_t j = 0;
  uint8_t len = msg.length();
  while(i<len){
    if(msg[i] == ','){
      tokens[j] = msg.substring(0,i);
      msg = msg.substring(i+1);
      len = msg.length();
      i = 0;
      j++;
    }
    i++;
  }
  tokens[j] = msg;
  num_tokens = j+1;
}

enum cmd_t {
  CMD_POWER,
  CMD_REVERSE,
  CMD_REGEN,
  CMD_THROTTLE,
  CMD_STATE,
  CMD_SPARE,
  CMD_UNKNOWN
};

cmd_t parse_cmd(String cmd){
  if(cmd == "power") return CMD_POWER;
  if(cmd == "reverse") return CMD_REVERSE;
  if(cmd == "regen") return CMD_REGEN;
  if(cmd == "throttle") return CMD_THROTTLE;
  if(cmd == "state") return CMD_STATE;
  if(cmd == "spare") return CMD_SPARE;
  return CMD_UNKNOWN;
}

#define UPDATE_ON_OFF_FIELD(f,v) if(v!=on_off_t::error) engine_state.f = v;

on_off_t parse_on_off(String val){
  if(val == "on") return on_off_t::on;
  if(val == "off") return on_off_t::off;
  return on_off_t::error;
}
void send_serial_field(Print * p, String name, String val, bool last = false, const String delim = ",", const String f_delim = ":"){
  p->print(name);
  p->print(f_delim);
  p->print(val);
  if(last) p->println();
  else p->print(delim);
}

void update_throttle_value(int val){
  if(val < 0 || val > 255){
    Serial.println("ERROR: Throttle value out of range");
    return;
  }
  engine_state.throttle = (uint8_t)val;
}

void read_serial_command(){
  msgToTokens(Serial.readStringUntil('\n'));
  cmd_t cmd = parse_cmd(tokens[0]);
  on_off_t val = parse_on_off(tokens[1]);
  switch(cmd){
    case CMD_POWER:
      if(engine_state.power && engine_state.throttle > 1){
        Serial.println("ERROR: Cannot force power off when engine is running");
        break;
      }
      UPDATE_ON_OFF_FIELD(power, val);
      break;
    case CMD_REVERSE:
      if(engine_state.power==0) break;
      if(engine_state.throttle > 1){
        Serial.println("ERROR: Cannot force reverse when engine is running");
        break;
      } else if(engine_state.regen){
        Serial.println("ERROR: Cannot force reverse when regen is on");
        break;
      }
      UPDATE_ON_OFF_FIELD(reverse, val);
      break;
    case CMD_REGEN:
      if(engine_state.power==0) break;
      if(engine_state.throttle > 1){
        Serial.println("ERROR: Cannot force regen when engine is running");
        break;
      } else if(engine_state.reverse){
        Serial.println("ERROR: Cannot force regen when reverse is on");
        break;
      }
      UPDATE_ON_OFF_FIELD(regen, val);
      break;
    case CMD_THROTTLE:
      if(engine_state.power==0) break;
      if(engine_state.regen){
        Serial.println("ERROR: Cannot set throttle when regen is on");
        break;
      }
      update_throttle_value(tokens[1].toInt());
      break;
    case CMD_STATE:
      UPDATE_ON_OFF_FIELD(power, parse_on_off(tokens[1]));
      UPDATE_ON_OFF_FIELD(reverse, parse_on_off(tokens[2]));
      UPDATE_ON_OFF_FIELD(regen, parse_on_off(tokens[3]));
      update_throttle_value(tokens[4].toInt());
      break;
    default:
      Serial.print("WARNING: Command not supported: ");
      Serial.println(tokens[0]);
  }
}


void loop() {
  if(Serial.available()){
    read_serial_command();
    serial_sent = false;
    set_state();
  }
  if (serial_sent) return;
  serial_sent = true;
  send_serial_field(&Serial, "T", String(millis()));
  send_serial_field(&Serial, "rpm", String(engine_state.rpm));
  send_serial_field(&Serial, "power", engine_state.power?"on":"off");
  send_serial_field(&Serial, "reverse", engine_state.reverse?"on":"off");
  send_serial_field(&Serial, "regen", engine_state.regen?"on":"off");
  send_serial_field(&Serial, "throttle", String(engine_state.throttle));
  send_serial_field(&Serial, "throttle_val", String(map(engine_state.throttle_val,0,1023,0,5000)));
  send_serial_field(&Serial, "vcc48v", String(map(engine_state.vcc48v,0,1023,0,100000)), true);
  }
