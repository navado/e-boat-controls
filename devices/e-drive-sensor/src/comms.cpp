#include <Arduino.h>
#include "comms.h"
#include "models.h"


uint8_t tokenize(String msg, char delim , String * tok,uint8_t max_tok){
  uint8_t i = 0;
  uint8_t j = 0;
  uint8_t len = msg.length();
  while(i<len && j<max_tok){
    if(msg[i] == delim){
      tok[j] = msg.substring(0,i);
      msg = msg.substring(i+1);
      len = msg.length();
      i = 0;
      j++;
    }
    i++;
  }
  tok[j] = msg;
  return j+1;
}

cmd_t parse_cmd(String cmd){
  if(cmd == "pow") return CMD_POWER;
  if(cmd == "rev") return CMD_REVERSE;
  if(cmd == "reg") return CMD_REGEN;
  if(cmd == "thr") return CMD_THROTTLE;
  if(cmd == "rst") return CMD_RESET;
  return CMD_UNKNOWN;
}

String bool_to_on_of(bool value){
  return value?"on":"off";
}

void send_serial_error(String msg){
  #if defined(DEBUG)
  Serial.print("ERROR: ");
  Serial.println(msg);
  #endif
}


void handle_command(String token){
  String tok[4];
  uint8_t num_tokens = tokenize(token, ':', tok, 4);
  if(num_tokens != 2){
    send_serial_error("wrong comand format");
    return;
  }
  cmd_t cmd = parse_cmd(tok[0]);
  on_off_t val = parse_on_off(tok[1]);
  int tv = 0;
  switch(cmd){
    case CMD_POWER:
      if(engine_state.power && engine_state.throttle > 1){
       send_serial_error("Cannot force power off when engine is running");
        break;
      }
      UPDATE_ON_OFF_FIELD(power, val);
      break;
    case CMD_REVERSE:
      if(engine_state.power==0) break;
      if(engine_state.throttle > 1){
        send_serial_error("Cannot force reverse when engine is running");
        break;
      } else if(engine_state.regen){
        send_serial_error("Cannot force reverse when regen is on");
        break;
      }
      UPDATE_ON_OFF_FIELD(reverse, val);
      break;
    case CMD_REGEN:
      if(engine_state.power==0) break;
      if(engine_state.throttle > 1){
        send_serial_error("Cannot force regen when engine is running");
        break;
      } else if(engine_state.reverse){
        send_serial_error("Cannot force regen when reverse is on");
        break;
      }
      UPDATE_ON_OFF_FIELD(regen, val);
      break;
    case CMD_THROTTLE:
      if(engine_state.power==0){
        send_serial_error("Cannot set throttle when engine is off");
        break;
      }
      if(engine_state.regen){
        send_serial_error("Cannot set throttle when regen is on");
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
    delay(1); // Do nothing but to compile correctly w/o DEBUG
  }
}

void send_serial_field(
        Print * p,
        String name,
        String val,
        bool last,
        String delim,
        String f_delim){
  p->print(name);
  p->print(f_delim);
  p->print(val);
  if(last) p->println();
  else p->print(delim);
}

String tokens[MAX_TOKENS];
void read_serial_commands(){

  uint8_t num_tokens = tokenize(Serial.readStringUntil('\n'),',',tokens,MAX_TOKENS);
  Serial.print(num_tokens);
  Serial.print("Tokens: ");
  for(uint8_t i=0; i< num_tokens; i++){
    Serial.print(tokens[i]);
    Serial.print(" ");
  }
  Serial.println();
  if(num_tokens == 0) return;
  if(tokens[0] == "$ENCMD"){
    char _msg[] = "ENACK,st:OK";
    send_msg(&Serial,_msg);
    for(uint8_t i=1; i< num_tokens; i++){
      handle_command(tokens[i]);
    }
  }
}

void send_msg(Print * p, const char * msg){
  char checksum = msg_checksum(msg);
  p->print("$");
  p->print(msg);
  p->print("*");
  p->println(checksum, HEX);
}


char msg_checksum(const char * msg, char initial, int len)
{
  if(len == 0) len = strlen(msg);
  for(int i=0; i<len; i++){
    initial ^= msg[i];
  }
  return initial;
}