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
  if(j < max_tok) tok[j] = msg;
  return (j < max_tok) ? j + 1 : max_tok;
}

String bool_to_on_of(bool value){
  return value?"on":"off";
}


const char * log_level_str[5] = {"DEBUG","INFO","WARN","ERROR","NONE"};

void send_serial_dbg(String msg,log_level_t level){
  #if (LOG_LEVEL==DEBUG || LOG_LEVEL==INFO || LOG_LEVEL==WARN || LOG_LEVEL==ERROR)
    char _msg[128];
    sprintf(_msg, "ENDBG,lvl:%s,msg:%s",log_level_str[level], msg.c_str());
    send_msg(&Serial, _msg);
  #endif
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

void send_msg(Print * p, const char * msg){
  char checksum = msg_checksum(msg);
  p->print("$");
  // write to print in chunks of 32 bytes
  for(size_t i=0; i<strlen(msg); i+=32){
    p->write(msg+i, min(32,strlen(msg)-i));
  }
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

cmd_t parse_cmd(String cmd){
  if(cmd == "pow")    return CMD_POWER;
  if(cmd == "rev")    return CMD_REVERSE;
  if(cmd == "reg")    return CMD_REGEN;
  if(cmd == "thr")    return CMD_THROTTLE;
  if(cmd == "rst")    return CMD_RESET;
  if(cmd == "mode")   return CMD_MODE;
  if(cmd == "target") return CMD_TARGET;
  return CMD_UNKNOWN;
}

String tokens[MAX_TOKENS];
void read_engine_commands(bool (*handle_cmd)(String token)){

  uint8_t num_tokens = tokenize(Serial.readStringUntil('\n'),',',tokens,MAX_TOKENS);
  Serial.print(num_tokens);
  Serial.print("Tokens: ");
  for(uint8_t i=0; i< num_tokens; i++){
    Serial.print(tokens[i]);
    Serial.print(" ");
  }
  Serial.println();
  if(num_tokens == 0) return;
  uint8_t good_tokens = 0;
  // Accept both manual panel commands (ENCMD) and throttle device commands (THRCMD)
  if(tokens[0] == "$" MSG_ENG_CMD || tokens[0] == "$" MSG_THR_CMD){
    for(uint8_t i=1; i< num_tokens; i++){
      if(handle_cmd(tokens[i])) good_tokens++;
    }

    char _msg[128];
    sprintf(_msg, "ENINF,st:COMMANDS_HANDLED,gd:%d,tt:%d", good_tokens, num_tokens-1);
    send_msg(&Serial,_msg);
  } else {
    char _msg[] = "ENERR,st:UNKNOWN_MSG";
    send_msg(&Serial,_msg);
  }
}