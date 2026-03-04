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
  #if (LOG_LEVEL < 4)   /* 4 = LOG_NONE */
    char _msg[128];
    sprintf(_msg, "ENDBG,lvl:%s,msg:%s",log_level_str[level], msg.c_str());
    send_msg(&Serial, _msg);
  #else
    (void)msg; (void)level;
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
    p->write(msg+i, min((size_t)32, strlen(msg)-i));
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

// ── GPS source arbitration ─────────────────────────────────────────────────────
void send_gpsrpt(Print * p, uint8_t src, const gps_state_t * g) {
  char msg[80];
  uint8_t valid = (g->valid ? 1u : 0u) | (g->sow_valid ? 2u : 0u);
  snprintf(msg, sizeof(msg),
    "%s,src:%c,sog:%u,cog:%u,sow:%u,valid:%u",
    MSG_GPS_RPT, (char)src,
    g->sog_kn10, g->cog_deg, g->sow_kn10, valid);
  send_msg(p, msg);
}

bool parse_gpsrpt(String * tok, uint8_t ntok, uint8_t * src, gps_state_t * g) {
  *src = GPS_SRC_NONE;
  if (ntok < 2) return false;
  for (uint8_t i = 1; i < ntok; i++) {
    String kv[2];
    if (tokenize(tok[i], ':', kv, 2) != 2) continue;
    String val = kv[1];
    int star = val.indexOf('*');
    if (star >= 0) val = val.substring(0, star);
    const String & key = kv[0];
    if      (key == "src")   *src         = (uint8_t)val[0];
    else if (key == "sog")   g->sog_kn10  = (uint16_t)val.toInt();
    else if (key == "cog")   g->cog_deg   = (uint16_t)val.toInt();
    else if (key == "sow")   g->sow_kn10  = (uint16_t)val.toInt();
    else if (key == "valid") {
      uint8_t v    = (uint8_t)val.toInt();
      g->valid     = (v & 1u) ? 1 : 0;
      g->sow_valid = (v & 2u) ? 1 : 0;
    }
  }
  return (*src != GPS_SRC_NONE);
}

// ── NMEA 0183 input parser ─────────────────────────────────────────────────────
bool parse_nmea0183(const String & s, gps_state_t * g) {
  int star = s.indexOf('*');
  if (star < 1 || s[0] != '$') return false;
  String body = s.substring(1, star);
  char calc_cs = msg_checksum(body.c_str());
  long rx_cs   = strtol(s.substring(star + 1).c_str(), NULL, 16);
  if (calc_cs != (char)rx_cs) return false;

  String tok[16];
  uint8_t n = tokenize(body, ',', tok, 16);
  if (n < 2) return false;

  if (tok[0] == "GPRMC" || tok[0] == "GNRMC") {
    if (n >= 9 && tok[2] == "A") {
      g->valid    = 1;
      g->sog_kn10 = (uint16_t)(tok[7].toFloat() * 10.0f);
      g->cog_deg  = (uint16_t)tok[8].toFloat();
    } else {
      g->valid = 0;
    }
    return true;
  }
  if (tok[0] == "GPVTG" || tok[0] == "GNVTG") {
    if (n >= 6) {
      g->sog_kn10 = (uint16_t)(tok[5].toFloat() * 10.0f);
      g->valid    = 1;
      return true;
    }
  }
  if (tok[0] == "VHW") {
    // $VHW,<hdg_T>,T,<hdg_M>,M,<sow_kn>,N,<sow_kmh>,K*cs
    if (n >= 6) {
      g->sow_kn10  = (uint16_t)(tok[5].toFloat() * 10.0f);
      g->sow_valid = 1;
      return true;
    }
  }
  return false;
}

// ── Motor NMEA 0183 output sentences ──────────────────────────────────────────
void send_nmea0183_rpm(Print * p, uint16_t rpm) {
  char body[40];
  snprintf(body, sizeof(body), "IIRPM,E,1,%u,1.00,A", rpm);
  char line[56];
  snprintf(line, sizeof(line), "$%s*%02X\r\n", body, (uint8_t)msg_checksum(body));
  p->print(line);
}

void send_nmea0183_xdr(Print * p, uint16_t vcc_mv, uint16_t curr_ma, uint16_t power_w) {
  char body[80];
  snprintf(body, sizeof(body),
    "IIXDR,V,%u.%u,V,BATT,U,%u.%u,A,BATT,P,%u,W,BATT",
    vcc_mv  / 1000, (vcc_mv  % 1000) / 100,
    curr_ma / 1000, (curr_ma % 1000) / 100,
    power_w);
  char line[96];
  snprintf(line, sizeof(line), "$%s*%02X\r\n", body, (uint8_t)msg_checksum(body));
  p->print(line);
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