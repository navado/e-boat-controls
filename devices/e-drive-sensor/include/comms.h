#ifndef COMMS_H
#define COMMS_H
#include <Arduino.h>


enum cmd_t {
  CMD_POWER,
  CMD_REVERSE,
  CMD_REGEN,
  CMD_THROTTLE,
  CMD_RESET,
  CMD_UNKNOWN
};

void send_serial_field(
    Print * p,
    String name,
    String val,
    bool last = false,
    String delim = ",",
    String f_delim = ":");

typedef enum{
  DEBUG,
  INFO,
  WARN,
  ERROR,
  NONE
} log_level_t;
extern const char * log_level_str[]; 
void send_serial_dbg(String msg,log_level_t level);
void read_serial_commands();
inline bool serial_available(){return Serial.available();}
String bool_to_on_of(bool value);

#define MAX_TOKENS 16
uint8_t tokenize(String msg, char delim, String * tok, uint8_t max_tok = MAX_TOKENS);
char msg_checksum(const char * msg, char initial = 0, int len = 0);
void send_msg(Print * p, const char * msg);
cmd_t parse_cmd(String cmd);
bool handle_command(String token);
void read_engine_commands(bool (*handle_cmd)(String token));
#endif