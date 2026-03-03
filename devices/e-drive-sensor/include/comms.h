#ifndef COMMS_H
#define COMMS_H
#include <Arduino.h>


enum cmd_t {
  CMD_POWER,
  CMD_REVERSE,
  CMD_REGEN,
  CMD_THROTTLE,
  CMD_RESET,
  CMD_MODE,    // mode:<0-4>   — throttle control mode
  CMD_TARGET,  // target:<val> — mode target (RPM / W / kn*10)
  CMD_UNKNOWN
};

// Message type identifiers (all frames: $<TYPE>,<fields>*<CRC>\n)
#define MSG_ENG_INFO  "ENINF"   // Sensor   → all    : engine telemetry
#define MSG_ENG_CMD   "ENCMD"   // Panel    → sensor : manual control
#define MSG_THR_CMD   "THRCMD"  // Throttle → sensor : primary control
#define MSG_THR_INFO  "THRINF"  // Throttle → all    : mode / GPS status
#define MSG_PAN_CTL   "PANCTL"  // Throttle → panel  : panel power control

void send_serial_field(
    Print * p,
    String name,
    String val,
    bool last = false,
    String delim = ",",
    String f_delim = ":");

typedef enum{
  LOG_DEBUG,
  LOG_INFO,
  LOG_WARN,
  LOG_ERROR,
  LOG_NONE
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