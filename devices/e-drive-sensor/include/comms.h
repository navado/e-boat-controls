#ifndef COMMS_H
#define COMMS_H
#include <Arduino.h>
#include "models.h"   // gps_state_t, gps_arb_t, GPS_SRC_*


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
#define MSG_GPS_RPT   "GPSRPT"  // Any node → all    : GPS broadcast for source arbitration
//   Format: $GPSRPT,src:<T|P|S>,sog:<kn10>,cog:<deg>,sow:<kn10>,valid:<0-3>*XX
//   valid bitmask: bit0 = GPS fix valid, bit1 = water-speed valid

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

// ── GPS source arbitration bus messages ───────────────────────────────────────
// Broadcast local GPS state on the shared internal bus.
// src = GPS_SRC_THROTTLE / GPS_SRC_PANEL / GPS_SRC_SENSOR
void send_gpsrpt(Print * p, uint8_t src, const gps_state_t * g);

// Parse a GPSRPT field array (already tokenised at commas; tok[0] == "$GPSRPT"
// or "GPSRPT").  Fills *src and *g; returns true on success.
bool parse_gpsrpt(String * tok, uint8_t ntok, uint8_t * src, gps_state_t * g);

// ── NMEA 0183 input parser (shared across nodes) ──────────────────────────────
// Parse one complete NMEA 0183 sentence (including leading '$' and trailing '*XX').
// Updates *g on recognised sentences (RMC, VTG, VHW).
// Returns true if *g was modified.
bool parse_nmea0183(const String & sentence, gps_state_t * g);

// ── Motor NMEA 0183 output sentences (external navigation bus, talker II) ─────
// $IIRPM,E,1,<rpm>,1.00,A*XX\r\n
void send_nmea0183_rpm(Print * p, uint16_t rpm);
// $IIXDR,V,<V.d>,V,BATT,U,<A.d>,A,BATT,P,<W>,W,BATT*XX\r\n
void send_nmea0183_xdr(Print * p, uint16_t vcc_mv, uint16_t curr_ma, uint16_t power_w);
#endif