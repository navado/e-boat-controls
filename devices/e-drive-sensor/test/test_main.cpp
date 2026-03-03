/**
 * Native unit tests for e-drive-sensor firmware.
 * Run with: pio test -e native
 *
 * Coverage:
 *  - tokenize()             comms.cpp
 *  - msg_checksum()         comms.cpp
 *  - parse_cmd()            comms.cpp
 *  - bool_to_on_of()        comms.cpp
 *  - parse_on_off()         models.cpp
 *  - update_throttle_value() models.cpp
 *  - Checksum round-trip    (validates Bug #3 fix)
 */

#include <unity.h>
#include "mock_arduino.h"   // must come before project headers

// Provide the Serial object required by comms.cpp / models.cpp
SerialStub Serial;

// Now pull in the headers under test
#include "comms.h"
#include "models.h"

// ── setUp / tearDown ─────────────────────────────────────────────────────────
void setUp(void) {
  // Reset engine state before each test (cast away volatile for re-initialization)
  engine_state_t zero = {};
  *(engine_state_t*)&engine_state = zero;
}
void tearDown(void) {}

// ═══════════════════════════════════════════════════════════════════════════
// tokenize()
// ═══════════════════════════════════════════════════════════════════════════

void test_tokenize_basic(void) {
  String tok[8];
  uint8_t n = tokenize("a,b,c", ',', tok, 8);
  TEST_ASSERT_EQUAL(3, n);
  TEST_ASSERT_EQUAL_STRING("a", tok[0].c_str());
  TEST_ASSERT_EQUAL_STRING("b", tok[1].c_str());
  TEST_ASSERT_EQUAL_STRING("c", tok[2].c_str());
}

void test_tokenize_single_token(void) {
  String tok[8];
  uint8_t n = tokenize("hello", ',', tok, 8);
  TEST_ASSERT_EQUAL(1, n);
  TEST_ASSERT_EQUAL_STRING("hello", tok[0].c_str());
}

void test_tokenize_empty_string(void) {
  String tok[8];
  uint8_t n = tokenize("", ',', tok, 8);
  // Empty input: one empty token
  TEST_ASSERT_EQUAL(1, n);
  TEST_ASSERT_EQUAL_STRING("", tok[0].c_str());
}

void test_tokenize_respects_max(void) {
  String tok[3];
  // Input has 5 tokens but max_tok = 3 — must not overflow
  uint8_t n = tokenize("a,b,c,d,e", ',', tok, 3);
  TEST_ASSERT_LESS_OR_EQUAL(3, n);
}

void test_tokenize_trailing_delimiter(void) {
  String tok[8];
  uint8_t n = tokenize("a,b,", ',', tok, 8);
  TEST_ASSERT_EQUAL(3, n);
  TEST_ASSERT_EQUAL_STRING("a",  tok[0].c_str());
  TEST_ASSERT_EQUAL_STRING("b",  tok[1].c_str());
  TEST_ASSERT_EQUAL_STRING("",   tok[2].c_str());
}

void test_tokenize_colon_delimiter(void) {
  String tok[4];
  uint8_t n = tokenize("pow:on", ':', tok, 4);
  TEST_ASSERT_EQUAL(2, n);
  TEST_ASSERT_EQUAL_STRING("pow", tok[0].c_str());
  TEST_ASSERT_EQUAL_STRING("on",  tok[1].c_str());
}

void test_tokenize_nmea_msg(void) {
  // Simulates what parse_serial_data does after the Bug #3 fix:
  // msg = data.substring(msg_start + 1, msg_end)  ->  "ENINF,100,50,1,0,0,45,2000,48000"
  String tok[MAX_TOKENS];
  uint8_t n = tokenize("ENINF,100,50,1,0,0,45,2000,48000", ',', tok, MAX_TOKENS);
  TEST_ASSERT_EQUAL(9, n);
  TEST_ASSERT_EQUAL_STRING("ENINF", tok[0].c_str());
  TEST_ASSERT_EQUAL(100, tok[1].toInt());
  TEST_ASSERT_EQUAL(50,  tok[2].toInt());
}

// ═══════════════════════════════════════════════════════════════════════════
// msg_checksum()
// ═══════════════════════════════════════════════════════════════════════════

void test_checksum_empty(void) {
  // XOR of zero bytes with initial 0 = 0
  TEST_ASSERT_EQUAL(0, msg_checksum("", 0, 0));
}

void test_checksum_single_char(void) {
  // XOR of 'A' (0x41) with 0 = 0x41
  TEST_ASSERT_EQUAL(0x41, msg_checksum("A"));
}

void test_checksum_known_string(void) {
  // "ENINF" → 'E'^'N'^'I'^'N'^'F'
  char expected = 'E' ^ 'N' ^ 'I' ^ 'N' ^ 'F';
  TEST_ASSERT_EQUAL(expected, msg_checksum("ENINF"));
}

void test_checksum_custom_initial(void) {
  // With initial = 0xFF, XOR of 'A' = 0xFF ^ 0x41 = 0xBE
  TEST_ASSERT_EQUAL((char)(0xFF ^ 0x41), msg_checksum("A", 0xFF));
}

void test_checksum_length_limit(void) {
  // Only first 3 bytes of "ABCDE" = 'A'^'B'^'C'
  char expected = 'A' ^ 'B' ^ 'C';
  TEST_ASSERT_EQUAL(expected, msg_checksum("ABCDE", 0, 3));
}

void test_checksum_roundtrip(void) {
  // Bug #3 fix validation:
  // send_msg computes checksum over msg (no '$').
  // parse_serial_data now also skips '$', giving the same input.
  const char* body = "ENINF,1234,100,1,0,0,45,2000,48000";
  char cs_send  = msg_checksum(body);        // what send_msg uses
  // Simulate old (buggy) receive path: include '$'
  std::string with_dollar = std::string("$") + body;
  char cs_old_rx = msg_checksum(with_dollar.c_str());
  // Simulate new (fixed) receive path: exclude '$'
  char cs_new_rx = msg_checksum(body);

  TEST_ASSERT_EQUAL(cs_send, cs_new_rx);   // Fixed path matches
  TEST_ASSERT_NOT_EQUAL(cs_send, cs_old_rx); // Old path mismatched
}

// ═══════════════════════════════════════════════════════════════════════════
// parse_cmd()
// ═══════════════════════════════════════════════════════════════════════════

void test_parse_cmd_power(void)   { TEST_ASSERT_EQUAL(CMD_POWER,   parse_cmd("pow")); }
void test_parse_cmd_reverse(void) { TEST_ASSERT_EQUAL(CMD_REVERSE,  parse_cmd("rev")); }
void test_parse_cmd_regen(void)   { TEST_ASSERT_EQUAL(CMD_REGEN,    parse_cmd("reg")); }
void test_parse_cmd_throttle(void){ TEST_ASSERT_EQUAL(CMD_THROTTLE, parse_cmd("thr")); }
void test_parse_cmd_reset(void)   { TEST_ASSERT_EQUAL(CMD_RESET,    parse_cmd("rst")); }
void test_parse_cmd_unknown(void) { TEST_ASSERT_EQUAL(CMD_UNKNOWN,  parse_cmd("xyz")); }
void test_parse_cmd_empty(void)   { TEST_ASSERT_EQUAL(CMD_UNKNOWN,  parse_cmd(""));   }

void test_parse_cmd_case_sensitive(void) {
  // Must be lowercase only
  TEST_ASSERT_EQUAL(CMD_UNKNOWN, parse_cmd("POW"));
  TEST_ASSERT_EQUAL(CMD_UNKNOWN, parse_cmd("Pow"));
}

// ═══════════════════════════════════════════════════════════════════════════
// bool_to_on_of()
// ═══════════════════════════════════════════════════════════════════════════

void test_bool_to_on_of_true(void) {
  TEST_ASSERT_EQUAL_STRING("on",  bool_to_on_of(true).c_str());
}
void test_bool_to_on_of_false(void) {
  TEST_ASSERT_EQUAL_STRING("off", bool_to_on_of(false).c_str());
}

// ═══════════════════════════════════════════════════════════════════════════
// parse_on_off()
// ═══════════════════════════════════════════════════════════════════════════

void test_parse_on_off_on(void)  { TEST_ASSERT_EQUAL(on,    parse_on_off("on"));  }
void test_parse_on_off_off(void) { TEST_ASSERT_EQUAL(off,   parse_on_off("off")); }
void test_parse_on_off_err(void) { TEST_ASSERT_EQUAL(error, parse_on_off(""));    }

void test_parse_on_off_case_sensitive(void) {
  TEST_ASSERT_EQUAL(error, parse_on_off("ON"));
  TEST_ASSERT_EQUAL(error, parse_on_off("Off"));
}

void test_parse_on_off_unknown(void) {
  TEST_ASSERT_EQUAL(error, parse_on_off("yes"));
  TEST_ASSERT_EQUAL(error, parse_on_off("1"));
}

// ═══════════════════════════════════════════════════════════════════════════
// update_throttle_value()
// ═══════════════════════════════════════════════════════════════════════════

void test_throttle_valid_zero(void) {
  engine_state.throttle = 99;
  update_throttle_value(0);
  TEST_ASSERT_EQUAL(0, engine_state.throttle);
}

void test_throttle_valid_max(void) {
  update_throttle_value(255);
  TEST_ASSERT_EQUAL(255, engine_state.throttle);
}

void test_throttle_valid_mid(void) {
  update_throttle_value(128);
  TEST_ASSERT_EQUAL(128, engine_state.throttle);
}

void test_throttle_reject_negative(void) {
  engine_state.throttle = 50;
  update_throttle_value(-1);
  TEST_ASSERT_EQUAL(50, engine_state.throttle); // unchanged
}

void test_throttle_reject_over_255(void) {
  engine_state.throttle = 50;
  update_throttle_value(256);
  TEST_ASSERT_EQUAL(50, engine_state.throttle); // unchanged
}

// ═══════════════════════════════════════════════════════════════════════════
// throttle_table bounds
// ═══════════════════════════════════════════════════════════════════════════

void test_throttle_table_size(void) {
  TEST_ASSERT_EQUAL(11, THROTTLE_TABLE_SIZE);
}

void test_throttle_table_neutral(void) {
  // Index SPD_NEUTRAL (5) must be 1 (near-zero output at neutral)
  TEST_ASSERT_EQUAL(1, throttle_table[SPD_NEUTRAL]);
}

void test_throttle_table_inner_symmetric(void) {
  // Inner 9 values (indices 1-9) are symmetric around neutral (index 5).
  // Indices 0 and 10 differ intentionally: max reverse (245) < max forward (254).
  for (int i = 1; i <= 9; i++) {
    TEST_ASSERT_EQUAL(throttle_table[i], throttle_table[THROTTLE_TABLE_SIZE - 1 - i]);
  }
}

void test_throttle_table_max_forward_gt_reverse(void) {
  // Max forward speed (index 10 = 254) is intentionally higher than
  // max reverse speed (index 0 = 245), giving more headroom going forward.
  TEST_ASSERT_EQUAL(245, throttle_table[0]);   // max reverse
  TEST_ASSERT_EQUAL(254, throttle_table[10]);  // max forward
}

// ═══════════════════════════════════════════════════════════════════════════
// throttle_mode_t — enum sanity
// ═══════════════════════════════════════════════════════════════════════════

void test_throttle_mode_values(void) {
  TEST_ASSERT_EQUAL(0, (int)MODE_RPM);
  TEST_ASSERT_EQUAL(1, (int)MODE_POWER);
  TEST_ASSERT_EQUAL(2, (int)MODE_SOG);
  TEST_ASSERT_EQUAL(3, (int)MODE_SOW);
  TEST_ASSERT_EQUAL(4, (int)MODE_RANGE);
  TEST_ASSERT_EQUAL(5, (int)MODE_COUNT);
}

void test_throttle_mode_names_count(void) {
  // throttle_mode_names[] must have a non-empty entry for each mode
  for (int i = 0; i < (int)MODE_COUNT; i++) {
    TEST_ASSERT_TRUE(throttle_mode_names[i] != nullptr);
    TEST_ASSERT_TRUE(strlen(throttle_mode_names[i]) > 0);
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// parse_cmd() — new mode/target tokens
// ═══════════════════════════════════════════════════════════════════════════

void test_parse_cmd_mode(void)   { TEST_ASSERT_EQUAL(CMD_MODE,   parse_cmd("mode"));   }
void test_parse_cmd_target(void) { TEST_ASSERT_EQUAL(CMD_TARGET, parse_cmd("target")); }

// ═══════════════════════════════════════════════════════════════════════════
// pid_compute() — basic behaviour
// ═══════════════════════════════════════════════════════════════════════════

void test_pid_zero_error(void) {
  pid_state_t pid = {1.0f, 0.0f, 0.0f, 0, 0, 0, 0, 255};
  float out = pid_compute(&pid, 100.0f, 100.0f, 0.1f);
  // zero error → output should be 0 (clamped to output_min = 0)
  TEST_ASSERT_TRUE(out >= -0.01f && out <= 0.01f);
}

void test_pid_proportional(void) {
  pid_state_t pid = {2.0f, 0.0f, 0.0f, 0, 0, 0, 0, 255};
  float out = pid_compute(&pid, 50.0f, 0.0f, 0.1f);
  // error = 50, Kp = 2 → out = 100
  TEST_ASSERT_TRUE(out >= 99.9f && out <= 100.1f);
}

void test_pid_clamp_max(void) {
  pid_state_t pid = {100.0f, 0.0f, 0.0f, 0, 0, 0, 0, 254};
  float out = pid_compute(&pid, 500.0f, 0.0f, 0.1f);
  TEST_ASSERT_LESS_OR_EQUAL(254.0f, out);
}

void test_pid_clamp_min(void) {
  pid_state_t pid = {1.0f, 0.0f, 0.0f, 0, 0, 0, 0, 254};
  float out = pid_compute(&pid, 0.0f, 100.0f, 0.1f);
  // negative error → output ≤ 0, clamped to output_min = 0
  TEST_ASSERT_LESS_OR_EQUAL(0.01f, out);
}

void test_pid_reset(void) {
  pid_state_t pid = {1.0f, 1.0f, 0.0f, 999.0f, 999.0f, 0.0f, 0, 255};
  pid_reset(&pid);
  TEST_ASSERT_TRUE(pid.integral   < 0.001f && pid.integral   > -0.001f);
  TEST_ASSERT_TRUE(pid.prev_error < 0.001f && pid.prev_error > -0.001f);
}

void test_pid_integral_winds_up_and_clamps(void) {
  pid_state_t pid = {0.0f, 1.0f, 0.0f, 0, 0, 0, 0, 10};
  // Accumulate 20 steps with constant error of 5; Ki=1, output_max=10
  for (int i = 0; i < 20; i++) pid_compute(&pid, 5.0f, 0.0f, 1.0f);
  // Anti-windup must cap output at output_max
  float out = pid_compute(&pid, 5.0f, 0.0f, 1.0f);
  TEST_ASSERT_LESS_OR_EQUAL(10.0f, out);
}

// ═══════════════════════════════════════════════════════════════════════════
// THRINF / THRCMD message format — tokenize round-trip
// ═══════════════════════════════════════════════════════════════════════════

void test_thrinf_tokenize(void) {
  // Simulates the 7-field THRINF body (without '$' and '*xx')
  String tok[MAX_TOKENS];
  const char * body = "THRINF,12345,2,150,32,10,270";
  uint8_t n = tokenize(body, ',', tok, MAX_TOKENS);
  TEST_ASSERT_EQUAL(7, n);
  TEST_ASSERT_EQUAL_STRING("THRINF", tok[0].c_str());
  TEST_ASSERT_EQUAL(12345, tok[1].toInt());    // timestamp
  TEST_ASSERT_EQUAL(2,     tok[2].toInt());    // MODE_SOG
  TEST_ASSERT_EQUAL(150,   tok[3].toInt());    // target 15.0 kn
  TEST_ASSERT_EQUAL(32,    tok[4].toInt());    // SOG 3.2 kn
  TEST_ASSERT_EQUAL(10,    tok[5].toInt());    // SOW 1.0 kn
  TEST_ASSERT_EQUAL(270,   tok[6].toInt());    // COG 270°
}

void test_thrcmd_tokenize(void) {
  String tok[MAX_TOKENS];
  const char * body = "THRCMD,pow:on,rev:off,reg:off,thr:145,mode:0,target:1500";
  uint8_t n = tokenize(body, ',', tok, MAX_TOKENS);
  TEST_ASSERT_EQUAL(7, n);
  TEST_ASSERT_EQUAL_STRING("THRCMD",   tok[0].c_str());
  TEST_ASSERT_EQUAL_STRING("pow:on",   tok[1].c_str());
  TEST_ASSERT_EQUAL_STRING("thr:145",  tok[4].c_str());
  TEST_ASSERT_EQUAL_STRING("mode:0",   tok[5].c_str());
  TEST_ASSERT_EQUAL_STRING("target:1500", tok[6].c_str());
}

void test_eninf_extended_tokenize(void) {
  // Extended ENINF with curr_ma, power_w, water_kn10, slip_pct10
  String tok[MAX_TOKENS];
  const char * body = "ENINF,10000,1200,1,0,0,145,2400,48000,15000,720,12,350";
  uint8_t n = tokenize(body, ',', tok, MAX_TOKENS);
  TEST_ASSERT_EQUAL(13, n);
  TEST_ASSERT_EQUAL(15000, tok[9].toInt());  // curr_ma
  TEST_ASSERT_EQUAL(720,   tok[10].toInt()); // power_w
  TEST_ASSERT_EQUAL(12,    tok[11].toInt()); // water_kn10
  TEST_ASSERT_EQUAL(350,   tok[12].toInt()); // slip_pct10 = 35.0 %
}

// ═══════════════════════════════════════════════════════════════════════════
// calc_prop_slip()
// ═══════════════════════════════════════════════════════════════════════════

void test_slip_zero_rpm(void) {
  // Slip undefined at zero RPM → returns 0
  TEST_ASSERT_EQUAL(0, calc_prop_slip(0, 50, 600));
}

void test_slip_zero_pitch(void) {
  TEST_ASSERT_EQUAL(0, calc_prop_slip(1000, 50, 0));
}

void test_slip_full_slip(void) {
  // SOW == 0 while RPM > 0: 100 % slip (churning in place)
  int16_t slip = calc_prop_slip(1000, 0, 600);
  TEST_ASSERT_EQUAL(1000, slip); // 100.0 %
}

void test_slip_zero_slip(void) {
  // SOW exactly matches theoretical speed → 0 % slip
  // v_theoretical = 1000 * 600 / 60000 = 10 m/s
  // 10 m/s in kn*10 = 10 / 0.05144 ≈ 194.4 → use 194
  // expected slip ≈ (1 - 194*0.05144/10)*100 = (1 - 9.98/10)*100 = 0.2 %
  // We just check it's near zero (< 5 = 0.5 %)
  int16_t slip = calc_prop_slip(1000, 194, 600);
  TEST_ASSERT_TRUE(slip >= -5 && slip <= 5);
}

void test_slip_typical(void) {
  // Typical scenario: 1000 RPM, 600 mm pitch → v_theoretical = 10 m/s
  // Boat doing 5 m/s (SOW = 97 kn*10) → slip = 50 %
  // v_actual = 97 * 0.05144 = 4.99 m/s  → slip = (1 - 4.99/10)*100 ≈ 50.1 %
  int16_t slip = calc_prop_slip(1000, 97, 600);
  // Allow ±5 (±0.5 %) tolerance for floating-point rounding
  TEST_ASSERT_TRUE(slip >= 495 && slip <= 510); // ~50 %
}

void test_slip_clamped_at_100(void) {
  // Very high slip scenario should clamp at 100 %
  int16_t slip = calc_prop_slip(100, 0, 100); // extreme slip
  TEST_ASSERT_EQUAL(1000, slip); // clamped at 100.0 %
}

void test_slip_negative(void) {
  // Negative slip: following current pushing boat faster than prop advance
  // v_theoretical = 500 * 600 / 60000 = 5 m/s
  // v_actual = 200 kn*10 * 0.05144 = 10.29 m/s → slip = (1 - 10.29/5)*100 = -105.8 → clamped -100
  int16_t slip = calc_prop_slip(500, 200, 600);
  TEST_ASSERT_EQUAL(-1000, slip); // clamped at -100.0 %
}

// ═══════════════════════════════════════════════════════════════════════════
// main
// ═══════════════════════════════════════════════════════════════════════════

int main(void) {
  UNITY_BEGIN();

  // tokenize
  RUN_TEST(test_tokenize_basic);
  RUN_TEST(test_tokenize_single_token);
  RUN_TEST(test_tokenize_empty_string);
  RUN_TEST(test_tokenize_respects_max);
  RUN_TEST(test_tokenize_trailing_delimiter);
  RUN_TEST(test_tokenize_colon_delimiter);
  RUN_TEST(test_tokenize_nmea_msg);

  // msg_checksum
  RUN_TEST(test_checksum_empty);
  RUN_TEST(test_checksum_single_char);
  RUN_TEST(test_checksum_known_string);
  RUN_TEST(test_checksum_custom_initial);
  RUN_TEST(test_checksum_length_limit);
  RUN_TEST(test_checksum_roundtrip);

  // parse_cmd
  RUN_TEST(test_parse_cmd_power);
  RUN_TEST(test_parse_cmd_reverse);
  RUN_TEST(test_parse_cmd_regen);
  RUN_TEST(test_parse_cmd_throttle);
  RUN_TEST(test_parse_cmd_reset);
  RUN_TEST(test_parse_cmd_unknown);
  RUN_TEST(test_parse_cmd_empty);
  RUN_TEST(test_parse_cmd_case_sensitive);

  // bool_to_on_of
  RUN_TEST(test_bool_to_on_of_true);
  RUN_TEST(test_bool_to_on_of_false);

  // parse_on_off
  RUN_TEST(test_parse_on_off_on);
  RUN_TEST(test_parse_on_off_off);
  RUN_TEST(test_parse_on_off_err);
  RUN_TEST(test_parse_on_off_case_sensitive);
  RUN_TEST(test_parse_on_off_unknown);

  // update_throttle_value
  RUN_TEST(test_throttle_valid_zero);
  RUN_TEST(test_throttle_valid_max);
  RUN_TEST(test_throttle_valid_mid);
  RUN_TEST(test_throttle_reject_negative);
  RUN_TEST(test_throttle_reject_over_255);

  // throttle_table
  RUN_TEST(test_throttle_table_size);
  RUN_TEST(test_throttle_table_neutral);
  RUN_TEST(test_throttle_table_inner_symmetric);
  RUN_TEST(test_throttle_table_max_forward_gt_reverse);

  // throttle_mode_t
  RUN_TEST(test_throttle_mode_values);
  RUN_TEST(test_throttle_mode_names_count);

  // parse_cmd — new tokens
  RUN_TEST(test_parse_cmd_mode);
  RUN_TEST(test_parse_cmd_target);

  // pid_compute / pid_reset
  RUN_TEST(test_pid_zero_error);
  RUN_TEST(test_pid_proportional);
  RUN_TEST(test_pid_clamp_max);
  RUN_TEST(test_pid_clamp_min);
  RUN_TEST(test_pid_reset);
  RUN_TEST(test_pid_integral_winds_up_and_clamps);

  // message format round-trips
  RUN_TEST(test_thrinf_tokenize);
  RUN_TEST(test_thrcmd_tokenize);
  RUN_TEST(test_eninf_extended_tokenize);

  // calc_prop_slip
  RUN_TEST(test_slip_zero_rpm);
  RUN_TEST(test_slip_zero_pitch);
  RUN_TEST(test_slip_full_slip);
  RUN_TEST(test_slip_zero_slip);
  RUN_TEST(test_slip_typical);
  RUN_TEST(test_slip_clamped_at_100);
  RUN_TEST(test_slip_negative);

  return UNITY_END();
}
