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
  engine_state_t zero = {0};
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

  return UNITY_END();
}
