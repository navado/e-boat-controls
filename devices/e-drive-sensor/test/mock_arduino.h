/**
 * Minimal Arduino stub for native (host) unit testing.
 * Only the subset used by comms.cpp and models.cpp is implemented.
 */
#pragma once

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <string>

// ── Primitive types ─────────────────────────────────────────────────────────
typedef unsigned char  byte;
typedef unsigned long  ulong;

// ── Arduino min/max (avoid conflicts with <algorithm>) ───────────────────────
#ifndef min
#  define min(a,b) ((a)<(b)?(a):(b))
#endif
#ifndef max
#  define max(a,b) ((a)>(b)?(a):(b))
#endif

// ── Arduino String class ─────────────────────────────────────────────────────
class String {
public:
  std::string s;

  String() = default;
  String(const char* c)  : s(c ? c : "") {}
  String(const std::string& ss) : s(ss) {}
  String(char c)         : s(1, c) {}
  String(int v)          : s(std::to_string(v)) {}
  String(unsigned int v) : s(std::to_string(v)) {}
  String(long v)         : s(std::to_string(v)) {}

  // Comparison
  bool operator==(const String& o)  const { return s == o.s; }
  bool operator==(const char* c)    const { return s == c; }
  bool operator!=(const String& o)  const { return s != o.s; }
  bool operator!=(const char* c)    const { return s != c; }

  // Access
  char operator[](int i) const { return s[i]; }
  const char* c_str()    const { return s.c_str(); }
  int length()           const { return (int)s.size(); }

  // Substring — same semantics as Arduino: substring(from, to)
  String substring(int from, int to = -1) const {
    if (to < 0) to = (int)s.size();
    if (from < 0) from = 0;
    if (from > (int)s.size()) return String("");
    if (to > (int)s.size()) to = (int)s.size();
    return String(s.substr(from, to - from));
  }

  int indexOf(char c, int from = 0) const {
    auto p = s.find(c, from);
    return p == std::string::npos ? -1 : (int)p;
  }

  int   toInt()   const { return atoi(s.c_str()); }
  float toFloat() const { return (float)atof(s.c_str()); }

  String operator+(const String& o) const { return String(s + o.s); }
  String& operator+=(const String& o) { s += o.s; return *this; }
};

inline bool operator==(const char* c, const String& s) { return s == c; }
inline bool operator!=(const char* c, const String& s) { return s != c; }

// ── Arduino print-base constants ─────────────────────────────────────────────
#define HEX 16
#define DEC 10
#define OCT  8
#define BIN  2

// ── Print stub ───────────────────────────────────────────────────────────────
class Print {
public:
  virtual size_t write(uint8_t) { return 1; }
  // Accept both signed and unsigned char pointers (comms.cpp passes const char*)
  virtual size_t write(const uint8_t* buf, size_t n) { (void)buf; return n; }
  virtual size_t write(const char*    buf, size_t n) { (void)buf; return n; }
  size_t print(const char* s)               { (void)s; return strlen(s); }
  size_t print(const String& s)             { (void)s; return s.length(); }
  size_t print(int v, int base = 10)          { (void)v; (void)base; return 1; }
  size_t print(unsigned int v, int base = 10) { (void)v; (void)base; return 1; }
  size_t print(char c)                        { (void)c; return 1; }
  size_t println(const char* s = "")          { (void)s; return strlen(s) + 2; }
  size_t println(const String& s)             { (void)s; return s.length() + 2; }
  size_t println(int v, int base = 10)        { (void)v; (void)base; return 3; }
};

// ── Serial stub ──────────────────────────────────────────────────────────────
struct SerialStub : public Print {
  bool   available()           { return false; }
  int    read()                { return -1; }
  String readStringUntil(char) { return String(""); }
  void   begin(unsigned long)  {}
};
extern SerialStub Serial;

// ── Arduino helpers not needed by tested code but required to compile ────────
inline void delay(unsigned long) {}
inline unsigned long millis()    { return 0; }
