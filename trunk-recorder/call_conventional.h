#ifndef CALL_CONVENTIONAL_H
#define CALL_CONVENTIONAL_H
#include "global_structs.h"
class System;
class Recorder;

#include "call.h"
#include "call_impl.h"
#include <string>

class Call_conventional : public Call_impl {
public:
  Call_conventional(long t, double f, System *s, Config c, double squelch_db, bool signal_detection);
  time_t get_start_time();
  bool is_conventional() { return true; }
  void restart_call();
  void set_recorder(Recorder *r);
  void recording_started();
  double get_squelch_db();
  bool get_signal_detection();
  bool get_call_start_sent() { return call_start_sent; }
  void set_call_start_sent(bool v) { call_start_sent = v; }
  time_t get_recording_start_time() { return recording_start_time; }
  void set_recording_start_time(time_t t) { recording_start_time = t; }
private:
  double squelch_db;
  bool signal_detection;
  bool call_start_sent;
  time_t recording_start_time;
};

#endif
