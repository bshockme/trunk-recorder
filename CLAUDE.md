# trunk-recorder — Claude Context

## Project Overview
trunk-recorder is a GNU Radio-based SDR application that records trunked and conventional radio systems. Source lives in `trunk-recorder/trunk-recorder/` (the inner directory).

## Build
```bash
cd trunk-recorder && mkdir build && cd build
cmake ..
make -j$(nproc)
```
Uses CMake. Primary dependencies: GNU Radio, Boost, libsndfile, osmosdr.

## Source Layout
```
trunk-recorder/        ← repo root
  trunk-recorder/      ← source root (working directory)
    trunk-recorder/    ← core C++ source
      main.cc
      call.h / call_impl.h / call_impl.cc       ← base call class
      call_conventional.h / call_conventional.cc ← conventional channel calls
      monitor_systems.cc / monitor_systems.h     ← main loop, call management
      setup_systems.cc / setup_systems.h         ← startup, conventional channel init
      plugin_manager/                            ← plugin API and dispatch
      recorders/                                 ← analog, P25, DMR, SIGMF recorders
      gr_blocks/                                 ← GNU Radio blocks, MDC1200/decoders
      systems/                                   ← P25, SmartNet system implementations
    plugins/                                     ← built-in plugins (stat_socket, etc.)
    user_plugins/                                ← user-provided plugins (MQTT, etc.)
    tests/mqtt/                                  ← MQTT test config
```

## Startup Order (main.cc)
1. `load_config()` — parse JSON config
2. `initialize_plugins()` — load plugin .so files, call `parse_config` + `init`
3. `start_plugins()` — call `start()`, `setup_sources()`, `setup_systems()` on each plugin
4. `setup_systems()` — create conventional channel Call objects, start analog recorders
5. `tb->start()` — start GNU Radio flow graph
6. `monitor_messages()` — main loop (runs forever)

## Key Architecture Concepts

### Call Types
- `Call` — abstract interface (`call.h`)
- `Call_impl` — base implementation, used for trunked calls
- `Call_conventional` — extends `Call_impl` for all conventional system types

### Conventional System Types
- `conventional` — analog FM, recorder starts immediately at setup
- `conventionalP25` — P25 digital conventional, recorder deferred to main loop
- `conventionalDMR` — DMR conventional, recorder deferred to main loop
- `conventionalSIGMF` — IQ capture conventional, recorder deferred to main loop

### Main Loop Timing
- `monitor_messages()` sleeps 10ms per iteration
- `manage_calls()` runs every ~1 second
- `check_conventional_channel_detection()` runs every 100ms

### Plugin System
Plugins implement `Plugin_Api` (`plugin_manager/plugin_api.h`). Key callbacks:
- `call_start(Call*)` — fires when a new call/segment begins recording
- `call_end(Call_Data_t)` — fires after a recording is concluded and uploaded
- `calls_active(vector<Call*>)` — fires when the active call list changes
- `signal(unitId, signaling_type, sig_type, call, system, recorder)` — fires on MDC1200/FSYNC/STAR/TPS decode
- `setup_recorder(Recorder*)` — fires when a recorder is assigned to a call

### plugman_call_start Behavior
- **Trunked calls**: fires in `handle_call_grant()` after recorder starts
- **conventionalP25/DMR/SIGMF**: fires in `manage_conventional_call()` when `!recorder->is_active()`
- **conventional (analog)**: fires in `manage_conventional_call()` when `current_length > 0` AND `time - recording_start_time >= 1s` (1-second delay to allow MDC1200 decode)

### MDC1200 / Signaling Decoders
- Decoded in GR processing thread via `decoder_callback_handler` in `analog_recorder.cc`
- On decode: sets `wav_sink->set_source(unitId)`, `call->set_current_source_id(unitId)`, fires `plugman_signal`
- `curr_src_id` on the call is updated so it appears in `get_stats()["srcId"]` when `plugman_call_start` fires

### Call_conventional State Fields
- `call_start_sent` — true once `plugman_call_start` has been fired for the current segment; reset in `restart_call()`
- `recording_start_time` — `time(NULL)` when `current_length` first exceeds 0; reset in `restart_call()`
- `squelch_db`, `signal_detection` — per-channel squelch settings

### Recorder States
- `INACTIVE` — not started
- `ACTIVE` — running in flow graph
- `is_idle()` — ACTIVE but squelch closed (no audio passing)
- `is_active()` — state == ACTIVE

## Common Patterns

### Adding a new Call_impl virtual method
1. Add pure virtual to `call.h`
2. Declare in `call_impl.h`
3. Implement in `call_impl.cc`

### Conventional call segment lifecycle
1. `setup_systems.cc` creates Call_conventional, starts recorder
2. Squelch opens → `current_length > 0` → `recording_start_time` set
3. After 1 second → `plugman_call_start` fires (with MDC1200 unit ID if present)
4. Squelch closes → idle_count increments each manage loop
5. `idle_count > call_timeout` → `conclude_call()` + `restart_call()` → new segment begins
6. Repeat from step 2

## Notes
- MQTT plugin lives in `user_plugins/` (not in this repo); communicates via `plugman_call_start` / `plugman_call_end`
- `plugman_calls_active` is NOT called for conventional calls (only trunked)
- Threading: GR processing thread calls decoder callbacks; main thread runs manage loop. `curr_src_id` writes from GR thread are not mutex-protected but are safe in practice for scalar types.
- `Call_conventional::recording_started()` exists but is unused — was intended as a hook for when squelch opens
