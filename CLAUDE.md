# trunk-recorder — Claude Code Notes

## Project Overview
trunk-recorder is a GNU Radio-based application for recording trunked and conventional radio systems. The main source lives under `trunk-recorder/` relative to this file.

## Active Feature Branch
`feat/conventional-call-start-mqtt` — base for current work.

## DCS Squelch Feature (implemented on this branch)

### What was added
DCS (Digital Coded Squelch / CDCSS per EIA/TIA-603) support for conventional analog channels, reusing the existing `Tone` CSV column.

### CSV Format
The `Tone` column in the channel file now accepts three formats:
| Value | Meaning |
|-------|---------|
| `94.8` | CTCSS tone at 94.8 Hz (existing behaviour) |
| `D023N` | DCS code 023, normal polarity |
| `D023I` | DCS code 023, inverted polarity |

The `###` digits are the standard 3-digit octal DCS code (e.g. `023`, `127`, `754`).

### Files changed / created
| File | Change |
|------|--------|
| `trunk-recorder/gr_blocks/dcs_squelch_ff.h` | New — GR block API |
| `trunk-recorder/gr_blocks/dcs_squelch_ff_impl.cc` | New — Golay decoder + signal processing |
| `trunk-recorder/talkgroup.h` | Added `dcs_code` (int) and `dcs_inverted` (bool) fields |
| `trunk-recorder/talkgroup.cc` | Updated conventional constructor signature |
| `trunk-recorder/talkgroups.cc` | Parse `D###N`/`D###I` strings from `Tone` column |
| `trunk-recorder/setup_systems.cc` | Extract DCS fields from talkgroup; pass to recorder |
| `trunk-recorder/source.h` / `source.cc` | New `create_conventional_recorder` overload with DCS params |
| `trunk-recorder/recorders/analog_recorder.h` | DCS members + factory overload |
| `trunk-recorder/recorders/analog_recorder.cc` | Constructor + signal chain wiring |
| `CMakeLists.txt` | Added `dcs_squelch_ff_impl.cc` to source list |

### Signal chain
```
demod → [dcs_squelch_ff?] → deemph → [ctcss_squelch_ff?] → decim_audio → ...
```
DCS squelch runs **before** de-emphasis so the <300 Hz subcarrier is still present.
CTCSS squelch remains **after** de-emphasis (unchanged from original).

### DCS decoder internals (`dcs_squelch_ff_impl.cc`)
- Decimates 96 kHz → 4800 Hz (factor 20)
- IIR low-pass to isolate <250 Hz subcarrier
- Manchester decoding at 134.4 bps (~35.7 samples/bit at 4800 Hz)
- (23,12) Golay error-correcting code decode with single-bit correction
- Opens squelch after 2 consecutive matching codewords; closes after 5 misses

## Build System
CMake. Build directory convention: `build/` in repo root.
Standard build:
```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
```

## Key Directories
```
trunk-recorder/recorders/   — analog, DMR, P25 recorder implementations
trunk-recorder/gr_blocks/   — custom GNU Radio blocks
trunk-recorder/systems/     — system type definitions
trunk-recorder/plugins/     — plugin interface
```
