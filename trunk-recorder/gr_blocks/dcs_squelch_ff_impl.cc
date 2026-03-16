/* -*- c++ -*- */
/*
 * DCS (CDCSS) squelch block — implementation
 *
 * EIA/TIA-603 CDCSS:
 *   - 134.4 bps, Manchester-encoded NRZ
 *   - 23-bit codeword: (23,12) Golay code
 *   - Message word layout (12 bits):
 *       bits 11-3 : 9-bit DCS code (MSB first)
 *       bits 2-1  : polarity (00 = normal, 11 = inverted)
 *       bit  0    : complement of bit 11
 *
 * Generator polynomial: x^11 + x^10 + x^6 + x^5 + x^4 + x^2 + 1
 *   = 0xC75 (bits 11..0)
 */

#include "dcs_squelch_ff.h"
#include <gnuradio/io_signature.h>
#include <cstring>
#include <cmath>
#include <boost/log/trivial.hpp>

// ---------------------------------------------------------------------------
// Factory
// ---------------------------------------------------------------------------
dcs_squelch_ff_sptr make_dcs_squelch_ff(int sample_rate, int dcs_code, bool inverted, bool gate) {
#if GNURADIO_VERSION < 0x030900
  return dcs_squelch_ff_sptr(new dcs_squelch_ff(sample_rate, dcs_code, inverted, gate));
#else
  return std::make_shared<dcs_squelch_ff>(sample_rate, dcs_code, inverted, gate);
#endif
}

// ---------------------------------------------------------------------------
// Golay (23,12) implementation
// Generator poly: x^11+x^10+x^6+x^5+x^4+x^2+1 = 0xC75
// ---------------------------------------------------------------------------
static const uint32_t GOLAY_POLY = 0xC75u;

// Multiply one data bit into the 11-bit LFSR
static inline uint32_t golay_next(uint32_t reg, int bit) {
  int feedback = ((reg >> 10) & 1) ^ bit;
  reg = ((reg << 1) & 0x7FFu) ^ (feedback ? GOLAY_POLY : 0u);
  return reg;
}

uint32_t dcs_squelch_ff::golay_encode(uint16_t data12) {
  uint32_t reg = 0;
  // Feed 12 data bits MSB first
  for (int i = 11; i >= 0; i--) {
    reg = golay_next(reg, (data12 >> i) & 1);
  }
  // 23-bit word: 12 data bits (MSB) + 11 parity bits
  return ((uint32_t)data12 << 11) | reg;
}

// Syndrome: feed all 23 bits into the LFSR, result should be 0 if valid
int dcs_squelch_ff::golay_syndrome(uint32_t word) {
  uint32_t reg = 0;
  for (int i = 22; i >= 0; i--) {
    reg = golay_next(reg, (word >> i) & 1);
  }
  return (int)reg;
}

// Attempt single-bit-error correction; returns true if word is valid (or correctable)
bool dcs_squelch_ff::golay_decode(uint32_t word, uint16_t &data12) {
  if (golay_syndrome(word) == 0) {
    data12 = (uint16_t)(word >> 11);
    return true;
  }
  // Try flipping each of the 23 bits
  for (int i = 0; i < 23; i++) {
    uint32_t candidate = word ^ (1u << i);
    if (golay_syndrome(candidate) == 0) {
      data12 = (uint16_t)(candidate >> 11);
      return true;
    }
  }
  return false;
}

// ---------------------------------------------------------------------------
// DCS code → 12-bit Golay message
//   code      : decimal integer whose digits are the octal code (e.g. 23 → D023)
//   inverted  : false = normal (polarity bits 00), true = inverted (polarity bits 11)
//
// 12-bit message layout:
//   bits 11..3 = 9-bit DCS value (octal digits → actual value)
//   bits 2..1  = polarity: normal=0b00, inverted=0b11
//   bit  0     = complement of bit 11
// ---------------------------------------------------------------------------
uint16_t dcs_squelch_ff::dcs_to_message(int code, bool inverted) {
  // Convert decimal-octal notation to actual 9-bit value
  int hundreds = (code / 100) & 7;
  int tens      = (code /  10) % 10 & 7;
  int ones      =  code        % 10 & 7;
  uint16_t val9 = (uint16_t)((hundreds << 6) | (tens << 3) | ones);

  uint16_t polarity = inverted ? 0x3u : 0x0u; // bits 2..1
  uint16_t msg = (val9 << 3) | (polarity << 0);

  // bit 0 = complement of bit 11
  int bit11 = (msg >> 11) & 1;
  msg = (msg & ~0x1u) | (uint16_t)(bit11 ^ 1);

  return msg & 0x0FFFu;
}

// ---------------------------------------------------------------------------
// Constructor / destructor
// ---------------------------------------------------------------------------
dcs_squelch_ff::dcs_squelch_ff(int sample_rate, int dcs_code, bool inverted, bool gate)
    : gr::block("dcs_squelch_ff",
                gr::io_signature::make(1, 1, sizeof(float)),
                gr::io_signature::make(1, 1, sizeof(float))),
      d_sample_rate(sample_rate),
      d_dcs_code(dcs_code),
      d_inverted(inverted),
      d_gate(gate),
      d_unmuted(false) {
  uint16_t msg = dcs_to_message(dcs_code, inverted);
  d_target_word = (int)golay_encode(msg);

  d_samples_per_bit = (double)sample_rate / DECIM / 134.4;
  reset();

  BOOST_LOG_TRIVIAL(info) << "DCS squelch: code D" << dcs_code
                          << (inverted ? "I" : "N")
                          << "  12-bit msg=0x" << std::hex << msg
                          << "  23-bit word=0x" << d_target_word << std::dec;
}

dcs_squelch_ff::~dcs_squelch_ff() {}

void dcs_squelch_ff::reset() {
  d_decim_count   = 0;
  d_decim_acc     = 0.0f;
  d_lp_state      = 0.0f;
  d_bit_phase     = 0.0;
  d_last_sample   = 0.0f;
  d_last_bit      = 0;
  d_shift_reg     = 0;
  d_match_count   = 0;
  d_loss_count    = 0;
}

void dcs_squelch_ff::set_code(int dcs_code, bool inverted) {
  d_dcs_code   = dcs_code;
  d_inverted   = inverted;
  uint16_t msg = dcs_to_message(dcs_code, inverted);
  d_target_word = (int)golay_encode(msg);
  reset();
}

int  dcs_squelch_ff::get_code()     const { return d_dcs_code; }
bool dcs_squelch_ff::is_inverted()  const { return d_inverted; }
bool dcs_squelch_ff::unmuted()      const { return d_unmuted; }

// ---------------------------------------------------------------------------
// Scheduling
// ---------------------------------------------------------------------------
void dcs_squelch_ff::forecast(int noutput_items, gr_vector_int &ninput_items_required) {
  ninput_items_required[0] = noutput_items;
}

// ---------------------------------------------------------------------------
// Signal processing
// ---------------------------------------------------------------------------
int dcs_squelch_ff::general_work(int noutput_items,
                                  gr_vector_int &ninput_items,
                                  gr_vector_const_void_star &input_items,
                                  gr_vector_void_star &output_items) {
  const float *in  = (const float *)input_items[0];
  float       *out = (float *)output_items[0];

  int n_in   = ninput_items[0];
  int n_out  = 0;
  int in_idx = 0;

  // We produce one output sample per input sample (gate or zero)
  // but do DCS decoding on the decimated side-channel
  while (in_idx < n_in && n_out < noutput_items) {
    float sample = in[in_idx];

    // ---- Decimation accumulator ----
    d_decim_acc += sample;
    d_decim_count++;

    if (d_decim_count >= DECIM) {
      float decimated = d_decim_acc / DECIM;
      d_decim_count = 0;
      d_decim_acc   = 0.0f;

      // ---- Simple IIR low-pass (cutoff ~250 Hz at 4800 Hz) ----
      d_lp_state += LP_ALPHA * (decimated - d_lp_state);
      float subcarrier = d_lp_state;

      // ---- Manchester bit clock recovery ----
      // Advance bit phase by 1 sample (at decimated rate)
      d_bit_phase += 1.0;

      // Detect zero crossing for edge alignment
      bool crossing = (subcarrier > 0) != (d_last_sample > 0);
      if (crossing) {
        // Snap phase: Manchester edge at mid-bit means phase ~0.5
        double expected_edge = fmod(d_bit_phase, d_samples_per_bit);
        double half_bit = d_samples_per_bit * 0.5;
        // If we're near a half-bit boundary, this is the Manchester transition (ignore)
        // If we're near a full-bit boundary, align the clock
        if (fabs(expected_edge - d_samples_per_bit) < d_samples_per_bit * 0.25 ||
            expected_edge < d_samples_per_bit * 0.25) {
          // Near bit boundary — re-align
          d_bit_phase = fmod(d_bit_phase, d_samples_per_bit);
          if (d_bit_phase > d_samples_per_bit * 0.5)
            d_bit_phase -= d_samples_per_bit;
        }
        (void)half_bit;
      }

      // Sample bit value at the mid-point of each bit period
      if (d_bit_phase >= d_samples_per_bit) {
        d_bit_phase -= d_samples_per_bit;

        // Manchester decode: sample at 1/4 and 3/4 of bit, use sign at 3/4
        // Simplified: use current sign as the data bit
        int bit = (subcarrier > 0) ? 1 : 0;

        // Shift into 23-bit register
        d_shift_reg = ((d_shift_reg << 1) | (uint32_t)bit) & 0x7FFFFFu;

        // Check for valid Golay word in the shift register
        uint16_t decoded;
        if (golay_decode(d_shift_reg, decoded)) {
          // Check if this is our target code
          uint16_t target_msg = dcs_to_message(d_dcs_code, d_inverted);
          if (decoded == target_msg) {
            d_loss_count = 0;
            if (!d_unmuted) {
              d_match_count++;
              if (d_match_count >= MATCH_THRESH) {
                d_unmuted = true;
                BOOST_LOG_TRIVIAL(debug) << "DCS squelch opened: D" << d_dcs_code << (d_inverted ? "I" : "N");
              }
            }
          } else {
            // A valid word but wrong code — treat as loss
            if (d_unmuted) {
              d_loss_count++;
              if (d_loss_count >= LOSS_THRESH) {
                d_unmuted     = false;
                d_match_count = 0;
                BOOST_LOG_TRIVIAL(debug) << "DCS squelch closed (wrong code)";
              }
            } else {
              d_match_count = 0;
            }
          }
        }
        // No valid Golay word — don't immediately close (carrier may be absent)
        d_last_bit = bit;
      }

      d_last_sample = subcarrier;
    }

    // ---- Gate output ----
    if (d_unmuted) {
      out[n_out] = sample;
    } else {
      if (d_gate) {
        // gate mode: don't output (handled by returning fewer items)
        // For simplicity we output zeros; caller should use gate=false
        out[n_out] = 0.0f;
      } else {
        out[n_out] = 0.0f;
      }
    }

    in_idx++;
    n_out++;
  }

  consume_each(in_idx);
  return n_out;
}
