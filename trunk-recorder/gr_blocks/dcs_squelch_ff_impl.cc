/* -*- c++ -*- */
/*
 * DCS (CDCSS) squelch block — implementation
 *
 * EIA/TIA-603 CDCSS:
 *   - 134.4 bps, NRZ encoding
 *   - 23-bit codeword: (23,12) Golay code, transmitted continuously
 *   - Message word layout (12 bits):
 *       bits 11-3 : 9-bit DCS code (MSB first)
 *       bits 2-1  : polarity (00 = normal, 11 = inverted)
 *       bit  0    : complement of bit 11
 *
 * Generator polynomial for (23,12) Golay code:
 *   g(x) = x^11 + x^10 + x^6 + x^5 + x^4 + x^2 + 1 = 0xC75
 *
 * After FM demodulation, the DCS subcarrier appears as a low-frequency
 * (~134 Hz fundamental) signal. The absolute polarity depends on the
 * FM demodulator convention, so we check both the shift register value
 * and its bitwise complement.
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
  return dcs_squelch_ff_sptr(new dcs_squelch_ff(sample_rate, dcs_code, inverted, gate));
}

// ---------------------------------------------------------------------------
// Golay (23,12) implementation
// Generator poly: g(x) = x^11+x^10+x^6+x^5+x^4+x^2+1 = 0xC75
//
// Systematic encoding: codeword = [data(12) | remainder(11)]
// where remainder = (data * x^11) mod g(x)
// ---------------------------------------------------------------------------
static const uint32_t GOLAY_POLY = 0xC75u;

// Compute the 11-bit remainder of a 23-bit word divided by g(x)
static uint32_t golay_remainder(uint32_t word) {
  for (int i = 22; i >= 11; i--) {
    if (word & (1u << i)) {
      word ^= (GOLAY_POLY << (i - 11));
    }
  }
  return word & 0x7FFu;
}

uint32_t dcs_squelch_ff::golay_encode(uint16_t data12) {
  // Systematic encode: [data | parity]
  uint32_t shifted = (uint32_t)(data12 & 0xFFFu) << 11;
  uint32_t parity = golay_remainder(shifted);
  return shifted | parity;
}

int dcs_squelch_ff::golay_syndrome(uint32_t word) {
  return (int)golay_remainder(word);
}

// Attempt to decode a 23-bit word. Returns true if valid or single-bit correctable.
bool dcs_squelch_ff::golay_decode(uint32_t word, uint16_t &data12) {
  if (golay_syndrome(word) == 0) {
    data12 = (uint16_t)((word >> 11) & 0xFFFu);
    return true;
  }
  // Try flipping each of the 23 bits for single-bit error correction
  for (int i = 0; i < 23; i++) {
    uint32_t candidate = word ^ (1u << i);
    if (golay_syndrome(candidate) == 0) {
      data12 = (uint16_t)((candidate >> 11) & 0xFFFu);
      return true;
    }
  }
  return false;
}

// ---------------------------------------------------------------------------
// DCS code → 12-bit Golay message
//
// 12-bit message layout (MSB to LSB):
//   bits 11..3 = 9-bit DCS value (octal digits → binary)
//   bits 2..1  = polarity: normal=0b00, inverted=0b11
//   bit  0     = complement of bit 11
// ---------------------------------------------------------------------------
uint16_t dcs_squelch_ff::dcs_to_message(int code, bool inverted) {
  // Convert decimal-stored-octal to 9-bit binary value
  // e.g. code=23 → octal 023 → binary 0*64 + 2*8 + 3 = 19
  int d2 = (code / 100) % 10; // hundreds digit (octal)
  int d1 = (code / 10) % 10;  // tens digit
  int d0 = code % 10;         // ones digit
  // Clamp each digit to 0-7 for safety
  d2 &= 7; d1 &= 7; d0 &= 7;
  uint16_t val9 = (uint16_t)((d2 << 6) | (d1 << 3) | d0);

  uint16_t polarity = inverted ? 0x3u : 0x0u;
  uint16_t msg = (val9 << 3) | (polarity << 1);

  // bit 0 = complement of bit 11
  int bit11 = (msg >> 11) & 1;
  msg = (msg & 0xFFFEu) | (uint16_t)(bit11 ^ 1);

  return msg & 0x0FFFu;
}

// Validate that a decoded 12-bit message has valid DCS structure
static bool validate_dcs_message(uint16_t msg) {
  // Check polarity bits (bits 2..1): must be 00 or 11
  int pol = (msg >> 1) & 3;
  if (pol != 0 && pol != 3)
    return false;

  // Check bit 0 = complement of bit 11
  int bit11 = (msg >> 11) & 1;
  int bit0 = msg & 1;
  if (bit0 != (bit11 ^ 1))
    return false;

  // Check that the 9-bit code (bits 11..3) is a valid octal value (each 3-bit group 0-7)
  // This is inherently true for 9 bits split into 3 groups of 3, so just check non-zero
  int val9 = (msg >> 3) & 0x1FF;
  if (val9 == 0)
    return false;

  return true;
}

// ---------------------------------------------------------------------------
// Constructor / destructor
// ---------------------------------------------------------------------------
dcs_squelch_ff::dcs_squelch_ff(int sample_rate, int dcs_code, bool inverted, bool gate)
    : gr::sync_block("dcs_squelch_ff",
                     gr::io_signature::make(1, 1, sizeof(float)),
                     gr::io_signature::make(1, 1, sizeof(float))),
      d_sample_rate(sample_rate),
      d_dcs_code(dcs_code),
      d_inverted(inverted),
      d_unmuted(false) {
  uint16_t msg = dcs_to_message(dcs_code, inverted);
  d_target_word = golay_encode(msg);

  d_samples_per_bit = (double)(sample_rate / DECIM) / 134.4;
  reset();

  BOOST_LOG_TRIVIAL(info) << "DCS squelch: code D"
                          << ((dcs_code < 100) ? "0" : "") << ((dcs_code < 10) ? "0" : "") << dcs_code
                          << (inverted ? "I" : "N")
                          << "  12-bit msg=0x" << std::hex << msg
                          << "  23-bit target=0x" << d_target_word << std::dec
                          << "  samples_per_bit=" << d_samples_per_bit;
}

dcs_squelch_ff::~dcs_squelch_ff() {}

void dcs_squelch_ff::reset() {
  d_decim_count      = 0;
  d_decim_acc        = 0.0f;
  d_lp_state         = 0.0f;
  d_bit_phase        = 0.0;
  d_last_sample      = 0.0f;
  d_last_bit         = 0;
  d_shift_reg        = 0;
  d_match_count      = 0;
  d_loss_count       = 0;
  d_bits_since_match = 0;
}

void dcs_squelch_ff::set_code(int dcs_code, bool inverted) {
  d_dcs_code   = dcs_code;
  d_inverted   = inverted;
  uint16_t msg = dcs_to_message(dcs_code, inverted);
  d_target_word = golay_encode(msg);
  reset();
}

int  dcs_squelch_ff::get_code()    const { return d_dcs_code; }
bool dcs_squelch_ff::is_inverted() const { return d_inverted; }
bool dcs_squelch_ff::unmuted()     const { return d_unmuted; }

// ---------------------------------------------------------------------------
// Signal processing — 1:1 sync block
// ---------------------------------------------------------------------------
int dcs_squelch_ff::work(int noutput_items,
                          gr_vector_const_void_star &input_items,
                          gr_vector_void_star &output_items) {
  const float *in  = (const float *)input_items[0];
  float       *out = (float *)output_items[0];

  const uint16_t target_msg = dcs_to_message(d_dcs_code, d_inverted);

  for (int i = 0; i < noutput_items; i++) {
    float sample = in[i];

    // ---- Decimation accumulator (96kHz → 4800 Hz) ----
    d_decim_acc += sample;
    d_decim_count++;

    if (d_decim_count >= DECIM) {
      float decimated = d_decim_acc / DECIM;
      d_decim_count = 0;
      d_decim_acc   = 0.0f;

      // ---- IIR low-pass to isolate DCS subcarrier (<200 Hz at 4800 Hz) ----
      d_lp_state += LP_ALPHA * (decimated - d_lp_state);
      float subcarrier = d_lp_state;

      // ---- NRZ bit clock recovery ----
      d_bit_phase += 1.0;

      // Zero-crossing detection for clock alignment
      if ((subcarrier > 0) != (d_last_sample > 0)) {
        // Edge detected — nudge clock toward bit boundary
        double phase_in_bit = fmod(d_bit_phase, d_samples_per_bit);
        double half_bit = d_samples_per_bit * 0.5;
        // If crossing is in the first or last quarter, it's near a boundary
        if (phase_in_bit < d_samples_per_bit * 0.25) {
          // Crossing just after boundary — phase is slightly ahead, slow down
          d_bit_phase -= phase_in_bit * 0.5;
        } else if (phase_in_bit > d_samples_per_bit * 0.75) {
          // Crossing just before next boundary — phase is slightly behind, speed up
          d_bit_phase += (d_samples_per_bit - phase_in_bit) * 0.5;
        }
        // Crossings near mid-bit are normal NRZ transitions, ignore them
      }

      // Sample bit at mid-point of each bit period
      if (d_bit_phase >= d_samples_per_bit) {
        d_bit_phase -= d_samples_per_bit;

        // NRZ: sample the sign of the filtered subcarrier
        int bit = (subcarrier > 0) ? 1 : 0;

        // Shift into 23-bit register (MSB first)
        d_shift_reg = ((d_shift_reg << 1) | (uint32_t)bit) & 0x7FFFFFu;
        d_bits_since_match++;

        // Try to decode the current 23-bit window.
        // Because FM demod polarity is unknown, check both orientations.
        bool matched = false;
        for (int try_inv = 0; try_inv < 2 && !matched; try_inv++) {
          uint32_t word = (try_inv == 0) ? d_shift_reg : (d_shift_reg ^ 0x7FFFFFu);
          uint16_t decoded;
          if (golay_decode(word, decoded)) {
            // Structural validation: reject random Golay-valid words
            if (!validate_dcs_message(decoded))
              continue;

            if (decoded == target_msg) {
              matched = true;
              d_loss_count = 0;
              d_bits_since_match = 0;
              d_match_count++;
              if (!d_unmuted && d_match_count >= MATCH_THRESH) {
                d_unmuted = true;
                BOOST_LOG_TRIVIAL(info) << "DCS squelch OPEN: D"
                    << ((d_dcs_code < 100) ? "0" : "") << ((d_dcs_code < 10) ? "0" : "") << d_dcs_code
                    << (d_inverted ? "I" : "N")
                    << (try_inv ? " (inverted FM)" : "");
              }
            } else {
              // Valid DCS word but wrong code — count as loss
              if (d_unmuted) {
                d_loss_count++;
                if (d_loss_count >= LOSS_THRESH) {
                  d_unmuted     = false;
                  d_match_count = 0;
                  BOOST_LOG_TRIVIAL(info) << "DCS squelch CLOSED (wrong code detected)";
                }
              } else {
                d_match_count = 0;
              }
            }
          }
        }

        // Timeout: close squelch if no matching word for ~3 codeword periods
        // 3 * 23 = 69 bits ≈ 513 ms at 134.4 bps
        if (d_unmuted && d_bits_since_match >= TIMEOUT_BITS) {
          d_unmuted     = false;
          d_match_count = 0;
          d_loss_count  = 0;
          BOOST_LOG_TRIVIAL(info) << "DCS squelch CLOSED (timeout — no code for "
              << (d_bits_since_match / 134.4) << "s)";
        }

        d_last_bit = bit;
      }

      d_last_sample = subcarrier;
    }

    // ---- Gate: pass audio when unmuted, zero when muted ----
    out[i] = d_unmuted ? sample : 0.0f;
  }

  return noutput_items;
}
