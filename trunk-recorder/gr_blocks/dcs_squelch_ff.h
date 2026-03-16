/* -*- c++ -*- */
/*
 * DCS (Digital Coded Squelch / CDCSS) squelch block for trunk-recorder
 * Implements EIA/TIA-603 CDCSS detection on FM-demodulated audio.
 */

#ifndef INCLUDED_DCS_SQUELCH_FF_H
#define INCLUDED_DCS_SQUELCH_FF_H

#include <gnuradio/block.h>

class dcs_squelch_ff;

#if GNURADIO_VERSION < 0x030900
typedef boost::shared_ptr<dcs_squelch_ff> dcs_squelch_ff_sptr;
#else
typedef std::shared_ptr<dcs_squelch_ff> dcs_squelch_ff_sptr;
#endif

/*!
 * \brief DCS/CDCSS squelch block.
 *
 * Accepts float audio samples (post FM-demod) at the given sample_rate.
 * Gates output: passes samples when the configured DCS code is detected,
 * mutes (outputs zeros) otherwise.
 *
 * \param sample_rate  Input sample rate in Hz (typically 96000)
 * \param dcs_code     3-digit octal code stored as decimal integer (e.g., 23 for D023)
 * \param inverted     true for inverted polarity (D###I), false for normal (D###N)
 * \param gate         if true, block output when squelched; if false, output zeros
 */
dcs_squelch_ff_sptr make_dcs_squelch_ff(int sample_rate, int dcs_code, bool inverted, bool gate = false);

class dcs_squelch_ff : public gr::block {
  friend dcs_squelch_ff_sptr make_dcs_squelch_ff(int sample_rate, int dcs_code, bool inverted, bool gate);

protected:
  dcs_squelch_ff(int sample_rate, int dcs_code, bool inverted, bool gate);

public:
  ~dcs_squelch_ff();

  void set_code(int dcs_code, bool inverted);
  int  get_code() const;
  bool is_inverted() const;
  bool unmuted() const;

  void forecast(int noutput_items, gr_vector_int &ninput_items_required) override;
  int general_work(int noutput_items,
                   gr_vector_int &ninput_items,
                   gr_vector_const_void_star &input_items,
                   gr_vector_void_star &output_items) override;

private:
  int   d_sample_rate;
  int   d_dcs_code;       // as entered (e.g. 23 for D023)
  int   d_target_word;    // 23-bit Golay-encoded target codeword
  bool  d_inverted;
  bool  d_gate;
  bool  d_unmuted;

  // Decimation / subcarrier extraction
  static const int DECIM = 20;          // 96000 -> 4800 Hz
  static const int BAUD_RATE_NUM = 672; // 134.4 * 5 numerator (4800/134.4 = ~35.71 samp/bit)
  static const int BAUD_RATE_DEN = 5;

  int   d_decim_count;
  float d_decim_acc;

  // Simple IIR low-pass to isolate <250 Hz subcarrier (at 4800 Hz)
  float d_lp_state;
  static constexpr float LP_ALPHA = 0.34f; // cutoff ~250 Hz at 4800 Hz

  // Manchester / bit clock recovery
  double d_bit_phase;       // 0.0 .. 1.0 within a bit period
  double d_samples_per_bit; // 4800 / 134.4 ≈ 35.714

  float  d_last_sample;
  int    d_last_bit;

  // 23-bit shift register for Golay word detection
  uint32_t d_shift_reg;

  // Lock state
  int  d_match_count;   // consecutive matching words needed to open squelch
  int  d_loss_count;    // consecutive missing words to close squelch
  static const int MATCH_THRESH = 2;
  static const int LOSS_THRESH  = 5;

  // Golay encode/decode helpers
  static uint32_t golay_encode(uint16_t data12);
  static int      golay_syndrome(uint32_t word);
  static bool     golay_decode(uint32_t word, uint16_t &data12);

  // Convert 3-digit decimal-octal DCS code (e.g. 23) to 12-bit Golay message
  static uint16_t dcs_to_message(int code, bool inverted);

  void reset();
};

#endif /* INCLUDED_DCS_SQUELCH_FF_H */
