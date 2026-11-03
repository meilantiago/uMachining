#pragma once

// On-device relearning of the anti-cogging tables (M59), run at the end of each calibration (M56).
// The tracking error is folded over the electrical cycle for each direction and corrected harmonic
// by harmonic with a Newton step, using pre-measured harmonic responses G (these depend on the
// motor and the loop tuning, not on the calibration).

#include <cstdint>

class CoggingLearner {
  public:
    static constexpr int N = 64;   // = ServoController::COGGING_TABLE_SIZE

    // Prepares a pass: field phase at position 0 [rad], field rad per mm, cruise feed [mm/s].
    void begin(float field_at_zero_rad, float field_rad_per_mm, float feed_mm_s);
    void reset();

    // Called from the profile ISR (Core 0, 2 kHz); accumulates only while 'active' and cruising.
    volatile bool active = false;
    void add(double target_mm, double pos_mm, float vel_mm_s);

    // Mean-free error pattern [um] over the electrical cycle; dir 0 = forward (+), 1 = backward (-).
    // Returns false if there were too few samples in that direction.
    bool pattern(int dir, float e_um[N]) const;
    float rms_um() const;

    // In-place Newton step on the tables [electrical degrees].
    static void newton_step(float table_deg[N], float table_dir_deg[N], const float e_um[2][N], float gain);

  private:
    float field_at_zero = 0.0f;
    float field_per_mm = 0.0f;
    float feed = 0.0f;
    float sum[2][N] = {};
    uint32_t cnt[2][N] = {};
    float sq_sum = 0.0f;
    uint32_t sq_cnt = 0;
};
