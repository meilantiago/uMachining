// --------------------------------------------------------------------------------------
// Project: MicroManipulatorStepper
// License: MIT (see LICENSE file for full description)
//          All text in here must be included in any redistribution.
// Author:  M. S. (diffraction limited)
// --------------------------------------------------------------------------------------

#pragma once

#include <vector>
#include <cstdint>
#include <cmath>

//*** CLASS *****************************************************************************

class LookupTable {
  public:
    LookupTable() {}

    // initializes the lookup table to a given size and input range
    bool init(int32_t table_size, float input_min, float input_max);

    // initializes the lookup table from a list of input output value pairs
    // approximation/interpolation will be used to sample the input range in
    // equidistant steps. 
    bool init_interpolating(std::vector<std::pair<float, float>> in_out_pairs, 
                            int table_size, bool sort_input);
    
    // fits the lookup table to the provided data (exact least squares)
    bool optimize_lut(const std::vector<std::pair<float, float>>& in_out_pairs, float& rms_error);

    // makes the table periodic: outside the input range, x is shifted by whole multiples
    // of 'period_in' and the output by the same multiples of 'period_out'. The input
    // range must be at least one period long. period_in = 0 disables it.
    void set_period(float period_in, float period_out);
    float get_period_in() const { return period_in; }
    float get_period_out() const { return period_out; }

    // clear the lookup table, use init to use it again
    void clear();

    // returns the size of the lookup table
    uint32_t size() const;

    // set an entry of the lookup table
    void set_entry(int32_t idx, float v);

    // get an entry of the lookup table
    float get_entry(int32_t idx) const;

    // evaluate the lookup table at a given position with linear interpolation
    float evaluate(float x) const;

    // Like evaluate(), for a periodic table with integer input (encoder counts) and double
    // output. Whole periods are removed with integer arithmetic before converting to float,
    // because float(x) keeps only 24 bits and a float output of hundreds of rad only resolves
    // ~3e-5 rad. Without a period it falls back to evaluate().
    double evaluate_periodic(int64_t x) const;
    // Same, split for the control loop: returns the part within one period and writes the
    // whole periods to 'periods' (result = periods * period_out + return value).
    float evaluate_periodic_split(int64_t x, int32_t& periods) const;

    // evaluate the inverse of the lookup table function (very slow), the LUT must be monotonic
    float evaluate_inverse(float y) const;

    // inverts the lookup table so it represents the function x = fi(y) given y = f(x) 
    bool invert(int new_size);

    // check if the lookup table is monotonic
    bool is_monotonic() const;

    // check if the given value is inside the input range
    bool in_input_range(float x) const;

    // check if the given value is inside the output range
    bool in_output_range(float x) const;

    // get input range
    void get_intput_range(float& input_min, float& input_max) const;

    // prints the lookup table using the logger
    void print_to_log() const;

  private:
    void linear_interpolate(float x, int& idx_a, int& idx_b, float& weight_a, float& weight_b) const;
    // same, with the input already measured from input_min
    void linear_interpolate_from_min(float dx, int& idx_a, int& idx_b, float& weight_a, float& weight_b) const;

  private:
    float input_min = 0.0f;
    float input_max = 0.0f;
    float one_over_input_range = 1.0f;
    float period_in = 0.0f;
    float period_out = 0.0f;
    // precomputed in set_period() for evaluate_periodic()
    int64_t period_in_counts = 0;
    int64_t input_min_counts = 0;
    float one_over_period_in = 0.0f;
    std::vector<float> lookup_table;

    friend bool save_lut_to_file(const LookupTable& lut, const char* filename);
    friend bool load_lut_from_file(LookupTable& lut, const char* filename);
};

//*** FUNCTION ***********************************************************************************/

void build_linear_lut(LookupTable& lut, float in_min, float in_max, float out_min, float out_max);
bool save_lut_to_file(const LookupTable& lut, const char* filename);
bool load_lut_from_file(LookupTable& lut, const char* filename);

