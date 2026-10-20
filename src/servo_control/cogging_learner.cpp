#include <cmath>
#include <algorithm>
#include "cogging_learner.h"
#include "utilities/math_constants.h"

// Respuesta de cada armonico h = 1..8 del ciclo electrico: G = (cambio del error plegado [um]) /
// (cambio de la tabla [grados el.]), en la convencion de numpy.fft.rfft, a 2 mm/s. Medidas por el
// metodo de la secante en mediciones/2026-10-20/cogging_153911_dos_tablas (campo "G" del json).
// {0, 0} = no medido (armonicos impares 3, 5, 7: el error casi no tiene); esos no se corrigen.
// Los armonicos 1 y 2 (20 y 40 Hz) responden con ~180 grados: por eso copiar el patron del PI empeoraba.
static const float G_RESPONSE[2][8][2] = {
  {{-0.10987f, -0.18450f}, {-0.38355f, -0.21930f}, {0.0f, 0.0f}, {0.45491f, 0.02540f}, {0.0f, 0.0f}, {-0.47737f, -0.67642f}, {0.0f, 0.0f}, {1.55821f, 1.57119f}},   // ida
  {{-0.12927f, 0.06388f}, {-0.28522f, 0.02128f}, {0.0f, 0.0f}, {0.41736f, -0.02107f}, {0.0f, 0.0f}, {-0.31865f, 0.30916f}, {0.0f, 0.0f}, {1.87295f, -0.77941f}},   // vuelta
};
static constexpr float MAX_TABLE_DEG = 20.0f;      // mismo tope que tools/cogging_learn.py
static constexpr uint32_t MIN_SAMPLES = 2000;      // por sentido

void CoggingLearner::begin(float field_at_zero_rad, float field_rad_per_mm, float feed_mm_s) {
  active = false;
  field_at_zero = field_at_zero_rad;
  field_per_mm = field_rad_per_mm;
  feed = feed_mm_s;
  reset();
}

void CoggingLearner::reset() {
  active = false;
  for (int d = 0; d < 2; d++)
    for (int i = 0; i < N; i++) {
      sum[d][i] = 0.0f;
      cnt[d][i] = 0;
    }
  sq_sum = 0.0f;
  sq_cnt = 0;
}

void CoggingLearner::add(float target_mm, float pos_mm, float vel_mm_s) {
  // crucero: velocidad del perfil a menos del 2 % de la pedida (como closed_loop_test.py)
  if (fabsf(fabsf(vel_mm_s) - feed) >= 0.02f * feed)
    return;
  const int d = vel_mm_s > 0.0f ? 0 : 1;
  const float err_um = 1000.0f * (target_mm - pos_mm);
  float ph = (field_at_zero + pos_mm * field_per_mm) * (1.0f / Constants::TWO_PI_F);
  ph -= floorf(ph);
  const int b = int(floorf(ph * N + 0.5f)) % N;   // intervalos centrados en i/N, como la tabla
  sum[d][b] += err_um;
  cnt[d][b]++;
  sq_sum += err_um * err_um;
  sq_cnt++;
}

bool CoggingLearner::pattern(int dir, float e_um[N]) const {
  uint32_t total = 0;
  float mean = 0.0f;
  for (int i = 0; i < N; i++) {
    total += cnt[dir][i];
    e_um[i] = cnt[dir][i] ? sum[dir][i] / float(cnt[dir][i]) : 0.0f;
    mean += e_um[i];
  }
  mean /= float(N);
  for (int i = 0; i < N; i++)
    e_um[i] -= mean;
  return total >= MIN_SAMPLES;
}

float CoggingLearner::rms_um() const {
  return sq_cnt ? sqrtf(sq_sum / float(sq_cnt)) : 0.0f;
}

void CoggingLearner::newton_step(float table_deg[N], float table_dir_deg[N], const float e_um[2][N], float gain) {
  // tabla efectiva de cada sentido: T_ida = tabla + tabla_sentido, T_vuelta = tabla - tabla_sentido
  float T[2][N];
  for (int i = 0; i < N; i++) {
    T[0][i] = table_deg[i] + table_dir_deg[i];
    T[1][i] = table_deg[i] - table_dir_deg[i];
  }
  for (int d = 0; d < 2; d++) {
    for (int h = 1; h <= 8; h++) {
      const float gr = G_RESPONSE[d][h - 1][0], gi = G_RESPONSE[d][h - 1][1];
      const float g2 = gr * gr + gi * gi;
      if (g2 < 1e-12f)
        continue;
      // E = sum e_i exp(-j 2 pi h i / N)   (rfft)
      float er = 0.0f, ei = 0.0f;
      for (int i = 0; i < N; i++) {
        const float a = Constants::TWO_PI_F * float(h * i) / float(N);
        er += e_um[d][i] * cosf(a);
        ei -= e_um[d][i] * sinf(a);
      }
      // delta = -gain * E / G
      const float dr = -gain * (er * gr + ei * gi) / g2;
      const float di = -gain * (ei * gr - er * gi) / g2;
      // irfft: el armonico h aporta (2/N) Re(delta exp(j 2 pi h i / N))
      for (int i = 0; i < N; i++) {
        const float a = Constants::TWO_PI_F * float(h * i) / float(N);
        T[d][i] += (2.0f / float(N)) * (dr * cosf(a) - di * sinf(a));
      }
    }
  }
  float peak = 0.0f;
  for (int i = 0; i < N; i++) {
    table_deg[i] = 0.5f * (T[0][i] + T[1][i]);
    table_dir_deg[i] = 0.5f * (T[0][i] - T[1][i]);
    peak = std::max(peak, fabsf(table_deg[i]) + fabsf(table_dir_deg[i]));
  }
  if (peak > MAX_TABLE_DEG) {
    for (int i = 0; i < N; i++) {
      table_deg[i] *= MAX_TABLE_DEG / peak;
      table_dir_deg[i] *= MAX_TABLE_DEG / peak;
    }
  }
}
