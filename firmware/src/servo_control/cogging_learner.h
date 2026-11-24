#pragma once

// Reaprendizaje de las tablas anti-cogging (M59) en el firmware, al final de cada calibracion (M56).
// Es la version embebida de tools/cogging_learn.py --g-from: el error de seguimiento se pliega sobre
// el ciclo electrico en cada sentido y se corrige armonico por armonico con un paso de Newton usando
// las respuestas G medidas en el aprendizaje del 20/10/2026 (dependen del motor y del lazo, no de la
// calibracion). Ver docs/CAMBIOS_CALIBRACION_2026-10-20.md.

#include <cstdint>

class CoggingLearner {
  public:
    static constexpr int N = 64;   // = ServoController::COGGING_TABLE_SIZE

    // Prepara una pasada: velocidad de crucero que se mide [mm/s] y fase del campo en la posicion 0.
    void begin(float field_at_zero_rad, float field_rad_per_mm, float feed_mm_s);
    void reset();

    // Desde el ISR del perfil (Core 0, 2 kHz): solo acumula mientras 'active' y en crucero.
    volatile bool active = false;
    void add(double target_mm, double pos_mm, float vel_mm_s);

    // Patron del error [um] sobre el ciclo electrico, sin la media; dir 0 = ida (+), 1 = vuelta (-).
    // Devuelve false si hubo pocas muestras en ese sentido.
    bool pattern(int dir, float e_um[N]) const;
    float rms_um() const;

    // Paso de Newton sobre las tablas [grados electricos], en el lugar.
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
