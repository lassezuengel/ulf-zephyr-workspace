#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

#include <math.h>
#include <errno.h>

#include <app/lib/localization/least_squares.h>

LOG_MODULE_REGISTER(localization_least_squares);

static float __attribute__((unused)) distance(const struct node_position* a, const struct node_position* b) {
  float dx = a->position.x - b->position.x;
  float dy = a->position.y - b->position.y;
  float dz = a->position.z - b->position.z;
  return sqrtf(dx*dx + dy*dy + dz*dz);
}

static void vector_subtract(const struct node_position* a, const struct node_position* b, float out[3]) {
  out[0] = a->position.x - b->position.x;
  out[1] = a->position.y - b->position.y;
  out[2] = a->position.z - b->position.z;
}

static float vector_norm(const float v[3]) {
  return sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

static void vector_scale(const float v[3], float scale, float out[3]) {
  out[0] = v[0] * scale;
  out[1] = v[1] * scale;
  out[2] = v[2] * scale;
}

static float __attribute__((unused)) vector_dot(const float a[3], const float b[3]) {
  return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

static void matrix_add_outer_product(float mat[3][3], const float v[3]) {
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      mat[i][j] += v[i] * v[j];
}

static void matrix_add_vector_product(float out[3], const float v[3], float scale) {
  for (int i = 0; i < 3; ++i)
    out[i] += v[i] * scale;
}

// Solve 3x3 system using LDL' decomposition (simplified)
static int solve_ldlt(const float A[3][3], const float b[3], float x[3]) {
  float L[3][3] = {0}, D[3] = {0};
  float y[3] = {0}, z[3] = {0};

  // Decompose A = LDLᵀ
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < i; ++j) {
      float sum = A[i][j];
      for (int k = 0; k < j; ++k)
        sum -= L[i][k] * D[k] * L[j][k];
      L[i][j] = sum / D[j];
    }
    float sum = A[i][i];
    for (int k = 0; k < i; ++k)
      sum -= L[i][k] * L[i][k] * D[k];
    D[i] = sum;
    L[i][i] = 1.0f;
  }

  // Solve Ly = b
  for (int i = 0; i < 3; ++i) {
    y[i] = b[i];
    for (int j = 0; j < i; ++j)
      y[i] -= L[i][j] * y[j];
  }

  // Solve Dz = y
  for (int i = 0; i < 3; ++i)
    z[i] = y[i] / D[i];

  // Solve Lᵀx = z
  for (int i = 2; i >= 0; --i) {
    x[i] = z[i];
    for (int j = i + 1; j < 3; ++j)
      x[i] -= L[j][i] * x[j];
  }

  return 1;  // success
}

int cholesky_linear_localization(deca_short_addr_t target_address,
    const struct node_position *known_positions, size_t known_positions_length,
    const struct measurement *measurements, size_t measurement_count,
    struct node_position *estimate, uint8_t flags) {

    if(estimate == NULL || known_positions == NULL || measurements == NULL) {
        LOG_ERR("Invalid arguments");
        return -EINVAL;
    }

    struct node_position _estimate = {
        .addr = target_address,
        .position = {
            .x = 0.5f,
            .y = 0.5f,
            .z = 0.5f,
        }
    };

    for (uint8_t iter = 0; iter < 20; ++iter) {
        float JTJ[3][3] = {0};
        float JTr[3] = {0};

        for (size_t i = 0; i < known_positions_length; i++) {
            const struct node_position *pos = &known_positions[i];
            deca_short_addr_t addr = pos->addr;
            for (size_t m = 0; m < measurement_count; ++m) {
                const struct measurement *meas = &measurements[m];
                deca_short_addr_t initiator = meas->ranging_initiator_id;
                deca_short_addr_t responder = meas->ranging_responder_id;
                if (!((initiator == addr || responder == addr) &&
                      (initiator == target_address || responder == target_address))) {
                    continue;
                }

                float delta[3];
                vector_subtract(&_estimate, pos, delta);
                float dist = vector_norm(delta);

                if (dist < 1e-6f)
                    continue;

                float J[3];
                vector_scale(delta, 1.0f / dist, J);
                float residual = time_to_dist(meas->tof) - dist;

                matrix_add_outer_product(JTJ, J);
                matrix_add_vector_product(JTr, J, residual);
            }
        }

        // Regularization
        JTJ[0][0] += 0.1f;
        JTJ[1][1] += 0.1f;
        JTJ[2][2] += 0.1f;

        float delta[3];
        if (!solve_ldlt(JTJ, JTr, delta))
            break;

        _estimate.position.x += delta[0];
        _estimate.position.y += delta[1];
        _estimate.position.z += delta[2];

        /* Apply Z constraint if enabled */
        if ((flags & LS_FLAG_CONSTRAIN_Z_POSITIVE) && _estimate.position.z < 0.0f) {
            _estimate.position.z = 0.0f;
        }

        if (vector_norm(delta) < 0.001f)
            break;
    }

    *estimate = _estimate;

    return 0;
}
