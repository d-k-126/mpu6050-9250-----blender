#ifndef _KALMAN_H
#define _KALMAN_H

#include <Arduino.h>

class Kalman {
private:
  uint8_t state_dim; 
  float* x; 
  float* p; 
  float* k; 
  float* q; 
  float* r; 
  bool initialized;

public:
  Kalman(uint8_t dim = 4) {
    state_dim = dim;
    initialized = false;
    
    // Cấp phát bộ nhớ
    x = new float[state_dim];
    p = new float[state_dim * state_dim];
    k = new float[state_dim * state_dim];
    q = new float[state_dim * state_dim];
    r = new float[state_dim * state_dim];
    
    // Khởi tạo mặc định
    for (uint8_t i = 0; i < state_dim; i++) {
      x[i] = 0.0;
      for (uint8_t j = 0; j < state_dim; j++) {
        q[i * state_dim + j] = (i == j) ? 0.001 : 0.0;
        r[i * state_dim + j] = (i == j) ? 0.03 : 0.0;
        p[i * state_dim + j] = (i == j) ? 1.0 : 0.0;
      }
    }
  }

  ~Kalman() {
    delete[] x;
    delete[] p;
    delete[] k;
    delete[] q;
    delete[] r;
  }

  void setParameters(float processNoise, float measurementNoise) {
    for (uint8_t i = 0; i < state_dim; i++) {
      for (uint8_t j = 0; j < state_dim; j++) {
        q[i * state_dim + j] = (i == j) ? processNoise : 0.0;
        r[i * state_dim + j] = (i == j) ? measurementNoise : 0.0;
      }
    }
  }

  void reset() {
    for (uint8_t i = 0; i < state_dim; i++) {
      x[i] = 0.0;
      for (uint8_t j = 0; j < state_dim; j++) {
        p[i * state_dim + j] = (i == j) ? 1.0 : 0.0;
      }
    }
    initialized = false;
  }

  void updateEstimate(float* measurement, float* output) {
    if (!initialized) {
      for (uint8_t i = 0; i < state_dim; i++) {
        x[i] = measurement[i];
      }
      initialized = true;
      memcpy(output, x, state_dim * sizeof(float));
      return;
    }

    // Bước dự đoán
    float p_temp[state_dim * state_dim];
    matrixAdd(p_temp, p, q, state_dim, state_dim);

    // Bước cập nhật
    float denominator[state_dim * state_dim];
    matrixAdd(denominator, p_temp, r, state_dim, state_dim);

    float inv_denominator[state_dim * state_dim];
    matrixInverse(inv_denominator, denominator, state_dim);

    matrixMultiply(k, p_temp, inv_denominator, state_dim, state_dim, state_dim);

    float innovation[state_dim];
    for (uint8_t i = 0; i < state_dim; i++) {
      innovation[i] = measurement[i] - x[i];
    }

    float correction[state_dim];
    matrixMultiply(correction, k, innovation, state_dim, state_dim, 1);

    for (uint8_t i = 0; i < state_dim; i++) {
      x[i] += correction[i];
    }

    float identity[state_dim * state_dim];
    for (uint8_t i = 0; i < state_dim; i++) {
      for (uint8_t j = 0; j < state_dim; j++) {
        identity[i * state_dim + j] = (i == j) ? 1.0 : 0.0;
      }
    }

    float k_p[state_dim * state_dim];
    matrixMultiply(k_p, k, p_temp, state_dim, state_dim, state_dim);

    float p_new[state_dim * state_dim];
    matrixSubtract(p_new, identity, k_p, state_dim, state_dim);
    matrixMultiply(p, p_new, p_temp, state_dim, state_dim, state_dim);

    memcpy(output, x, state_dim * sizeof(float));
  }

  void getEstimate(float* output) {
    memcpy(output, x, state_dim * sizeof(float));
  }

private:
  void matrixAdd(float* result, float* a, float* b, uint8_t rows, uint8_t cols) {
    for (uint8_t i = 0; i < rows * cols; i++) {
      result[i] = a[i] + b[i];
    }
  }

  void matrixMultiply(float* result, float* a, float* b, uint8_t r1, uint8_t c1, uint8_t c2) {
    for (uint8_t i = 0; i < r1; i++) {
      for (uint8_t j = 0; j < c2; j++) {
        result[i * c2 + j] = 0;
        for (uint8_t k = 0; k < c1; k++) {
          result[i * c2 + j] += a[i * c1 + k] * b[k * c2 + j];
        }
      }
    }
  }

  void matrixSubtract(float* result, float* a, float* b, uint8_t rows, uint8_t cols) {
    for (uint8_t i = 0; i < rows * cols; i++) {
      result[i] = a[i] - b[i];
    }
  }

  float matrixInverse(float* result, float* a, uint8_t n) {
    float temp[n * 2 * n];
    for (uint8_t i = 0; i < n; i++) {
      for (uint8_t j = 0; j < n; j++) {
        temp[i * 2 * n + j] = a[i * n + j];
        temp[i * 2 * n + j + n] = (i == j) ? 1.0 : 0.0;
      }
    }

    for (uint8_t i = 0; i < n; i++) {
      float pivot = temp[i * 2 * n + i];
      if (abs(pivot) < 0.0001) return false;
      for (uint8_t j = 0; j < 2 * n; j++) {
        temp[i * 2 * n + j] /= pivot;
      }
      for (uint8_t k = 0; k < n; k++) {
        if (k != i) {
          float factor = temp[k * 2 * n + i];
          for (uint8_t j = 0; j < 2 * n; j++) {
            temp[k * 2 * n + j] -= factor * temp[i * 2 * n + j];
          }
        }
      }
    }

    for (uint8_t i = 0; i < n; i++) {
      for (uint8_t j = 0; j < n; j++) {
        result[i * n + j] = temp[i * 2 * n + j + n];
      }
    }
    return true;
  }
};

#endif // _KALMAN_H
