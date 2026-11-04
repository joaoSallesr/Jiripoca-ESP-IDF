#ifndef TEST_METRICS_H
#define TEST_METRICS_H

#include <math.h>
#include <stddef.h>
#include <stdio.h>

#include "unity.h"

typedef struct {
    size_t sample_count;
    double sum_error;
    double sum_squared_error;
    double max_absolute_error;
} test_error_metrics_t;

#define TEST_ERROR_METRICS_INIT {0, 0.0, 0.0, 0.0}

static inline void test_error_metrics_add(test_error_metrics_t *metrics, double actual, double expected) {
    double error = actual - expected;
    double absolute_error = fabs(error);

    metrics->sample_count++;
    metrics->sum_error += error;
    metrics->sum_squared_error += error * error;
    if (absolute_error > metrics->max_absolute_error) {
        metrics->max_absolute_error = absolute_error;
    }
}

static inline double test_error_metrics_bias(const test_error_metrics_t *metrics) {
    return metrics->sample_count == 0 ? NAN : metrics->sum_error / (double)metrics->sample_count;
}

static inline double test_error_metrics_rmse(const test_error_metrics_t *metrics) {
    return metrics->sample_count == 0 ? NAN : sqrt(metrics->sum_squared_error / (double)metrics->sample_count);
}

static inline double test_error_metrics_max_absolute_error(const test_error_metrics_t *metrics) {
    return metrics->sample_count == 0 ? NAN : metrics->max_absolute_error;
}

static inline void test_error_metrics_print(const char *label, const test_error_metrics_t *metrics) {
    printf("\n--- %s ---\n", label);
    printf("Samples:              %zu\n", metrics->sample_count);
    printf("Max Absolute Error:   %.6f\n", test_error_metrics_max_absolute_error(metrics));
    printf("RMSE:                 %.6f\n", test_error_metrics_rmse(metrics));
    printf("Bias:                 %.6f\n", test_error_metrics_bias(metrics));
}

#define TEST_ASSERT_ERROR_METRICS_WITHIN(metrics, max_abs_tol, rmse_tol, bias_tol) \
    do {                                                                          \
        TEST_ASSERT_TRUE((metrics)->sample_count > 0);                            \
        TEST_ASSERT_FLOAT_WITHIN((max_abs_tol), 0.0, test_error_metrics_max_absolute_error(metrics)); \
        TEST_ASSERT_FLOAT_WITHIN((rmse_tol), 0.0, test_error_metrics_rmse(metrics));                  \
        TEST_ASSERT_FLOAT_WITHIN((bias_tol), 0.0, test_error_metrics_bias(metrics));                  \
    } while (0)

#endif