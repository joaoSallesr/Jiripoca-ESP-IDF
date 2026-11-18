#ifndef TEST_METRICS_VECT3_H
#define TEST_METRICS_VECT3_H

#include <math.h>
#include <stddef.h>
#include <stdio.h>

#include "test_metrics.h"
#include "unity.h"

typedef struct {
    test_error_metrics_t x;
    test_error_metrics_t y;
    test_error_metrics_t z;
} test_error_metrics_vect3_t;

#define TEST_ERROR_METRICS_VECT3_INIT { TEST_ERROR_METRICS_INIT, TEST_ERROR_METRICS_INIT, TEST_ERROR_METRICS_INIT }

static inline void test_error_metrics_vect3_add(test_error_metrics_vect3_t *metrics, float actual[3], float expected[3]) {
    test_error_metrics_add(&metrics->x, actual[0], expected[0]);
    test_error_metrics_add(&metrics->y, actual[1], expected[1]);
    test_error_metrics_add(&metrics->z, actual[2], expected[2]);
}

// Combined Vector RMSE across all 3 axes
static inline double test_error_metrics_vect3_rmse(const test_error_metrics_vect3_t *metrics) {
    double rmse_x = test_error_metrics_rmse(&metrics->x);
    double rmse_y = test_error_metrics_rmse(&metrics->y);
    double rmse_z = test_error_metrics_rmse(&metrics->z);

    if (isnan(rmse_x) || isnan(rmse_y) || isnan(rmse_z) ? 1 : 0) return NAN;

    return sqrt((rmse_x * rmse_x) + (rmse_y * rmse_y) + (rmse_z * rmse_z));
}

// Maximum absolute error across all 3 axes
static inline double test_error_metrics_vect3_max_absolute_error(const test_error_metrics_vect3_t *metrics) {
    double max_x = test_error_metrics_max_absolute_error(&metrics->x);
    double max_y = test_error_metrics_max_absolute_error(&metrics->y);
    double max_z = test_error_metrics_max_absolute_error(&metrics->z);

    double max_val = max_x;
    if (max_y > max_val) max_val = max_y;
    if (max_z > max_val) max_val = max_z;
    return max_val;
}

static inline void test_error_metrics_vect3_print(const test_error_metrics_vect3_t *metrics) {
    printf("\n=== Vector Metrics ===\n");
    test_error_metrics_print("X-Axis Error Summary", &metrics->x);
    test_error_metrics_print("Y-Axis Error Summary", &metrics->y);
    test_error_metrics_print("Z-Axis Error Summary", &metrics->z);
}

#define TEST_ASSERT_ERROR_METRICS_VECT3_WITHIN(metrics, max_abs_tol, rmse_tol, bias_tol) \
    do {                                                                            \
        TEST_ASSERT_FLOAT_WITHIN((max_abs_tol), 0.0, test_error_metrics_vect3_max_absolute_error(metrics)); \
        TEST_ASSERT_FLOAT_WITHIN((rmse_tol), 0.0, test_error_metrics_vect3_rmse(metrics));                  \
        TEST_ASSERT_FLOAT_WITHIN((bias_tol), 0.0, test_error_metrics_bias(&(metrics)->x));                  \
        TEST_ASSERT_FLOAT_WITHIN((bias_tol), 0.0, test_error_metrics_bias(&(metrics)->y));                  \
        TEST_ASSERT_FLOAT_WITHIN((bias_tol), 0.0, test_error_metrics_bias(&(metrics)->z));                  \
    } while (0)

#endif // TEST_METRICS_VECT3_H