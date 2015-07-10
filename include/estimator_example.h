#ifndef ESTIMATOR_EXAMPLE_H
#define ESTIMATOR_EXAMPLE_H

#include "estimator_base.h"
#include <lib/mathlib/mathlib.h>

#define N 10 // number of prediction steps

class estimator_example : public estimator_base
{
public:
    estimator_example();

private:
    virtual void estimate(const params_s &params, const input_s &input, output_s &output);

    float alpha;
    float alpha1;
    float lpf_gyro_x;
    float lpf_gyro_y;
    float lpf_gyro_z;
    float lpf_static;
    float lpf_diff;
    float lpf_accel_x;
    float lpf_accel_y;
    float lpf_accel_z;
    math::Vector<2> xhat_a;
    math::Matrix<2,2> P_a;
    math::Vector<7> xhat_p;
    math::Matrix<7,7> P_p;
    float gps_n_old;
    float gps_e_old;
    float gps_Vg_old;
    float gps_course_old;
};

#endif // ESTIMATOR_EXAMPLE_H
