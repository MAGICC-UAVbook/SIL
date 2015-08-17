/**
 * @file estimator_base.h
 *
 * Base class definition for autopilot estimator in chapter 8 of UAVbook, see http://uavbook.byu.edu/doku.php
 *
 * @author Gary Ellingson <gary.ellingson@byu.edu>
 */

#ifndef ESTIMATOR_BASE_H
#define ESTIMATOR_BASE_H

//#include <nuttx/config.h>
//#include <unistd.h>
//#include <stdlib.h>
//#include <stdio.h>
//#include <stdbool.h>
//#include <poll.h>
//#include <drivers/drv_hrt.h>
//#include <fcntl.h>
//#include <nuttx/sched.h>
//#include <sys/prctl.h>
//#include <termios.h>
//#include <errno.h>
//#include <limits.h>
//#include <math.h>
//#include <float.h>

//#include <uORB/uORB.h>
//#include <uORB/topics/parameter_update.h>
//#include <uORB/topics/sensor_combined.h>
//#include <uORB/topics/airspeed.h>
//#include <uORB/topics/vehicle_gps_position.h>
//#include <uORB/topics/vehicle_state.h>

//#include <systemlib/param/param.h>
//#include <systemlib/err.h>
//#include <systemlib/perf_counter.h>
//#include <systemlib/systemlib.h>
#include <lib/mathlib/mathlib.h>

#define EARTH_RADIUS 6378145.0f

class estimator_base
{
public:
    estimator_base();
    //void spin();

    struct input_s{
        float gyro_x;
        float gyro_y;
        float gyro_z;
        float accel_x;
        float accel_y;
        float accel_z;
        float static_pres;
        float diff_pres;
        bool gps_new;
        float gps_n;
        float gps_e;
        float gps_h;
        float gps_Vg;
        float gps_course;
        float Ts;
    };

    struct output_s{
        float pn;
        float pe;
        float h;
        float Va;
        float alpha;
        float beta;
        float phi;
        float theta;
        float psi;
        float chi;
        float p;
        float q;
        float r;
        float Vg;
        float wn;
        float we;
    };

    struct params_s{
        float gravity;
        float rho;
        float sigma_accel;
        float sigma_gyro;
        float sigma_n_gps;
        float sigma_e_gps;
        float sigma_Vg_gps;
        float sigma_course_gps;
    };

    virtual void estimate(const struct params_s &params, const struct input_s &input, struct output_s &output) = 0;

//private:
//    int _params_sub;            /**< parameter updates subscription */
//    int _sensor_combined_sub;
////    int _airspeed_sub;
//    int _gps_sub;
//    struct pollfd fds[1];
//    int poll_error_counter;

//    orb_advert_t _vehicle_state_pub; /**< vehicle state publication */

//    struct {
//        param_t gravity;
//        param_t rho;
//        param_t sigma_accel;
//        param_t sigma_n_gps;
//        param_t sigma_e_gps;
//        param_t sigma_Vg_gps;
//        param_t sigma_course_gps;
//    } _params_handles; /**< handles for interesting parameters */

//    struct sensor_combined_s        _sensor_combined;
////    struct airspeed_s               _airspeed;
////    bool                            _airspeed_new;
//    struct vehicle_gps_position_s   _gps;
//    bool                            _gps_new;
//    bool                            _gps_init;
//    int32_t                         _init_lat;	/**< Initial latitude in 1E-7 degrees */
//    int32_t                         _init_lon;	/**< Initial longitude in 1E-7 degrees */
//    int32_t                         _init_alt;	/**< Initial altitude in 1E-3 meters (millimeters) above MSL  */
////    struct baro_report              _baro;
//    struct vehicle_state_s          _vehicle_state;
//    struct params_s                 _params;

//    /**
//    * Update our local parameter cache.
//    */
//    int parameters_update();

//    /**
//    * Check for parameter update and handle it.
//    */
//    void parameter_update_poll();

//    /**
//    * Check for changes in sensor combined.
//    */
//    void sensor_combined_poll();

//    /**
//    * Check for changes in airspeed.
//    */
//    //void airspeed_poll();

//    /**
//    * Check for changes in gps.
//    */
//    void gps_poll();

//    /**
//    * Publish the outputs
//    */
//    void vehicle_state_publish(struct output_s &output);
};

#endif // ESTIMATOR_BASE_H
