/**
 * @file path_manager_base.h
 *
 * Base class definition for autopilot path follower in chapter 10 of UAVbook, see http://uavbook.byu.edu/doku.php
 *
 * @author Gary Ellingson <gary.ellingson@byu.edu>
 */

#ifndef PATH_MANAGER_BASE_H
#define PATH_MANAGER_BASE_H

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
//#include <uORB/topics/vehicle_state.h>
//#include <uORB/topics/new_waypoint.h>
//#include <uORB/topics/current_path.h>

//#include <systemlib/param/param.h>
//#include <systemlib/err.h>
//#include <systemlib/perf_counter.h>
//#include <systemlib/systemlib.h>
//#include <lib/mathlib/mathlib.h>
//#include <lib/geo/geo.h>

#define SIZE_WAYPOINT_ARRAY 100 //20

class path_manager_base
{
public:
    path_manager_base();
    float spin();

//protected:

    struct waypoint_s{
        float w[3];
        float chi_d;
        bool chi_valid;
        float Va_d;
    };

    struct waypoint_s _waypoints[SIZE_WAYPOINT_ARRAY];
    int _num_waypoints;
    struct waypoint_s* _ptr_a;

    struct input_s{
        float pn;               /** position north */
        float pe;               /** position east */
        float h;                /** altitude */
        float chi;              /** course angle */
    };

    struct output_s{
        bool flag;              /** Inicates strait line or orbital path (true is line, false is orbit) */
        float Va_d;             /** Desired airspeed (m/s) */
        float r[3];             /** Vector to origin of straight line path (m) */
        float q[3];             /** Unit vector, desired direction of travel for line path */
        float c[3];             /** Center of orbital path (m) */
        float rho;              /** Radius of orbital path (m) */
        int lambda;          /** Direction of orbital path (cw is 1, ccw is -1) */
    };

    struct params_s {
        float R_min;
        float k_path;
        float k_orbit;
    };

    virtual void manage(const struct params_s &params, const struct input_s &input, struct output_s &output) = 0;

//private:
//    int _params_sub;            /**< parameter updates subscription */
//    int _vehicle_state_sub;     /**< vehicle state subscription */
//    int _new_waypoint_sub;      /**< new waypoint subscription */
//    struct pollfd fds[1];
//    int poll_error_counter;

//    orb_advert_t _current_path_pub; /**< controller commands publication */

//    struct {
//        param_t chi_infty;
//        param_t k_path;
//        param_t k_orbit;
//    } _params_handles; /**< handles for interesting parameters */

//    struct vehicle_state_s             _vehicle_state;     /**< vehicle state */
//    struct current_path_s              _current_path;      /**< current path */
//    struct params_s                    _params;            /**< params */

//    /**
//    * Update our local parameter cache.
//    */
//    int parameters_update();

//    /**
//    * Check for parameter update and handle it.
//    */
//    void parameter_update_poll();

//    /**
//    * Check for changes in vehicle state.
//    */
//    void vehicle_state_poll();

//    /**
//    * Check for new waypoints.
//    */
//    void new_waypoint_poll();

//    /**
//    * Publish the outputs
//    */
//    void current_path_publish(struct output_s &output);
};

#endif // PATH_MANAGER_BASE_H
