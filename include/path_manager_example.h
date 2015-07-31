#ifndef PATH_MANAGER_EXAMPLE_H
#define PATH_MANAGER_EXAMPLE_H

#include "path_manager_base.h"
#include <lib/mathlib/mathlib.h>

#define M_PI_F 3.14159265358979323846f

enum class fillet_state {
    Straight,
    Orbit
};

enum class dubin_state {
    First,
    Before_H1,
    Before_H1_wrong_side,
    Straight,
    Before_H3,
    Before_H3_wrong_side
};

class path_manager_example : public path_manager_base
{
public:
    path_manager_example();
private:
    virtual void manage(const struct params_s &params, const struct input_s &input, struct output_s &output);

    void manage_line(const struct params_s &params, const struct input_s &input, struct output_s &output);
    void manage_fillet(const struct params_s &params, const struct input_s &input, struct output_s &output);
    fillet_state _fil_state;
    void manage_dubins(const struct params_s &params, const struct input_s &input, struct output_s &output);
    dubin_state _dub_state;
    struct dubinspath_s {
        math::Vector<3> ps;         /** the start position */
        float chis;                 /** the start course angle */
        math::Vector<3> pe;                   /** the end position */
        float chie;                 /** the end course angle */
        float R;                    /** turn radius */
        float L;                    /** length of the path */
        math::Vector<3> cs;         /** center of the start circle */
        int lams;                 /** direction of the start circle */
        math::Vector<3> ce;         /** center of the enc circle */
        int lame;                 /** direction of the end circle */
        math::Vector<3> w1;         /** vector defining half plane H1 */
        math::Vector<3> q1;         /** unit vector along striaght line path */
        math::Vector<3> w2;         /** vector defining half plane H2 */
        math::Vector<3> w3;         /** vector defining half plane H3 */
        math::Vector<3> q3;         /** unit vector defining direction of half plane H3 */
    };
    struct dubinspath_s _dubinspath;
    void dubinsParameters(const struct waypoint_s start_node, const struct waypoint_s end_node, float R);

    math::Matrix<3,3> rotz(float theta);
    float mo(float in);
    float dot(math::Vector<3>, math::Vector<3>);
};

#endif // PATH_MANAGER_EXAMPLE_H
