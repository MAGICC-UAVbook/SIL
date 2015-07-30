#ifndef PATH_MANAGER_EXAMPLE_H
#define PATH_MANAGER_EXAMPLE_H

#include "path_manager_base.h"
#include <lib/mathlib/mathlib.h>

enum class fillet_state {
    Straight,
    Orbit
};

enum class dubin_state {
    First_Orbit_before_H1,
    Straight,
    Second_Orbit
};

class path_manager_example : public path_manager_base
{
public:
    path_manager_example();
private:
    virtual void manage(const struct params_s &params, const struct input_s &input, struct output_s &output);

    void manage_line(const struct params_s &params, const struct input_s &input, struct output_s &output);
    void manage_fillet(const struct params_s &params, const struct input_s &input, struct output_s &output);
    fillet_state fil_state;
    void manage_dubins(const struct params_s &params, const struct input_s &input, struct output_s &output);

    float dot(math::Vector<3>, math::Vector<3>);
};

#endif // PATH_MANAGER_EXAMPLE_H
