#ifndef PATH_FOLLOWER_EXAMPLE_H
#define PATH_FOLLOWER_EXAMPLE_H

#include "path_follower_base.h"

#define M_PI_F 3.1415927f
class path_follower_example : public path_follower_base
{
public:
    path_follower_example();
private:
    virtual void follow(const struct params_s &params, const struct input_s &input, struct output_s &output);
};

#endif // PATH_FOLLOWER_EXAMPLE_H
