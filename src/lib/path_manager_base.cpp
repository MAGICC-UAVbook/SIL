#include "path_manager_base.h"

path_manager_base::path_manager_base()
{

    for(int i=0;i<SIZE_WAYPOINT_ARRAY;i++)
    {
        _waypoints[i].w[0] = 0.0f;
        _waypoints[i].w[1] = 0.0f;
        _waypoints[i].w[2] = 30.0f;
        _waypoints[i].chi_d = 0.0f;
        _waypoints[i].chi_valid = false;
        _waypoints[i].Va_d = 0.0f;
    }
    _num_waypoints = 0;
    _ptr_a = &_waypoints[0];

}

