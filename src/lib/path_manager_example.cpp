#include "path_manager_example.h"
#include <iostream>

path_manager_example::path_manager_example() : path_manager_base()
{
    fil_state = fillet_state::Straight;
}

void path_manager_example::manage(const params_s &params, const input_s &input, output_s &output)
{
    if(_num_waypoints == 0)
    {
        output.flag = true;
        output.Va_d = 11;
        output.r[0] = input.pn;
        output.r[1] = input.pe;
        output.r[2] = -input.h;
        output.q[0] = cosf(input.chi);
        output.q[1] = sinf(input.chi);
        output.q[2] = 0.0f;
        output.c[0] = 0.0f;
        output.c[1] = 0.0f;
        output.c[2] = 0.0f;
        output.rho = 0;
        output.lambda = 0;
    } else {
        if(_ptr_a->chi_valid)
        {
            manage_dubins(params, input, output);
        } else {
            //manage_line(params, input, output);
            manage_fillet(params, input, output);
        }
    }
}

void path_manager_example::manage_line(const params_s &params, const input_s &input, output_s &output)
{
    math::Vector<3> p;
    p(0) = input.pn;
    p(1) = input.pe;
    p(2) = -input.h;

    waypoint_s* ptr_b;
    waypoint_s* ptr_c;
    if(_ptr_a == &_waypoints[_num_waypoints - 1])
    {
        ptr_b = &_waypoints[0];
        ptr_c = &_waypoints[1];
    } else if(_ptr_a == &_waypoints[_num_waypoints - 2])
    {
        ptr_b = &_waypoints[_num_waypoints - 1];
        ptr_c = &_waypoints[0];
    }
    else
    {
        ptr_b = _ptr_a + 1;
        ptr_c = ptr_b + 1;
    }

    math::Vector<3> w_im1;
    w_im1(0) = _ptr_a->w[0];
    w_im1(1) = _ptr_a->w[1];
    w_im1(2) = _ptr_a->w[2];
    math::Vector<3> w_i;
    w_i(0) = ptr_b->w[0];
    w_i(1) = ptr_b->w[1];
    w_i(2) = ptr_b->w[2];
    math::Vector<3> w_ip1;
    w_ip1(0) = ptr_c->w[0];
    w_ip1(1) = ptr_c->w[1];
    w_ip1(2) = ptr_c->w[2];
    //std::cout << w_ip1(0) << " " << w_ip1(1) << " " <<  w_ip1(2) << " " <<std::endl;

    output.flag = true;
    output.Va_d = _ptr_a->Va_d;
    output.r[0] = w_im1(0);
    output.r[1] = w_im1(1);
    output.r[2] = w_im1(2);
    math::Vector<3> q_im1 = (w_i - w_im1).normalized();
    math::Vector<3> q_i = (w_ip1 - w_i).normalized();
    output.q[0] = q_im1(0);
    output.q[1] = q_im1(1);
    output.q[2] = q_im1(2);
    output.c[0] = 1;
    output.c[1] = 1;
    output.c[2] = 1;
    output.rho = 1;
    output.lambda = 1;

    math::Vector<3> n_i = (q_im1 + q_i).normalized();
    if(dot((p - w_i),n_i) > 0.0f)
    {
        if(_ptr_a == &_waypoints[_num_waypoints - 1])
            _ptr_a = &_waypoints[0];
        else
            _ptr_a++;
    }

}

void path_manager_example::manage_fillet(const params_s &params, const input_s &input, output_s &output)
{
    math::Vector<3> p;
    p(0) = input.pn;
    p(1) = input.pe;
    p(2) = -input.h;

    waypoint_s* ptr_b;
    waypoint_s* ptr_c;
    if(_ptr_a == &_waypoints[_num_waypoints - 1])
    {
        ptr_b = &_waypoints[0];
        ptr_c = &_waypoints[1];
    } else if(_ptr_a == &_waypoints[_num_waypoints - 2])
    {
        ptr_b = &_waypoints[_num_waypoints - 1];
        ptr_c = &_waypoints[0];
    }
    else
    {
        ptr_b = _ptr_a + 1;
        ptr_c = ptr_b + 1;
    }

    math::Vector<3> w_im1;
    w_im1(0) = _ptr_a->w[0];
    w_im1(1) = _ptr_a->w[1];
    w_im1(2) = _ptr_a->w[2];
    math::Vector<3> w_i;
    w_i(0) = ptr_b->w[0];
    w_i(1) = ptr_b->w[1];
    w_i(2) = ptr_b->w[2];
    math::Vector<3> w_ip1;
    w_ip1(0) = ptr_c->w[0];
    w_ip1(1) = ptr_c->w[1];
    w_ip1(2) = ptr_c->w[2];

    float R = params.R_min;

    output.Va_d = _ptr_a->Va_d;
    output.r[0] = w_im1(0);
    output.r[1] = w_im1(1);
    output.r[2] = w_im1(2);
    math::Vector<3> q_im1 = (w_i - w_im1).normalized();
    math::Vector<3> q_i = (w_ip1 - w_i).normalized();
    float beta = acosf(dot(-q_im1, q_i));

    math::Vector<3> z;
    switch (fil_state) {
    case fillet_state::Straight:
        output.flag = true;
        output.q[0] = q_im1(0);
        output.q[1] = q_im1(1);
        output.q[2] = q_im1(2);
        output.c[0] = 1;
        output.c[1] = 1;
        output.c[2] = 1;
        output.rho = 1;
        output.lambda = 1;
        z = w_i - q_im1*(R/tanf(beta/2));
        if(dot((p-z),q_im1) > 0)
            fil_state = fillet_state::Orbit;
        break;
    case fillet_state::Orbit:
        output.flag = false;
        output.q[0] = q_i(0);
        output.q[1] = q_i(1);
        output.q[2] = q_i(2);
        math::Vector<3> c = w_i - (q_im1-q_i).normalized()*(R/sinf(beta/2));
        output.c[0] = c(0);
        output.c[1] = c(1);
        output.c[2] = c(2);
        output.rho = R;
        output.lambda = ((q_im1(0)*q_i(1) - q_im1(1)*q_i(0)) > 0 ? 1 : -1);
        z = w_i + q_i*(R/tanf(beta/2));
        if(dot((p-z),q_i) > 0)
        {
            if(_ptr_a == &_waypoints[_num_waypoints - 1])
                _ptr_a = &_waypoints[0];
            else
                _ptr_a++;
            fil_state = fillet_state::Straight;
        }
        break;
    }
}

void path_manager_example::manage_dubins(const params_s &params, const input_s &input, output_s &output)
{
    math::Vector<3> p;
    p(0) = input.pn;
    p(1) = input.pe;
    p(2) = -input.h;

    waypoint_s* ptr_b;
    waypoint_s* ptr_c;
    if(_ptr_a == &_waypoints[_num_waypoints - 1])
    {
        ptr_b = &_waypoints[0];
        ptr_c = &_waypoints[1];
    } else if(_ptr_a == &_waypoints[_num_waypoints - 2])
    {
        ptr_b = &_waypoints[_num_waypoints - 1];
        ptr_c = &_waypoints[0];
    }
    else
    {
        ptr_b = _ptr_a + 1;
        ptr_c = ptr_b + 1;
    }

}

float path_manager_example::dot(math::Vector<3> first, math::Vector<3> second)
{
    return first(0)*second(0) + first(1)*second(1) + first(2)*second(2);
}
