#include "estimator_example.h"
#include <iostream>

estimator_example::estimator_example()
{
    alpha = 0;
    alpha1 = 0;
    lpf_gyro_x = 0;
    lpf_gyro_y = 0;
    lpf_gyro_z = 0;
    lpf_static = 0;
    lpf_diff = 0;
    lpf_accel_x = 0;
    lpf_accel_y = 0;
    lpf_accel_z = 0;
    xhat_a.zero();
    P_a.identity();
    P_a *= powf(math::radians(20.0f),2);
    xhat_p.zero();
    xhat_p(2) = 10.5;
    P_p.identity();
    P_p(0,0) = 3;
    P_p(1,1) = 3;
    P_p(2,2) = 1;
    P_p(3,3) = math::radians(5.0f);
    P_p(4,4) = 4;
    P_p(5,5) = 4;
    P_p(6,6) = math::radians(5.0f);
    gps_n_old = -9999;
    gps_e_old = -9999;
    gps_Vg_old = -9999;
    gps_course_old = -9999;
}

void estimator_example::estimate(const params_s &params, const input_s &input, output_s &output)
{
    if(alpha == 0)
    {
        float lpf_a = 50;
        float lpf_a1 = 50;
        alpha = exp(-lpf_a*input.Ts);
        alpha1 = exp(-lpf_a1*input.Ts);

        lpf_static = params.rho*params.gravity*100;
        lpf_diff = 1/2 * params.rho*11*11;
    }

    // low pass filter gyros to estimate angular rates
    lpf_gyro_x = alpha*lpf_gyro_x + (1-alpha)*input.gyro_x;
    lpf_gyro_y = alpha*lpf_gyro_y + (1-alpha)*input.gyro_y;
    lpf_gyro_z = alpha*lpf_gyro_z + (1-alpha)*input.gyro_z;

    float phat = lpf_gyro_x;
    float qhat = lpf_gyro_y;
    float rhat = lpf_gyro_z;

    // low pass filter static pressure sensor and invert to esimate altitude
    lpf_static = alpha1*lpf_static + (1-alpha)*input.static_pres;
    float hhat = lpf_static/params.rho/params.gravity;

    // low pass filter diff pressure sensor and invert to extimate Va
    lpf_diff = alpha1*lpf_diff + (1-alpha)*input.diff_pres;
    float Vahat = sqrt(2/params.rho*lpf_diff);

    // low pass filter accelerometers
//    lpf_accel_x = alpha*lpf_accel_x + (1-alpha)*input.accel_x;
//    lpf_accel_y = alpha*lpf_accel_y + (1-alpha)*input.accel_y;
//    lpf_accel_z = alpha*lpf_accel_z + (1-alpha)*input.accel_z;

    // inplement continuous-discrete EKF to estimate roll and pitch angles
    math::Matrix<2,2> Q_a;
    Q_a.zero();
    Q_a(0,0) = 0.0000001;
    Q_a(1,1) = 0.0000000001;
    math::Matrix<2,2> R_accel;
    R_accel.zero();
    R_accel(0,0) = powf(params.sigma_accel,2);
    R_accel(1,1) = powf(params.sigma_accel,2);
//    R_accel(2,2) = powf(params.sigma_accel,2);

    // prediction step
    float cp; // cos(phi)
    float sp; // sin(phi)
    float tt; // tan(thata)
    float ct; // cos(thata)
    for(int i=0;i<N;i++)
    {
        cp = cos(xhat_a(0)); // cos(phi)
        sp = sin(xhat_a(0)); // sin(phi)
        tt = tan(xhat_a(1)); // tan(thata)
        ct = cos(xhat_a(1)); // cos(thata)
        math::Vector<2> f_a;
        f_a(0) = phat + (qhat*sp + rhat*cp)*tt;
        f_a(1) = qhat*cp - rhat*sp;
        math::Matrix<2,2> A_a;
        A_a.zero();
        A_a(0,0) = (qhat*cp - rhat*sp)*tt;
        A_a(0,1) = (qhat*sp + rhat*cp)/ct/ct;
        A_a(1,0) = -qhat*sp - rhat*cp;
//        math::Matrix<2,3> G_a;
//        G_a.zero();
//        G_a(0,0) = 1;
//        G_a(0,1) = sp*tt;
//        G_a(0,2) = cp*tt;
//        G_a(1,1) = cp;
//        G_a(1,2) = -sp;

        xhat_a += f_a *(input.Ts/N/10);
        P_a += (A_a*P_a + P_a*A_a.transposed() + Q_a)*(input.Ts/N/10);
    }
    // measurement updates
    cp = cos(xhat_a(0));
    sp = sin(xhat_a(0));
    ct = cos(xhat_a(1));
    float st = sin(xhat_a(1)); // sin(theta)
    math::Matrix<2,2> I;
    I.identity();

    // x and y-axix accelerometers
    math::Vector<2> h_a;
    h_a(0) = qhat*Vahat*st + params.gravity*st;
    h_a(1) = rhat*Vahat*ct - phat*Vahat*st - params.gravity*ct*sp;
//    h_a(2) = -qhat*Vahat*ct - params.gravity*ct*cp;
    math::Matrix<2,2> C_a;  // this should be a vector but its a matrix to get the library to work
    C_a.zero();
    C_a(0,1) = qhat*Vahat*ct + params.gravity*ct;
    C_a(1,0) = -params.gravity*cp*ct;
    C_a(1,1) = -rhat*Vahat*st - phat*Vahat*ct + params.gravity*st*sp;
//    C_a(2,0) = params.gravity*sp*ct;
//    C_a(2,1) = (qhat*Vahat + params.gravity*cp)*st;
    math::Matrix<2,2> L_a;  // this should be a vector but its a matrix to get the library to work
    L_a = (P_a*C_a.transposed()) * (R_accel + C_a*P_a*C_a.transposed()).inversed();
    math::Vector<2> y_a;
    y_a(0) = input.accel_x;
    y_a(1) = input.accel_y;
//    y_a(2) = input.accel_z;
    xhat_a += L_a *(y_a - h_a);
    P_a = (I - L_a*C_a)*P_a;

//    // x-axis accelerometer
//    float h_a = qhat*Vahat*st + params.gravity*st;
//    math::Matrix<2,2> C_a;  // this should be a vector but its a matrix to get the library to work
//    C_a.zero();
//    C_a(0,1) = qhat*Vahat*ct + params.gravity*ct;
//    math::Matrix<2,2> L_a;  // this should be a vector but its a matrix to get the library to work
//    L_a = (P_a*C_a.transposed()) * (R_accel + C_a*P_a*C_a.transposed()).inversed();
//    math::Vector<2> K_a;
//    K_a(0) = L_a(0,0);
//    K_a(1) = L_a(1,0);
//    xhat_a += K_a *(input.accel_x - h_a);
//    P_a = (I - L_a*C_a)*P_a;

//    // y-axis accelerometer
//    h_a = rhat*Vahat*ct - phat*Vahat*st - params.gravity*ct*sp;
//    C_a.zero();
//    C_a(0,0) = -params.gravity*cp*ct;
//    C_a(0,1) = -rhat*Vahat*st - phat*Vahat*ct + params.gravity*st*sp;
//    L_a = (P_a*C_a.transposed()) * (R_accel + C_a*P_a*C_a.transposed()).inversed();
//    K_a(0) = L_a(0,0);
//    K_a(1) = L_a(1,0);
//    xhat_a += K_a *(input.accel_y - h_a);
//    P_a = (I - L_a*C_a)*P_a;

    // z-axis accelerometer
    float fh_a = -qhat*Vahat*ct - params.gravity*ct*cp;
    C_a.zero();
    C_a(0,0) = params.gravity*sp*ct;
    C_a(0,1) = (qhat*Vahat + params.gravity*cp)*st;
    L_a = (P_a*C_a.transposed()) * (R_accel + C_a*P_a*C_a.transposed()).inversed();
    math::Vector<2> K_a;
    K_a(0) = L_a(0,0);
    K_a(1) = L_a(1,0);
    xhat_a += K_a *(input.accel_z - fh_a);
    P_a = (I - L_a*C_a)*P_a;

    if(xhat_a(0) > math::radians(90.0) || xhat_a(0) < math::radians(-90.0))
        xhat_a(0) = 0;
    if(xhat_a(1) > math::radians(80.0) || xhat_a(1) < math::radians(-80.0))
        xhat_a(1) = 0;
    float phihat = xhat_a(0);
    float thetahat = xhat_a(1);

    // implement continous-discrete EKF to estimate pn, pe, chi, Vg
    math::Matrix<7,7> Q_p;
    Q_p.identity();
    Q_p *= 0.0001f;
    Q_p(3,3) = 0.00000001f;
    math::Matrix<7,7> R_p;
    R_p.zero();
    R_p(0,0) = powf(params.sigma_n_gps,2);
    R_p(1,1) = powf(params.sigma_e_gps,2);
    R_p(2,2) = powf(params.sigma_Vg_gps,2);
    R_p(3,3) = powf(params.sigma_course_gps,2);
    R_p(4,4) = 0.001;
    R_p(5,5) = 0.001;

    // prediction step
    float psidot, tmp, Vgdot;
    for(int i=0;i<N;i++)
    {
        psidot = (qhat*sin(phihat) + rhat*cos(phihat))/cos(thetahat);
        tmp = -psidot*Vahat*(xhat_p(4)*cos(xhat_p(6)) + xhat_p(5)*sin(xhat_p(6)))/xhat_p(2);
        Vgdot = ((Vahat*cos(xhat_p(6)) + xhat_p(4))*(-psidot*Vahat*sin(xhat_p(6))) + (Vahat*sin(xhat_p(6)) + xhat_p(5))*(psidot*Vahat*cos(xhat_p(6))))/xhat_p(2);

        math::Vector<7> f_p;
        f_p.zero();
        f_p(0) = xhat_p(2)*cos(xhat_p(3));
        f_p(1) = xhat_p(2)*sin(xhat_p(3));
        f_p(2) = Vgdot;
        f_p(3) = params.gravity/xhat_p(2)*tan(phihat)*cos(xhat_p(3) - xhat_p(6));
        f_p(6) = psidot;

        math::Matrix<7,7> A_p;
        A_p.zero();
        A_p(0,2) = cos(xhat_p(3));
        A_p(0,3) = -xhat_p(2)*sin(xhat_p(3));
        A_p(1,2) = sin(xhat_p(3));
        A_p(1,3) = xhat_p(2)*cos(xhat_p(3));
        A_p(2,2) = -Vgdot/xhat_p(2);
        A_p(2,4) = -psidot*Vahat*sin(xhat_p(6))/xhat_p(2);
        A_p(2,5) = psidot*Vahat*cos(xhat_p(6))/xhat_p(2);
        A_p(2,6) = tmp;
        A_p(3,2) = -params.gravity/pow(xhat_p(2),2)*tan(phihat)*cos(xhat_p(3) - xhat_p(6));
        A_p(3,3) = -params.gravity/xhat_p(2)*tan(phihat)*sin(xhat_p(3) - xhat_p(6));
        A_p(3,6) = params.gravity/xhat_p(2)*tan(phihat)*sin(xhat_p(3) - xhat_p(6));

        xhat_p += f_p *(input.Ts/N/5);
        P_p += (A_p*P_p + P_p*A_p.transposed() + Q_p)*(input.Ts/N/5);
    }
    std::cout << "2" << std::endl;
    // measurement updates
    if(input.gps_n != gps_n_old ||
            input.gps_e != gps_e_old ||
            input.gps_Vg != gps_Vg_old ||
            input.gps_course != gps_course_old)
    {
        // all
        math::Vector<7> h_p;
        h_p(0) = xhat_p(0);
        h_p(1) = xhat_p(1);
        h_p(2) = xhat_p(2);
        h_p(3) = xhat_p(3);
        h_p(4) = Vahat*cos(xhat_p(6)) + xhat_p(4) - xhat_p(2)*cos(xhat_p(3));  // pseudo measurement
        h_p(5) = Vahat*sin(xhat_p(6)) + xhat_p(5) - xhat_p(2)*sin(xhat_p(3));
        //h_p(6) = 1;
        //std::cout << h_p(4) << " !!" << std::endl;
        math::Matrix<7,7> I_p;
        I_p.identity();
        math::Matrix<7,7> C_p;
        C_p.identity();
        C_p(4,2) = -cos(xhat_p(3));
        C_p(4,3) = xhat_p(2)*sin(xhat_p(3));
        C_p(4,6) = -Vahat*sin(xhat_p(6));
        C_p(5,2) = -sin(xhat_p(3));
        C_p(5,3) = -xhat_p(2)*cos(xhat_p(3));
        C_p(5,6) = Vahat*cos(xhat_p(6));
        C_p(6,6) = 0.05;
        math::Matrix<7,7> L_p;
        L_p = (P_p*C_p.transposed()) * (R_p + C_p*P_p*C_p.transposed()).inversed();
        P_p = (I_p - L_p*C_p)*P_p;
        math::Vector<7> y;
        y.zero();
        y(0) = input.gps_n;
        y(1) = input.gps_e;
        y(2) = input.gps_Vg;
        y(3) = input.gps_course;
        xhat_p = xhat_p + L_p*(y - h_p);

        // gps North position
//        float h_p = xhat_p(0);
//        math::Matrix<7,7> I_p;
//        I_p.identity();
//        math::Matrix<7,7> C_p;
//        C_p.zero();
//        C_p(0,0) = 1;
//        math::Matrix<7,7> L_p;
//        math::Matrix<7,7> R_tmp;
//        R_tmp.identity();
//        R_tmp *= powf(params.sigma_n_gps,2);
//        math::Matrix<1,1> denom;
//        denom(0,0) = (R_tmp + C_p*P_p*C_p.transposed())(0,0);
//        L_p = (P_p*C_p.transposed());
//        L_p(0,0) = L_p(0,0)/denom(0,0);
//        L_p(1,0) = L_p(1,0)/denom(0,0);
//        L_p(2,0) = L_p(2,0)/denom(0,0);
//        L_p(3,0) = L_p(3,0)/denom(0,0);
//        L_p(4,0) = L_p(4,0)/denom(0,0);
//        L_p(5,0) = L_p(5,0)/denom(0,0);
//        L_p(6,0) = L_p(6,0)/denom(0,0);
//        P_p = (I_p - L_p*C_p)*P_p;
//        math::Vector<7> K_p;
//        K_p(0) = L_p(0,0);
//        K_p(1) = L_p(1,0);
//        K_p(2) = L_p(2,0);
//        K_p(3) = L_p(3,0);
//        K_p(4) = L_p(4,0);
//        K_p(5) = L_p(5,0);
//        K_p(6) = L_p(6,0);
//        xhat_p = xhat_p + K_p*(input.gps_n - h_p);

//        // gps East position
//        h_p = xhat_p(1);
//        I_p.identity();
//        C_p.zero();
//        C_p(0,1) = 1;
//        R_tmp.identity();
//        R_tmp *= powf(params.sigma_e_gps,2);
//        denom(0,0) = (R_tmp + C_p*P_p*C_p.transposed())(0,0);
//        L_p = (P_p*C_p.transposed());
//        L_p(0,0) = L_p(0,0)/denom(0,0);
//        L_p(1,0) = L_p(1,0)/denom(0,0);
//        L_p(2,0) = L_p(2,0)/denom(0,0);
//        L_p(3,0) = L_p(3,0)/denom(0,0);
//        L_p(4,0) = L_p(4,0)/denom(0,0);
//        L_p(5,0) = L_p(5,0)/denom(0,0);
//        L_p(6,0) = L_p(6,0)/denom(0,0);
//        P_p = (I_p - L_p*C_p)*P_p;
//        K_p(0) = L_p(0,0);
//        K_p(1) = L_p(1,0);
//        K_p(2) = L_p(2,0);
//        K_p(3) = L_p(3,0);
//        K_p(4) = L_p(4,0);
//        K_p(5) = L_p(5,0);
//        K_p(6) = L_p(6,0);
//        xhat_p = xhat_p + K_p*(input.gps_e - h_p);

//        // gps ground speed
//        h_p = xhat_p(2);
//        I_p.identity();
//        C_p.zero();
//        C_p(0,2) = 1;
//        R_tmp.identity();
//        R_tmp *= powf(params.sigma_Vg_gps,2);
//        //L_p = (P_p*C_p.transposed()) * (R_tmp + C_p*P_p*C_p.transposed()).inversed();
//        denom(0,0) = (R_tmp + C_p*P_p*C_p.transposed())(0,0);
//        L_p = (P_p*C_p.transposed());
//        L_p(0,0) = L_p(0,0)/denom(0,0);
//        L_p(1,0) = L_p(1,0)/denom(0,0);
//        L_p(2,0) = L_p(2,0)/denom(0,0);
//        L_p(3,0) = L_p(3,0)/denom(0,0);
//        L_p(4,0) = L_p(4,0)/denom(0,0);
//        L_p(5,0) = L_p(5,0)/denom(0,0);
//        L_p(6,0) = L_p(6,0)/denom(0,0);
//        P_p = (I_p - L_p*C_p)*P_p;
//        K_p(0) = L_p(0,0);
//        K_p(1) = L_p(1,0);
//        K_p(2) = L_p(2,0);
//        K_p(3) = L_p(3,0);
//        K_p(4) = L_p(4,0);
//        K_p(5) = L_p(5,0);
//        K_p(6) = L_p(6,0);
//        xhat_p = xhat_p + K_p*(input.gps_e - h_p);

//        // gps course
//        //wrap course measurement
//        float temp = input.gps_course;
//        //while(temp - xhat_p(3) > 3.1415) temp = temp - 2*3.1415;
//        //while(temp - xhat_p(3) < -3.1415) temp = temp + 2*3.1415;
//        h_p = xhat_p(3);
//        I_p.identity();
//        C_p.zero();
//        C_p(0,3) = 1;
//        R_tmp.identity();
//        R_tmp *= powf(params.sigma_course_gps,2);
//        //L_p = (P_p*C_p.transposed()) * (R_tmp + C_p*P_p*C_p.transposed()).inversed();
//        denom(0,0) = (R_tmp + C_p*P_p*C_p.transposed())(0,0);
//        L_p = (P_p*C_p.transposed());
//        L_p(0,0) = L_p(0,0)/denom(0,0);
//        L_p(1,0) = L_p(1,0)/denom(0,0);
//        L_p(2,0) = L_p(2,0)/denom(0,0);
//        L_p(3,0) = L_p(3,0)/denom(0,0);
//        L_p(4,0) = L_p(4,0)/denom(0,0);
//        L_p(5,0) = L_p(5,0)/denom(0,0);
//        L_p(6,0) = L_p(6,0)/denom(0,0);
//        P_p = (I_p - L_p*C_p)*P_p;
//        K_p(0) = L_a(0,0);
//        K_p(1) = L_a(1,0);
//        K_p(2) = L_a(2,0);
//        K_p(3) = L_a(3,0);
//        K_p(4) = L_a(4,0);
//        K_p(5) = L_a(5,0);
//        K_p(6) = L_a(6,0);
//        xhat_p = xhat_p + K_p*(temp - h_p);

//        // pseudo measurement #1 y_1 = Va*cos(psi)+wn-Vg*cos(chi)
//        h_p = Vahat*cos(xhat_p(6)) + xhat_p(4) - xhat_p(2)*cos(xhat_p(3));  // pseudo measurement
//        I_p.identity();
//        C_p.zero();
//        C_p(0,2) = -cos(xhat_p(3));
//        C_p(0,3) = xhat_p(2)*sin(xhat_p(3));
//        C_p(0,4) = 1;
//        C_p(0,6) = -Vahat*sin(xhat_p(6));
//        R_tmp.identity();
//        R_tmp *= 0.000001;
//        L_p = (P_p*C_p.transposed()) * (R_tmp + C_p*P_p*C_p.transposed()).inversed();
//        P_p = (I_p - L_p*C_p)*P_p;
//        K_p(0) = L_a(0,0);
//        K_p(1) = L_a(1,0);
//        K_p(2) = L_a(2,0);
//        K_p(3) = L_a(3,0);
//        K_p(4) = L_a(4,0);
//        K_p(5) = L_a(5,0);
//        K_p(6) = L_a(6,0);
//        xhat_p = xhat_p + K_p*(0 - h_p);

//        // pseudo measurement #2 y_2 = Va*sin(psi) + we - Vg*sin(chi)
//        h_p = Vahat*sin(xhat_p(6))+xhat_p(5)-xhat_p(2)*sin(xhat_p(3));  // pseudo measurement
//        I_p.identity();
//        C_p.zero();
//        C_p(0,2) = -sin(xhat_p(3));
//        C_p(0,3) = -xhat_p(2)*cos(xhat_p(3));
//        C_p(0,5) = 1;
//        C_p(0,6) = Vahat*cos(xhat_p(6));
//        R_tmp.identity();
//        R_tmp *= 0.000001;
//        L_p = (P_p*C_p.transposed()) * (R_tmp + C_p*P_p*C_p.transposed()).inversed();
//        P_p = (I_p - L_p*C_p)*P_p;
//        K_p(0) = L_a(0,0);
//        K_p(1) = L_a(1,0);
//        K_p(2) = L_a(2,0);
//        K_p(3) = L_a(3,0);
//        K_p(4) = L_a(4,0);
//        K_p(5) = L_a(5,0);
//        K_p(6) = L_a(6,0);
//        xhat_p = xhat_p + K_p*(0 - h_p);

        gps_n_old      = input.gps_n;
        gps_e_old      = input.gps_e;
        gps_Vg_old     = input.gps_Vg;
        gps_course_old = input.gps_course;
    }

    float pnhat = xhat_p(0);
    float pehat = xhat_p(1);
    float Vghat = xhat_p(2);
    float chihat = xhat_p(3);
    float wnhat = xhat_p(4);
    float wehat = xhat_p(5);
    float psihat = xhat_p(6);

    output.pn = pnhat;
    output.pe = pehat;
    output.h = hhat;
    output.Va = Vahat;
    output.alpha = 0;
    output.beta = 0;
    output.phi = phihat;
    output.theta = thetahat;
    output.chi = chihat;
    output.p = phat;
    output.q = qhat;
    output.r = rhat;
    output.Vg = Vghat;
    output.wn = wnhat;
    output.we = wehat;
    output.psi = psihat;
}
