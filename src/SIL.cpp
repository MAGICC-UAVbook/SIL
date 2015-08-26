#define S_FUNCTION_NAME SIL
#define S_FUNCTION_LEVEL 2

#include <simstruc.h>
#include <ctime>
#include "controller_base.h"
#include "controller_example.h"
#include <iostream>

#define NUM_PARAMS (28)
#define IS_PARAM_DOUBLE(pVal) (mxIsNumeric(pVal) && !mxIsLogical(pVal) &&\
!mxIsEmpty(pVal) && !mxIsSparse(pVal) && !mxIsComplex(pVal) && mxIsDouble(pVal))

#define ALT_HZ 0
#define ALT_TOZ 1
#define K_THETA_DC 2
#define TS 3
#define TAU 4
#define C_kp 5
#define C_kd 6
#define C_ki 7
#define R_kp 8
#define R_ki 9
#define R_kd 10
#define P_kp 11
#define P_ki 12
#define P_kd 13
#define A_p_kp 14
#define A_p_ki 15
#define A_p_kd 16
#define A_t_kp 17
#define A_t_ki 18
#define A_t_kd 19
#define A_kp 20
#define A_ki 21
#define A_kd 22
#define B_kp 23
#define B_ki 24
#define B_kd 25
#define TRIM_E 26
#define TRIM_T 27


#define MDL_CHECK_PARAMETERS
#if defined(MDL_CHECK_PARAMETERS)
/*
 * Check to make sure that each parameter is 1-d and positive
 */
static void mdlCheckParameters(SimStruct *S)
{

    for(int i=0;i<NUM_PARAMS;i++)
    {
        const mxArray *pVal0 = ssGetSFcnParam(S,i);

        if ( !IS_PARAM_DOUBLE(pVal0)) {
            ssSetErrorStatus(S, "Parameter to S-function must be a double scalar");
            return;
        }
    }
}
#endif

static void mdlInitializeSizes(SimStruct *S)
{
    // sets the number of parameters that the S-Function Has.
    // For now, we don't have any, but in the future, we might need some.
    // Look at the the template file for guidance on how to deal with parameters
    // passed to the function
    ssSetNumSFcnParams(S,  NUM_PARAMS);
    // check to make sure that the right number of parameters were passed
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return; /* Parameter mismatch reported by the Simulink engine*/
    } else {
        mdlCheckParameters(S);
        if (ssGetErrorStatus(S) != NULL) {
            return;
        }
    }

    /***************************************************/
    /* Take Care of the Continuous and Discrete States */
    /***************************************************/
    ssSetNumContStates(S, 0);   /* number of continuous states           */
    ssSetNumDiscStates(S, 0);   /* number of discrete states             */



    /*****************************/
    /* Configure the input ports */
    /*****************************/
    if(!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, 20);
    ssSetInputPortDirectFeedThrough(S, 0, 1); // we will use the input in the output step

    /******************************/
    /* Configure the output ports */
    /******************************/
    if (!ssSetNumOutputPorts(S, 1)) return;
    if(!ssSetOutputPortVectorDimension(S, 0, 1)) return;
    ssSetOutputPortWidth(S, 0, 16); // set the output to be a dynamically sized vector
    ssSetNumSampleTimes(S, 1);

    /* specify the sim state compliance to be same as a built-in block */
    //ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);

    /**************************/
    /* Configure work vectors */
    /**************************/
    ssSetNumDWork(S,1);
    ssSetDWorkDataType(S,0,SS_POINTER);
    ssSetDWorkWidth(S,0,1);

    // see MATLABROOT/toolbox/simulink/simdemos/simfeatures/src/sfun_counter_cpp.cpp for an example
    ssSetNumPWork(S,1);

    //ssSetNumDWork(         S, 1);   /* number of DWork Vectors (persistent memory) */
    //ssSetNumRWork(         S, 0);   /* number of real work vector elements   */
    //ssSetNumIWork(         S, 0);   /* number of integer work vector elements*/
    //ssSetNumPWork(         S, 1);   /* number of pointer work vector elements*/
    //ssSetNumModes(         S, 0);   /* number of mode work vector elements   */
    //ssSetNumNonsampledZCs( S, 0);   /* number of nonsampled zero crossings   */

    // ssSetDworkWidth(S, 0, 256);
    // ssSetDworkDataType(S, 0, SS_DOUBLE);
    // ssSetDworkName(S, 0, "payload_data");

}

#define MDL_START  /* Change to #undef to remove function */
#if defined(MDL_START)
static void mdlStart(SimStruct *S)
{
    mexLock();
    controller_base* pController = new controller_example;
    ssGetPWork(S)[0] = (void *) pController;

    //pController->time = std::clock();
}
#endif /*  MDL_START */

static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, 0.01);//INHERITED_SAMPLE_TIME);//mxGetScalar(ssGetSFcnParam(S, 0)));
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S);
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    UNUSED_ARG(tid);
    /**************/
    /* Get Controller out of PWork */
    /**************/
    controller_base* pController = (controller_base*) ssGetPWork(S)[0];

    /**************/
    /* Grab Input */
    /**************/

    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0); // this returns a pointer to the data on the input port;

    struct controller_base::input_s input;

    input.h = *uPtrs[2];
    input.va = *uPtrs[3];
    input.phi = *uPtrs[6];
    input.theta = *uPtrs[7];
    input.chi = *uPtrs[8];
    input.p = *uPtrs[9];
    input.q = *uPtrs[10];
    input.r = *uPtrs[11];
    input.Va_c = *uPtrs[16];
    input.h_c = *uPtrs[17];
    input.chi_c = *uPtrs[18];

    /***************/
    /* Grab Params */
    /***************/

    struct controller_base::params_s params;

    params.alt_hz = (float) mxGetScalar( ssGetSFcnParam(S, ALT_HZ));
    params.alt_toz = mxGetScalar( ssGetSFcnParam(S, ALT_TOZ));
    params.tau = mxGetScalar(ssGetSFcnParam(S, TAU));
    params.c_kp = mxGetScalar(ssGetSFcnParam(S, C_kp));
    params.c_kd = mxGetScalar(ssGetSFcnParam(S, C_kd));
    params.c_ki = mxGetScalar(ssGetSFcnParam(S, C_ki));
    params.r_kp = mxGetScalar(ssGetSFcnParam(S, R_kp));
    params.r_kd = mxGetScalar(ssGetSFcnParam(S, R_kd));
    params.r_ki = mxGetScalar(ssGetSFcnParam(S, R_ki));
    params.p_kp = mxGetScalar(ssGetSFcnParam(S, P_kp));
    params.p_kd = mxGetScalar(ssGetSFcnParam(S, P_kd));
    params.p_ki = mxGetScalar(ssGetSFcnParam(S, P_ki));
    params.a_p_kp = mxGetScalar(ssGetSFcnParam(S, A_p_kp));
    params.a_p_kd = mxGetScalar(ssGetSFcnParam(S, A_p_kd));
    params.a_p_ki = mxGetScalar(ssGetSFcnParam(S, A_p_ki));
    params.a_t_kp = mxGetScalar(ssGetSFcnParam(S, A_t_kp));
    params.a_t_kd = mxGetScalar(ssGetSFcnParam(S, A_t_kd));
    params.a_t_ki = mxGetScalar(ssGetSFcnParam(S, A_t_ki));
    params.a_kp = mxGetScalar(ssGetSFcnParam(S, A_kp));
    params.a_kd = mxGetScalar(ssGetSFcnParam(S, A_kd));
    params.a_ki = mxGetScalar(ssGetSFcnParam(S, A_ki));
    params.b_kp = mxGetScalar(ssGetSFcnParam(S, B_kp));
    params.b_kd = mxGetScalar(ssGetSFcnParam(S, B_kd));
    params.b_ki = mxGetScalar(ssGetSFcnParam(S, B_ki));
    params.trim_e = mxGetScalar(ssGetSFcnParam(S, TRIM_E));
    params.trim_t = mxGetScalar(ssGetSFcnParam(S, TRIM_T));
    params.max_e = 0.67;//10; // 35*pi/180
    params.max_a = 0.523; // 30*pi/180
    params.max_r = 0.523; // 30*pi/180
    params.max_t = 1;

    //std::clock_t now = std::clock();
    input.Ts = (const float) mxGetScalar( ssGetSFcnParam(S, TS));//(now - pController->time)/1000000; //because this will be an input on the autopilot
    //pController->time = now;

    /***********************/
    /* Receive from Output */
    /***********************/

    struct controller_base::output_s output;
    pController->control(params, input, output);

    /********************************************/
    /* Pack Received message into Output Vector */
    /********************************************/
    real_T *out = ssGetOutputPortRealSignal(S,0);
    out[0] = output.delta_e;
    out[1] = output.delta_a;
    out[2] = output.delta_r;
    out[3] = output.delta_t;

    //commands for ploting
    out[4] = 0; // pn
    out[5] = 0; // pe
    out[6] = input.h_c; // h
    out[7] = input.Va_c; // Va
    out[8] = 0; // alpha
    out[9] = 0; // beta
    out[10] = output.phi_c; // phi_c
    out[11] = output.theta_c*mxGetScalar(ssGetSFcnParam(S, K_THETA_DC)); // theta;
    out[12] = input.chi_c;
    out[13] = 0; // p
    out[14] = 0; // q
    out[15] = 0; // r
}


static void mdlTerminate(SimStruct *S)
{
    controller_base* pController = (controller_base*) ssGetPWork(S)[0];
    delete pController;
    mexUnlock();
}

/* Required S-function trailer */

#ifdef	MATLAB_MEX_FILE
#include "simulink.c"    
#else
#include "cg_sfun.h"     
#endif

/* eof: sdotproduct.c */
