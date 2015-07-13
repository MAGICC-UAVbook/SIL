#define S_FUNCTION_NAME SIL
#define S_FUNCTION_LEVEL 2

#include <simstruc.h>
#include <ctime>
#include "estimator_base.h"
#include "estimator_example.h"
#include <iostream>

#define NUM_PARAMS (9)
#define IS_PARAM_DOUBLE(pVal) (mxIsNumeric(pVal) && !mxIsLogical(pVal) &&\
!mxIsEmpty(pVal) && !mxIsSparse(pVal) && !mxIsComplex(pVal) && mxIsDouble(pVal))

#define GRAVITY 0
#define RHO 1
#define SIGMA_ACCEL 2
#define SIGMA_GYRO 3
#define SIGMA_N_GPS 4
#define SIGMA_E_GPS 5
#define SIGMA_VG_GPS 6
#define SIGMA_COURSE_GPS 7
#define TS 8

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
    ssSetInputPortWidth(S, 0, 14);
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
    estimator_base* pEstimator = new estimator_example;
    ssGetPWork(S)[0] = (void *) pEstimator;

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
    estimator_base* pEstimator = (estimator_base*) ssGetPWork(S)[0];

    /**************/
    /* Grab Input */
    /**************/

    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0); // this returns a pointer to the data on the input port;

    struct estimator_base::input_s input;

    input.gyro_x = *uPtrs[0];
    input.gyro_y = *uPtrs[1];
    input.gyro_z = *uPtrs[2];
    input.accel_x = *uPtrs[3];
    input.accel_y = *uPtrs[4];
    input.accel_z = *uPtrs[5];
    input.static_pres = *uPtrs[6];
    input.diff_pres = *uPtrs[7];
    input.gps_n = *uPtrs[8];
    input.gps_e = *uPtrs[9];
    input.gps_h = *uPtrs[10];
    input.gps_Vg = *uPtrs[11];
    input.gps_course = *uPtrs[12];

    /***************/
    /* Grab Params */
    /***************/

    struct estimator_base::params_s params;

    params.gravity = (float) mxGetScalar( ssGetSFcnParam(S, GRAVITY));
    params.rho = (float) mxGetScalar( ssGetSFcnParam(S, RHO));
    params.sigma_accel = (float) mxGetScalar( ssGetSFcnParam(S, SIGMA_ACCEL));
    params.sigma_gyro = (float) mxGetScalar( ssGetSFcnParam(S, SIGMA_GYRO));
    params.sigma_n_gps = mxGetScalar( ssGetSFcnParam(S, SIGMA_N_GPS));
    params.sigma_e_gps = mxGetScalar( ssGetSFcnParam(S, SIGMA_E_GPS));
    params.sigma_Vg_gps = mxGetScalar( ssGetSFcnParam(S, SIGMA_VG_GPS));
    params.sigma_course_gps = mxGetScalar( ssGetSFcnParam(S, SIGMA_COURSE_GPS));

    input.Ts = (const float) mxGetScalar( ssGetSFcnParam(S, TS));

    /***********************/
    /* Receive from Output */
    /***********************/

    struct estimator_base::output_s output;
    pEstimator->estimate(params, input, output);

    /********************************************/
    /* Pack Received message into Output Vector */
    /********************************************/
    real_T *out = ssGetOutputPortRealSignal(S,0);
    //    std::cout << input.gps_course << std::endl;
    out[0] = output.pn;
    out[1] = output.pe;
    out[2] = output.h;
    out[3] = output.Va;
    out[4] = output.alpha;
    out[5] = output.beta;
    out[6] = output.phi;
    out[7] = output.theta;
    out[8] = output.chi;
    out[9] = output.p;
    out[10] = output.q;
    out[11] = output.r;
    out[12] = output.Vg;
    out[13] = output.wn;
    out[14] = output.we;
    out[15] = output.psi;
}


static void mdlTerminate(SimStruct *S)
{
    estimator_base* pEstimator = (estimator_base*) ssGetPWork(S)[0];
    delete pEstimator;
    mexUnlock();
}

/* Required S-function trailer */

#ifdef	MATLAB_MEX_FILE
#include "simulink.c"    
#else
#include "cg_sfun.h"     
#endif

/* eof: sdotproduct.c */
