#define S_FUNCTION_NAME SIL
#define S_FUNCTION_LEVEL 2

#include <simstruc.h>
#include <ctime>
#include "path_follower_base.h"
#include "path_follower_example.h"
#include <iostream>

#define NUM_PARAMS (3)
#define IS_PARAM_DOUBLE(pVal) (mxIsNumeric(pVal) && !mxIsLogical(pVal) &&\
!mxIsEmpty(pVal) && !mxIsSparse(pVal) && !mxIsComplex(pVal) && mxIsDouble(pVal))

#define CHI_INFTY 0
#define K_PATH 1
#define K_ORBIT 2

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
    ssSetInputPortWidth(S, 0, 30);
    ssSetInputPortDirectFeedThrough(S, 0, 1); // we will use the input in the output step

    /******************************/
    /* Configure the output ports */
    /******************************/
    if (!ssSetNumOutputPorts(S, 1)) return;
    if(!ssSetOutputPortVectorDimension(S, 0, 1)) return;
    ssSetOutputPortWidth(S, 0, 3); // set the output to be a dynamically sized vector
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
    path_follower_base* pFollower = new path_follower_example;
    ssGetPWork(S)[0] = (void *) pFollower;

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
    path_follower_base* pFollower = (path_follower_base*) ssGetPWork(S)[0];

    /**************/
    /* Grab Input */
    /**************/

    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0); // this returns a pointer to the data on the input port;

    struct path_follower_base::input_s input;

    float flag_f = *uPtrs[0];
    if(flag_f == 1)
        input.flag = true;
    else if(flag_f == 2)
        input.flag = false;
    input.Va_d = *uPtrs[1];
    input.r_path[0] = *uPtrs[2];
    input.r_path[1] = *uPtrs[3];
    input.r_path[2] = *uPtrs[4];
    input.q_path[0] = *uPtrs[5];
    input.q_path[1] = *uPtrs[6];
    input.q_path[2] = *uPtrs[7];
    input.c_orbit[0] = *uPtrs[8];
    input.c_orbit[1] = *uPtrs[9];
    input.c_orbit[2] = *uPtrs[10];
    input.rho_orbit = *uPtrs[11];
    input.lam_orbit = *uPtrs[12];
    input.pn = *uPtrs[13];
    input.pe = *uPtrs[14];
//    input.h = *uPtrs[15];
//    input.Va = *uPtrs[16];
//    input.phi = *uPtrs[19];
//    input.theta = *uPtrs[20];
    input.chi = *uPtrs[21];
//    input.r = *uPtrs[24];

    /***************/
    /* Grab Params */
    /***************/

    struct path_follower_base::params_s params;

    params.chi_infty = (float) mxGetScalar( ssGetSFcnParam(S, CHI_INFTY));
    params.k_path = (float) mxGetScalar( ssGetSFcnParam(S, K_PATH));
    params.k_orbit = (float) mxGetScalar( ssGetSFcnParam(S, K_ORBIT));

    //input.Ts = (const float) mxGetScalar( ssGetSFcnParam(S, TS));

    /***********************/
    /* Receive from Output */
    /***********************/

    struct path_follower_base::output_s output;
    pFollower->follow(params, input, output);

    /********************************************/
    /* Pack Received message into Output Vector */
    /********************************************/
    real_T *out = ssGetOutputPortRealSignal(S,0);

    out[0] = output.Va_c;
    out[1] = output.h_c;
    out[2] = output.chi_c;
}


static void mdlTerminate(SimStruct *S)
{
    path_follower_base* pFollower = (path_follower_base*) ssGetPWork(S)[0];
    delete pFollower;
    mexUnlock();
}

/* Required S-function trailer */

#ifdef	MATLAB_MEX_FILE
#include "simulink.c"    
#else
#include "cg_sfun.h"     
#endif

/* eof: sdotproduct.c */
