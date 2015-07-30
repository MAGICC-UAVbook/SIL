#define S_FUNCTION_NAME SIL
#define S_FUNCTION_LEVEL 2

#include <simstruc.h>
#include <ctime>
#include "path_manager_base.h"
#include "path_manager_example.h"
#include <iostream>

#define NUM_PARAMS (1)
#define IS_PARAM_DOUBLE(pVal) (mxIsNumeric(pVal) && !mxIsLogical(pVal) &&\
!mxIsEmpty(pVal) && !mxIsSparse(pVal) && !mxIsComplex(pVal) && mxIsDouble(pVal))

#define R_MIN 0
//#define K_PATH 1
//#define K_ORBIT 2

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
    ssSetInputPortWidth(S, 0, 1 + 5*SIZE_WAYPOINT_ARRAY + 17);
    ssSetInputPortDirectFeedThrough(S, 0, 1); // we will use the input in the output step

    /******************************/
    /* Configure the output ports */
    /******************************/
    if (!ssSetNumOutputPorts(S, 1)) return;
    if(!ssSetOutputPortVectorDimension(S, 0, 1)) return;
    ssSetOutputPortWidth(S, 0, 30); // set the output to be a dynamically sized vector
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
    path_manager_base* pManager = new path_manager_example;
    ssGetPWork(S)[0] = (void *) pManager;

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
    path_manager_base* pManager = (path_manager_base*) ssGetPWork(S)[0];

    /**************/
    /* Grab Input */
    /**************/

    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0); // this returns a pointer to the data on the input port;

    struct path_manager_base::input_s input;

    int num_waypoints = (int)*uPtrs[0];
    if(num_waypoints > 0 &&
       (pManager->_waypoints[0].w[0] != *uPtrs[1] ||
        pManager->_waypoints[0].w[1] != *uPtrs[2] ||
        pManager->_waypoints[0].w[2] != *uPtrs[3]))
    {
        std::cout << "received new waypoints " << num_waypoints << std::endl;
        for(int i=0;i<num_waypoints;i++)
        {
            pManager->_waypoints[i].w[0] = *uPtrs[i*5 + 1];
            pManager->_waypoints[i].w[1] = *uPtrs[i*5 + 2];
            pManager->_waypoints[i].w[2] = *uPtrs[i*5 + 3];
            pManager->_waypoints[i].chi_d = *uPtrs[i*5 + 4];
            std::cout << pManager->_waypoints[i].chi_d << std::endl;
            if(*uPtrs[i*5 + 4] >= 2*3.14159 || *uPtrs[i*5 + 4] <= -2*3.14159)
                pManager->_waypoints[i].chi_valid = false;
            else
                pManager->_waypoints[i].chi_valid = true;
            pManager->_waypoints[i].Va_d = *uPtrs[i*5 + 5];
            pManager->_num_waypoints++;
        }
    }
    input.pn = *uPtrs[1 + 5*SIZE_WAYPOINT_ARRAY ];
    input.pe = *uPtrs[1 + 5*SIZE_WAYPOINT_ARRAY + 1];
    input.h = *uPtrs[1 + 5*SIZE_WAYPOINT_ARRAY + 2];
    input.chi = *uPtrs[1 + 5*SIZE_WAYPOINT_ARRAY + 8];

    /***************/
    /* Grab Params */
    /***************/

    struct path_manager_base::params_s params;

    params.R_min = (float) mxGetScalar( ssGetSFcnParam(S, R_MIN));
//    params.k_path = (float) mxGetScalar( ssGetSFcnParam(S, K_PATH));
//    params.k_orbit = (float) mxGetScalar( ssGetSFcnParam(S, K_ORBIT));

    /***********************/
    /* Receive from Output */
    /***********************/

    struct path_manager_base::output_s output;
    pManager->manage(params, input, output);

    /********************************************/
    /* Pack Received message into Output Vector */
    /********************************************/
    real_T *out = ssGetOutputPortRealSignal(S,0);

    bool flag;              /** Inicates strait line or orbital path (true is line, false is orbit) */
    float Va_d;             /** Desired airspeed (m/s) */
    float r[3];             /** Vector to origin of straight line path (m) */
    float q[3];             /** Unit vector, desired direction of travel for line path */
    float c[3];             /** Center of orbital path (m) */
    float rho;              /** Radius of orbital path (m) */
    int lambda;          /** Direction of orbital path (cw is 1, ccw is -1) */
    if(output.flag)
        out[0] = 1;
    else
        out[0] = 2;
    out[1] = output.Va_d;
    out[2] = output.r[0];
    out[3] = output.r[1];
    out[4] = output.r[2];
    out[5] = output.q[0];
    out[6] = output.q[1];
    out[7] = output.q[2];
    //std::cout << out[2] << " " << out[3] << " " << out[4] << " " << out[5] << " " << out[6] << " " << out[7] << std::endl;
    out[8] = output.c[0];
    out[9] = output.c[1];
    out[10] = output.c[2];
    out[11] = output.rho;
    out[12] = output.lambda;
    out[13] = input.pn;
    out[14] = input.pe;
    out[15] = input.h;
    out[21] = input.chi;
    for(int i=16;i<30;i++)
        out[i] = 0;
    out[21] = input.chi;
}


static void mdlTerminate(SimStruct *S)
{
    path_manager_base* pManager = (path_manager_base*) ssGetPWork(S)[0];
    delete pManager;
    mexUnlock();
}

/* Required S-function trailer */

#ifdef	MATLAB_MEX_FILE
#include "simulink.c"    
#else
#include "cg_sfun.h"     
#endif

/* eof: sdotproduct.c */
