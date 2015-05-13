/*
 * File : timestwo.c
 * Abstract:
 *       An example C-file S-function for multiplying an input by 2,
 *       y  = 2*u
 *
 * Real-Time Workshop note:
 *   This file can be used as is (noninlined) with the Real-Time Workshop
 *   C rapid prototyping targets, or it can be inlined using the Target 
 *   Language Compiler technology and used with any target. See 
 *     matlabroot/toolbox/simulink/blocks/tlc_c/timestwo.tlc   
 *   the TLC code to inline the S-function.
 *
 * Copyright 1990-2013 The MathWorks, Inc.
 */


#define S_FUNCTION_NAME  Get_Data_From_Motive_API_v2
#define S_FUNCTION_LEVEL 2

#include "simstruc.h" //for s-function building

//#include "mex.h" //for mexing

//#include <stdio.h> //for printf function
#include <windows.h> //for sleep function
//#include <math.h> //for isnan function

#include "NPTrackingTools.h" //for motive API

/*================*
 * Build checking *
 *================*/


/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *   Setup sizes of the various vectors.
 */
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 0);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return; /* Parameter mismatch will be reported by Simulink */
    }

    if (!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, 1);
    ssSetInputPortDataType(S, 0, DYNAMICALLY_TYPED );
    ssSetInputPortDirectFeedThrough(S, 0, 1);

    if (!ssSetNumOutputPorts(S,8)) return; // insert here to change number of output ports
    
    ssSetOutputPortWidth(S, 0, 1);
    ssSetOutputPortDataType(S, 0, DYNAMICALLY_TYPED );
    
    ssSetOutputPortWidth(S, 1, 1);
    ssSetOutputPortDataType(S, 1, DYNAMICALLY_TYPED );
    
    ssSetOutputPortWidth(S, 2, 1);
    ssSetOutputPortDataType(S, 2, DYNAMICALLY_TYPED );
    
    ssSetOutputPortWidth(S, 3, 1);
    ssSetOutputPortDataType(S, 3, DYNAMICALLY_TYPED );
    
    ssSetOutputPortWidth(S, 4, 1);
    ssSetOutputPortDataType(S, 4, DYNAMICALLY_TYPED );
    
    ssSetOutputPortWidth(S, 5, 1);
    ssSetOutputPortDataType(S, 5, DYNAMICALLY_TYPED );
    
    ssSetOutputPortWidth(S, 6, 1);
    ssSetOutputPortDataType(S, 6, DYNAMICALLY_TYPED );
    
    ssSetOutputPortWidth(S, 7, 1);
    ssSetOutputPortDataType(S, 7, DYNAMICALLY_TYPED );
    
    ssSetNumSampleTimes(S, 1);

    /* specify the sim state compliance to be same as a built-in block */
    ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);

    
    ssSetOptions(S,
                 SS_OPTION_CALL_TERMINATE_ON_EXIT |
                 SS_OPTION_EXCEPTION_FREE_CODE);

//                removed SS_OPTION_USE_TLC_WITH_ACCELERATOR 
//                removed SS_OPTION_WORKS_WITH_CODE_REUSE
    
    // set inner value for old_timestamp to messure delta time btween ithreations
    ssSetNumDWork(S, 1);
    ssSetDWorkWidth(S, 0, 1);
    
    ssSetDWorkDataType(S, 0, SS_DOUBLE);
    
}


/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    Specifiy that we inherit our sample time from the driving block.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S); 
}



#define MDL_START
void mdlStart(SimStruct *S)
{
    int ret1;
    const char project_file_name[130] = "c:\\Users\\sasson\\onedrive\\work control lab aero summer 2014 technion\\C_code\\Motive-S-function\\MOTIVE15.ttp";
    double *TimeStamp_old = (double*) ssGetDWork(S,0);
    
    TimeStamp_old[0]=0;
    
    ret1=TT_Initialize();
    
    Sleep(100);
    
    ret1=TT_LoadProject(project_file_name);
    
    Sleep(1000);
}


/* Function: mdlOutputs =======================================================
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);
    real_T *X         = ssGetOutputPortRealSignal(S,0);
    real_T *Y         = ssGetOutputPortRealSignal(S,1);
    real_T *Z         = ssGetOutputPortRealSignal(S,2);
    real_T *Yaw       = ssGetOutputPortRealSignal(S,3);
    real_T *Pitch     = ssGetOutputPortRealSignal(S,4);
    real_T *Roll      = ssGetOutputPortRealSignal(S,5);
    real_T *TimeStamp = ssGetOutputPortRealSignal(S,6);
    real_T *DeltaTime = ssGetOutputPortRealSignal(S,7);
    double *TimeStamp_old = (double*) ssGetDWork(S,0);
    
    float x1=999, y1=999, z1=999, qx1=999, qy1=999, qz1=999, qw1=999, yaw1=999, pitch1=999, roll1=999;
    double          TimeStamp1=0;
    double          DeltaTime1=0;
    
    int_T           width = ssGetOutputPortWidth(S,0);
    int             ret2;
    int             ID_num;
    
    ID_num=(int) *uPtrs[0]; //obtain input    
    
    ret2=TT_UpdateSingleFrame();
    
    TT_TrackableLocation(ID_num, &x1, &y1, &z1, &qx1, &qy1, &qz1, &qw1, &yaw1, &pitch1, &roll1);
    
    TimeStamp1=TT_FrameTimeStamp();
    
    DeltaTime1=TimeStamp1-TimeStamp_old[0];
    
    TimeStamp_old[0]=TimeStamp1;
    
    *X=(real_T) x1;
    *Y=(real_T) y1;
    *Z=(real_T) z1;
    *Yaw=(real_T) yaw1;
    *Pitch=(real_T) pitch1;
    *Roll=(real_T) roll1;
    *TimeStamp=(real_T) TimeStamp1;
    *DeltaTime=(real_T) DeltaTime1;
    
    
    Sleep(100);
 
}


/* Function: mdlTerminate =====================================================
 * Abstract:
 *    No termination needed, but we are required to have this routine.
 */
static void mdlTerminate(SimStruct *S)
{
    int_T             ret3;
            
//    ret3=TT_FinalCleanup(); //create error
    ret3=TT_Shutdown();
    
}



#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
