//--------------------------------------------------------------------
//Copyright(c)2020,Andar Technologise Inc.
//All Rights Reserved
//Confidential Property of Andar Technologies Inc.
//
//Module Description: calculate phase and angle
//Created by :wuhao
//$Revision: 1.0
//$Data: 2020/11/9
//--------------------------------------------------------------------
//
//All include header files
#include "adt3102_type_define.h"
#include "adt3102.h"
#include "adt3102_adc.h"
#include "adt3102_dma.h"
#include "math.h"
#include "arm_math.h"
#include "adt3102_phase_cal.h"
#include "dsp_ctype_map.h"
#include "rfc_ctype_map.h"
#include "dma_ctype_map.h"

//------Global variable declaration

PHASE_TypeDef phaseCal(int16 iCh0,int16 qCh0, int16 iCh1, int16 qCh1, float phaseCor)
{
    PHASE_TypeDef phaseResult;
    float phaseCh0,phaseCh1;
    float phaseDiff;
    float angle;
    phaseCh0 = atan2f((float)qCh0,(float)iCh0);
    phaseCh1 = atan2f((float)qCh1,(float)iCh1);
    phaseDiff = phaseCh0 - phaseCh1 + phaseCor;
    while(phaseDiff>=PI)
    {
        phaseDiff = phaseDiff - 2*PI;
    }
    while(phaseDiff <= -PI)
    {
        phaseDiff = phaseDiff + 2*PI;
    }
    angle = phaseDiff/PI;
    angle = asinf(angle);
    angle = angle*180/PI;
    phaseResult.phaseCh0  = phaseCh0;
    phaseResult.phaseCh1  = phaseCh1;
    phaseResult.phaseDiff = phaseDiff;
    phaseResult.angle      = angle;
    return phaseResult;
}

float angleCorrect1(float angleIn)
{	
    static float angleDly1,angleDly2,angleCur;
    float d[3];
    angleDly2 = angleDly1;
    angleDly1 = angleCur;
    angleCur = angleIn;
    //Hamming distance of 3 points
    d[0]=fabsf(angleCur-angleDly1);
    d[1]=fabsf(angleDly1-angleDly2);
    d[2]=fabsf(angleCur-angleDly2);
    if((d[0]<=d[1]) && (d[0]<=d[2]))
    {
        return  (angleCur+angleDly1)/2;
    }
    else if((d[1]<=d[0]) && (d[1]<=d[2]))
    {
        return  (angleDly1+angleDly2)/2;
    }
    else
    {
        return  (angleCur+angleDly2)/2;        
    }
}



