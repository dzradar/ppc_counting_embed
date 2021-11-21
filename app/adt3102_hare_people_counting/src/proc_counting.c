//--------------------------------------------------------------------
//Copyright(c)2020,Andar Technologise Inc.
//All Rights Reserved
//Confidential Property of Andar Technologies Inc.
//
//Module Description: People Counting for ADT3102 Hare DEV board
//
//Created by :WanzhiQiu
//$Revision: 0.1
//$Data: 2021/06/18
//--------------------------------------------------------------------

#include "stdio.h"
//#include "cfar.h"
#include "string.h"
//#include "uart.h"
//#include "hw_cfg.h"
//#include "Tx.h"
//#include "Timer.h"
#include "proc_counting.h"
//#include "phase_cal.h"
#include "math.h"
#include "stdint.h"
#include "arm_math.h"
//#include "define.h"
//#include "velo_fft_gest.h"
//#include "arm_const_structs.h"
//#include "angle_proc.h"
//#include "mled.h"
#include "stdlib.h"
#include "adt3102.h"
#include "adt3102_phase_cal.h"
#include "db_scan.h"
#include "adt3102_timer.h"
#include "adt3102_rx.h"
#include "adt3102_dsp.h"
#include "dsp_ctype_map.h"


extern uint32 g_absMergeArray[RANGE_MAX][CHIRP_NUM2];

OBJ_TypeDef objResult[OBJ_MAX];



void procCounting(float ThFac_rng, float ThFac_dop, uint32 thNoCfar)
{
    static uint16 s_resultN;

	static uint16 s_frameCnt=1;
 
	static uint16 s_prevFrameCnt=0;
		
    int16 frElaps = 0;
    
//    uint32 timePrint;

    frElaps = s_frameCnt - s_prevFrameCnt;
	
    if(frElaps < 0)
    {
        frElaps += MAX_FRAME_CNT;
    }
    
    if(frElaps <= NUM_FRAME_CLUSTER) //(frElaps <= NUM_FRAME_CLUSTER) //s_resultN < OBJ_MAX)
    {
        if(s_resultN < OBJ_MAX)
        {
//                timerOff(Timer1);
//                timerInit(Timer1,(float)100000,HIGH_CLOCK);
            
            s_resultN = detectPeaks(s_resultN, s_frameCnt, ThFac_rng, ThFac_dop,thNoCfar);
            
//                timePrint=(uint32)(100000-Timer1->VALUE/125);
//                printfDebug("Frame time %d \r\n", timePrint);
            
            #ifdef PRINT_DEBUG_INFO
                printfDebug("finish pk search,  frame cnt : %d, s_resultN : %d : \r\n", s_frameCnt, s_resultN);
            #endif
        }
        else
        {
            #ifdef PRINT_DEBUG_INFO
                printfDebug("enough pks - not search anymore, frElaps : %d \r\n",frElaps);
            #endif
        }
    }
    
    if(frElaps == NUM_FRAME_CLUSTER)
    {	
        if(s_resultN >= MIN_PK_FOR_DOING_CLUSTER)
        {			
            #ifdef PRINT_DEBUG_INFO
                printfDebug("clustering, s_resultN : %d \r\n",s_resultN);
            #endif
            
//            #ifdef TIMING_TEST
//                timerOff(Timer1);
//                timerInit(Timer1,(float)100000,HIGH_CLOCK);
//            #endif
            
            #ifdef DBSCAN_METHOD_RNG_ANG
                dbScan(objResult, s_resultN, DBSCAN_EPS_R, DBSCAN_EPS_D, DBSCAN_EPS_A, DBSCAN_MIN_PTS, euclideanDist);
            #else
                dbScan(objResult, s_resultN, DBSCAN_EPS_X, DBSCAN_EPS_Y, DBSCAN_EPS_D, DBSCAN_MIN_PTS, euclideanDist);
            #endif
            
//            #ifdef TIMING_TEST
//                timePrint=(uint32)(100000-Timer1->VALUE/125);
//                printfDebug("Frame time %d \r\n", timePrint);
//            #endif
            
            printPoints(objResult, s_resultN);
            
            
            
        }
        else
        {
            #ifdef  SYSTEM_USING_CLUSTER_SE
			        printf("s");              
		            printf("e");
            #endif
            #ifdef PRINT_DEBUG_INFO
                printfDebug("too few pks after time elaps, s_resultN :  %d \r\n",s_resultN);
            #endif
        }
        //end add
        s_resultN = 0;
        s_prevFrameCnt = s_frameCnt;
    }

    if(s_frameCnt == MAX_FRAME_CNT - 1 )
    {
        s_frameCnt = 0;
    }
    else
    {
        s_frameCnt += 1;
    }
}


uint16 detectPeaks(uint16 resultN, uint16 frameCnt, float ThFac_rng, float ThFac_dop, uint32 thNoCfar)
{
	uint16 y,x;
    int16 qPeak1,iPeak1,qPeak2,iPeak2, qPeak3,iPeak3,qPeak4,iPeak4;		
//	PHASE_TypeDef phaseResult; 
	
	uint16 rngIdx = 0;
	uint16 dopIdx = 0;
	//uint32 cfarInputDop[CHIRP_NUM2] = {0};
	//uint8 cfarDetectDop[CHIRP_NUM2] = {0};
	//float cfarThrDop[CHIRP_NUM2] = {0};
	//uint32 cfarInputRng[RANGE_MAX] = {0};    
	//uint8 cfarDetectRng[RANGE_MAX] = {0};
	//float cfarThrRng[RANGE_MAX] = {0};	
//    uint32 curTiaGain = TIA_GAIN_6DB;
//    uint32 curVgaGain = PGA_GAIN_44DB;
//    int32 powerThdAdj=-200;
//    float sigPower = 0;
    
    //powerThdAdj = -32+(curTiaGain*6+curVgaGain*2-50) - 2; 
    //OBJ_MAXPEAK_TypeDef obj;
    
	enum DetectionMethod detMethod = DETECT_METHOD_NOCFAR;
	
//    uint32 timePrint;
    
//    int32 accRealCh0[RANGE_MAX] = {0};
//    int32 accImagCh0[RANGE_MAX] = {0};
//    int32 accRealCh1[RANGE_MAX] = {0};
//    int32 accImagCh1[RANGE_MAX] = {0};
//    uint32 temp0 = 0;
//    uint32 temp1 = 0;
    
    uint8 numAngByFft = 0;
    float angByFft1 = 0;
    float angByFft2 = 0;
    float ang1 = 0;
    float ang2 = 0;
    float ang3 = 0;
    
    for(int16 i=0; i<RANGE_MAX; i++)
    {
      g_absMergeArray[i][CHIRP_NUM2/2] = 0;
    }
    
    if(detMethod == DETECT_METHOD_NOCFAR)
	{
		for(rngIdx = RANGE_MIN-1; rngIdx< RANGE_MAX; rngIdx++)
		{
			for(dopIdx = 0; dopIdx < CHIRP_NUM2; dopIdx++)
			{
				//if(g_absMergeArray[rngIdx][dopIdx] >= C_MIMOTHDABS_NOCFAR)
                if(g_absMergeArray[rngIdx][dopIdx]>=thNoCfar)
				{		
//	     //hack
//        detectMaxPeak(&obj);
//        rngIdx = obj.rngIdx;
//        dopIdx = obj.dopIdx;
        
					y = rngIdx; 
					x = dopIdx < CHIRP_NUM2/2 ? dopIdx + CHIRP_NUM2/2 : dopIdx - CHIRP_NUM2/2;
     			
        	
                    iPeak1   = (*( ((int16 *)DOP_FFT_CH0_ODD_ADDR)  + (y*CHIRP_NUM2+x)*2 ));
                    qPeak1   = (*( ((int16 *)DOP_FFT_CH0_ODD_ADDR)  + (y*CHIRP_NUM2+x)*2 + 1 ));																						
                    iPeak2   = (*( ((int16 *)DOP_FFT_CH1_ODD_ADDR)  + (y*CHIRP_NUM2+x)*2));
                    qPeak2   = (*( ((int16 *)DOP_FFT_CH1_ODD_ADDR)  + (y*CHIRP_NUM2+x)*2 + 1 ));
                    iPeak3   = (*( ((int16 *)DOP_FFT_CH0_EVEN_ADDR)  + (y*CHIRP_NUM2+x)*2 ));
                    qPeak3   = (*( ((int16 *)DOP_FFT_CH0_EVEN_ADDR)  + (y*CHIRP_NUM2+x)*2 + 1 ));																						
                    iPeak4   = (*( ((int16 *)DOP_FFT_CH1_EVEN_ADDR)  + (y*CHIRP_NUM2+x)*2));
                    qPeak4   = (*( ((int16 *)DOP_FFT_CH1_EVEN_ADDR)  + (y*CHIRP_NUM2+x)*2 + 1 ));
                    
                    numAngByFft = doAngFft(&ang1, &ang2, &ang3, &angByFft1, &angByFft2, rngIdx, dopIdx, iPeak1,qPeak1,iPeak2,qPeak2, iPeak3,qPeak3,iPeak4,qPeak4);
                    
                    
                    if(numAngByFft == 0)
                    {
                        //non-mimo angle easurement
                        objResult[resultN].hori = (ang1+ang2)/2;
                        
                        objResult[resultN].peak = g_absMergeArray[rngIdx][dopIdx];
                        objResult[resultN].rng_idx = rngIdx;
                        objResult[resultN].dop_idx = dopIdx;
                        objResult[resultN].valid = 1;
                        objResult[resultN].frame_cnt = frameCnt;
                        objResult[resultN].cluster_id = UNCLASSIFIED;
                        
                        objResult[resultN].x = RANGE_STEP*rngIdx*sin( objResult[resultN].hori * COEF_DEGREE_TO_RAD);
                        objResult[resultN].y = RANGE_STEP*rngIdx*cos( objResult[resultN].hori * COEF_DEGREE_TO_RAD);
                        //printfDebug("target %.2f \r\n", 10*log(g_absMergeArray[rngIdx][dopIdx]*1.0/thNoCfar));
                        #ifdef PRINT_DEBUG_EN
                        //printfDebug("FrameCnt: %d RngIdx: %d DopIdx: %d Power: %d, angle1 : %0.1f, angle2 : %0.1f, angle3 : %0.1f, HoriAngle: %.1f \r\n", objResult[resultN].frame_cnt, objResult[resultN].rng_idx, (objResult[resultN].dop_idx- CHIRP_NUM2/2), objResult[resultN].peak, ang1, ang2, ang3, objResult[resultN].hori);
                        #endif
                        resultN += 1;
                        if(resultN == OBJ_MAX)
                        {
                            return resultN;
                        }
                    }
                    else if(numAngByFft == 1 || numAngByFft == 2)
                    {
                        
                        #ifdef PRINT_DEBUG_EN
                            if(numAngByFft == 1)
                            {
                            //    printfDebug("FrameCnt: %d RngIdx: %d DopIdx: %d Power: %d, angle1 : %0.1f, angle2 : %0.1f, angle3 : %0.1f, angByFFT1: %.1f \r\n", objResult[resultN].frame_cnt, objResult[resultN].rng_idx, (objResult[resultN].dop_idx- CHIRP_NUM2/2), objResult[resultN].peak, ang1, ang2, ang3, angByFft1);
                            }
                            else
                            {
                            //   printfDebug("FrameCnt: %d RngIdx: %d DopIdx: %d Power: %d, angle1 : %0.1f, angle2 : %0.1f, angle3 : %0.1f, angByFFT1: %.1f, angByFFT2: %.1f \r\n", objResult[resultN].frame_cnt, objResult[resultN].rng_idx, (objResult[resultN].dop_idx- CHIRP_NUM2/2), objResult[resultN].peak, ang1, ang2, ang3, angByFft1, angByFft2);
 
                            }
                        #endif

                        objResult[resultN].peak = g_absMergeArray[rngIdx][dopIdx];
                        objResult[resultN].rng_idx = rngIdx;
                        objResult[resultN].dop_idx = dopIdx;
                        objResult[resultN].valid = 1;
                        objResult[resultN].frame_cnt = frameCnt;
                        objResult[resultN].cluster_id = UNCLASSIFIED;
                        objResult[resultN].hori = angByFft1;
                        
                        objResult[resultN].x = RANGE_STEP*rngIdx*sin(angByFft1 * COEF_DEGREE_TO_RAD);
                        objResult[resultN].y = RANGE_STEP*rngIdx*cos(angByFft1 * COEF_DEGREE_TO_RAD);
                        //printfDebug("target %.2f \r\n", 10*log(g_absMergeArray[rngIdx][dopIdx]*1.0/thNoCfar));
                        #ifdef PRINT_DEBUG_EN
                            //printfDebug("FrameCnt: %d RngIdx: %d DopIdx: %d Power: %d, angle1 : %0.1f, angle2 : %0.1f, angle3 : %0.1f, angByFFT1: %.1f \r\n", objResult[resultN].frame_cnt, objResult[resultN].rng_idx, (objResult[resultN].dop_idx- CHIRP_NUM2/2), objResult[resultN].peak, ang1, ang2, ang3, angByFft1);
                        #endif
                        resultN += 1;
                        if(resultN == OBJ_MAX)
                        {
                            return resultN;
                        }
                    
                        if(numAngByFft == 2)
                        {
                            objResult[resultN].peak = g_absMergeArray[rngIdx][dopIdx];
                            objResult[resultN].rng_idx = rngIdx;
                            objResult[resultN].dop_idx = dopIdx;
                            objResult[resultN].valid = 1;
                            objResult[resultN].frame_cnt = frameCnt;
                            objResult[resultN].cluster_id = UNCLASSIFIED;
                            objResult[resultN].hori = angByFft2;
                            
                            objResult[resultN].x = RANGE_STEP*rngIdx*sin(angByFft2 * COEF_DEGREE_TO_RAD);
                            objResult[resultN].y = RANGE_STEP*rngIdx*cos(angByFft2 * COEF_DEGREE_TO_RAD);
                            //printfDebug("target %.2f \r\n", 10*log(g_absMergeArray[rngIdx][dopIdx]*1.0/thNoCfar));
                            #ifdef PRINT_DEBUG_EN
                                //printfDebug("FrameCnt: %d RngIdx: %d DopIdx: %d Power: %d, angle1 : %0.1f, angle2 : %0.1f, angle3 : %0.1f,angByFFT2: %.1f \r\n", objResult[resultN].frame_cnt, objResult[resultN].rng_idx, (objResult[resultN].dop_idx- CHIRP_NUM2/2), objResult[resultN].peak, ang1, ang2, ang3, angByFft2);
                            #endif
                            resultN += 1;
                            if(resultN == OBJ_MAX)
                            {
                                return resultN;
                            }
                        }
                    }
                    else
                    {
                        printf("num of angles return by angle FFT is inccorrect");
                    }
				}
			}
		}	
	}
 
	return resultN;
	 
}


void detectMaxPeak(OBJ_MAXPEAK_TypeDef *obj)
{
	uint16 rngIdx = 0;
	uint16 dopIdx = 0;
		
 //   uint32 curTiaGain = TIA_GAIN_6DB;
//    uint32 curVgaGain = PGA_GAIN_44DB;
//    int32 powerThdAdj=-200;
    
    //powerThdAdj = -32+(curTiaGain*6+curVgaGain*2-50) - 2; 
    
    uint32 tempdata;
    uint16 maxPosRng;
    uint16 maxPosDop;
    uint32 maxSignal;
   // float phase;
    
//    for(int16 i=0; i<RANGE_MAX; i++)
//    {
//      g_absMergeArray[i][CHIRP_NUM2/2] = 0;
//    }
    
     maxSignal = 0;
    for(rngIdx = RANGE_MIN - 1; rngIdx < RANGE_MAX; rngIdx++)
    {
        for(dopIdx = 0; dopIdx<CHIRP_NUM2; dopIdx++) 
        {
            tempdata = g_absMergeArray[rngIdx][dopIdx];
            if(tempdata > maxSignal)
            {
                 maxSignal   = tempdata; 
                 maxPosRng  = rngIdx;
                 maxPosDop  = dopIdx; 
            }
        }
    }
    
    rngIdx =  maxPosRng;
    dopIdx =  maxPosDop;
    if(maxSignal > 200000)
    {
        obj->powerSignal = g_absMergeArray[rngIdx][dopIdx];
        obj->rngIdx = rngIdx;
        obj->dopIdx = dopIdx;
        obj->valid = 1;                           
    }
    else
    {
        obj->valid = 0;
    }
}

void complexMultiply(int16 *outR, int16 *outI, int16 inR1, int16 inI1,float inR2,float inI2)
{
    *outR = (int16)(inR1*inR2 - inI1*inI2);
    *outI = (int16)(inR1*inI2 + inI1*inR2);
}    

int cmpfunc_uint32(const void * a, const void * b) 
{
   return ( *(uint32*)a - *(uint32*)b );
}

/*********************************************************
Function name: dopplerSwapMimo
Description:   serach for single peak
Paramater:     dopFftLen : doppler fft length
               rngMax    : max range
			   absMergeArray : abs array, size[RANGE_MAX][CHIRP_NUM2]
Return:        none
*********************************************************/
void dopplerSwapMimo(int32 dopFftLen, int32 rngMax, uint32 absMergeArray[][CHIRP_NUM2])
{
    uint32 tempdata2Dfft1,tempdata2Dfft2;
	for(uint32 count1 =0; count1< rngMax; count1++)
    {
        for(unsigned char count2 =0; count2< dopFftLen/2; count2++)
        {
            //swap velocity.After swap, absMergeArray[][dopFftLen/2] is static 
            tempdata2Dfft1  = absMergeArray[count1][count2];
            tempdata2Dfft2  = absMergeArray[count1][count2+dopFftLen/2];
            absMergeArray[count1][count2] = tempdata2Dfft2;
            absMergeArray[count1][count2+dopFftLen/2] = tempdata2Dfft1 ;
        }
    }
}

uint8 doAngFft(float *ang1, float *ang2, float *ang3, float *angByFft1, float *angByFft2, uint16 rngIdx, uint16 dopIdx, int16 iPeak1, int16 qPeak1, int16 iPeak2, int16 qPeak2, int16 iPeak3, int16 qPeak3, int16 iPeak4, int16 qPeak4)
{
    int16 angFftInput[2*ANGLE_FFT_LEN] = {0};
    int16 angFftOutput[2*ANGLE_FFT_LEN] = {0};
    uint32 angFftAbs[ANGLE_FFT_LEN] = {0};
    uint32 maxVal;
    int16 angIdx;
    uint32 tempdata1,tempdata2;
    
    float phaseDeltaRx = 14.5;
    float phaseDeltaTx = 9.0;
    
    float phaseCompRealAnt2 = 0;
    float phaseCompImagAnt2 = 0;
    float phaseCompAnt2 = sinf(phaseDeltaRx/180*PI)*PI; 
    phaseCompRealAnt2 = cosf(phaseCompAnt2); 
    phaseCompImagAnt2 = sinf(phaseCompAnt2);
    
    float phaseCompRealAnt3 = 0;
    float phaseCompImagAnt3 = 0;
    float phaseCompAnt3 = sinf(phaseDeltaTx/180*PI)*PI; 
    //phaseCompRealAnt3 = cosf(phaseCompAnt3); 
    //phaseCompImagAnt3 = sinf(phaseCompAnt3);
    
    float phaseCompRealAnt4 = 0;
    float phaseCompImagAnt4 = 0;
    float phaseCompAnt4 = sinf((phaseDeltaRx + 3 + phaseDeltaTx)/180*PI)*PI; 
    //phaseCompRealAnt4 = cosf(phaseCompAnt4); 
    //phaseCompImagAnt4 = sinf(phaseCompAnt4);
    
    int8 df1[ANGLE_FFT_LEN-1] = {0};
   // uint16 pkIdx[ANGLE_FFT_LEN-2] = {0};
    uint32 pkPow[ANGLE_FFT_LEN-2] = {0};
   // uint32 pkPowSort[ANGLE_FFT_LEN-2] = {0};
  //  uint32 pkPowOut[MAX_ANG_FFT_PKS] = {0};
   // uint16 pkIdxOut[MAX_ANG_FFT_PKS] = {0};
    int32 diff = 0;
  //  uint16 numPk = 0;
    uint32 powThr = 0;
    uint16 firstPkIdx = 0;
    
//    float fdEst = 0;
    float phaseCompDop = 0;
//    float phaseCompDopReal = 0;
//    float phaseCompDopImag = 0;
//    uint8 numDetectedAng = 0;
    
    PHASE_TypeDef phaseResult1, phaseResult2, phaseResult3; 

    #ifdef EN_DOP_COMP
        //fdEst = (dopIdx-CHIRP_NUM2/2)*FD_STEP;
        //phaseCompDop = -2*PI*fdEst*(CHIRP_T0 + CHIRP_T1 + CHIRP_T2);
        phaseCompDop = -PI*(dopIdx-CHIRP_NUM2/2)/CHIRP_NUM2; //hack -
    
        phaseCompAnt3 += phaseCompDop;
        phaseCompAnt4 += phaseCompDop;
    #endif
    
    phaseCompRealAnt3 = cosf(phaseCompAnt3); 
    phaseCompImagAnt3 = sinf(phaseCompAnt3);
    
    phaseCompRealAnt4 = cosf(phaseCompAnt4); 
    phaseCompImagAnt4 = sinf(phaseCompAnt4);
    
    angFftInput[0] = iPeak1;
    angFftInput[1] = qPeak1;
    angFftInput[2] = iPeak2;
    angFftInput[3] = qPeak2;
    angFftInput[4] = iPeak3;
    angFftInput[5] = qPeak3;
    angFftInput[6] = iPeak4;
    angFftInput[7] = qPeak4;
    
    complexMultiply(&angFftInput[2], &angFftInput[3], angFftInput[2], angFftInput[3], phaseCompRealAnt2, phaseCompImagAnt2);
    complexMultiply(&angFftInput[4], &angFftInput[5], angFftInput[4], angFftInput[5], phaseCompRealAnt3, phaseCompImagAnt3);
    complexMultiply(&angFftInput[6], &angFftInput[7], angFftInput[6], angFftInput[7], phaseCompRealAnt4, phaseCompImagAnt4);

    phaseResult1 = phaseCal(angFftInput[0], angFftInput[1], angFftInput[2], angFftInput[3], 0); 
    phaseResult2 = phaseCal(angFftInput[2], angFftInput[3], angFftInput[4], angFftInput[5], 0);
    phaseResult3 = phaseCal(angFftInput[4], angFftInput[5], angFftInput[6], angFftInput[7], 0);

    *ang1 = phaseResult1.angle;
    *ang2 = phaseResult2.angle;
    *ang3 = phaseResult3.angle;
    
    fftSingle(ANGLE_FFT_LEN, (uint32)&angFftInput, (uint32)&angFftOutput, 4, 4);
    DSP->dsp_fft_ctrl &= ~dsp_fft_ctrl_fft_en_bit;
    
    //abs
    absAdt3102(ANGLE_FFT_LEN, 1, (uint32)&angFftOutput[0], (uint32)&angFftAbs[0]); 
    
    //fftshift
    for(int16 k =0; k< ANGLE_FFT_LEN2; k++)
    {
        tempdata1  = angFftAbs[k];
        tempdata2  = angFftAbs[k + ANGLE_FFT_LEN2];
        angFftAbs[k] = tempdata2;
        angFftAbs[k + ANGLE_FFT_LEN2] = tempdata1 ;
    }
    
    if(0)
    {                
        printf("\r\n");
        printf("\r\n");
        for(int16 k =0; k< ANGLE_FFT_LEN; k++)
        {
             printf("%d \r\n", angFftAbs[k]);
        }
    }           
            
     if(0)
     {
        maxVal = 0;
        for(uint16 k=0; k<ANGLE_FFT_LEN; k++)
        {
            if(angFftAbs[k] > maxVal)
            {
                angIdx = k;
                maxVal = angFftAbs[k];
            }                    
        }           
        angIdx = ANGLE_FFT_LEN2 - angIdx + 1;

        *angByFft1 = asin(angIdx*ANGLE_FFT_COEFF)/PI*180;
                 
        printfDebug("dopIdx :  %d, angle1 :  %.1f,  angle2 :  %.1f,  angle3 :  %.1f, angByFft : %.1f \r\n",  dopIdx-CHIRP_NUM2/2, *ang1, *ang2, *ang3, *angByFft1);
    }
        
    for(uint16 k = 0; k < ANGLE_FFT_LEN-1; k++) 
    {
        df1[k] = 0;
    }
    
    for(uint16 k = 1; k < ANGLE_FFT_LEN; k++) 
    {
        diff = angFftAbs[k] - angFftAbs[k-1];
        if(diff > 0)
        {
            df1[k-1] = 1;
        }
        else //(diff <= 0)
        {
            df1[k-1] = -1;
        }
    }
            
    for(uint16 k = 0; k < ANGLE_FFT_LEN-2; k++)
    {
        //pkIdx[k] = 0;
        pkPow[k] = 0;
        //pkPowSort[k] = 0;
    }
    
    for(uint16 k = 1; k < ANGLE_FFT_LEN-1; k++) 
    {
        diff = df1[k] - df1[k-1];
        if(diff < 0)
        {
            //pkIdx[k-1] = k;
            pkPow[k-1] = angFftAbs[k];
            //pkPowSort[k-1] = angFftAbs[k];
        }
    }
    
    //do without qsort
    maxVal = 0;
    angIdx = ANGLE_FFT_LEN-2;
    for(uint16 k=0; k<ANGLE_FFT_LEN-2; k++)
    {
        if(pkPow[k] >= MIN_ANG_FFT_POW)
        {
            if(pkPow[k] > maxVal)
            {
                angIdx = k;
                maxVal = pkPow[k];
            }
        }            
    }
    if(angIdx == ANGLE_FFT_LEN-2)
    {
        #ifdef ANGLE_TESTMODE
            printfDebug("rngIdx : %d, dopIdx :  %d, angle1 :  %.1f,  angle2 :  %.1f,  angle3 :  %.1f, num of angFFT peak Candidates : %d \r\n",  rngIdx, dopIdx-CHIRP_NUM2/2, *ang1, *ang2, *ang3, 0);
        #endif
        return 0; // no pk
    }
    else
    {
        firstPkIdx = angIdx;
        powThr = pkPow[angIdx] >> 2;
        
        //clear for looking for 2nd pk
        pkPow[angIdx] = 0;
        
        angIdx = ANGLE_FFT_LEN2 - angIdx;

        *angByFft1 = asin(angIdx*ANGLE_FFT_COEFF)/PI*180;
        
         #ifdef ANGLE_TESTMODE
            printfDebug("rngIdx : %d, dopIdx :  %d, angle1 :  %.1f,  angle2 :  %.1f,  angle3 :  %.1f \r\n",  rngIdx, dopIdx-CHIRP_NUM2/2, *ang1, *ang2, *ang3);
            printfDebug("Angle_FFT_1st_Peak ---  AngFFTPow, %d, angByFft1 : %.1f \r\n",  maxVal, *angByFft1);
        #endif
        
        //find 2nd pk
        maxVal = 0;
        angIdx = ANGLE_FFT_LEN-2;
        for(uint16 k=0; k<ANGLE_FFT_LEN-2; k++)
        {
            if(abs(k - firstPkIdx) <= MIN_ANG_FFT_PK_DISTANCE && pkPow[k] >= powThr)
            {
                if(pkPow[k] > maxVal)
                {
                    angIdx = k;
                    maxVal = pkPow[k];
                }
            }            
        }
        if(angIdx == ANGLE_FFT_LEN-2)
        {         
            return 1;
        }
        else
        {
            angIdx = ANGLE_FFT_LEN2 - angIdx;

            *angByFft2 = asin(angIdx*ANGLE_FFT_COEFF)/PI*180;
            #ifdef ANGLE_TESTMODE
              printfDebug("Angle_FFT_2nd_Peak ---  AngFFTPow, %d, angByFft2 : %.1f \r\n",  maxVal, *angByFft2);
            #endif
            return 2;
        }
    }
    
//    qsort(pkPowSort, ANGLE_FFT_LEN-2, sizeof(uint32), cmpfunc_uint32);
//    
//    powThr = pkPowSort[ANGLE_FFT_LEN-3] >> 2;
//    numPk = 0;
//   
//    for(uint16 k = ANGLE_FFT_LEN-3; k >= ANGLE_FFT_LEN-3 - MAX_ANG_FFT_PKS + 1; k--)
//    {
//        if(pkPowSort[k] > MIN_ANG_FFT_POW && pkPowSort[k] > powThr)
//        {
//            for(uint16 k1 = 0; k1 < ANGLE_FFT_LEN-2; k1++)
//            {
//                if(pkPowSort[k] == pkPow[k1])
//                {
//                    pkIdxOut[numPk] = pkIdx[k1];
//                    pkPowOut[numPk] = pkPow[k1];
//                    numPk += 1;
//                    break;
//                }
//            }
//        }
//        else
//        {
//            //break;
//        }
//    }
//    
//    #ifdef ANGLE_TESTMODE
//        printfDebug("rngIdx : %d, dopIdx :  %d, angle1 :  %.1f,  angle2 :  %.1f,  angle3 :  %.1f, num of angFFT peak Candidates : %d \r\n",  rngIdx, dopIdx-CHIRP_NUM2/2, *ang1, *ang2, *ang3, numPk);
//    #endif
//    
//    if(numPk > 0)
//    {    
//        angIdx = pkIdxOut[0];
//        
//        angIdx = ANGLE_FFT_LEN2 - angIdx; // + 1;

//        *angByFft1 = asin(angIdx*ANGLE_FFT_COEFF)/PI*180;
//        
//        #ifdef ANGLE_TESTMODE
//            printfDebug("AngFFTPow, %d, angByFft1 : %.1f \r\n",  pkPowOut[0], *angByFft1);
//        #endif
//        
//        // printfDebug(" fdEst : %0.6f, phaseCompDop : %.1f, AngFftPow :  %d,  angByFft : %.1f \r\n",   fdEst, phaseCompDop, pkPowOut[0], (*angByFft1));
//        numDetectedAng += 1;
//        
//        if(numPk > 1)
//        {
//            for(uint16 k = 1; k < numPk; k++)
//            {
//                angIdx = pkIdxOut[k];
//                
//                angIdx = ANGLE_FFT_LEN2 - angIdx; // + 1;
//    
//                *angByFft2 = asin(angIdx*ANGLE_FFT_COEFF)/PI*180;
//                
//                if(fabs(*angByFft2 - *angByFft1) > 29 && fabs(*angByFft2) < 60)
//                {
//                    #ifdef ANGLE_TESTMODE
//                      printfDebug("AngFFTPow, %d, angByFft2 : %.1f \r\n",  pkPowOut[k], *angByFft2);
//                    #endif
//                    
//                   // printfDebug("fdEst : %0.6f, phaseCompDop : %.1f, AngFftPow :  %d,  angByFft : %.1f \r\n",  fdEst, phaseCompDop, pkPowOut[k], (*angByFft2));
//                    numDetectedAng += 1;
//                    break;
//                }
//            }
//        }
//    }

//    return numDetectedAng;
    
}
