//--------------------------------------------------------------------
//Copyright(c)2020,Andar Technologise Inc.
//All Rights Reserved
//Confidential Property of Andar Technologies Inc.
//
//Module Description:
//---------------------------------------------------------------
// To search for single max peak in 2D FFT table
//  
//
//Created by :jiangqi
//$Revision: 1.5
//$Data: 2020/11/9
//--------------------------------------------------------------------
#include "stdio.h"
#include "string.h"
#include "peak_single.h"
#include "math.h"
#include "stdint.h"
#include "arm_math.h"
#include "adt3102.h"
#include "adt3102_uart.h"

void peakSingle(int32 dopFftLen, int32 dopMin, int32 rngMin, int32 rngMax, uint32 absMergeArray[][CHIRP_NUM], int32 powerThdAdj, uint32 nearest, uint32 distComp, OBJ_PEAKSINGLE_TypeDef *obj)
{
    const int32 DBFS=84; //dBFS=84db
    uint32 newData;
    uint32 max2DfftRow;
    uint16 maxPosRng = 0xfff;
    uint16 maxPosDop = 0xfff;
	uint16 maxPosDopRow = 0;
    uint32 maxPower = 0;
    uint8  peakVld = 0;
    uint16 firstPeakFnd = 0;
    uint16 seachEnable;
    float maxPowerDb=-200;
    //absMergeArray, row 8 is static, 9~15 are positive speed, 0~7 are negetive speed
    for(int i=0; i<rngMax; i++)
    {
        for(int j=dopFftLen/2-dopMin+1; j<dopFftLen/2+dopMin;j++)
        {
            absMergeArray[i][j] = 0;
        }
    }
    for(int32 rngIdx = rngMin; rngIdx< rngMax; rngIdx++)
    {
        max2DfftRow = 0;
        //find the highest peak in every row
        for(uint16 i =0; i< dopFftLen; i++)
		{
            newData = absMergeArray[rngIdx][i];
            if(newData > max2DfftRow)
            {
			    max2DfftRow   = newData;  
	            maxPosDopRow  = i;                
            }
        }
        if(distComp==1)
        {
		    max2DfftRow=max2DfftRow*rngIdx*rngIdx; //Compensate for distance, should be ^4 ideally. 
        }
        seachEnable= (nearest==0) || ((nearest==1) && (firstPeakFnd==0)) ? 1 : 0;
        if((max2DfftRow > maxPower) && (seachEnable==1))
        {
            maxPosRng  = rngIdx;
            maxPosDop  = maxPosDopRow;
			maxPower   = max2DfftRow;
            maxPowerDb =10*log10(maxPower)-DBFS;
            if(maxPowerDb > powerThdAdj)
            {
                firstPeakFnd=1;
            }
        }
	}
    if (maxPowerDb > powerThdAdj)
    {
       peakVld = 1; 
    }
    obj->valid=peakVld;
    obj->powerSignal=maxPowerDb;
    obj->rngIdx=maxPosRng;
    obj->dopIdx=maxPosDop; ////static at [CHIRP_NUM/2]

//    if(maxPosDop!=0xfff)
//    {
//       printfDebug("Max ---- maxPower: %.1f, Distance: %.0f, Detect status: %d \r\n", maxPowerDb, (float)maxPosRng*RANGE_STEP, peakVld);
//    }
}
