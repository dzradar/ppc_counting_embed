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
#include "adt3102.h" 
#include "ca_cfar.h"
#include "stdio.h"
void caCfar(uint32 *data, uint8 *detect, float *threshold, uint16 numCells, uint16 startK, uint16 endK, float ThFac, uint16 numTrainCells, uint16 numGuardCells, uint8 cell_det_cell)
{
	uint16 cutIdx;
	uint16 oneSideGuardCell;
	uint16 oneSideTrainCell;
	uint16 numFrontCells;
	uint16 numRearCells;
	int16 guardFrontIdx;
	int16 guardRearIdx;
	uint16 traningCellInds[numTrainCells];
	uint16 numFrontTrainingCells;
	uint16 numRearTrainingCells;
	int16 k;
	uint16 cnt;
	uint32 pw = 0;
	
//	if(TRAIN_CELL_NUM + GUARD_CELL_NUM + 1 > numCells )
//	{
//		error;
//	}
	
	oneSideGuardCell = numGuardCells/2;
	oneSideTrainCell = numTrainCells/2;
	
	//for(cutIdx = 1; cutIdx <= numCells; cutIdx++)    //index starts from 1
	for(cutIdx = startK + 1 ; cutIdx <= endK + 1; cutIdx++)
	{	
			guardFrontIdx = cutIdx - oneSideGuardCell;  //front end of guard cells
			guardRearIdx = cutIdx + oneSideGuardCell;   //rear end of guard cells
	
			numFrontCells = max(guardFrontIdx-1, 0);       //number of training cells in front of guard cells	
			numRearCells = max(numCells-guardRearIdx, 0);  //number of training cells behind guard cells
			
			if(numFrontCells < numRearCells)  // front remaining cells are less
			{
				// Use half of training cells or all available cells in front
				numFrontTrainingCells = min(oneSideTrainCell, numFrontCells);  
				// Make up the rest or take all rear cells if not enough
				numRearTrainingCells = min(numTrainCells-numFrontTrainingCells, numRearCells);
			}
			else   // rear remaining cells are less
			{
				// Use half of training cells or all available rear cells
				numRearTrainingCells = min(oneSideTrainCell, numRearCells);  
				// Make up the rest or take all front cells if not enough	
				numFrontTrainingCells = min(numTrainCells-numRearTrainingCells, numFrontCells); 
			}
		
			//assert(numFrontTrainingCells <  NUM_TRAIN_CELLS+1); //Help compiler compute upperbounds
			//front_training_cell_inds = guardFrontIdx + (-numFrontTrainingCells:-1);
			//assert(numRearTrainingCells <  NUM_TRAIN_CELLS+1);
			//rear_training_cell_inds = guardRearIdx + (1:numRearTrainingCells);			 
			//traningCellInds = [front_training_cell_inds rear_training_cell_inds];
			cnt = 0;
			for(k = -numFrontTrainingCells; k <= -1; k++)
			{
				traningCellInds[cnt] = guardFrontIdx + k -1;   //-1 to shift by 1
				cnt += 1;
			}
			for(k = 1; k <= numRearTrainingCells; k++)
			{
				traningCellInds[cnt] = guardRearIdx + k -1;   //-1 to shift by 1
				cnt += 1;
			}
			
			//assert(cnt == NUM_TRAIN_CELLS);
			pw = 0;
			for(k = 0; k < cnt; k++)
			{
				pw += data[traningCellInds[k]];
			}
			//pw = pw/numTrainCells; already factored into ThFac
			threshold[cutIdx-1] =  ThFac*pw;   //shift 1
			if(data[cutIdx-1] > threshold[cutIdx-1])
			{
					detect[cutIdx-1] = 1;
			}
			else
			{
                if(cell_det_cell == CFAR_CLEAR_CELL_ON)
                {
                    detect[cutIdx-1] = 0; 
                }
			}
				
		}
	}


 uint8 caCfarAsic(uint32 *data, uint8 *detect, float *threshold, uint16 numCells, uint16 startK, uint16 endK, float ThFac, uint16 numTrainCells, uint16 numGuardCells, uint8 cell_det_cell)
{
    volatile uint32 NarrayLenLeft=0;
    volatile uint32 NarrayLenRight=0;
    volatile uint32 NarrayAddrLeft=0;
    volatile uint32 NarrayAddrRight=0;
    volatile uint64 NcfarSumValueLeft=0;
    volatile uint64 NcfarSumValueRight=0;
    volatile uint64 NcfarSumValue=0;
    uint32 targetVelue;
    int32  detectFlag = 1;
    int32  fronty1, fronty2, reary1, reary2;
    uint16 trainCell;
    uint16 guardCell;
    uint32 srcAddr;
    uint16 srcLen;
    uint8  *det;
    trainCell   = numTrainCells/2;
    guardCell   = numGuardCells/2;
    srcAddr     = (uint32)&(*data);
    srcLen      = numCells;
    det         = detect;
    
    if((srcLen > trainCell + guardCell+1) && (srcLen > (guardCell<<1) + 2))
    {            
        fronty1 =  guardCell;
        fronty2 = (trainCell + guardCell) ;
        /*  rear side edge points */
        reary1 = (srcLen - trainCell) - guardCell - 1;
        reary2 = srcLen - (guardCell<<1) + 1;
    }
    else
    {
        fronty1 = 0;
        fronty2 = 0;
        reary1 = 0;
        reary2 = 0;
    }
    //for(int32 arrayIndex=startK; arrayIndex<endK; arrayIndex++)
    for(int32 arrayIndex=startK; arrayIndex<=endK; arrayIndex++)  //hack
    {
        if(arrayIndex<fronty1)
        {
            NarrayLenLeft      = 0;
            NarrayLenRight     = trainCell<<1;
            NarrayAddrLeft     = 0;
            NarrayAddrRight    = srcAddr+((guardCell+arrayIndex+1)<<2);
        }else if(arrayIndex<=fronty2)
        {
            NarrayLenLeft      = arrayIndex-guardCell;
            NarrayLenRight     = (trainCell<<1) - (arrayIndex-guardCell);
            NarrayAddrLeft     = srcAddr;
            NarrayAddrRight    = srcAddr+((guardCell+arrayIndex+1)<<2);  
        }else if(arrayIndex<reary1)//21
        {
            NarrayLenLeft      = trainCell;
            NarrayLenRight     = trainCell;
            NarrayAddrLeft     = srcAddr+((arrayIndex-guardCell-trainCell)<<2);
            NarrayAddrRight    = srcAddr+((guardCell+arrayIndex+1)<<2);
        }else if(arrayIndex<reary2)//29
        {
            NarrayLenLeft      = arrayIndex-reary1+trainCell;
            NarrayLenRight     = trainCell-(arrayIndex-reary1);
            NarrayAddrLeft     = srcAddr+((srcLen-(trainCell<<1)-(guardCell<<1)-1)<<2);
            NarrayAddrRight    = srcAddr+((arrayIndex+guardCell+1)<<2);
        }else
        {
            NarrayLenLeft      = trainCell<<1;
            NarrayLenRight     = 0;
            NarrayAddrLeft     = srcAddr+((arrayIndex-guardCell-(trainCell<<1))<<2);
            NarrayAddrRight    = 0;      
        }
        NcfarSumValueLeft  = accSum(NarrayAddrLeft,  NarrayLenLeft);
        NcfarSumValueRight = accSum(NarrayAddrRight, NarrayLenRight);       
        NcfarSumValue = (NcfarSumValueLeft + NcfarSumValueRight);
        targetVelue = *((uint32 *)srcAddr + arrayIndex);
        threshold[arrayIndex] =  ThFac*NcfarSumValue; //targetVelue;
        if (targetVelue>threshold[arrayIndex]) //(NcfarSumValue*ThFac)) 
        {
            det[arrayIndex]=1;        
            detectFlag = 0;
        }else
        {
            if(cell_det_cell == CFAR_CLEAR_CELL_ON)
            {
                det[arrayIndex] = 0; 
            }
        }
    }
    return detectFlag;
}


int16 max(int16 num1, int16 num2)
{
    return (num1 > num2 ) ? num1 : num2;
}


int16 min(int16 num1, int16 num2) 
{
    return (num1 > num2 ) ? num2 : num1;
}

    
void cohAccuAdt3102(uint32 srcAddr, int32 *CohAccuReal, int32 *CohAccuImag)
{
    uint32 cnt = 0;
    uint16 row = 0;
    uint16 col = 0;
    int16 tmpR = 0;
    int16 tmpI = 0;
    uint32 addr_idx = 0;
    
    for(col = 0; col <= RANGE_MAX; col++)
    {
        CohAccuReal[col] = 0;
        CohAccuImag[col] = 0;
    }
    
    for (col = 0; col <= RANGE_MAX; col++)
    {
        for(row = 0; row <= CHIRP_NUM; row++)
        //for(int32 i =0; i<fftLen*rngMax*2; i=i+2)
        {
            addr_idx = cnt*2; 
            tmpR = (*( ((int16 *)srcAddr) + addr_idx ));
            tmpI = (*( (int16 *)srcAddr + addr_idx + 1 ));
            CohAccuReal[col] = CohAccuReal[col] +  tmpR;
            CohAccuImag[col] = CohAccuImag[col] +  tmpI;
            //*( ((int32 *)dstAddr) + cnt )=tmp_r*tmp_r + tmp_i*tmp_i;
            cnt++;
        }
    }
    for(col = 0; col <= RANGE_MAX; col++)
    {
        CohAccuReal[col]  = CohAccuReal[col]/CHIRP_NUM;
        CohAccuImag[col]  = CohAccuImag[col]/CHIRP_NUM;
    }
}
