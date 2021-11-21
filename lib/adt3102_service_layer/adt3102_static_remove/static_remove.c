//--------------------------------------------------------------------
//Copyright(c)2020,Andar Technologise Inc.
//All Rights Reserved
//Confidential Property of Andar Technologies Inc.
//
//Module Description:
//---------------------------------------------------------------
//To remove static object in rangeMax*nFftVel range. 
//Data arrangement (x0,0),(x0,1),(x0,2), ....,(x0,15),(x1,0),(x1,1)....
//  
//
//Created by :jiangqi
//$Revision: 1.5
//$Data: 2020/11/9
//--------------------------------------------------------------------
//
//All include header files
#include "math.h"
#include "arm_math.h"
#include "stdio.h"
#include "string.h"
#include "static_remove.h"


void background_removal(int32 srcAddr, int32 bgAddr, uint16 nFftRange,uint16 rangeMax, uint16 nFftVel,uint16 nLog2FftVel, float iqcoef, uint8 update_flag)
{
	volatile int32* pData;
    volatile int32* pDCdata;
	int32 iqRead, iqBg;
	int16 iRead,qRead,iBg,qBg;
    int16 iCal, qCal;
    
    // estimate background for several minutes
    // if doppler is zero, the frame is used for background estimation!
    if (update_flag == 1)
    {
        for(unsigned int count1 =0; count1< rangeMax; count1++)
        {
            //dcSumI = 0;
            //dcSumQ = 0;
            pData = (volatile int32*)(srcAddr + (count1<<2));
            pDCdata = (volatile int32*)(bgAddr + (count1<<2));

            // read backgrond removal I,Q int data
            iqBg = *pDCdata;
            iBg  = (*((int16 *)pDCdata )) ; //iqBg & (0xffff);
            qBg  = (*((int16 *)pDCdata+1 )) ; //iqBg >> 16;
            
            for(uint16 count2 = 0; count2 < nFftVel; count2++)
            {
                // read src I, Q int data
                iqRead = *pData;
                iRead  = iqRead & (0xffff);
                qRead  = iqRead >> 16;
                
                //dcSumI = dcSumI + iRead;
                //dcSumQ = dcSumQ + qRead;
                
                
                //dcI = (int16)(dcSumI>>nLog2FftVel);
                //dcQ = (int16)(dcSumQ>>nLog2FftVel);   
                    
                    
                // weighting the background
                iBg   = (int16)((iBg * (1-iqcoef)) + (iRead * iqcoef));
                qBg   = (int16)((qBg * (1-iqcoef)) + (qRead * iqcoef));
                *pDCdata = (qBg<<16) + iBg;
                
                // update 
                pData = pData + nFftRange;
            }                
            
        }	
    }
    
	//background removal
    
    for(unsigned int count1 =0; count1< rangeMax; count1++)
    {
		pData = (volatile int32*)(srcAddr + (count1<<2));
        pDCdata = (volatile int32*)(bgAddr + (count1<<2));
        // read backgrond removal I,Q int data
        iqBg = *pDCdata;
		iBg  = iqBg & (0xffff);
		qBg  = iqBg >> 16; 
        for(uint16 count2 = 0; count2 < nFftVel; count2++)
        {
            // read src I, Q int data
			iqRead = *pData;
			iRead  = iqRead & (0xffff);
			qRead  = iqRead >> 16; 
            
            // remove background
			iCal   = iRead - iBg;
			qCal   = qRead - qBg;
			*pData = (qCal<<16) + iCal;
		    pData = pData + nFftRange;
        }
    }
}


void staticRemove(int32 srcAddr,uint16 nFftRange,uint16 rangeMax, uint16 nFftVel,uint16 nLog2FftVel)
{
	volatile int32* pData;
	int32 iqRead;
	int16 iRead,qRead,iCal,qCal;
	int32 dcSumI;
	int32 dcSumQ;
	int16 dcAvgI[rangeMax];
	int16 dcAvgQ[rangeMax];
    int16 dcI,dcQ;
	
    //replace 1st dc with 2nd one because it's different from others.
    //*(volatile int32 *)srcAddr = *(volatile int32*)(srcAddr+nFftRange*4);
    //*(volatile int32 *)(srcAddr+4) = *(volatile int32*)(srcAddr+nFftRange*4+4);
    //*(volatile int32 *)(srcAddr+8) = *(volatile int32*)(srcAddr+nFftRange*4+8);
	//calculate the average noise floor using all lines, 
    //printfDebug("iread \r\n",iRead); 	
    for(unsigned int count1 =0; count1< rangeMax; count1++)
    {
		dcSumI = 0;
		dcSumQ = 0;
		pData = (volatile int32*)(srcAddr + (count1<<2));
        for(uint16 count2 = 0; count2 < nFftVel; count2++)
        {
			iqRead = *pData;
			iRead  = iqRead & (0xffff);
			qRead  = iqRead >> 16;
            dcSumI = dcSumI + iRead;
            dcSumQ = dcSumQ + qRead;
		    pData = pData + nFftRange;
            if(count1==1)
            {
                //printfDebug("%d ",iRead);
            }                
        }
		dcAvgI[count1] = (int16)(dcSumI>>nLog2FftVel);
		dcAvgQ[count1] = (int16)(dcSumQ>>nLog2FftVel);
	    //printfDebug("dcSumI %d, dcSumQ %d,  dcAvgI[count1] %d, dcAvgQ[count1] %d \r\n",dcSumI,dcSumQ,dcAvgI[count1],dcAvgQ[count1]);  
    }
    //printfDebug(" \r\n"); 
    //printfDebug("qread \r\n"); 	
	//remove dc 
    for(unsigned int count1 =0; count1< rangeMax; count1++)
    {
		pData = (volatile int32*)(srcAddr + (count1<<2));
        dcI=dcAvgI[count1];
        dcQ=dcAvgQ[count1];
        for(uint16 count2 = 0; count2 < nFftVel; count2++)
        {
			iqRead = *pData;
			iRead  = iqRead & (0xffff);
			qRead  = iqRead >> 16;
			iCal   = iRead - dcI;
			qCal   = qRead - dcQ;
			*pData = (qCal<<16) + iCal;
		    pData = pData + nFftRange;
        }
    }
    //printfDebug(" \r\n"); 
}
