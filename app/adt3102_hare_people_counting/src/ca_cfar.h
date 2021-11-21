#ifndef __CA_CFAR_H__  
#define __CA_CFAR_H__ 

//#include "define.h"
#include "assert.h"
#include "adt3102_type_define.h"

#define NUM_TRAIN_CELLS_RNG 20 //training cell number for rng cfar - 2 side
#define NUM_GUARD_CELLS_RNG 4 //guard cell number for rng cfar- 2 side
#define NUM_TRAIN_CELLS_DOP 10 //training cell number for rng cfar - 2 side
#define NUM_GUARD_CELLS_DOP 2 //guard cell number for rng cfar- 2 side

enum CfarClearCell {
    CFAR_CLEAR_CELL_ON = 0,      //clear detect cell after CFAR
    CFAR_CLEAR_CELL_OFF = 1,      //detect cell not cleared after CFAR 
};

/*********************************************************
Function name: caCfar
Description:   CA-CFAR algorithm
Paramater:     *data            : pointer to data cell buffer
               *detect          : pointer to detection outcome buffer
               *threshold       : pointer to threshold buffer
               numCells         : number of input data
               startK           : index of starting cell to do CFAR
               endK             : index of ending cell to do CFAR
               numTrainCells    : number of training cells (2-side)
               numGuardCells    : number of guard cells (2-side)
Return:        void
*********************************************************/
void caCfar(uint32 *data, uint8 *detect, float *threshold, uint16 numCells, uint16 startK, uint16 endK, float ThFac, uint16 numTrainCells, uint16 numGuardCells, uint8 cell_det_cell);

uint8 caCfarAsic(uint32 *data, uint8 *detect, float *threshold, uint16 numCells, uint16 startK, uint16 endK, float ThFac, uint16 numTrainCells, uint16 numGuardCells, uint8 cell_det_cell);

/*********************************************************
Function name: cohAccuAdt3102
Description:   coherent accumulation of 2DFFT matrix along doppler axis
Paramater:     srcAddr          : address 2DFFT matrx
               CohAccuReal      : pointer to accumulated real part
               CohAccuImag      : pointer to accumulated imag part
Return:        void
*********************************************************/
void cohAccuAdt3102(uint32 srcAddr, int32 *CohAccuReal, int32 *CohAccuImag);

/*********************************************************
Function name: max
Description:   Find maximum between two numbers
Paramater:     num1            : input number 1
               num2            : input number 2
Return:        max of num1 and num2
*********************************************************/
int16 max(int16 num1, int16 num2);

/*********************************************************
Function name: min
Description:   Find minimum between two numbers
Paramater:     num1            : input number 1
               num2            : input number 2
Return:        min of num1 and num2
*********************************************************/
int16 min(int16 num1, int16 num2); 


#endif

