#ifndef __PROC_COUNT_H__  
#define __PROC_COUNT_H__ 

#include <adt3102_type_define.h>
#include "adt3102_uart.h"
#include "adt3102.h"

#define OBJ_MAX          100 //hack32 50   storage only
#define MAX_FRAME_CNT    10000    //used in tracking frame number
#define C_MIMOTHDABS_CFAR1     100000 // 100000 //100000 //250000 //200000 //100000   //thr for pk detection
#define C_MIMOTHDABS_CFAR2     100000 //250000 //200000 //100000   //thr for pk detection
#define C_MIMOTHDABS_CFAR3     100000
#define C_MIMOTHDABS_NOCFAR    200000 //100000 //250000 //200000 //100000   //thr for pk detection

enum DetectionMethod {
    DETECT_METHOD_CFAR1 = 0,      //dop cfar, then rng cfar 
    DETECT_METHOD_CFAR2 = 1,      // non-coherent accumulation, range cfar, then dop cfar
    DETECT_METHOD_CFAR3 = 2,      // coherent accumulation, range cfar, then dop cfar
    DETECT_METHOD_NOCFAR = 3,      //threshoulding only
};

#define NUM_FRAME_CLUSTER  1//3//2//3 //5
#define MIN_PK_FOR_DOING_CLUSTER  8//1//8 //6
#define DBSCAN_EPS_R 2
#define DBSCAN_EPS_A 10
#define DBSCAN_MIN_PTS 6

#define DBSCAN_EPS_X (4*RANGE_STEP + 0.0001)  
#define DBSCAN_EPS_Y (4*RANGE_STEP + 0.0001)
//#define DBSCAN_EPS_X (2*RANGE_STEP + 0.0001)  
//#define DBSCAN_EPS_Y (2*RANGE_STEP + 0.0001)
//#define DBSCAN_EPS_X (400*RANGE_STEP + 0.0001)  
//#define DBSCAN_EPS_Y (400*RANGE_STEP + 0.0001)

#define DBSCAN_EPS_D 2 //2
//#define DBSCAN_EPS_D 400 //2

#define COEF_DEGREE_TO_RAD 0.0175    //pi/180

#define PHASE_COR 0.2531    //14.5/180*pi

//#define PRINT_DEBUG_INFO
//#define TIMING_TEST

//#define DBSCAN_METHOD_RNG_ANG

#define ANGLE_FFT_LEN 128
#define ANGLE_FFT_LEN2 (ANGLE_FFT_LEN/2)
#define ANGLE_FFT_COEFF 0.0156// =2/N c/(128*d*fc)=0.0156; c/(64*d*fc)=0.0313; c/(32*d*fc)=0.0625; c/(16*d*fc)=0.0125  
#define MAX_ANG_FFT_PKS 50
#define MIN_ANG_FFT_POW 5
#define ANG_FFT_PK_SEARCH_LEN 10  //one-side
#define EN_DOP_COMP
#define MIN_ANG_FFT_PK_DISTANCE 29 

//#define ANGLE_TESTMODE  

typedef struct point_t
 {
    uint8  valid; 
    uint32  peak;  
    uint16  rng_idx; 
    uint16  dop_idx; 
    float  hori;  // angle
	uint16 frame_cnt;
	int cluster_id;
	float x;
	float y;
} OBJ_TypeDef;

typedef struct
{
    uint8  valid;
    uint16  rngIdx;
    uint16  dopIdx;
    uint32  powerSignal;
    float  snrSignal;
} OBJ_MAXPEAK_TypeDef;


/*********************************************************
Function name: procCounting
Description:   perform people counting
Paramater:     ThFac_rng    : Range CFAR coefficient
               ThFac_dop    : Doppler CFAR coefficient
Return:        void
*********************************************************/
void procCounting(float ThFac_rng, float ThFac_dop, uint32 thNoCfar);

/*********************************************************
Function name: detectPeaks
Description:   peak detection
Paramater:     resultN      : num of peaks detected previously
               frame_cnt    : frame count of current frame
               ThFac_rng    : Range CFAR coefficient
               ThFac_dop    : Doppler CFAR coefficient
Return:        total number of detected peaks (including previously detected)
*********************************************************/
uint16 detectPeaks(uint16 resultN, uint16 frame_cnt, float ThFac_rng, float ThFac_dop, uint32 thNoCfar);
void detectMaxPeak(OBJ_MAXPEAK_TypeDef *obj);
void complexMultiply(int16 *outR, int16 *outI, int16 inR1, int16 inI1,float inR2,float inI2);
int cmpfunc_uint32(const void * a, const void * b); 
void dopplerSwapMimo(int32 dopFftLen, int32 rngMax, uint32 absMergeArray[][CHIRP_NUM2]);
uint8 doAngFft(float *ang1, float *ang2, float *ang3, float *angByFft1, float *angByFft2, uint16 rngIdx, uint16 dopIdx, int16 iPeak1, int16 qPeak1, int16 iPeak2, int16 qPeak2, int16 iPeak3, int16 qPeak3, int16 iPeak4, int16 qPeak4);
#endif

