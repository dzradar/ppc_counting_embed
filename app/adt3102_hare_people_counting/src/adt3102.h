#ifndef __ADT3102__H
#define __ADT3102__H
#include "adt3102_adc.h"
//<<< Use Configuration Wizard in Context Menu >>>

//<s>Define Adt3102 Config File Version
//  <i>version
#define ADT3102_VERSION "1.0.0"

//--Debug define e.g.,printf, output result, analog IF sign enbuffer
//<q.0> Enable debug mode
#define DEBUGMODE 1

//--High clock
#define HIGH_CLOCK_50M  1
#define HIGH_CLOCK_125M 0

#define UART0_PRINTF
//#define UART1_PRINTF

#define HIGH_CLOCK HIGH_CLOCK_125M

//---Tx
//<e>FMCW config
// <o>FSTART
//   <i>Default: 76G(Uint:GHz)
//   <76-80>
#define FSTART    76
// <o>FM
//   <i>Default: 2000(Uint:MHz)
//   <0-4000>
#define FM        4000
#define LOOP_FILTER_1M
// <o>CHIRP_NUM
//   <i>Default: 32
//   <0-256>
#define CHIRP_NUM         64 
#define CHIRP_NUM_LOG2    6 
#define CHIRP_NUM2        (CHIRP_NUM/2) 
#define CHIRP_NUM2_LOG2   5 

#define CHIRP_T0          96
#define CHIRP_T1          (120+216)//492//120 //336//120//12
//#define CHIRP_T1          (120)//492//120 //336//120//12
#define CHIRP_T2          0//696
#define CHIRP_PERIOD      (CHIRP_T0+CHIRP_T1+CHIRP_T2)
#define MIMO              1          // 1: MIMO enable
#define PA1_ONLY          0

#define ADC_SAMPLE_RATE   ADC_8P3M       //ADC_16P6M: 16.667mhz.   ADC_8P3M: 8.333mhz
#define REMOVE_FIRST_CHIRP 1 //2db better
#define RANGE_MIN         10 //must be >= 1
#define RANGE_MAX         128 
#define DOWN_SAMPLE       3  //actual down sample rate = 1/(DOWN_SAMPLE+1).
#define SAMPLE_POINT      128 // 512 
#define RANGE_STEP        0.0586//0.1172//0.0586 //5.86 
//#define FD_STEP           (3.6169e-05) //delta_fd = 1/32/(96+120+216)/2
#define SPEED_STEP        0.07 //delta_fd*lambda/2
#define WN_BYPASS         0
#define FRAME_PERIOD      150//us
#define SAGC_EN           1
#define TF_ENABLE         0
#define FRAME_LOWPOWER    0// 1
#define CHIRP_LOWPOWER    0
#define PRINT_DEBUG_EN    0
#define BG_REMOVAL_ENABLE 0
#define LOG2_NFFT_VEL     5 //6
#define NFFT_VEL          (CHIRP_NUM/2)  //divid 2 for mimo

//memory address
#define DATA_MOVE_ADDR            0x20018000

//#define RANGE_ADDR_MAX              64  //no less than RANGE_MAX

#define RNG_FFT_CH0_ADDR            0x20008000                                         
#define RNG_FFT_CH1_ADDR            (RNG_FFT_CH0_ADDR + SAMPLE_POINT*CHIRP_NUM*4)   //0x20010000

#define RNG_FFT_CH0_ODD_ADDR      RNG_FFT_CH0_ADDR                                  //[0x20008000-0x20010000]
#define RNG_FFT_CH0_EVEN_ADDR     (RNG_FFT_CH0_ODD_ADDR  + SAMPLE_POINT*CHIRP_NUM*2)//[0x20010000-0x20018000]
#define RNG_FFT_CH1_ODD_ADDR 	  (RNG_FFT_CH0_EVEN_ADDR + SAMPLE_POINT*CHIRP_NUM*2)//[0x20018000-0x20020000]
#define RNG_FFT_CH1_EVEN_ADDR 	  (RNG_FFT_CH1_ODD_ADDR  + SAMPLE_POINT*CHIRP_NUM*2)//[0x20020000-0x20028000]

#define DOP_FFT_CH0_ODD_ADDR      0x20018000	//[0x20028000-0x2002C000]  16KB(0x4000)
#define DOP_FFT_CH0_EVEN_ADDR	  0x20008000	//[0x2002c000-0x20030000]  16KB(0x4000)
#define DOP_FFT_CH1_ODD_ADDR	  0x2000c000	//[0x20008000-0x2000C000]  16KB(0x4000)
#define DOP_FFT_CH1_EVEN_ADDR 	  0x20010000	//[0x2000c000-0x20010000]  16KB(0x4000)

//#define ABS_CH0_ODD_ADDR     	  0x20014000  //-0x20013fff  16KB
//#define ABS_CH0_EVEN_ADDR		  0x20018000  //-0x20018fff  16KB
#define ABS_CH1_ODD_ADDR  		  0x20014000  //-0x2001cfff  16KB
#define ABS_CH1_EVEN_ADDR 		  0x2001c000  //-0x2001ffff  16KB

#define ABS_MERGE_ADDR            0x20020000  //-0x20030000  16KB 

#define BG_REMOVAL_CH1_ADD    (0x2001c000 + SAMPLE_POINT*4)   // 0x2002A000+128*4 +128*4  
#define BG_REMOVAL_CH2_ADD    (BG_REMOVAL_CH1_ADD + SAMPLE_POINT*4)   // 0x2002A000+128*4 +128*4*2  
#define BG_REMOVAL_CH3_ADD    (BG_REMOVAL_CH2_ADD + SAMPLE_POINT*4)   // 0x2002A000+128*4 +128*4*3  
#define BG_REMOVAL_CH4_ADD    (BG_REMOVAL_CH3_ADD + SAMPLE_POINT*4)   // 0x2002A000+128*4 +128*4*4  

//#define RNG_FFT_CH0_ADDR  (RNG_FFT_CH1_ADDR + SAMPLE_POINT*CHIRP_NUM*4) //start 0x20018000,size 0x10000
//#define DOP_FFT_CH1_ADDR  (RNG_FFT_CH0_ADDR + SAMPLE_POINT*CHIRP_NUM*4) //start 0x20028000,size RANGE_ADDR_MAX*CHIRP_NUM*4=0x4000
//#define DOP_FFT_CH0_ADDR  (DOP_FFT_CH1_ADDR + RANGE_ADDR_MAX*CHIRP_NUM*4)    //start 0x2002C000,size RANGE_ADDR_MAX*CHIRP_NUM*4=0x4000
//#define ABS_CH1_ADDR      RNG_FFT_CH1_ADDR                              //start 0x20008000,size RANGE_ADDR_MAX*CHIRP_NUM*4=0x4000
//#define ABS_CH0_ADDR      (ABS_CH1_ADDR+RANGE_ADDR_MAX*CHIRP_NUM*4)   //start 0x2000C000,size RANGE_ADDR_MAX*CHIRP_NUM*4=0x4000
//#define ABS_MERGE_ADDR    (ABS_CH0_ADDR+RANGE_ADDR_MAX*CHIRP_NUM*4)   //start 0x20010000,size RANGE_ADDR_MAX*CHIRP_NUM*4=0x4000

//</e>
//---Rx

//---Adc
#define FFT_BUF_HF1 0x20030000         //FFT buffer start address input for 1st half
#define FFT_BUF_HF2 0x20030000 + 256*4 //FFT buffer start address input for 2nd half
#if (ADC_SAMPLE_RATE == ADC_16P6M)
  #define SAMPLE_END_NUM  ceil((CHIRP_T1+CHIRP_T2)*16.666)
  #define INTERVALNUM     (CHIRP_T0+CHIRP_T1+CHIRP_T2)*1000/60
#else
  #define SAMPLE_END_NUM  ceil((CHIRP_T1+CHIRP_T2)*8.333)
  #define INTERVALNUM     (CHIRP_T0+CHIRP_T1+CHIRP_T2)*1000/120
#endif
//512 1
//#define FIRREG0  ((0<<13)     +0x1fff)
//#define FIRREG1  ((1<<13)     +0x1)
//#define FIRREG2  ((0x1ffd<<13)+0x1fff)
//#define FIRREG3  ((5<<13)     +0x0)
//#define FIRREG4  ((0x1ffa<<13)+0x3)
//#define FIRREG5  ((0x4<<13)   +0x1ff7)
//#define FIRREG6  ((0x3<<13)   +0x10)
//#define FIRREG7  ((0x1fef<<13)+0x1fea)
//#define FIRREG8  ((0x25<<13)  +0x16)
//#define FIRREG9  ((0x1fc6<<13)+0x1ff6)
//#define FIRREG10 ((0x46<<13)  +0x1fee)
//#define FIRREG11 ((0x1fc2<<13)+0x40)
//#define FIRREG12 ((0x11<<13)  +0x1f83)
//#define FIRREG13 ((0x58<<13)  +0xc0)
//#define FIRREG14 ((0x1ec2<<13)+0x1f03)
//#define FIRREG15 ((0x4ef<<13) +0x129)
//#define FC       (0x6c8)

// Dn = 2
#define FIRREG0   0x1FFF
#define FIRREG1   0x2001
#define FIRREG2   0x3FFBFFF
#define FIRREG3   0xA000
#define FIRREG4   0x3FF4003
#define FIRREG5   0x9FF7
#define FIRREG6   0x6010
#define FIRREG7   0x3FDFFEA
#define FIRREG8   0x4A016
#define FIRREG9   0x3F8DFF6
#define FIRREG10  0x8DFEE
#define FIRREG11  0x3F84040
#define FIRREG12  0x23F83
#define FIRREG13  0xB00C0
#define FIRREG14  0x3D85F03
#define FIRREG15  0x9DE129
#define FIRREG16  0x6C8

//#define FIRREG0   0x0
//#define FIRREG1   0x0
//#define FIRREG2   0x0
//#define FIRREG3   0x0
//#define FIRREG4   0x0
//#define FIRREG5   0x0
//#define FIRREG6   0x0
//#define FIRREG7   0x0
//#define FIRREG8   0x0
//#define FIRREG9   0x0
//#define FIRREG10  0x0
//#define FIRREG11  0x0
//#define FIRREG12  0x0
//#define FIRREG13  0x0
//#define FIRREG14  0x0
//#define FIRREG15  0x0
//#define FIRREG16  0xfff


#define FC        (FIRREG16 | (DOWN_SAMPLE << fir_cof_sample_rate_down_sample_rate_shift))


//---FFT

//---Cfar
#define CFAR_ADDR 0x40024000
#endif
