//--------------------------------------------------------------------
//Copyright(c)2020,Andar Technologise Inc.
//All Rights Reserved
//Confidential Property of Andar Technologies Inc.
//
//Module Description: People Counting for ADT3102 Hare DEV board
//
//Created by :JiangQi
//$Revision: 0.1
//$Data: 2021/3/25
//--------------------------------------------------------------------
//
//------All include header files
#include "CM3DS_MPS2.h"
#include "stdio.h"
#include "math.h"
#include "adt3102.h"
#include "adt3102_system.h"
#include "adt3102_tx.h"
#include "adt3102_rx.h"
#include "adt3102_pmu.h"
#include "adt3102_timer.h"
#include "adt3102_dsp.h"
#include "adt3102_gpio.h"
#include "adt3102_memscheduling.h"
#include "adt3102_phase_cal.h"
#include "adt3102_type_define.h"
#include "adt3102_uart.h"
#include "adt3102_system.h"
#include "adt3102_rtc.h"
#include "adt3102_sagc.h"
#include "adt3102_trng.h"
#include "adt3102_watchdog.h"
#include "pmu_ctype_map.h"
#include "rfc_ctype_map.h"
#include "dsp_ctype_map.h"
#include "rtc_ctype_map.h"
#include "static_remove.h"
#include "peak_single.h"
#include "gpio_ctype_map.h"
#include "adt3102_uart.h"
#include "TinyFrame.h"
#include "adt3102_utilities.h"
#include "proc_counting.h"
#include "arm_math.h"
#include "stdlib.h"
#include "track.h"

//------Global variable declaration
int32 g_jlinkDataFlag __attribute__((at(0x20007800)));
int32 g_rx0En=1;
int32 g_rx1En=1;
int32 g_tx0En=1;
int32 g_tx1En=1;
uint32 g_tiaGain0=TIA_GAIN_6DB; //0,6,12,18
uint32 g_pgaGain0=PGA_GAIN_44DB;//PGA_GAIN_30DB;
uint32 g_tiaGain1=TIA_GAIN_6DB;
uint32 g_pgaGain1=PGA_GAIN_44DB;//PGA_GAIN_30DB;
uint32 g_tiaHpfEn=0;
uint32 g_rcHpfEn=1;
uint32 g_pgaHpfEn=1;
uint32 g_tiaHpf=TIA_HPF_100KHZ; //25,50,100,300
uint32 g_rcHpf=RC_HPF_150KHZ;   //150, 400
uint32 g_pgaHpf=PGA_HPF_100KHZ; //25,50,100,500
uint32 g_pgaLpf=PGA_LPF_4M; //PGA_LPF_2M; 
uint32 g_PaGain0=14;
uint32 g_PaGain1=14;
uint32 g_phaseShftLut0=0;
uint32 g_phaseShftLut1=0;
uint32 g_absMergeArray[RANGE_MAX][CHIRP_NUM2] __attribute__((at(ABS_MERGE_ADDR)));

#if REMOVE_FIRST_CHIRP
uint32 g_discardChirpReserve0[SAMPLE_POINT] __attribute__((at(RNG_FFT_CH0_ADDR-SAMPLE_POINT*4)));
uint32 g_discardChirpReserve1[SAMPLE_POINT] __attribute__((at(RNG_FFT_CH1_ADDR-SAMPLE_POINT*4))); //overlap with end of ch0
#endif

Raw_Target_List target_list __attribute__((at(TARGET_LIST_ADDR)));

volatile int32 g_sendCount = 0;

volatile uint32 g_frameStart=0;  


uint8 initial_flag = 1;
float iqcoef = 0.001f;
uint8 update_flag = 1;
uint8 bg_estimate_time = 10; //100 -> 5s

uint32_t work_times = 0;
int main(void)
{
    //const char VER[4]="v1.0";
    //-----Local variable declaration-----
    //float powerThd= 0.01;//1;//100 no center point;//breath 10;//0.5 no person
    float powerThd= 0.05;//breath 0.05;//0.03 no person
    uint32 powerThdNoCfar;
//    uint32 dopMin=0;
//    uint32 firstPeakOnly=0;
//    uint32 distComp=0;
//    uint16 frameCnt=0;
    uint32 curTiaGain;
    uint32 curVgaGain;
//    OBJ_PEAKSINGLE_TypeDef obj;	
//   uint16 y,x;
//    int16 qPeakCh0,iPeakCh0,qPeakCh1,iPeakCh1;	
    uint32 timePrint;
//    PHASE_TypeDef phaseResult; 
    uint32 sagcTarget;     
//    multTargetInfo myMultTargetInfo;
//    myMultTargetInfo.targetNum = 1; 
    MY_NVIC_SetVectorTable(0x00000000, 0x8000);
    
//	float ThFac_rng = 0;    //CFAR thr factor
//	float ThFac_dop = 0;


//   int32 powerThdAdj;
//    uint32 dopMin=0;
//    uint32 firstPeakOnly=0;
 //   uint32 distComp=0;

    
   // OBJ_MAXPEAK_TypeDef obj;	
   // PHASE_TypeDef phaseResult1, phaseResult2, phaseResult3; 

    //-----Initialize some basic functions -----
    setInterrupt(INT_UART1, ENABLEINT);
    setInterrupt(INT_TIMER0, ENABLEINT);
    setInterrupt(INT_TIMER1, ENABLEINT);
    setInterrupt(INT_DUALTIMER, ENABLEINT);
    setInterrupt(INT_FFT, ENABLEINT);
    setInterrupt(INT_PMU, ENABLEINT);
    //LED init
    ledInit();
    //-----init some module-----
    //power on
    adt3102PowerSw(ON); 
    //turn on RF and analog, config adc   
    adt3102DigiInit();
	#if REMOVE_FIRST_CHIRP
        adt3102RfInit(g_rx0En, g_rx1En, g_tx0En, g_tx1En, g_phaseShftLut0, g_phaseShftLut1,CHIRP_NUM+1);
	#else
        adt3102RfInit(g_rx0En, g_rx1En, g_tx0En, g_tx1En, g_phaseShftLut0, g_phaseShftLut1,CHIRP_NUM);
	#endif
    adt3102Analog(); 
    #if REMOVE_FIRST_CHIRP
        adt3102AdcSetting(g_rx0En, g_rx1En,1,ADC_SAMPLE_RATE,CHIRP_NUM+1);
	#else
        adt3102AdcSetting(g_rx0En, g_rx1En,1,ADC_SAMPLE_RATE,CHIRP_NUM);
	#endif    
    if(HIGH_CLOCK==HIGH_CLOCK_125M)
    {
        switch125MHclk();
    }
    uartInit(UART0,1382400, HIGH_CLOCK); 
    //printfDebug("ADT3102 people counting %s \r\n", VER);
    timerInit(Timer0,(float)FRAME_PERIOD,HIGH_CLOCK);
    gpioDirection(9,0);
    gpioDirection(2,0);   
    //single tone
    //fmcwChirpSingleFrequency(); 
    //bistOn(BIST_AMP, BIST_DIV128_SEL);
    //siggen(0x1000, 0x0000, 0, 1, 0); DSP->ds_adc_format_ctrl = ds_adc_format_ctrl_offset_auto_bit;//uint16 step, uint16 chgStep, uint16 comp, uint16 siggenEn, uint16 ang

    //int16 qPeak1,iPeak1,qPeak2,iPeak2, qPeak3,iPeak3,qPeak4,iPeak4;		
#ifdef ANGLE_TESTMODE  
    uint8 numAngByFft = 0;
    float angByFft1 = 0;
    float angByFft2 = 0;
    float ang1 = 0;
    float ang2 = 0;
    float ang3 = 0;
		   uint16 y,x;
#endif
    
    target_list.magicword[0] = 0x0102;
    target_list.magicword[1] = 0x0304;
    target_list.magicword[2] = 0x0506;
    target_list.magicword[3] = 0x0708;
    target_list.framenumber = 1;
    

    tr_obj_list.magicword[0] = 0x0807;
    tr_obj_list.magicword[1] = 0x0605;
    tr_obj_list.magicword[2] = 0x0403;
    tr_obj_list.magicword[3] = 0x0201;
    tr_obj_list.framenumber = 1;
    
    func_track_init();
	while(1)//repeat sending chirps
    {
        //Frame rate control using timer0
        //while(g_frameStart==0);
        //g_frameStart=0;
        //timerInit(Timer0,(float)FRAME_PERIOD,HIGH_CLOCK);
        #if FRAME_LOWPOWER
            lowPowerSwitchPll(1,1);
            lowPowerSwitchFast(g_rx0En, g_rx1En, g_tx0En, g_tx1En, g_tiaHpf,g_rcHpf,g_pgaHpf);                
			//rfPhaseShiftLUT(g_phaseShftLut0, g_phaseShftLut1);
        #endif
			//frame rate test
        timePrint=(uint32)(10000000-Timer0->VALUE/125);
        printfDebug("Frame time %d \r\n", timePrint);
        timerOff(Timer0);
        timerInit(Timer0,(float)10000000,HIGH_CLOCK);
        if (PA1_ONLY==1)
        {
            //enable PA0 and disable PA1
            txPaEnable(DISABLE, ENABLE);
            RFC->rf_buf77g_ctrl_reg &= ~(rf_buf77g_ctrl_reg_buff77g_en_tx0_bit);//tx0 buff disable            
            RFC->rf_buf77g_ctrl_reg |= rf_buf77g_ctrl_reg_buff77g_en_tx1_bit;   //tx1 buff enable
        }
        else
        {
            //enable PA1 and disable PA0
            txPaEnable(ENABLE, DISABLE);
            RFC->rf_buf77g_ctrl_reg |= rf_buf77g_ctrl_reg_buff77g_en_tx0_bit;   //tx0 buff enable            
            RFC->rf_buf77g_ctrl_reg &= ~rf_buf77g_ctrl_reg_buff77g_en_tx1_bit;//tx1 buff disable
        } 
        #if REMOVE_FIRST_CHIRP
            g_sendCount=-1;
            timerInit(Timer1,CHIRP_PERIOD,HIGH_CLOCK);
            adcSampleStart(ADC01SEL, SAMPLE_POINT, CHIRP_NUM+1, ADC0_FIFO, RNG_FFT_CH0_ADDR-SAMPLE_POINT*4, ADC1_FIFO, RNG_FFT_CH1_ADDR-SAMPLE_POINT*4);   
        #else
            g_sendCount=0;
            timerInit(Timer1,CHIRP_PERIOD,HIGH_CLOCK);
            adcSampleStart(ADC01SEL, SAMPLE_POINT, CHIRP_NUM, ADC0_FIFO, RNG_FFT_CH0_ADDR, ADC1_FIFO, RNG_FFT_CH1_ADDR); 
        #endif
        timerOff(Timer1);
        //timePrint=(uint32)(100000-Timer1->VALUE/125);
        //printfDebug("Sample done %d \r\n", timePrint);
        #if FRAME_LOWPOWER
            lowPowerSwitchPll(1,0);
            lowPowerSwitchFast(0, 0, 0, 0, g_tiaHpf,g_rcHpf,g_pgaHpf);
        #endif
        
        
        curTiaGain=g_tiaGain1;
        curVgaGain=g_pgaGain1;
        #if SAGC_EN
            #if WN_BYPASS
                sagcTarget=800;
            #else
                sagcTarget=500;
            #endif  
            sAgc(0,RNG_FFT_CH1_ODD_ADDR, SAMPLE_POINT*8,sagcTarget,&g_tiaGain1,&g_pgaGain1);
            g_tiaGain0=g_tiaGain1;
            g_pgaGain0=g_pgaGain1;
            rfTiaGainSetting(g_tiaGain0,g_tiaGain0,g_tiaGain1,g_tiaGain1);
            rfVgaGainSetting(g_pgaGain0,g_pgaGain0,g_pgaGain1,g_pgaGain1);  
        #endif
        #if TF_ENABLE
            tinyFramefTx(0x0B04, (uint8*)RNG_FFT_CH0_ODD_ADDR, SAMPLE_POINT*4);
            tinyFramefTx(0x0B05, (uint8*)RNG_FFT_CH1_EVEN_ADDR, SAMPLE_POINT*4);
        #endif
        
        //mimo data moving
        for(int i=0; i<CHIRP_NUM2; i++)
        {
            memScheduling(RNG_FFT_CH0_ODD_ADDR+(i*2+1)*SAMPLE_POINT*4, DATA_MOVE_ADDR+i*SAMPLE_POINT*4, 4, 4, SAMPLE_POINT);
        }
        for(int i=0; i<CHIRP_NUM2; i++)
        {
            memScheduling(RNG_FFT_CH0_ODD_ADDR+(i*2)*SAMPLE_POINT*4, RNG_FFT_CH0_ODD_ADDR+i*SAMPLE_POINT*4, 4, 4, SAMPLE_POINT);
        }
        for(int i=0; i<CHIRP_NUM2; i++)
        {
            memScheduling(DATA_MOVE_ADDR+i*SAMPLE_POINT*4, RNG_FFT_CH0_EVEN_ADDR+i*SAMPLE_POINT*4, 4, 4, SAMPLE_POINT);
        }
        for(int i=0; i<CHIRP_NUM2; i++)
        {
            memScheduling(RNG_FFT_CH1_ODD_ADDR+(i*2+1)*SAMPLE_POINT*4, DATA_MOVE_ADDR+i*SAMPLE_POINT*4, 4, 4, SAMPLE_POINT);
        }
        for(int i=0; i<CHIRP_NUM2; i++)
        {
            memScheduling(RNG_FFT_CH1_ODD_ADDR+(i*2)*SAMPLE_POINT*4, RNG_FFT_CH1_ODD_ADDR+i*SAMPLE_POINT*4, 4, 4, SAMPLE_POINT);
        }
        for(int i=0; i<CHIRP_NUM2; i++)
        {
            memScheduling(DATA_MOVE_ADDR+i*SAMPLE_POINT*4,RNG_FFT_CH1_EVEN_ADDR+i*SAMPLE_POINT*4, 4, 4, SAMPLE_POINT);
        }

        //timePrint=(uint32)(100000-Timer1->VALUE/125);
        //printfDebug("AGC done %d \r\n", timePrint);
        //save ..\matlab\adc_data_ch1.hex  0x20008000,0x20017fff;save ..\matlab\adc_data_ch0.hex   0x20018000,0x20027fff
        fftProcess(SAMPLE_POINT, CHIRP_NUM2, RNG_FFT_CH0_ODD_ADDR ,RNG_FFT_CH0_ODD_ADDR , 4, 4);//0x20008000
        fftProcess(SAMPLE_POINT, CHIRP_NUM2, RNG_FFT_CH0_EVEN_ADDR,RNG_FFT_CH0_EVEN_ADDR, 4, 4);//0x20010000
        fftProcess(SAMPLE_POINT, CHIRP_NUM2, RNG_FFT_CH1_ODD_ADDR ,RNG_FFT_CH1_ODD_ADDR , 4, 4);//0x20018000
        fftProcess(SAMPLE_POINT, CHIRP_NUM2, RNG_FFT_CH1_EVEN_ADDR,RNG_FFT_CH1_EVEN_ADDR, 4, 4);//0x20020000
        //timePrint=(uint32)(100000-Timer1->VALUE/125);
        //printfDebug("Range FFT done %d \r\n", timePrint);
        #if TF_ENABLE
            tinyFramefTx(0x0C00, (uint8*)RNG_FFT_CH0_ODD_ADDR, SAMPLE_POINT*4);
            tinyFramefTx(0x0D00, (uint8*)RNG_FFT_CH1_EVEN_ADDR, SAMPLE_POINT*4);
        #endif        
        //save ..\matlab\range_fft_data_ch0.hex  0x20008000,0x20017fff;save ..\matlab\range_fft_data_ch1.hex  0x20018000,0x20027fff
        //staticRemove(RNG_FFT_CH0_ODD_ADDR ,SAMPLE_POINT,RANGE_MAX,CHIRP_NUM, CHIRP_NUM2_LOG2); 
        //staticRemove(RNG_FFT_CH0_EVEN_ADDR,SAMPLE_POINT,RANGE_MAX,CHIRP_NUM, CHIRP_NUM2_LOG2);
        //staticRemove(RNG_FFT_CH1_ODD_ADDR ,SAMPLE_POINT,RANGE_MAX,CHIRP_NUM, CHIRP_NUM2_LOG2); 
        //staticRemove(RNG_FFT_CH1_EVEN_ADDR,SAMPLE_POINT,RANGE_MAX,CHIRP_NUM, CHIRP_NUM2_LOG2);
        
        #if BG_REMOVAL_ENABLE
        if (initial_flag == 1)
        {
            initial_flag++;
            // initial dc removal data
            for(int i= 0; i<RANGE_MAX; i++)
            {
                *((int32 *)(BG_REMOVAL_CH1_ADD+ i*4)) = *((int32 *)(RNG_FFT_CH0_ODD_ADDR  + i*4));
                *((int32 *)(BG_REMOVAL_CH2_ADD+ i*4)) = *((int32 *)(RNG_FFT_CH0_EVEN_ADDR  + i*4));
                *((int32 *)(BG_REMOVAL_CH3_ADD+ i*4)) = *((int32 *)(RNG_FFT_CH1_ODD_ADDR  + i*4));
                *((int32 *)(BG_REMOVAL_CH4_ADD+ i*4)) = *((int32 *)(RNG_FFT_CH1_EVEN_ADDR  + i*4));
            }
        }
        else if (initial_flag < bg_estimate_time)
        {
            // 10s big coeff
            initial_flag++;
            iqcoef = 0.3f;
        }
        else if (initial_flag == bg_estimate_time)
        {
            // small coeff
            initial_flag++;
            iqcoef = 0.05f;
        }
        else
        {
            update_flag = 0;
            iqcoef = 0.05f;
        }
        background_removal(RNG_FFT_CH0_ODD_ADDR , BG_REMOVAL_CH1_ADD, SAMPLE_POINT,SAMPLE_POINT, NFFT_VEL,LOG2_NFFT_VEL, iqcoef, update_flag);
        background_removal(RNG_FFT_CH0_EVEN_ADDR, BG_REMOVAL_CH2_ADD, SAMPLE_POINT,SAMPLE_POINT, NFFT_VEL,LOG2_NFFT_VEL, iqcoef, update_flag);
        background_removal(RNG_FFT_CH1_ODD_ADDR , BG_REMOVAL_CH3_ADD, SAMPLE_POINT,SAMPLE_POINT, NFFT_VEL,LOG2_NFFT_VEL, iqcoef, update_flag);
        background_removal(RNG_FFT_CH1_EVEN_ADDR, BG_REMOVAL_CH4_ADD, SAMPLE_POINT,SAMPLE_POINT, NFFT_VEL,LOG2_NFFT_VEL, iqcoef, update_flag);
        #endif
        
        
        velocityFftProcess(CHIRP_NUM2, RANGE_MAX, RNG_FFT_CH0_ODD_ADDR ,DOP_FFT_CH0_ODD_ADDR , 4*SAMPLE_POINT, 4);
        velocityFftProcess(CHIRP_NUM2, RANGE_MAX, RNG_FFT_CH0_EVEN_ADDR,DOP_FFT_CH0_EVEN_ADDR, 4*SAMPLE_POINT, 4);
        velocityFftProcess(CHIRP_NUM2, RANGE_MAX, RNG_FFT_CH1_ODD_ADDR ,DOP_FFT_CH1_ODD_ADDR , 4*SAMPLE_POINT, 4);
        velocityFftProcess(CHIRP_NUM2, RANGE_MAX, RNG_FFT_CH1_EVEN_ADDR,DOP_FFT_CH1_EVEN_ADDR, 4*SAMPLE_POINT, 4); 
        //save ..\matlab\velocity_data_ch1.hex  0x20028000,0x2002bfff;save ..\matlab\velocity_data_ch0.hex  0x2002c000,0x2002ffff
        //timePrint=(uint32)(100000-Timer1->VALUE/125);
        //printfDebug("Velocity FFT done %d \r\n", timePrint);
        
        //abs
        //absAdt3102(CHIRP_NUM2, RANGE_MAX, DOP_FFT_CH0_ODD_ADDR , ABS_CH0_ODD_ADDR );
        //absAdt3102(CHIRP_NUM2, RANGE_MAX, DOP_FFT_CH0_EVEN_ADDR, ABS_CH0_EVEN_ADDR);
        absAdt3102(CHIRP_NUM2, RANGE_MAX, DOP_FFT_CH1_ODD_ADDR , ABS_CH1_ODD_ADDR );
        absAdt3102(CHIRP_NUM2, RANGE_MAX, DOP_FFT_CH1_EVEN_ADDR, ABS_CH1_EVEN_ADDR); 
        //save ..\matlab\abs_merge_data_ceil_ch1_enen.hex 0x20010000,0x20020000
            

        //Average the results of Velocity_FFT of ADC0 and ADC1
 //       uint32 absCh00 = 0;
 //       uint32 absCh01 = 0; 
        uint32 absCh10 = 0;
        uint32 absCh11 = 0;
        for(int i =0;i<RANGE_MAX*CHIRP_NUM2;i=i+1)        
        {
            //uint32 absCh00, absCh01, absCh10, absCh11;
//            absCh00  = *((uint32 *)(ABS_CH0_ODD_ADDR  + i*4));
 //           absCh01  = *((uint32 *)(ABS_CH0_EVEN_ADDR  + i*4));
            absCh10  = *((uint32 *)(ABS_CH1_ODD_ADDR  + i*4));
            absCh11  = *((uint32 *)(ABS_CH1_EVEN_ADDR  + i*4));
            *((uint32 *)(ABS_MERGE_ADDR+ i*4))= (absCh10>>1) + (absCh11>>1);
            //*((uint32 *)(ABS_MERGE_ADDR+ i*4))= absCh00 + absCh01 + absCh10 + absCh11;
        }       
        
       
         //save ..\matlab\abs_merge_data.hex 0x20020000,0x20024000
         //save ..\matlab\abs_merge_data.hex 0x20010000,0x2001bfff
        dopplerSwapMimo(CHIRP_NUM2, RANGE_MAX, g_absMergeArray);
        //powerThdAdj=powerThd+(curTiaGain*6+curVgaGain*2-60);//powerThd is based on IFgain = 50db, if IFgain changed, thd changes automatically.
        //peakSingle(CHIRP_NUM, dopMin, RANGE_MIN, RANGE_MAX, g_absMergeArray, powerThdAdj, firstPeakOnly, distComp, &obj);
        //timePrint=(uint32)(100000-Timer1->VALUE/125);
        //printfDebug("Peak seach done %d \r\n", timePrint);
        
      #ifdef ANGLE_TESTMODE  
        
        detectMaxPeak(&obj);  //NOTE: static tgt NOT removed
        
        if(obj.valid==1)
        {
            y = obj.rngIdx;
	        x = obj.dopIdx < CHIRP_NUM2/2 ? obj.dopIdx + CHIRP_NUM2/2 : obj.dopIdx - CHIRP_NUM2/2;
            
	        //x = obj.dopIdx;
            iPeak1   = (*( ((int16 *)DOP_FFT_CH0_ODD_ADDR)  + (y*CHIRP_NUM2+x)*2 ));
	        qPeak1   = (*( ((int16 *)DOP_FFT_CH0_ODD_ADDR)  + (y*CHIRP_NUM2+x)*2 + 1 ));																						
	        iPeak2   = (*( ((int16 *)DOP_FFT_CH1_ODD_ADDR)  + (y*CHIRP_NUM2+x)*2));
	        qPeak2   = (*( ((int16 *)DOP_FFT_CH1_ODD_ADDR)  + (y*CHIRP_NUM2+x)*2 + 1 ));
            iPeak3   = (*( ((int16 *)DOP_FFT_CH0_EVEN_ADDR)  + (y*CHIRP_NUM2+x)*2 ));
	        qPeak3   = (*( ((int16 *)DOP_FFT_CH0_EVEN_ADDR)  + (y*CHIRP_NUM2+x)*2 + 1 ));																						
	        iPeak4   = (*( ((int16 *)DOP_FFT_CH1_EVEN_ADDR)  + (y*CHIRP_NUM2+x)*2));
	        qPeak4   = (*( ((int16 *)DOP_FFT_CH1_EVEN_ADDR)  + (y*CHIRP_NUM2+x)*2 + 1 ));
                       
            numAngByFft = doAngFft(&ang1, &ang2, &ang3, &angByFft1, &angByFft2, obj.rngIdx, obj.dopIdx, iPeak1,qPeak1,iPeak2,qPeak2, iPeak3,qPeak3,iPeak4,qPeak4);
            
            //printfDebug("rngIdx : %d , dopIdx :  %d, pow :  %d,  angle1 :  %.1f,  angle2 :  %.1f,  angle3 :  %.1f, numOfAngFFTPeaks : %d \r\n", obj.rngIdx, obj.dopIdx-CHIRP_NUM2/2, obj.powerSignal, ang1, ang2, ang3, numAngByFft);
            if(numAngByFft == 1 || numAngByFft == 2)
            {
               // printfDebug("numAngByFft1 : %.1f \r\n", angByFft1);
                if(numAngByFft == 2)
                {
                    //printfDebug("numAngByFft2 : %.1f \r\n", angByFft2);
                }
            }
            else if(numAngByFft == 0)
            {
                printfDebug("numAngByFft is 0!");
            }
            else
            {
                printfDebug("numAngByFft is WRONG");
            }
            
       
			//float powerDbDisp=obj.powerSignal-(int32)(curTiaGain*6+curVgaGain*2-60);
            //printfTf("tiaGain %d ,vgaGain %d, total gain %d.  Distance: %.3f, Speed: %.3f, Power(dBFS): %.0f, Angle: %.1f", curTiaGain,curVgaGain,curTiaGain*6+curVgaGain*2,obj.rngIdx*RANGE_STEP/100,(obj.dopIdx-CHIRP_NUM/2)*SPEED_STEP/100,powerDbDisp,phaseResult.angle);
            //printfDebug("tiaGain %d ,vgaGain %d, total gain %d.  Distance: %.3f, Speed: %.3f, Power(dBFS): %.0f, Angle: %.1f \r\n", curTiaGain,curVgaGain,curTiaGain*6+curVgaGain*2,obj.rngIdx*RANGE_STEP/100,(obj.dopIdx-CHIRP_NUM/2)*SPEED_STEP/100,powerDbDisp,phaseResult.angle);
           // printfTf("tiaGain %d ,vgaGain %d, total gain %d.  Distance: %.3f m, Speed: %.3f m/s, Power(dBFS): %.0f, Angle: %.1f \r\n", curTiaGain,curVgaGain,curTiaGain*6+curVgaGain*2,obj.rngIdx*RANGE_STEP/100,(obj.dopIdx-CHIRP_NUM/2)*SPEED_STEP/100,powerDbDisp,phaseResult.angle);
            //printfTf("tiaGain %d ,vgaGain %d, total gain %d.  Distance: %.3f, Power(dBFS): %.0f", curTiaGain,curVgaGain,curTiaGain*6+curVgaGain*2,obj.rngIdx*RANGE_STEP/100,powerDbDisp);
            
        }
        #else
            powerThdNoCfar=round(powerThd*pow(10,(curTiaGain*6+curVgaGain*2)/10)); 
                 
			procCounting(0, 0, powerThdNoCfar);  //NOTE:  static tgts are removed
        
        // copy to txframe buffer 
        target_list.Ntargets = 0;  
        target_list.framenumber = work_times;        
        if (cdi_data.raw_number > 0) 
        {
            // cluster for output
            //target_list.framenumber = cdi_data.index;
            for(int i = 0; i <cdi_data.raw_number; i++)
            {
                // i is the cluster number i 0-4
                for (int p_clu = 0; p_clu < cdi_data.point_number[i]; p_clu ++)
                {
                    target_list.points[target_list.Ntargets + p_clu].r_flt32 = cdi_data.cdi[i].raw[p_clu].y;
                    target_list.points[target_list.Ntargets + p_clu].v_flt32 = cdi_data.cdi[i].raw[p_clu].hori;
                    target_list.points[target_list.Ntargets + p_clu].x_flt32 = cdi_data.cdi[i].raw[p_clu].x;
                    target_list.points[target_list.Ntargets + p_clu].y_flt32 = cdi_data.cdi[i].raw[p_clu].y;
                    target_list.points[target_list.Ntargets + p_clu].z_flt32 = cdi_data.cdi[i].raw[p_clu].id;
                    target_list.points[target_list.Ntargets + p_clu].snr_flt32 = 10;
                }                
                target_list.Ntargets += cdi_data.point_number[i];
            }
        }      
        
        // output clusters 
        if (target_list.Ntargets > 0)
        {
            // data length : 9 bytes TF + 16 header + num Points * 6 float  * 4 bytes + 2 CRC
            // example: 1 target output 9+16+16+2 = 43
            tinyFramefTx(0xFF02, (uint8*)(&target_list), (24*target_list.Ntargets+16));
        }
        // ------------
        // tracking
        track_step();

       
        tr_obj_list.framenumber = work_times;
        // output clusters 
        if (tr_obj_list.N_Objs > 0)
        {
            // data length : 9 bytes TF + 16 header + num Points * 6 float  * 4 bytes + 2 CRC
            // example: 1 target output 9+16+16+2 = 43
            tinyFramefTx(0xFF03, (uint8*)(&tr_obj_list), (32*tr_obj_list.N_Objs+16));
        }
        tr_obj_list.N_Objs = 0;
        work_times++;
        #endif
        
        //save ..\matlab\abs_merge_data.hex 0x20020000,0x20024000
//        //save ..\matlab\abs_merge_data.hex 0x20020000,0x20030000
//        dopplerSwap(CHIRP_NUM, RANGE_MAX, g_absMergeArray);
//        powerThdNoCfar=round(powerThd*pow(10,(curTiaGain*6+curVgaGain*2)/10));        
//        //timerOff(Timer1);
//        //timerInit(Timer1,(float)100000,HIGH_CLOCK);
//        procCounting(ThFac_rng, ThFac_dop, powerThdNoCfar);
//        //timePrint=(uint32)(100000-Timer1->VALUE/125);
 //       printfDebug("procCounting time %d \r\n", timePrint);
    } 
}
