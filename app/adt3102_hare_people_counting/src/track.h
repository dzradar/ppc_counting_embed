#ifndef	  _TRACK_H_
#define   _TRACK_H_

#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <proc_counting.h>

#define SYSTEM_USING_TRACK	/*use track*/
//#define SYSTEM_USING_CLUSTER_SE     /*print se text data*/
#define MAX_OBJ		4
#define MAX_ARRAY   8
#define THRESHOLD_OBJ_DISTANCE  0.5

#define ALPHA_BETA_FILTER 0.5  // 1 is very stable, 0 is very fluctuant

#define MAX_OUTPUT_TARGET  5   //max object output
#define MAX_CLUSTER_ID    128

#define MAX_TARGET_NUM 100

#pragma pack(2)

typedef struct{
	float x;
	float y;
	float hori;  // doppler index
	uint16 id;   // cluster id
	uint32 id_count;
}track_measu_t;

typedef struct{
	track_measu_t raw[50];  // points in cluster
}track_cdi_t;  /*cluster points*/

//cluster
typedef struct{
	uint32 index;		           		//frame nr
	uint32 raw_number;             //clusters  less than 4
	uint16  point_number[MAX_OBJ];        //points in cluster
	track_cdi_t cdi[MAX_OBJ];              //cluster info

}track_cdi_pkg_t;        

/*before track centor - target*/
typedef struct {
	float x;
	float y;
	float hori;         //3
	float  clu_w;      //4
	float clu_h;       //5
	uint8 match_flag;  //6
}current_centor_t;

typedef struct {
		float x;          //1
		float y;          //2
		float hori;       //3
		float id;		      //4	//id
		float cur_id;     // 5 current frame
		float w_obj;      //6
		float h_obj;      //7
}last_center_t;
		
typedef struct{
	int8 confident;				        //1. confidence invalid -1, inconfident 0, confident 1,
	int32 lifetime;			          //2.lifetime 
	last_center_t current_center; //3
	track_cdi_t current_raw;      //4
	int8  active;   	            //5  0 invalid, 1 active object or new object
	int8  match;  	              //6   0 no match, 1 matched obj
	int8  moving;  	              //7 0 moving obj, 1 static obj 
}current_obj_list;              //current

typedef struct{
    uint32 index;		                      // frame nr
    uint32 raw_number;                  	  // obj nr
    track_cdi_t    raw_input[MAX_OBJ];        // points
	current_centor_t   current_ctr[MAX_OBJ];  // center

    void        *trk_data;
}target_obj_list;
// point clouds
typedef struct
{
    float         r_flt32;      
    float         v_flt32;   
    float         x_flt32;
    float         y_flt32;
    float         z_flt32;  // used for ID
    float         snr_flt32;
}Raw_Points;

typedef struct
{
    uint16       magicword[4];
    uint32       framenumber;
    int32        Ntargets;    
    Raw_Points   points[MAX_TARGET_NUM]; 
}Raw_Target_List;

// tracking objects
typedef struct
{
    uint16        obj_ID;  // 0-16
    uint16        confidence;  // 0 - 100
    float         x_flt32;
    float         y_flt32;
    float         z_flt32;
    float         vx_flt32; 
    float         vy_flt32; 
    float         vz_flt32; 
    uint16        obj_Rad;  // cm
    uint16        fall_flag;  // 0:normal 1: fall  
}tracker_Points;

typedef struct
{
    uint16       magicword[4];
    uint32       framenumber;
    int32        N_Objs;    
    tracker_Points   points[MAX_TARGET_NUM]; 
}tracker_obj_List;




typedef struct 
{
		int8 confident;              //1
		uint8 lifetime;              //2
		last_center_t current_center; 	 //3
		track_cdi_t current_raw; 		//4
		int8  active;					//5
		int8  match;					//6
		int8  ID;						//7
		int8  up_flag;					//8
		int8 silent_time;				//9
}live_people_list_t;

#pragma pack()

#define CURRENT_OBJ_TRACK_ADDRESS  0x20028000
#define CDI_DATA_TRACK_ADDRESS   (CURRENT_OBJ_TRACK_ADDRESS + 0x2500)
#define TARGETOBJ_TRACK_ADDRESS  (CDI_DATA_TRACK_ADDRESS + 0xF00)

#define OBJ_Track_LIST_ADDR   0x20024000  // size 0x90  // tracked object output 144 = 4*32+16
#define TARGET_LIST_ADDR   0x20025000  // cluster points output 100 points  100*24+16 = 2416 = 0x970
#define LIVE_PEOPLE_TRACK_ADDRESS (TARGET_LIST_ADDR + 0x970)
extern track_cdi_pkg_t cdi_data;
extern target_obj_list target_hist;
extern current_obj_list curr_obj_list[MAX_ARRAY+2];
extern tracker_obj_List tr_obj_list;

void func_track_init(void );
void func_track_clean(void );
void track_step(void);

#endif
