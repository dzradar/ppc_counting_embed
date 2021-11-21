#include <track.h>
#include <string.h>
#include <stdbool.h> // for bool
#include <stdlib.h>

extern uint32 work_times;
current_obj_list curr_obj_list[MAX_ARRAY+2]  __attribute__((at(CURRENT_OBJ_TRACK_ADDRESS)));
track_cdi_pkg_t cdi_data  __attribute__((at(CDI_DATA_TRACK_ADDRESS)));
target_obj_list target_hist __attribute__((at(TARGETOBJ_TRACK_ADDRESS))); //[MAX_ARRAY + 2];

tracker_obj_List tr_obj_list __attribute__((at(OBJ_Track_LIST_ADDR)));
live_people_list_t  live_people[MAX_ARRAY]   __attribute__((at(LIVE_PEOPLE_TRACK_ADDRESS)));

static bool is_Emptyframe = false; /*current frame empty*/

void func_track_init(void)
{
	uint8 i = 0;
	memset(&cdi_data, 0, sizeof(cdi_data));
	memset(&curr_obj_list, 0, sizeof(curr_obj_list));
	memset(&target_hist, 0 , sizeof(target_hist));
	memset(&live_people , 0 , sizeof(live_people));
    // changed by Kang 17.11.2021
	for(i=0 ; i< MAX_ARRAY; i++)
	{
		curr_obj_list[2+i].confident = -1;
		curr_obj_list[2+i].lifetime = 0;		
	}
	for(i=0; i < MAX_ARRAY; i++)
	{
		live_people[i].confident = -1;
		live_people[i].ID = -1;
		live_people[i].up_flag = -1;
	}
}


/*update current data*/ //OBJ_TypeDef* point , 
void func_track_update(track_cdi_pkg_t *input_data ,target_obj_list *output_data)
{	
	if(input_data->raw_number)
		is_Emptyframe = false;
	else
		is_Emptyframe = true;  /*current frame empty*/

	if(!is_Emptyframe)
	{
    	output_data->raw_number = input_data->raw_number; 
			output_data->index = input_data->index;
	}
	memcpy(&output_data->raw_input ,  input_data->cdi , MAX_OBJ *sizeof(track_cdi_t));
}

void func_track_clean(void )
{
	memset(&cdi_data , 0, sizeof(cdi_data.cdi));
}

void tracking_all_clusters(current_obj_list *curr_list ,target_obj_list *target_hist , uint8 frame_num)
{
//	uint8 n = frame_num;
	uint8 i = 0;
	uint8 j = 0;
	uint8 cluster = 0;
	float mean_x = 0.f;
	float mean_y = 0.f;
	float mean_hori = 0.f;
	float obj_w = 0.f;     //x max - xmin
	float obj_h = 0.f;    //y_max- y_min
	float obj_x_min, obj_x_max;
	float obj_y_min, obj_y_max;
	

	for(i=0;i< MAX_ARRAY;i++)  // 8 object
		curr_list[2+i].match = 0;  //0 no match, 1 matched obj
  	track_cdi_t point_clouds; 	 // current clusters

	uint8 active_obj_idx[MAX_ARRAY]={0,0,0,0,0,0,0,0};   //active=1 
	uint8 num_obj = 0;
	/*cluster loop*/
	memset(&point_clouds , 0 , sizeof(track_cdi_t));
	for(cluster =0; cluster < target_hist->raw_number; cluster++)
	{			
		memcpy(&point_clouds ,&target_hist->raw_input[cluster] , sizeof(track_cdi_t) );
		/*mean value*/
		i = 0;
		obj_x_min = point_clouds.raw[i].x;
		obj_x_max = point_clouds.raw[i].x;
		obj_y_min = point_clouds.raw[i].y;
		obj_y_max = point_clouds.raw[i].y;
		while(fabsf(point_clouds.raw[i].y) > 0.01 )
		{
			mean_x += point_clouds.raw[i].x;
			mean_y += point_clouds.raw[i].y;
			mean_hori += point_clouds.raw[i].hori;
			if(obj_x_min > point_clouds.raw[i].x)
				obj_x_min = point_clouds.raw[i].x;
			if(obj_x_max < point_clouds.raw[i].x)
				obj_x_max = point_clouds.raw[i].x;
			if(obj_y_min > point_clouds.raw[i].y)
				obj_y_min = point_clouds.raw[i].y;
			if(obj_y_max < point_clouds.raw[i].y)
				obj_y_max = point_clouds.raw[i].y;
			i++;
		}
		if(i){
			target_hist->current_ctr[cluster].x = mean_x/i;
			target_hist->current_ctr[cluster].y = mean_y/i;
			target_hist->current_ctr[cluster].hori = mean_hori/i;
		
			obj_w = obj_x_max - obj_x_min;
			obj_h = obj_y_max - obj_y_min;

			target_hist->current_ctr[cluster].clu_w = obj_w;
			target_hist->current_ctr[cluster].clu_h = obj_h;
			target_hist->current_ctr[cluster].match_flag = 0;
		}
	}

	uint8 leer_obj_idx[MAX_ARRAY]={0,0,0,0,0,0,0,0};   //active<1 
	uint8 num_leer_obj = 0;
	for(i = 0 ; i< MAX_ARRAY ;i++)
	{
		if(curr_list[2+i].active == 1){
			active_obj_idx[num_obj] = i;
			num_obj++;   /*size(active_obj_idx,1)*/
		}
		else if(curr_list[2+i].active < 1){
			leer_obj_idx[num_leer_obj] = i;
			num_leer_obj++;
		}
	}
	float x_obj;
	float y_obj;
	float w_obj;
	float h_obj;
	float clu_w,clu_h,xc,yc,dist_obj_cluster,dist_wh,dist_cost;

	for(uint8 obj_nr=0; obj_nr < num_obj; obj_nr++) /**/
	{
		x_obj = curr_list[2+active_obj_idx[obj_nr]].current_center.x;
		y_obj = curr_list[2+active_obj_idx[obj_nr]].current_center.y;
		w_obj = curr_list[2+active_obj_idx[obj_nr]].current_center.w_obj;
		h_obj = curr_list[2+active_obj_idx[obj_nr]].current_center.h_obj;

		//loop for all clusters
		float min_dist_cluster = 1000;
		int8 nearest_obj_idx = 0;
		
		for(cluster =0; cluster < target_hist->raw_number; cluster++)
		{
			memcpy(&point_clouds, &target_hist->raw_input[cluster], sizeof(track_cdi_t));
			clu_w = target_hist->current_ctr[cluster].clu_w;  //center
			clu_h = target_hist->current_ctr[cluster].clu_h;
			xc = target_hist->current_ctr[cluster].x;
			yc = target_hist->current_ctr[cluster].y;
			dist_obj_cluster = sqrtf((xc - x_obj) * (xc - x_obj) + (yc - y_obj) * (yc - y_obj));
			dist_wh = sqrtf((w_obj - clu_w) *(w_obj - clu_w) + (h_obj - clu_h) *(h_obj - clu_h));
			dist_cost = dist_obj_cluster*0.9 + dist_wh*0.1;
			if(min_dist_cluster > dist_cost)
			{
				min_dist_cluster = dist_cost;
				nearest_obj_idx = cluster; 
			}		
		}

		//differ points number
		int32 delta_frame = target_hist->raw_input[nearest_obj_idx].raw[0].id_count - curr_list[2+active_obj_idx[obj_nr]].current_raw.raw[0].id_count;
		
		//printf("delta_frame = %d\r\n", delta_frame);
		uint8 num_pts_obj = 0;
		while(fabsf(curr_list[2+active_obj_idx[obj_nr]].current_raw.raw[num_pts_obj].y) > 0.01f){
			num_pts_obj++;
		}
		/*current points number*/ 
		uint8 num_pts_clu = 0;
		while(fabsf(target_hist->raw_input[nearest_obj_idx].raw[num_pts_clu].y) > 0.01f){
			num_pts_clu++;
		}

		float xo_min,xo_max,yo_min,yo_max,xcl_min,xcl_max,ycl_min,ycl_max;
		float x_distance = 0.f;
		float y_distance = 0.f;
		float thres_dist_obj_clu = 0.f;
		/*curr_obj_list*/
		xo_min = curr_list[2+active_obj_idx[obj_nr]].current_raw.raw[0].x;
		xo_max = xo_min;
		yo_min = curr_list[2+active_obj_idx[obj_nr]].current_raw.raw[0].y;
		yo_max = yo_min;
		for(j = 1; j <num_pts_obj;j++ )
		{
			if(xo_min > curr_list[2+active_obj_idx[obj_nr]].current_raw.raw[j].x)
				xo_min = curr_list[2+active_obj_idx[obj_nr]].current_raw.raw[j].x;
			if(xo_max < curr_list[2+active_obj_idx[obj_nr]].current_raw.raw[j].x)
				xo_max = curr_list[2+active_obj_idx[obj_nr]].current_raw.raw[j].x;
			if(yo_min > curr_list[2+active_obj_idx[obj_nr]].current_raw.raw[j].y)
				yo_min = curr_list[2+active_obj_idx[obj_nr]].current_raw.raw[j].y;
			if(yo_max < curr_list[2+active_obj_idx[obj_nr]].current_raw.raw[j].y)
				yo_max = curr_list[2+active_obj_idx[obj_nr]].current_raw.raw[j].y;
		}
		xcl_min = target_hist->raw_input[nearest_obj_idx].raw[0].x;
		xcl_max = xcl_min;
		ycl_min = target_hist->raw_input[nearest_obj_idx].raw[0].y;
		ycl_max = ycl_min;
		for(j = 1; j < num_pts_clu;j++)
		{
			if(xcl_min > target_hist->raw_input[nearest_obj_idx].raw[j].x )
				xcl_min = target_hist->raw_input[nearest_obj_idx].raw[j].x;
			if(xcl_max < target_hist->raw_input[nearest_obj_idx].raw[j].x)
				xcl_max =  target_hist->raw_input[nearest_obj_idx].raw[j].x;
			if(ycl_min > target_hist->raw_input[nearest_obj_idx].raw[j].y)
				ycl_min = target_hist->raw_input[nearest_obj_idx].raw[j].y;
			if(ycl_max < target_hist->raw_input[nearest_obj_idx].raw[j].y )
				ycl_max = target_hist->raw_input[nearest_obj_idx].raw[j].y;
		}
		if(xo_min > xcl_max)
			x_distance = xo_min - xcl_max;
		else if(xo_max < xcl_min)
			x_distance = xcl_min - xo_max;
		else
			x_distance = 0;
		if(yo_min > ycl_max)
			y_distance = yo_min - ycl_max;
		else if(yo_max < ycl_min)
			y_distance = ycl_min - yo_max;
		else
			y_distance = 0;

		if(y_distance < 0.15 && delta_frame >= 2)
			y_distance = 0;
		if(delta_frame > 3 )
			thres_dist_obj_clu = THRESHOLD_OBJ_DISTANCE * 1.5;
		else
			thres_dist_obj_clu = THRESHOLD_OBJ_DISTANCE;

		if((y_distance == 0) &&  (x_distance < thres_dist_obj_clu/2) && (min_dist_cluster < thres_dist_obj_clu))
		{
			if(delta_frame > 3)
				delta_frame =0;

			curr_list[2+active_obj_idx[obj_nr]].lifetime += delta_frame;
			if( curr_list[2+active_obj_idx[obj_nr]].lifetime > 20)
				curr_list[2+active_obj_idx[obj_nr]].lifetime = 20;
			if(curr_list[2+active_obj_idx[obj_nr]].lifetime > 5)
				curr_list[2+active_obj_idx[obj_nr]].confident = 1; /*confidence 1*/
			
			j = 0;
	        float xmean = 0,y_mean = 0,horimean = 0,id_mean = 0,idcount_mean = 0;
			while(fabsf(target_hist->raw_input[nearest_obj_idx].raw[j].y) > 0.01)
			{		
				xmean += target_hist->raw_input[nearest_obj_idx].raw[j].x;
				y_mean += target_hist->raw_input[nearest_obj_idx].raw[j].y;
				horimean += target_hist->raw_input[nearest_obj_idx].raw[j].hori;
				id_mean += target_hist->raw_input[nearest_obj_idx].raw[j].id;
				idcount_mean += target_hist->raw_input[nearest_obj_idx].raw[j].id_count;
				j++;
			}
			if(j){
				xmean = xmean / j;
				y_mean = y_mean / j;
				horimean = horimean / j;
				id_mean = id_mean / j;
				idcount_mean = idcount_mean / j;
			}

			curr_list[2+active_obj_idx[obj_nr]].current_center.x = ALPHA_BETA_FILTER * curr_list[2+active_obj_idx[obj_nr]].current_center.x + \
								(1 - ALPHA_BETA_FILTER) * xmean;
			curr_list[2+active_obj_idx[obj_nr]].current_center.y = ALPHA_BETA_FILTER * curr_list[2+active_obj_idx[obj_nr]].current_center.y + \
								(1 - ALPHA_BETA_FILTER) * y_mean;
			curr_list[2+active_obj_idx[obj_nr]].current_center.hori = ALPHA_BETA_FILTER * curr_list[2+active_obj_idx[obj_nr]].current_center.hori + \
								(1 - ALPHA_BETA_FILTER) * horimean;		
			curr_list[2+active_obj_idx[obj_nr]].current_center.id = ALPHA_BETA_FILTER * curr_list[2+active_obj_idx[obj_nr]].current_center.id + \
								(1 - ALPHA_BETA_FILTER) * id_mean;
			curr_list[2+active_obj_idx[obj_nr]].current_center.cur_id = ALPHA_BETA_FILTER * curr_list[2+active_obj_idx[obj_nr]].current_center.cur_id + \
								(1 - ALPHA_BETA_FILTER) * idcount_mean;			

			curr_list[2+active_obj_idx[obj_nr]].current_center.w_obj = target_hist->current_ctr[nearest_obj_idx].clu_w;
			curr_list[2+active_obj_idx[obj_nr]].current_center.h_obj = target_hist->current_ctr[nearest_obj_idx].clu_h;

			/*save current obj list data*/
			memcpy(&curr_list[2+active_obj_idx[obj_nr]].current_raw, &target_hist->raw_input[nearest_obj_idx], sizeof(track_cdi_t));

			curr_list[2+active_obj_idx[obj_nr]].active = 1;
			curr_list[2+active_obj_idx[obj_nr]].match = 1;
			curr_list[2+active_obj_idx[obj_nr]].moving = 0;

            // this could be changed! by kang 16.11.2021
            //target_hist->current_ctr[nearest_obj_idx].match_flag++;
			if(target_hist->current_ctr[nearest_obj_idx].match_flag++ >= 125)
				target_hist->current_ctr[nearest_obj_idx].match_flag = 125;
		}
		else
		{
			curr_list[2+active_obj_idx[obj_nr]].lifetime -= delta_frame;
            // changed by kang 16.11.2021
			//if(curr_list[2+active_obj_idx[obj_nr]].lifetime < 0)
			//		curr_list[2+active_obj_idx[obj_nr]].lifetime = 0;
            
			if(curr_list[2+active_obj_idx[obj_nr]].lifetime < 5)                
				curr_list[2+active_obj_idx[obj_nr]].confident = 0;
			if(curr_list[2+active_obj_idx[obj_nr]].lifetime < 1 )
			{
				curr_list[2+active_obj_idx[obj_nr]].confident = -1;
				curr_list[2+active_obj_idx[obj_nr]].lifetime = 0;
				memset(&curr_list[2+active_obj_idx[obj_nr]].current_center, 0 , sizeof(last_center_t));
				memset(&curr_list[2+active_obj_idx[obj_nr]].current_raw, 0 , sizeof(track_cdi_t));
				curr_list[2+active_obj_idx[obj_nr]].active = 0;
				curr_list[2+active_obj_idx[obj_nr]].match = 0;
				curr_list[2+active_obj_idx[obj_nr]].moving = 0;
                // change by kang 16.11.2021
                //curr_list[1].confident--;
				if(curr_list[1].confident-- <= -125)
					curr_list[1].confident	= -125;
			}	
		}
	}


	/*merge all obj mapping to the same cluster*/
	i = 0;
	j = 0;
	for(i = 0 ;i < MAX_ARRAY ; i++){
		if( curr_list[2 + i].active == 1)
		{
			active_obj_idx[j] = i;
			j++;
		}		
	}
	num_obj = j;	

	/*active object*/
	for(uint8 obj_i = 0; obj_i < num_obj-1; obj_i++)
	{
		for(uint8 obj_j = obj_i+1; obj_j < num_obj;  obj_j++)
		{
			if(curr_list[2+active_obj_idx[obj_i]].confident > -1 &&  curr_list[2+active_obj_idx[obj_j]].confident > -1)
			{
				if((fabsf(curr_list[2+active_obj_idx[obj_i]].current_center.x - curr_list[2+active_obj_idx[obj_j]].current_center.x ) < 0.7) && \
						(fabsf(curr_list[2+active_obj_idx[obj_i]].current_center.y - curr_list[2+active_obj_idx[obj_j]].current_center.y) < 0.3))
				{
					//same
					//delete obj with small frequency
					if(curr_list[2+active_obj_idx[obj_i]].lifetime < curr_list[2+active_obj_idx[obj_j]].lifetime )
					{
						//if i < j delete i
						curr_list[2+active_obj_idx[obj_i]].confident = -1;
						curr_list[2+active_obj_idx[obj_i]].lifetime = 0;
						memset(&curr_list[2+active_obj_idx[obj_i]].current_center , 0 ,sizeof(last_center_t));
						memset(&curr_list[2+active_obj_idx[obj_i]].current_raw , 0 ,sizeof(track_cdi_t));
						curr_list[2+active_obj_idx[obj_i]].active = 0;
						curr_list[2+active_obj_idx[obj_i]].match = 0;
						curr_list[2+active_obj_idx[obj_i]].moving = 0;
						if(curr_list[1].confident-- <= -126)
							curr_list[1].confident = -126;
					}
					else //if(curr_list[2+active_obj_idx[obj_i]].lifetime >= curr_list[2+active_obj_idx[obj_j]].lifetime  )
					{
						curr_list[2+active_obj_idx[obj_j]].confident = -1;
						curr_list[2+active_obj_idx[obj_j]].lifetime = 0;
						memset(&curr_list[2+active_obj_idx[obj_j]].current_center , 0 ,sizeof(last_center_t));
						memset(&curr_list[2+active_obj_idx[obj_j]].current_raw , 0 ,sizeof(track_cdi_t));
						curr_list[2+active_obj_idx[obj_j]].active = 0;
						curr_list[2+active_obj_idx[obj_j]].match = 0;
						curr_list[2+active_obj_idx[obj_j]].moving = 0;	
						if(curr_list[1].confident-- <= -126)
							curr_list[1].confident = -126;
					}
				}
			}
		}
	}

	uint8 new_obj_num = 0;
	// check match flag of clusters -> new target from unmatched clusters
	for(cluster = 0; cluster < target_hist->raw_number; cluster++)
	{
		/**/
		if(target_hist->current_ctr[cluster].match_flag == 0)
		{
			memset(&point_clouds , 0, sizeof(track_cdi_t));
			memcpy(&point_clouds , &target_hist->raw_input[cluster],sizeof(track_cdi_t));
			obj_w = target_hist->current_ctr[cluster].clu_w;
			obj_h = target_hist->current_ctr[cluster].clu_h;
			//new target
            
            // changed by Kang 17.11.2021
            if(new_obj_num < num_leer_obj)
            {
                //initial new object
                if(curr_list[1].confident++ >= 125)
                     curr_list[1].confident = 125;

                //push in the list
                   //new_obj_num += 1; 
                //% incinfident
                curr_list[2+ leer_obj_idx[new_obj_num]].confident = 0;
                //life time
                curr_list[2+ leer_obj_idx[new_obj_num]].lifetime += 1;
                if(curr_list[2+ leer_obj_idx[new_obj_num]].lifetime  > 20)
                    curr_list[2+ leer_obj_idx[new_obj_num]].lifetime = 20;
                
                //last_center_t
                float mean_x = 0.0f;
                float mean_y = 0.0f;
                float mean_hori = 0.0f;
                float mean_id = 0.0f;
                float mean_count = 0.0f;
                i = 0;
                while(fabsf(point_clouds.raw[i].y > 0.01) ){
                    mean_x += point_clouds.raw[i].x;
                    mean_y += point_clouds.raw[i].y;
                    mean_hori += point_clouds.raw[i].hori;
                    mean_id+= point_clouds.raw[i].id;
                    mean_count += point_clouds.raw[i].id_count;
                    i++;//
                }
                if(i){
                    curr_list[2+ leer_obj_idx[new_obj_num]].current_center.x = mean_x / i;
                    curr_list[2+ leer_obj_idx[new_obj_num]].current_center.y = mean_y / i;
                    curr_list[2+ leer_obj_idx[new_obj_num]].current_center.hori = mean_hori / i;
                    curr_list[2+ leer_obj_idx[new_obj_num]].current_center.id = mean_id / i;
                    curr_list[2+ leer_obj_idx[new_obj_num]].current_center.cur_id = mean_count / i;
                }
                //w,h
                curr_list[2+ leer_obj_idx[new_obj_num]].current_center.w_obj = obj_w;
                curr_list[2+ leer_obj_idx[new_obj_num]].current_center.h_obj = obj_h;
                
                memcpy(&curr_list[2+ leer_obj_idx[new_obj_num]].current_raw , &point_clouds ,sizeof(track_cdi_t));

                curr_list[2+ leer_obj_idx[new_obj_num]].active = 1;
                curr_list[2+ leer_obj_idx[new_obj_num]].match = 1;
                curr_list[2+ leer_obj_idx[new_obj_num]].moving = 0;
            }

			new_obj_num += 1;
		}
	}

}

static void keep_still_obj(current_obj_list *curr_list, live_people_list_t *live_list)
{
	const float bounding_box_x = 1.5;
	const float bounding_box_y = 3.5;
	const uint8 bounding_stime = 60;
	//reset updated flag
	int8 valid_obj_idx[MAX_ARRAY] = {0, 0, 0, 0, 0, 0, 0, 0};
	uint8 i = 0;
	uint8 j = 0;
	for(i = 0;i< MAX_ARRAY ;i++)
	{
        // changed by Kang 17.11.2021
		if(live_list[i].ID > 0)
		{
			valid_obj_idx[j] = i;
			j++;
		}
	}
	if(j)
	{
		for(uint8 n = 0; n< j; n++)
			live_list[valid_obj_idx[n]].up_flag = 0;
	}
	//if obj life time bigger than 20, keep in memory
	int8 active_obj_idx[MAX_ARRAY] = {0, 0, 0, 0, 0, 0, 0, 0};
	int8 leer_obj_idx[MAX_ARRAY] = {0, 0, 0, 0, 0, 0, 0, 0};
	int8 idx_list[MAX_ARRAY] = {0, 0, 0, 0, 0, 0, 0, 0};
	uint8 active_obj_length = 0;
	j=0;
	for(i=0; i< MAX_ARRAY; i++)
	{
		if(curr_list[2+i].active == 1)
		{
			active_obj_idx[j] = i;
			j++;
		}
	}
	active_obj_length = j;
	j=0;
	for(i=0;i < MAX_ARRAY; i++)
	{
		if(live_list[i].ID  == -1)
		{
			leer_obj_idx[j] = i;
			j++;
		}
	}
	for(i=0;i< MAX_ARRAY ;i++)
	{
		idx_list[i] = live_list[i].ID;
	}
	uint8 new_obj_num = 0;
	uint8 idx_incl[MAX_ARRAY];
	j=0;
	
	for(uint8 obj_nr = 0; obj_nr < active_obj_length;obj_nr++)
	{
		if(curr_obj_list[2+active_obj_idx[obj_nr]].lifetime == 20)
		{
			//is this obj new?
			//curret obj ID
			uint8 cur_ID = active_obj_idx[obj_nr];

			for(i=0;i<MAX_ARRAY;i++)
			{
				if(idx_list[i] == cur_ID)
				{
					idx_incl[j] = i; /*return postion*/
					j++;
				}
			}
			if(!j){ //isempty
				//if it is a new obj do this
				//people record
				//+ ID -1 no use,1:exist
				live_list[leer_obj_idx[new_obj_num]].confident = curr_list[2+cur_ID].confident;
				live_list[leer_obj_idx[new_obj_num]].lifetime = curr_list[2+cur_ID].lifetime;
				memcpy(&live_list[leer_obj_idx[new_obj_num]].current_center, &curr_list[2+cur_ID].current_center ,sizeof(last_center_t));
				memcpy(&live_list[leer_obj_idx[new_obj_num]].current_raw, &curr_list[2+cur_ID].current_raw ,sizeof(track_cdi_t));
				live_list[leer_obj_idx[new_obj_num]].active = curr_list[2+cur_ID].active;
				live_list[leer_obj_idx[new_obj_num]].match = curr_list[2+cur_ID].match;
				live_list[leer_obj_idx[new_obj_num]].ID = cur_ID;
				live_list[leer_obj_idx[new_obj_num]].up_flag = 1;
				live_list[leer_obj_idx[new_obj_num]].silent_time = 1;
				new_obj_num++;
			}
			else
			{
				//the obj ID is already recorded
				//update the position
				live_list[idx_incl[0]].confident = curr_list[2+cur_ID].confident;
				live_list[idx_incl[0]].lifetime = curr_list[2+cur_ID].lifetime;
				memcpy(&live_list[idx_incl[0]].current_center, &curr_list[2+cur_ID].current_center ,sizeof(last_center_t));
				memcpy(&live_list[idx_incl[0]].current_raw, &curr_list[2+cur_ID].current_raw ,sizeof(track_cdi_t));
				live_list[idx_incl[0]].active = curr_list[2+cur_ID].active;
				live_list[idx_incl[0]].match = curr_list[2+cur_ID].match;
				live_list[idx_incl[0]].up_flag = 1;
				if(live_list[idx_incl[0]].silent_time > 1 )
					live_list[idx_incl[0]].silent_time = 1;
			}
		}
	}
	
	//decide whether to keep still object
	for(j=0,i=0; i<MAX_ARRAY; i++)
	{
		if(live_list[i].ID > 0)
		{
			valid_obj_idx[j] = i;
			j++;
		}	
	}
	if(j) //not empty
	{
		for(uint8 n=0; n < j; n++ )
		{
			if(live_list[valid_obj_idx[n]].up_flag == 0) //updated flag, 1 updated, 0 no show object
			{
				float xc = live_list[valid_obj_idx[n]].current_center.x;
				float yc = live_list[valid_obj_idx[n]].current_center.y;
                // this is not phi, this is v
				//float phi = live_list[valid_obj_idx[n]].current_center.hori;
				if(live_list[valid_obj_idx[n]].silent_time++ >= 125)
					live_list[valid_obj_idx[n]].silent_time = 125;
				if(yc < 0.8 && fabsf(xc) > 1.0)
				{
					live_list[valid_obj_idx[n]].confident = -1;
					live_list[valid_obj_idx[n]].lifetime = 0;
					memset(&live_list[valid_obj_idx[n]].current_center , 0, sizeof(last_center_t));
					memset(&live_list[valid_obj_idx[n]].current_raw , 0, sizeof(track_cdi_t));
					live_list[valid_obj_idx[n]].active = 0;
					live_list[valid_obj_idx[n]].match = 0;
					live_list[valid_obj_idx[n]].ID = -1; //7
					live_list[valid_obj_idx[n]].up_flag = -1;
					live_list[valid_obj_idx[n]].silent_time = 0;
				}
				else if(fabsf(xc) < bounding_box_x && fabsf(yc) < bounding_box_y && live_list[valid_obj_idx[n]].silent_time < bounding_stime)
				{
					curr_list[2+live_list[valid_obj_idx[n]].ID].confident = live_list[valid_obj_idx[n]].confident;
					curr_list[2+live_list[valid_obj_idx[n]].ID].lifetime = live_list[valid_obj_idx[n]].lifetime;
					memcpy(&curr_list[2+live_list[valid_obj_idx[n]].ID].current_center ,&live_list[valid_obj_idx[n]].current_center, sizeof(last_center_t) );
					memcpy(&curr_list[2+live_list[valid_obj_idx[n]].ID].current_raw ,&live_list[valid_obj_idx[n]].current_raw, sizeof(track_cdi_t) );
					curr_list[2+live_list[valid_obj_idx[n]].ID].active = live_list[valid_obj_idx[n]].active;
					curr_list[2+live_list[valid_obj_idx[n]].ID].match = live_list[valid_obj_idx[n]].match;
					curr_list[2+live_list[valid_obj_idx[n]].ID].moving = 1;
				}
				else
				{
					live_list[valid_obj_idx[n]].confident = -1;
					live_list[valid_obj_idx[n]].lifetime = 0;
					memset(&live_list[valid_obj_idx[n]].current_center , 0, sizeof(last_center_t));
					memset(&live_list[valid_obj_idx[n]].current_raw , 0, sizeof(track_cdi_t));
					live_list[valid_obj_idx[n]].active = 0;
					live_list[valid_obj_idx[n]].match = 0;
					live_list[valid_obj_idx[n]].ID = -1; //7
					live_list[valid_obj_idx[n]].up_flag = -1;
					live_list[valid_obj_idx[n]].silent_time = 0;
				}
			}
		}
	}
}

void show_obj_cluster(current_obj_list *curr_list )
{
	//keep_still_obj(curr_list , live_people);
	uint8 active_obj_idx[MAX_ARRAY] = {0, 0, 0, 0, 0, 0, 0, 0};
	uint8 i = 0;
	uint8 j = 0;

	for(i=0; i<MAX_ARRAY; i++)
	{
		if(curr_obj_list[2+i].active == 1)
		{
			active_obj_idx[j] = i;
			j++;
		}
	}
	//printf("\nST:\n");
    int8 valid_obj_num = 0;
	for(i=0; i< j; i++)
	{
		if (curr_obj_list[2+active_obj_idx[i]].lifetime > 5)
        {
            float xc = curr_obj_list[2+active_obj_idx[i]].current_center.x;
            float yc = curr_obj_list[2+active_obj_idx[i]].current_center.y;
            float hori = curr_obj_list[2+active_obj_idx[i]].current_center.hori;
            //float obj_w = curr_obj_list[2+active_obj_idx[i]].current_center.w_obj;
            //float obj_h = curr_obj_list[2+active_obj_idx[i]].current_center.h_obj;
            //printf(" %5.3f, %5.3f, %5.3f \n", xc, yc, hori);		
            tr_obj_list.points[i].obj_ID = active_obj_idx[i];
            tr_obj_list.points[i].confidence = 90;
            tr_obj_list.points[i].x_flt32 = xc;
            tr_obj_list.points[i].y_flt32 = yc;
            tr_obj_list.points[i].z_flt32 = 0;
            tr_obj_list.points[i].vx_flt32 = 0;
            tr_obj_list.points[i].vy_flt32 = hori;
            tr_obj_list.points[i].obj_Rad = 40;
            tr_obj_list.points[i].fall_flag = 0;  // not used
            valid_obj_num++;
        }
	}
    
    tr_obj_list.N_Objs = valid_obj_num;
	//printf("ET\n");
}

void track_step(void)
{
        /*run tracking algo*/
#ifdef SYSTEM_USING_TRACK
		
        func_track_update(&cdi_data ,&target_hist);
       if(!is_Emptyframe){
        	tracking_all_clusters(curr_obj_list ,&target_hist , 0);

			keep_still_obj(curr_obj_list, live_people);
			show_obj_cluster(curr_obj_list); /*print still obj */
		}		
	func_track_clean(); 
#endif
}
