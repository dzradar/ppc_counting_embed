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

#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <db_scan.h>
#include <adt3102.h>
#include <track.h>

extern uint32 work_times;
Node_TypeDef *createNode(uint16 index)
{
    Node_TypeDef *n = (Node_TypeDef *) calloc(1, sizeof(Node_TypeDef));
    if (n == NULL)
        perror("Failed to allocate node.");
    else {
        n->index = index;
        n->next = NULL;
    }
    return n;
}

int16 appendAtEnd(
     uint16 index,
     Epsilon_Neighbours_TypeDef *en)
{
    Node_TypeDef *n = createNode(index);
    if (n == NULL) {
        free(en);
        return FAILURE;
    }
    if (en->head == NULL) {
        en->head = n;
        en->tail = n;
    } else {
        en->tail->next = n;
        en->tail = n;
    }
    ++(en->num_members);
    return SUCCESS;
}

Epsilon_Neighbours_TypeDef *getEpsilonNeighbours(
    uint16 index,
    OBJ_TypeDef *points,
    uint16 numPoints,
    double epsilon1,
	double epsilon2,
	double epsilon3,
    double (*dist)(OBJ_TypeDef *a, OBJ_TypeDef *b))
{
    Epsilon_Neighbours_TypeDef *en = (Epsilon_Neighbours_TypeDef *)
        calloc(1, sizeof(Epsilon_Neighbours_TypeDef));
    if (en == NULL) {
        perror("Failed to allocate epsilon neighbours.");
        return en;
    }
    for (int16 i = 0; i < numPoints; ++i) {
        if (i == index)
            continue;
        //if (dist(&points[index], &points[i]) > epsilon)
				#ifdef DBSCAN_METHOD_RNG_ANG 
					if ( fabs(points[index].rng_idx - points[i].rng_idx) > epsilon1 || fabs(points[index].dop_idx - points[i].dop_idx) > epsilon2 || fabs(points[index].hori - points[i].hori) > epsilon3)
        #else
					if ( fabs(points[index].x - points[i].x) > epsilon1 || fabs(points[index].y - points[i].y) > epsilon2 || fabs(points[index].dop_idx - points[i].dop_idx) > epsilon3)
				#endif
				continue;
        else {
            if (appendAtEnd(i, en) == FAILURE) {
                destroyEpsilonNeighbours(en);
                en = NULL;
                break;
            }
        }
    }
    return en;
}


void destroyEpsilonNeighbours(Epsilon_Neighbours_TypeDef *en)
{
    if (en) {
        Node_TypeDef *t, *h = en->head;
        while (h) {
            t = h->next;
            free(h);
            h = t;
        }
        free(en);
    }
}

void dbScan(
    OBJ_TypeDef *points,
    uint16 numPoints,
    double epsilon1,
	double epsilon2,
    double epsilon3,			
    uint16 minpts,
    double (*dist)(OBJ_TypeDef *a, OBJ_TypeDef *b))
{
    uint16 i, clusterId = 0;
	  
	minpts = minpts - 1;  
	
    for (i = 0; i < numPoints; ++i) {
        if (points[i].cluster_id == UNCLASSIFIED) {
            if (expand(i, clusterId, points,
                       numPoints, epsilon1, epsilon2, epsilon3  , minpts,
                       dist) == CORE_POINT)
                ++clusterId;
        }
    }
}

int16 expand(
    uint16 index,
    uint16 clusterId,
    OBJ_TypeDef *points,
    uint16 numPoints,
    double epsilon1,
	double epsilon2,
    double epsilon3,	
    uint16 minpts,
    double (*dist)(OBJ_TypeDef *a, OBJ_TypeDef *b))
{
    int16 return_value = NOT_CORE_POINT;
    Epsilon_Neighbours_TypeDef *seeds =
        getEpsilonNeighbours(index, points,
                               numPoints, epsilon1, epsilon2, epsilon3  ,
                               dist);
    if (seeds == NULL)
        return FAILURE;

    if (seeds->num_members < minpts)
        points[index].cluster_id = NOISE;
    else {
        points[index].cluster_id = clusterId;
        Node_TypeDef *h = seeds->head;
        while (h) {
            points[h->index].cluster_id = clusterId;
            h = h->next;
        }

        h = seeds->head;
        while (h) {
            spread(h->index, seeds, clusterId, points,
                   numPoints, epsilon1, epsilon2, epsilon3  , minpts, dist);
            h = h->next;
        }

        return_value = CORE_POINT;
    }
    destroyEpsilonNeighbours(seeds);
    return return_value;
}

int16 spread(
    uint16 index,
    Epsilon_Neighbours_TypeDef *seeds,
    uint16 clusterId,
    OBJ_TypeDef *points,
    uint16 numPoints,
    double epsilon1,
	double epsilon2,
    double epsilon3,	
    uint16 minpts,
    double (*dist)(OBJ_TypeDef *a, OBJ_TypeDef *b))
{
    Epsilon_Neighbours_TypeDef *spread =
    getEpsilonNeighbours(index, points,
                       numPoints, epsilon1, epsilon2, epsilon3  ,
                       dist);
    if (spread == NULL)
    {
        return FAILURE;
    }
    if (spread->num_members >= minpts) {
        Node_TypeDef *n = spread->head;
        OBJ_TypeDef *d;
        while (n) {
            d = &points[n->index];
            if (d->cluster_id == NOISE ||
                d->cluster_id == UNCLASSIFIED) {
                if (d->cluster_id == UNCLASSIFIED) {
                    if (appendAtEnd(n->index, seeds)
                        == FAILURE) {
                        destroyEpsilonNeighbours(spread);
                        return FAILURE;
                    }
                }
                d->cluster_id = clusterId;
            }
            n = n->next;
        }
    }

    destroyEpsilonNeighbours(spread);
    return SUCCESS;
}

double euclideanDist(OBJ_TypeDef *a, OBJ_TypeDef *b)
{
//    return sqrt(pow(a->dop_idx - b->dop_idx, 2) +
//            pow(a->rng_idx - b->rng_idx, 2) +
//            pow(a->hori - b->hori, 2));
	
	return 0;
}

void printPoints(
    OBJ_TypeDef *points,
    uint16 numPoints)
{
    uint16 i = 0;
	uint16 numNoise = 0; 
	uint8 hasClass = 0;
    uint16 clu_id = 0;
    uint8 total_clusters = 0;
		
	#ifdef DBSCAN_METHOD_RNG_ANG
	printf("start\n");
				while (i < numPoints) {
						 // printf("%5.2lf %5.2lf %5.2lf: %d \r\n",
					if(points[i].cluster_id >= 0)
					{
						printf("%d,%d,%.1f,%d\n",
											 points[i].rng_idx,
											 (points[i].dop_idx - CHIRP_NUM/2), points[i].hori,
											 points[i].cluster_id);
					}
					else
						numNoise += 1;
								++i;
				}
		    printf("end\n");
			#else		
			  hasClass = 0;
			  for (i = 0; i < numPoints; i++)
				{
					if(points[i].cluster_id >= 0)
					{
						hasClass = 1;
						break;
					}
				}
				if(hasClass == 0)
				{
 #ifdef                   SYSTEM_USING_CLUSTER_SE
			        printf("s");              
		            printf("e");
#endif
					return;      
				}	
#ifdef                   SYSTEM_USING_CLUSTER_SE              	
			    printf("s");
#endif
                cdi_data.index = work_times;
				while (i < numPoints) {
					if(points[i].cluster_id >= 0)
					{
                    
                        clu_id = points[i].cluster_id;
                        if (clu_id < MAX_OBJ)
                        {
                            
                            cdi_data.cdi[clu_id].raw[cdi_data.point_number[clu_id]].x = points[i].x;
                            cdi_data.cdi[clu_id].raw[cdi_data.point_number[clu_id]].y = points[i].y;
                            cdi_data.cdi[clu_id].raw[cdi_data.point_number[clu_id]].hori = points[i].dop_idx - CHIRP_NUM2/2;;
                            cdi_data.cdi[clu_id].raw[cdi_data.point_number[clu_id]].id = points[i].cluster_id;
                            cdi_data.cdi[clu_id].raw[cdi_data.point_number[clu_id]].id_count = work_times;
                            cdi_data.point_number[clu_id] += 1;
                            
                            if (total_clusters < clu_id)
                                total_clusters = clu_id;
                        }
#ifdef  SYSTEM_USING_CLUSTER_SE 
						printf("%5.3f,%5.3f,%d,%d\n",
											points[i].x,
											points[i].y,
											(points[i].dop_idx - CHIRP_NUM2/2), 
											 points[i].cluster_id);
#endif //
                        cdi_data.raw_number  = total_clusters + 1;
					}
					else
						numNoise += 1;
								++i;
				}
#ifdef  SYSTEM_USING_CLUSTER_SE
		     printf("e");
#endif
			#endif					
	//printf("num of noise : %d \r\n", numNoise);
}

