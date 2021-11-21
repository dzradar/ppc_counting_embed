#ifndef __DBSCAN_H__  
#define __DBSCAN_H__ 

#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
//#include <define.h>
#include <proc_counting.h>

#define UNCLASSIFIED -1
#define NOISE -2

#define CORE_POINT 1
#define NOT_CORE_POINT 0

#define SUCCESS 0
#define FAILURE -3

typedef struct node_s Node_TypeDef;
struct node_s {
    uint16 index;
    Node_TypeDef *next;
};

typedef struct epsilon_neighbours_s Epsilon_Neighbours_TypeDef;
struct epsilon_neighbours_s {
    uint16 num_members;
    Node_TypeDef *head, *tail;
};

/*********************************************************
Function name:  createNode
Description:    create a node structure from a point index 
Paramater:      index   : index of the point 
Return:         node structure
*********************************************************/
Node_TypeDef *createNode(uint16 index);

/*********************************************************
Function name:  appendAtEnd
Description:    add node to neighbourhood 
Paramater:      index   : index of the point 
                *en     : pointer to neighbourhood structure  
Return:         SUCCESS - if successfully added
                FAILURE - if not successfully added
*********************************************************/
int16 appendAtEnd(
     uint16 index,
     Epsilon_Neighbours_TypeDef *en);

/*********************************************************
Function name:  getEpsilonNeighbours
Description:    find neighbours of a point
Paramater:      index       : index of the point 
                *points     : pointer to points 
                num_points  : number of data points 
                epsilon_1   : distance limit for parameter 1 
                epsilon_2   : distance limit for parameter 2
                epsilon_3   : distance limit for parameter 3     
                *dist   : pointer to the function for distance calculation (currently not used) 
Return:         neighbourhood structure
*********************************************************/
Epsilon_Neighbours_TypeDef *getEpsilonNeighbours(
    uint16 index,
    OBJ_TypeDef *points,
    uint16 num_points,
    double epsilon_1,
	double epsilon_2,
    double epsilon_3,	
    double (*dist)(OBJ_TypeDef *a, OBJ_TypeDef *b));
    

/*********************************************************
Function name:  destroyEpsilonNeighbours
Description:    release neighbours                    
Paramater:      *en     : pointer to neighbourhood structure 
Return:         void
*********************************************************/
void destroyEpsilonNeighbours(Epsilon_Neighbours_TypeDef *en);

/*********************************************************
Function name:  dbScan
Description:    perform DBSCAN clustering algorithm                    
Paramater:      *points     : pointer to points 
                num_points  : number of data points 
                epsilon_1   : distance limit for parameter 1 
                epsilon_2   : distance limit for parameter 2
                epsilon_3   : distance limit for parameter 3     
                minpts   : minimum number of points in a cluster 
                *dist   : pointer to the function for distance calculation (currently not used) 
Return:         void
*********************************************************/
void dbScan(
    OBJ_TypeDef *points,
    uint16 num_points,
    double epsilon_1,
	double epsilon_2,
    double epsilon_3,	
    uint16 minpts,
    double (*dist)(OBJ_TypeDef *a, OBJ_TypeDef *b));
    
/*********************************************************
Function name:  expand
Description:    expand cluster
Paramater:      index       : index of the point 
                cluster_id  : cluster id 
                *points     : pointer to points 
                num_points  : number of data points 
                epsilon_1   : distance limit for parameter 1 
                epsilon_2   : distance limit for parameter 2
                epsilon_3   : distance limit for parameter 3     
                minpts   : minimum number of points in a cluster 
                *dist   : pointer to the function for distance calculation (currently not used) 
Return:         expension result
*********************************************************/
int16 expand(
    uint16 index,
    uint16 cluster_id,
    OBJ_TypeDef *points,
    uint16 num_points,
    double epsilon_1,
	double epsilon_2,
    double epsilon_3,	
    uint16 minpts,
    double (*dist)(OBJ_TypeDef *a, OBJ_TypeDef *b));
    
    
/*********************************************************
Function name:  spread
Description:    update neighbours of a point
Paramater:      index       : index of the point 
                *seeds      : pointer to neighbours already found
                cluster_id  : cluster id 
                *points     : pointer to points 
                num_points  : number of data points 
                epsilon_1   : distance limit for parameter 1 
                epsilon_2   : distance limit for parameter 2
                epsilon_3   : distance limit for parameter 3     
                minpts   : minimum number of points in a cluster 
                *dist   : pointer to the function for distance calculation (currently not used) 
Return:         spread result
*********************************************************/
int16 spread(
    uint16 index,
    Epsilon_Neighbours_TypeDef *seeds,
    uint16 cluster_id,
    OBJ_TypeDef *points,
    uint16 num_points,
    double epsilon_1,
    double epsilon_2,
    double epsilon_3,	
    uint16 minpts,
    double (*dist)(OBJ_TypeDef *a, OBJ_TypeDef *b));
    
/*********************************************************
Function name: euclideanDist
Description:   calculate Euclidian distance of two points
Paramater:     *a   : pointer of 1st point 
               *b   : pointer of 2nd point
Return:        Euclidian distance
*********************************************************/
double euclideanDist(OBJ_TypeDef *a, OBJ_TypeDef *b);

/*********************************************************
Function name: printPoints
Description:   output clustered points
Paramater:     *points       : storage of the points
               num_points    : number of points
Return:        void
*********************************************************/
void printPoints(
    OBJ_TypeDef *points,
    uint16 num_points);


#endif

