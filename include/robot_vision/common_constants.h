#ifndef _COMMON_CONSTANTS_H_
#define _COMMON_CONSTANTS_H_

#define RED 0xFF0000
#define GREEN 0x00FF00
#define BLUE 0x0000FF
#define YELLOW 0xFFFF00
#define CYAN 0x00FFFF
#define MAGENTA 0xFF00FF

const double bowlLength=0.1282528;
const double bowlWidth=0.121;
const double bowlHeight=0.1157708;

const double mugLength=0.0943486;
const double mugWidth=0.127;
const double mugHeight=0.1155481;

static const int color_array[] = {RED,GREEN,BLUE,YELLOW,CYAN,MAGENTA};

extern double voxelLeafSize;
extern int removePlaneIterations;
extern double distanceThreshold;
extern double PercentageCloud;
extern double tolerance;
extern int minClustersSize ;
extern int maxClustersSize ;


#endif
