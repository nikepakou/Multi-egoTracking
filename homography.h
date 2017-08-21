#ifndef __HOMOGRAPHY__
#define __HOMOGRAPHY__

#include "FileIO.h"

extern Mat K;

extern CvMat* match(char* file1, char* file2);
extern int calcPosbyH();
extern Target& getTargetByIndex(int, int, int);

#endif