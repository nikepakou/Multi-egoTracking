#ifndef FILEIO_H_
#define FILEIO_H_

#include "common.h"

extern Mat K;

extern int getOcclusionTarget(vector<Target>& occlusion_target);

extern void loadAnchorPoint();

extern void export_2DTracklet(char* filename, vector<vector<Point2f> > &tracklet);

extern int loadObjectPos(char* path, int camera_index);

extern int findOcclusionTarget();

extern Target& getTargetByIndex(int camera_index, int img_index, int target_index);

extern void save_homo_result(vector<Target>& target, vector<CvMat*>& homo_set);

extern void save_structure(string file_name, vector<Mat>& rotations, vector<Mat>& motions, Mat& structure);

extern int drawHomoPos(vector<Target>& target);

extern void ShowTracklet();

extern void GetTracklet();

extern int drawFrom3DPos(vector<Target>& target);
#endif