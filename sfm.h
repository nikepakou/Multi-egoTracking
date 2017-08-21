#ifndef __SFM_H__
#define __SFM_H__

#include "common.h"



#ifdef __cplusplus
extern "C"{
#endif

	extern vector<vector<KeyPoint>> key_points_for_all;
	extern vector<Mat> descriptor_for_all;
	extern vector<vector<Vec3b>> colors_for_all;
	extern vector<DMatch> matches;

	extern vector<vector<Point2f> > resPos;   //ӳ�䵽��һ֡�Ĺ켣

	extern void calc2DPosby3D();

	extern int loadObjectPos(char* path, int camera_index);
	extern void extract_features(
		vector<string>& image_names,
		vector<vector<KeyPoint>>& key_points_for_all,
		vector<Mat>& descriptor_for_all,
		vector<vector<Vec3b>>& colors_for_all
		);
	extern void match_features(Mat& query, Mat& train, vector<DMatch>& matches);
	extern int get_matched_points(char* img1, char* img2, vector<Point2f>& p1, vector<Point2f>& p2);

	extern void showSiftMatch(string img1, string img2, vector<vector<KeyPoint>>& key_points_for_all,
		vector<DMatch> &matches);
	
	extern int save_3D_ret(string file_name, vector<Mat>& rotations, vector<Mat>& motions, vector<Mat>& structure);

	extern int drawFrom3DPos(vector<Target>& target);

#ifdef __cplusplus
}
#endif


#endif