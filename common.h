#ifndef COMMON_H
#define COMMON_H

#include <vector>
#include <iostream>
#include <opencv.hpp>
#include <opencv2\xfeatures2d\nonfree.hpp>
#include <opencv2\features2d\features2d.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\calib3d\calib3d.hpp>
#include <highgui.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <map>

using namespace std;
using namespace cv;

#define frame_num       220
#define frame_offset	2  //֡ͬ��


class Target{
public:
	int camera_index;
	int img_index;
	int target_index;
	Rect rt;
	Point trackPos;
	Target(){
		camera_index = -1;
		img_index = -1;
		target_index = -1;
	}
	Target& operator = (const Target& tg){
		camera_index = tg.camera_index;
		img_index = tg.img_index;
		target_index = tg.target_index;
		rt = tg.rt;
		trackPos = tg.trackPos;
		return *this;
	}
};

extern vector<Target> target_set;
extern Target& getTargetByIndex(int camera_index, int img_index, int target_index);

#define ERROR(format, ...) printf("File: "__FILE__", Line: %05d: "format"\n", __LINE__, ##__VA_ARGS__)

extern vector<vector<Point2f> > anchorPoint1;  //�洢��Ƶǰ����֡��Ӧ��ԣ���MatchPoint.txt�м���
extern vector<vector<Point2f> > anchorPoint2;
extern vector<Mat> matArray;
extern vector<vector<Point2f> > resPos;   //ӳ�䵽��һ֡�Ĺ켣


#endif