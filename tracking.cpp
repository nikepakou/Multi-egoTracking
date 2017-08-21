#include "kalman.h"
#include <stdio.h>
#include "sfm.h"

#pragma comment(lib,"opencv_tracking310d.lib")
#pragma comment(lib,"opencv_video310d.lib")

extern Target& getTargetByIndex(int, int, int);

static inline Point calcPoint(Point2f center, double R, double angle)
{
	return center + Point2f((float)cos(angle), (float)-sin(angle))*(float)R;
}

int kalmanTracking()
{
	int dynamParams = 4;//�仯�����ĸ���
	int measureParams = 2;
	int controlParams = 0;
	KalmanFilter KF(dynamParams, measureParams, controlParams);
	
	/*
	 * ��������
	 */
	Mat processNoise(2, 1, CV_32F);
	Mat measurement = Mat::zeros(measureParams, 1, CV_32F);  //����ֵ
	char code = (char)-1;

	//״̬ת�ƾ���A
	KF.transitionMatrix = (Mat_<float>(4, 4) << 1, 0, 1, 0, 
		0, 1, 0, 1, 
		0, 0, 1, 0, 
		0, 0, 0, 1);
	//����û�����ÿ��ƾ���B��Ĭ��Ϊ��
	
	/*�۲����,����Խ���ֵΪ1
	 * H=[1,0,0,0;0,1,0,0]
	 */
	setIdentity(KF.measurementMatrix); 

	/*��������Э����Q,����Խ���ֵΪ10^-5*/
	setIdentity(KF.processNoiseCov, Scalar::all(1e-5));
	
	/*�۲�����Э����R*/
	setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));
	/*P�������Э�����ʼΪ��λ��*/
	setIdentity(KF.errorCovPost, Scalar::all(1));
	//��ʼλ��Ϊ���ֵ
	randn(KF.statePost, Scalar::all(0), Scalar::all(0.1));

	for (;;)
	{
		Mat prediction = KF.predict();  //Computes a predicted state.
		double predictAngle = prediction.at<float>(0);
		Point predictPt = Point(prediction.at<float>(0), prediction.at<float>(1));

		//kalman����ά����ת��λ֮��Ĺ���Ԥ��
		measurement.at<float>(0) = (float)getTargetByIndex(1, 1, 1).trackPos.x;
		measurement.at<float>(1) = (float)getTargetByIndex(1, 1, 1).trackPos.y;
		KF.correct(measurement);
		cvSave("kalmanTrackingRet.txt", &predictPt);
		code = (char)waitKey(100);

		if (code > 0)
			break;
	}
	

	return 0;
}