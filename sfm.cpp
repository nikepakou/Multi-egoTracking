#include "sfm.h"

#include "detector\corner\Corner.h"
#include "detector\corner\utils.h"
#include "detector\corner\xform.h"
#include "detector\corner\sift.h"
#include "detector\corner\imgfeatures.h"
#include "detector\corner\kdtree.h"

#pragma comment(lib, "opencv_core310d.lib ")
#pragma comment(lib,"opencv_calib3d310d.lib")
#pragma comment(lib,"opencv_xfeatures2d310d.lib")
#pragma comment(lib,"opencv_features2d310d.lib")
#pragma comment(lib,"opencv_imgproc310d.lib")
#pragma comment(lib,"opencv_ximgproc310d.lib")
#pragma comment(lib,"opencv_highgui310d.lib")
#pragma comment(lib,"opencv_ml310d.lib")
#pragma comment(lib,"opencv_imgcodecs310d.lib")

//��������
Mat K(Matx33d(
	1459.9, 0, 624.0,
	0, 1468.9, 350.1,
	0, 0, 1));

int get_matched_points(char* file1, char* file2, vector<Point2f>& p1, vector<Point2f>& p2)
{
	IplImage *img1 = cvLoadImage(file1);
	IplImage* img2 = cvLoadImage(file2);
	IplImage *img1_Feat = cvCloneImage(img1);//����ͼ1�������������������  
	IplImage *img2_Feat = cvCloneImage(img2);//����ͼ2�������������������  

	IplImage* stacked, *stacked_ransac;
	struct feature *feat1, *feat2;//feat1��ͼ1�����������飬feat2��ͼ2������������  
	int n1, n2;//n1:ͼ1�е������������n2��ͼ2�е����������  
	struct feature *feat;//ÿ��������  
	struct kd_node *kd_root;//k-d��������  
	struct feature **nbrs;//��ǰ�����������ڵ�����  
	int matchNum;//�������ֵ��ɸѡ���ƥ���Եĸ���  
	struct feature **inliers;//��RANSACɸѡ����ڵ�����  
	int n_inliers;//��RANSAC�㷨ɸѡ����ڵ����,��feat1�о��з���Ҫ���������ĸ���  

	//Ĭ����ȡ����LOWE��ʽ��SIFT������  
	//��ȡ����ʾ��1��ͼƬ�ϵ�������  
	n1 = sift_features(img1, &feat1);//���ͼ1�е�SIFT������,n1��ͼ1�����������  
	//export_features("feature1.txt", feat1, n1);//��������������д�뵽�ļ�  
	draw_features(img1_Feat, feat1, n1);//����������  
	//cvShowImage("img1_Feat", img1_Feat);//��ʾ  

	//��ȡ����ʾ��2��ͼƬ�ϵ�������  
	n2 = sift_features(img2, &feat2);//���ͼ2�е�SIFT�����㣬n2��ͼ2�����������  
	//export_features("feature2.txt", feat2, n2);//��������������д�뵽�ļ�  
	draw_features(img2_Feat, feat2, n2);//����������  
	//cvShowImage("img2_Feat", img2_Feat);//��ʾ  

	CvPoint pt1, pt2;//���ߵ������˵�  
	double d0, d1;//feat1��ÿ�������㵽����ںʹν��ڵľ���  
	matchNum = 0;//�������ֵ��ɸѡ���ƥ���Եĸ���  

	//��2��ͼƬ�ϳ�1��ͼƬ,��������  
	stacked = stack_imgs(img1, img2);//�ϳ�ͼ����ʾ�������ֵ��ɸѡ���ƥ����  
	stacked_ransac = stack_imgs(img1, img2);//�ϳ�ͼ����ʾ��RANSAC�㷨ɸѡ���ƥ����  

	//����ͼ2�������㼯feat2����k-d��������k-d������kd_root  
	kd_root = kdtree_build(feat2, n2);

	//���������㼯feat1�����feat1��ÿ��������feat��ѡȡ���Ͼ����ֵ������ƥ��㣬�ŵ�feat��fwd_match����  
	for (int i = 0; i < n1; i++)
	{
		feat = feat1 + i;//��i���������ָ��  
		//��kd_root������Ŀ���feat��2������ڵ㣬�����nbrs�У�����ʵ���ҵ��Ľ��ڵ����  
		int k = kdtree_bbf_knn(kd_root, feat, 2, &nbrs, KDTREE_BBF_MAX_NN_CHKS);
		if (k == 2)
		{
			d0 = descr_dist_sq(feat, nbrs[0]);//feat������ڵ�ľ����ƽ��  
			d1 = descr_dist_sq(feat, nbrs[1]);//feat��ν��ڵ�ľ����ƽ��  
			//��d0��d1�ı�ֵС����ֵNN_SQ_DIST_RATIO_THR������ܴ�ƥ�䣬�����޳�  
			if (d0 < d1 * NN_SQ_DIST_RATIO_THR)
			{   //��Ŀ���feat������ڵ���Ϊƥ����  
				pt1 = CvPoint(cvRound(feat->x), cvRound(feat->y));//ͼ1�е������  
				pt2 = CvPoint(cvRound(nbrs[0]->x), cvRound(nbrs[0]->y));//ͼ2�е������(feat������ڵ�)  
				pt2.y += img1->height;//��������ͼ���������еģ�pt2�����������ͼ1�ĸ߶ȣ���Ϊ���ߵ��յ�  
				cvLine(stacked, pt1, pt2, CV_RGB(255, 0, 255), 1, 8, 0);//��������  
				matchNum++;//ͳ��ƥ���Եĸ���  
				feat1[i].fwd_match = nbrs[0];//ʹ��feat��fwd_match��ָ�����Ӧ��ƥ���  
			}
		}
		free(nbrs);//�ͷŽ�������  
	}
	printf("�������ֵ��ɸѡ���ƥ���Ը�����%d\n", matchNum);
	//��ʾ�����澭�����ֵ��ɸѡ���ƥ��ͼ  
	//cvNamedWindow("siftmatch");//��������  
	//cvShowImage("siftmatch", stacked);//��ʾ  
	//cvSaveImage("siftmatch.jpg", stacked);
	//����RANSAC�㷨ɸѡƥ���,����任����H  
	//CvMat * H = ransac_xform(feat1, n1, FEATURE_FWD_MATCH, lsq_homog, 4, 0.01, homog_xfer_err, 3.0, &inliers, &n_inliers);
	CvMat* H = ransac_xform(feat1, n1, FEATURE_FWD_MATCH, lsq_homog, 4, 0.2, homog_xfer_err, 16.0, &inliers, &n_inliers);
	if (H == NULL){
		ERROR("%s %s have not enough match points", file1, file2);
		return -1;
	}
	//printf("��RANSAC�㷨ɸѡ���ƥ���Ը�����%d\n", n_inliers);
	//������RANSAC�㷨ɸѡ��������㼯��inliers���ҵ�ÿ���������ƥ��㣬��������  
	for (int i = 0; i<n_inliers; i++)
	{
		feat = inliers[i];//��i��������  
		p1.push_back(CvPoint2D32f(feat->x, feat->y));//ͼ1�е������  
		p2.push_back(CvPoint2D32f(feat->fwd_match->x, feat->fwd_match->y));//ͼ2�е������(feat��ƥ���)  
		//printf("( %d, %d )--->( %d, %d )\n", pt1.x, pt1.y, pt2.x, pt2.y);
		//pt2.y += img1->height;//��������ͼ���������еģ�pt2�����������ͼ1�ĸ߶ȣ���Ϊ���ߵ��յ�  
		//cvLine(stacked_ransac, pt1, pt2, CV_RGB(255, 0, 255), 1, 8, 0);//��������  
	}
	//cvNamedWindow("RansacMatch");//��������  
	//cvShowImage("RansacMatch", stacked_ransac);//��ʾ 
	//cvSaveImage("ransacmatch.jpg", stacked_ransac);
	cvReleaseImage(&stacked_ransac);//��ʾ
	cvReleaseImage(&stacked);
	cvReleaseImage(&img1);
	cvReleaseImage(&img2);
	kdtree_release(kd_root);
	free(feat1);
	free(feat2);
	return 0;
}

/*���R,T*/
bool find_transform(Mat& K, vector<Point2f>& p1, vector<Point2f>& p2, Mat& R, Mat& T)
{
	//�����ڲξ����ȡ����Ľ���͹������꣨�������꣩
	double focal_length = 0.5*(K.at<double>(0) + K.at<double>(4));   //��λ������/���ף� //focal_length = 51.024;

	Point2d principle_point(K.at<double>(2), K.at<double>(5));

	//����ƥ�����ȡ��������ʹ��RANSAC����һ���ų�ʧ���
	Mat E = findEssentialMat(p1, p2, focal_length, principle_point, RANSAC, 0.999, 1.0);  //ʲô����
	//Mat E = findEssentialMat(p1, p2, focal_length, principle_point, RANSAC, 0.999, 1.0);
	if (E.empty())
		return false;

	//�ֽⱾ�����󣬻�ȡ��Ա任
	int pass_count = recoverPose(E, p1, p2, R, T, focal_length, principle_point);

	//ͬʱλ���������ǰ���ĵ������Ҫ�㹻��
	//if (((double)pass_count) / feasible_count < 0.7)
	//	return false;

	return true;
}

void reconstruct(Mat& K, Mat& R, Mat& T, vector<Point2f>& p1, vector<Point2f>& p2, Mat& structure)
{
	//���������ͶӰ����[R T]��triangulatePointsֻ֧��float��
	Mat proj1(3, 4, CV_32FC1);
	Mat proj2(3, 4, CV_32FC1);

	proj1(Range(0, 3), Range(0, 3)) = Mat::eye(3, 3, CV_32FC1);
	proj1.col(3) = Mat::zeros(3, 1, CV_32FC1);

	R.convertTo(proj2(Range(0, 3), Range(0, 3)), CV_32FC1);
	T.convertTo(proj2.col(3), CV_32FC1);

	Mat fK;
	K.convertTo(fK, CV_32FC1);
	proj1 = fK*proj1;
	proj2 = fK*proj2;

	//�����ؽ�
	triangulatePoints(proj1, proj2, p1, p2, structure);
}

/*
Mat R, T;	//��ת�����ƽ������
Mat structure;	//4��N�еľ���ÿһ�д���ռ��е�һ���㣨������꣩
*/
int calc3DPos(int img_index, Mat& R, Mat& T, Mat& structure)
{
	int ret = 0;
	char img1[128] = { 0 };
	char img2[128] = { 0 };
	//����任����
	vector<Point2f> p1, p2;
	
	sprintf(img1, "H:\\Master\\��Ƶ���ݼ�\\Ours\\Camera1\\img-%d.jpg", img_index + 2);
	sprintf(img2, "H:\\Master\\��Ƶ���ݼ�\\Ours\\Camera2\\img-%d.jpg", img_index);
	ret = get_matched_points(img1, img2, p1, p2);

	if (ret == -1 || !find_transform(K, p1, p2, R, T)){
		ERROR("cannot find transform");
		return -1;
	}
	
	Target tg1 = getTargetByIndex(1, img_index, 1);//��ӵ�һ��Ŀ�꣬�ڶ���Ŀ��켣����Ϊƥ���
	Target tg2 = getTargetByIndex(2, img_index-2, 1);
	if (tg1.camera_index == -1 || tg2.camera_index == -1)
	{
		ERROR("cannot calc target 3d pos");
		return -1;
	}
	p1.insert(p1.begin(), tg1.trackPos);
	p2.insert(p2.begin(), tg2.trackPos);
	
	tg1 = getTargetByIndex(1, img_index, 2);
	tg2 = getTargetByIndex(2, img_index - 2, 2);
	if (tg1.camera_index == -1 || tg2.camera_index == -1)
	{
		ERROR("cannot calc target 3d pos");
		return -1;
	}
	p1.insert(p1.begin(), tg1.trackPos);
	p2.insert(p2.begin(), tg2.trackPos);
	
	reconstruct(K, R, T, p1, p2, structure);   //���һ��
	
	return 0;
}

void calc2DPosby3D()
{
	//the sync frame reconstruct
	int img_index = 0;
	Mat R;
	Mat T;
	Mat structure;
	vector<Target> target_3D;
	vector<Mat> rotations;
	vector<Mat> motions;
	vector<Mat> target3DPos;
	vector<Mat> pos2D;
	FileStorage fs("H:\\ouweiqi\\LittlePaper\\Code\\Multi-egoTracking\\result\\3D\\3dpoints.yml", FileStorage::WRITE);
	while (++img_index < frame_num) {
		if (0 != calc3DPos(img_index, R, T, structure)){
			continue;
		}
		rotations.push_back(R);
		motions.push_back(T);
		target3DPos.push_back(structure.colRange(0,1));
		cout << "two egocentric " << img_index << "and " << img_index << "reconstruction over!" << endl;

		fs << "img_index" << img_index;
		fs << "Rotation" << R;
		fs << "Motion" << T;

		Target tg;
		Mat m;
		K.convertTo(m, CV_32F);
		Mat_<float> c = structure.col(0).rowRange(0,3);
		fs << "target1_3D" << Point3f(c(0), c(1), c(2));
		
		c /= c(2);	//������꣬��Ҫ�������һ��Ԫ�ز�������������ֵ
		c.convertTo(c, CV_32F);
		Mat_<float> ret = m*c;
		fs << "target1_2D" << ret;
		
		tg.camera_index = 1;
		tg.img_index = img_index;
		tg.target_index = 1;
		tg.trackPos = CvPoint(ret(0), ret(1));
		tg.rt = getTargetByIndex(1, img_index-1, 1).rt;
		target_3D.push_back(tg);

		c = structure.col(1).rowRange(0,3);
		fs << "target2_3D" << Point3f(c(0), c(1), c(2));
		c /= c(2);	//������꣬��Ҫ�������һ��Ԫ�ز�������������ֵ
		c.convertTo(c, CV_32F);
		ret = m * c;
		fs << "target1_2D" << m * c;
		tg.camera_index = 1;
		tg.img_index = img_index;
		tg.target_index = 2;
		tg.trackPos = CvPoint(ret(0), ret(1));
		tg.rt = getTargetByIndex(1, img_index - 1, 2).rt;
		target_3D.push_back(tg);
	}
	drawFrom3DPos(target_3D);
	fs.release();
}

/*
void calcConsistFrameRT()
{
	int Index = 0;
	//the continuous frame reconstruct in egocentric-1
	while (++Index < 220)
	{
		sprintf(task.img1, "D:\\ouweiqi\\LittlePaper\\data\\Camera1\\img-%d.jpg", Index + frame_offset);
		sprintf(task.img2, "D:\\ouweiqi\\LittlePaper\\data\\Camera1\\img-%d.jpg", Index + frame_offset + 1);
		sprintf(task.resultpath, ".\\result\\structure_C1_%d_C1_%d.yml", Index, Index + 1);
		try{
			adjustPosBy3D(Index);
			cout << "egocentric-1 " << Index << "and " << Index + 1 << " reconstruction over!" << endl;
		}
		catch (Exception e){
			cout << "egocentric-1 " << Index << "and " << Index + 1 << " reconstruction failed!" << endl;
		}
	}

	//the continuous frame reconstruct in egocentric-2
	Index = 0;
	while (++Index < 220){
		sprintf(task.img1, "D:\\ouweiqi\\LittlePaper\\data\\Camera2\\img-%d.jpg", Index);
		sprintf(task.img2, "D:\\ouweiqi\\LittlePaper\\data\\Camera2\\img-%d.jpg", Index + 1);
		sprintf(task.resultpath, ".\\result\\structure_C2_%d_C2_%d.yml", Index, Index + 1);
		try{
			adjustPosBy3D(Index);
			cout << "egocentric-2 " << Index << "and " << Index + 1 << " reconstruction over!" << endl;
		}
		catch (Exception e){
			cout << "egocentric-2 " << Index << "and " << Index + 1 << " reconstruction failed!" << endl;
		}
	}
}*/

void showSiftMatch(string img1, string img2, vector<vector<KeyPoint>>& key_points_for_all,
	vector<DMatch> &matches)
{
	namedWindow("SIFT_matches");
	Mat img_matches;
	//�����ͼ���л���ƥ����

	Mat img_1 = imread(img1);
	Mat img_2 = imread(img2);
	drawMatches(img_1, key_points_for_all[0], img_2, key_points_for_all[1], matches,
		img_matches, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);								//ƥ�����ͼ��);						//�ð�ɫֱ����������ͼ���е�������
	imshow("SIFT_matches", img_matches);
	waitKey();
}