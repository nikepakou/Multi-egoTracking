/*
Detects SIFT features in two images and finds matches between them.

Copyright (C) 2006-2012  Rob Hess <rob@iqengines.com>

@version 1.1.2-20100521
*/

/*��һ��RANSAC��ⵥӦ�Ծ���SIFT��ǰ�����㣬����RANSAC�㷨������Ź���*/
/*�������ֶ������־�㣬����RANSAC������Ź���*/

#include "sfm.h"
#include "homography.h"
#include "FileIO.h"
#include "detector\corner\Corner.h"
#include "detector\corner\sift.h"
#include "detector\corner\imgfeatures.h"
#include "detector\corner\kdtree.h"
#include "detector\corner\utils.h"
#include "detector\corner\xform.h"

vector<vector<Point2f> > anchorPoint1;  //�洢��Ƶǰ����֡��Ӧ��ԣ���MatchPoint.txt�м���
vector<vector<Point2f> > anchorPoint2;
vector<Mat> matArray;
vector<vector<Point2f> > resPos;   //ӳ�䵽��һ֡�Ĺ켣

static int currPicIndex; //���һ֡������Ӧ��Լ����֡�������±��1��ʼ

/*ƥ�䲢��ͼ*/
CvMat * SiftMatch(char* file1, char* file2)
{
	IplImage *img1 = cvLoadImage(file1);
	IplImage* img2 = cvLoadImage(file2);
	IplImage *img1_Feat = cvCloneImage(img1);//����ͼ1�������������������  
	IplImage *img2_Feat = cvCloneImage(img2);//����ͼ2�������������������  

	IplImage* stacked, *stacked_ransac;
	struct feature *feat1, *feat2;//feat1��ͼ1�����������飬feat2��ͼ2������������  
	int i, n1, n2;//n1:ͼ1�е������������n2��ͼ2�е����������  
	struct feature *feat;//ÿ��������  
	struct kd_node *kd_root;//k-d��������  
	struct feature **nbrs;//��ǰ�����������ڵ�����  
	int matchNum;//�������ֵ��ɸѡ���ƥ���Եĸ���  
	struct feature **inliers;//��RANSACɸѡ����ڵ�����  
	int n_inliers;//��RANSAC�㷨ɸѡ����ڵ����,��feat1�о��з���Ҫ���������ĸ���  

	//Ĭ����ȡ����LOWE��ʽ��SIFT������  
	//��ȡ����ʾ��1��ͼƬ�ϵ�������  
	n1 = sift_features(img1, &feat1);//���ͼ1�е�SIFT������,n1��ͼ1�����������  
	//export_features("siftResult\feature1.txt", feat1, n1);//��������������д�뵽�ļ�  
	//draw_features(img1_Feat, feat1, n1);//����������  
	//cvSaveImage(file1, img1_Feat);//��ʾ  

	//��ȡ����ʾ��2��ͼƬ�ϵ�������  
	n2 = sift_features(img2, &feat2);//���ͼ2�е�SIFT�����㣬n2��ͼ2�����������  
	//export_features("siftResult\feature2.txt", feat2, n2);//��������������д�뵽�ļ�  
	//draw_features(img2_Feat, feat2, n2);//����������  
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
	for (i = 0; i < n1; i++)
	{
		feat = feat1 + i;//��i���������ָ��  

		//�ǵ�����ڱ�����
		bool f1=false, f2=false;
		for (int j = 0; j < 2; j++){
			Target tg = getTargetByIndex(1, currPicIndex, j);
			f1 = (feat1[i].x > tg.rt.x && feat1[i].x < tg.rt.x + tg.rt.width);
			f2 = (feat1[i].y > tg.rt.y && feat1[i].y > tg.rt.y + tg.rt.height);
			if (f1&&f2)
				break;
		}
		if (f1&&f2){
			memcpy(feat1 + i, feat1 + n1 - 1, sizeof(feature));
			n1--;
			i--;
			continue;
		}

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
	cvNamedWindow("siftmatch");//��������
	char stackImagePath[128] = {0};
	int tmp;
	char* p1, *p2;
	p1 = strrchr(file1, '-');
	p2 = strrchr(file2, '-');
	sscanf(p1, "-%d.jpg", &tmp);
	sprintf(stackImagePath, "D:\\ouweiqi\\LittlePaper\\Code\\Multi-egoTracking\\siftResult\\consist_sift_%d_%s", tmp, p2 + 1); 
	//cvSaveImage(stackImagePath, stacked);//��ʾ  

	//����RANSAC�㷨ɸѡƥ���,����任����H  
	/*
	 * 0.01(p_badxform)������Ĵ�����ʣ�������RANSAC�㷨������ı任�������ĸ���
	 */
	CvMat * H = ransac_xform(feat1, n1, FEATURE_FWD_MATCH, lsq_homog, 4, 0.01, homog_xfer_err, 3.0, &inliers, &n_inliers);
	
	printf("��RANSAC�㷨ɸѡ���ƥ���Ը�����%d\n", n_inliers);
	//������RANSAC�㷨ɸѡ��������㼯��inliers���ҵ�ÿ���������ƥ��㣬��������  
	for (int i = 0; i<n_inliers; i++)
	{
		feat = inliers[i];//��i��������  
		pt1 = CvPoint(cvRound(feat->x), cvRound(feat->y));//ͼ1�е������  
		pt2 = CvPoint(cvRound(feat->fwd_match->x), cvRound(feat->fwd_match->y));//ͼ2�е������(feat��ƥ���)  
		//printf( "( %d, %d )--->( %d, %d )\n",pt1.x,pt1.y,pt2.x,pt2.y);
		pt2.y += img1->height;//��������ͼ���������еģ�pt2�����������ͼ1�ĸ߶ȣ���Ϊ���ߵ��յ�  
		cvLine(stacked_ransac, pt1, pt2, CV_RGB(255, 0, 255), 1, 8, 0);//��������  
	}
	sprintf(stackImagePath, "D:\\ouweiqi\\LittlePaper\\Code\\Multi-egoTracking\\siftResult\\consist_ransac_%d_%s", tmp, p2 + 1);
	//cvSaveImage(stackImagePath, stacked_ransac);//��ʾ  
	cvNamedWindow("RansacMatch");//��������  
	cvShowImage("RansacMatch", stacked_ransac);//��ʾ 
	cvReleaseImage( &stacked_ransac);//��ʾ
	cvReleaseImage( &stacked );
	cvReleaseImage(&img1);
	cvReleaseImage(&img2);
	kdtree_release(kd_root);
	free(feat1);
	free(feat2);
	return H;
}

/*
 *��Ƶ1ǰ����֡���㵥Ӧ��
 *

void GetTracklet()
{
	vector<Point2f> p1, p2;
	//�����ֹ���ǵ�
	//loadAnchorPoint();
	//int lenPtSet = anchorPoint1.size();
	
	std::vector<Point2f> obj_corners(2);
	std::vector<Point2f> scene_corners(2);
	
	scene_corners[0] = getTargetByIndex(1, 1, 1).trackPos;
	scene_corners[1] = getTargetByIndex(1, 1, 2).trackPos;

	resPos.push_back(scene_corners);
	int index = 21;
	for (int i = 0; i < 100; i++)
	{
		//p1 = anchorPoint1.at(0); //��һ֡��ǵ�
		//p2 = anchorPoint2.at(i+3); //�ڶ�֡��ǵ�
		//Mat homograpy = findHomography(p2, p1, CV_FM_LMEDS);//LMeDS

		//�Ľ�RANSAC��ȡƥ���
		//currPicIndex = i + 2;
		//���ú���֡�����һ֡ӳ��
		CvMat* H = SiftMatch(tracklet_Cam1[1].img, tracklet_Cam1[i + 2].img);
		Mat homog = cvarrToMat(H);
		
		matArray.push_back(homog);
		
		obj_corners[0] = tracklet_Cam1[i + 2].pos[0];  //tracklet_Cam1 �±��1��ʼ
		obj_corners[1] = tracklet_Cam1[i + 2].pos[1];
		
		Mat hInvert;
		invert(matArray.at(i), hInvert);
		perspectiveTransform(obj_corners, scene_corners, hInvert);//�ڶ���λ��ת������һ��λ��
		//int j = i;
		//while (j--){
		//	obj_corners[0] = scene_corners[0];
		//	obj_corners[1] = scene_corners[1];
		//	perspectiveTransform(obj_corners, scene_corners, matArray.at(i-1));
		//}
		//
		//if (tracklet_Cam1[i].flag % 2 == 1){
		//	obj_corners.push_back(tracklet_Cam2[i].pos[0]);
		//	
		//	scene_corners.push_back(tmpResult[0]);
		//}
		//if (tracklet_Cam1[Index1].flag >= 2 && tracklet_Cam2[Index2].flag >= 2){
		//	obj_corners.push_back(tracklet_Cam1[Index1].pos[1]);
		//}
		
		resPos.push_back(scene_corners);
		p1.clear();
		p2.clear();
	}
	
	export_2DTracklet("D:\\ouweiqi\\LittlePaper\\Code\\Multi-egoTracking\\2DResult\\Position.txt", resPos);
}
*/

CvMat* match(char* file1, char* file2)
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
		return NULL;
	}
	printf("��RANSAC�㷨ɸѡ���ƥ���Ը�����%d\n", n_inliers);
	//������RANSAC�㷨ɸѡ��������㼯��inliers���ҵ�ÿ���������ƥ��㣬��������  
	for (int i = 0; i<n_inliers; i++)
	{
		feat = inliers[i];//��i��������  
		pt1 = CvPoint(cvRound(feat->x), cvRound(feat->y));//ͼ1�е������  
		pt2 = CvPoint(cvRound(feat->fwd_match->x), cvRound(feat->fwd_match->y));//ͼ2�е������(feat��ƥ���)  
		printf("( %d, %d )--->( %d, %d )\n", pt1.x, pt1.y, pt2.x, pt2.y);
		pt2.y += img1->height;//��������ͼ���������еģ�pt2�����������ͼ1�ĸ߶ȣ���Ϊ���ߵ��յ�  
		cvLine(stacked_ransac, pt1, pt2, CV_RGB(255, 0, 255), 1, 8, 0);//��������  
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
	return H;
}

/* brief: ��Ӧ��Ŀ��λ�ù��ƣ�ͬ��֡��
 * ���Ŀ������ڵ�
 * pt:����һ��ͼƬ�е�Ŀ��켣��
 */
int calcPosbyH()
{
	int i;
	char img1[64] = { 0 };
	char img2[64] = { 0 };
	
	vector<Target> occlusion_target;
	vector<CvMat*> homo_set;
	getOcclusionTarget(occlusion_target);

	vector<Target>::iterator it_target = occlusion_target.begin();
	FILE* fp = fopen("H:\\ouweiqi\\LittlePaper\\Code\\Multi-egoTracking\\result\\homo\\log_faile.txt", "w+");
	if (fp == NULL){
		ERROR("cannot open log_faile.txt");
		return -1;
	}
	for (; it_target != occlusion_target.end(); it_target++) {
		i = 1;
		Target tg_pre;
		while (tg_pre.camera_index == -1){
			tg_pre = getTargetByIndex(it_target->camera_index, it_target->img_index - i, it_target->target_index);
			i++;
		}
		sprintf(img1, "H:\\Master\\��Ƶ���ݼ�\\Ours\\Camera%d\\img-%d.jpg", it_target->camera_index, it_target->img_index);
		if (it_target->camera_index == 1){
			sprintf(img2, "H:\\Master\\��Ƶ���ݼ�\\Ours\\Camera2\\img-%d.jpg", it_target->img_index - frame_offset);
		}
		else{
			sprintf(img2, "H:\\Master\\��Ƶ���ݼ�\\Ours\\Camera1\\img-%d.jpg", it_target->img_index + frame_offset);
		}
		cout << img1 << endl;
		cout << img2 << endl;
		CvMat* H = match(img1, img2);
		if (H == NULL){
			fprintf(fp, "File: "__FILE__", Line: %05d  camera_index:%d img_index : %d target_index : %d, H is null\n", __LINE__, it_target->camera_index, it_target->img_index, it_target->target_index);
			it_target->camera_index = -1;
			continue;
		}
		CvPoint2D64f pt = cvPoint2D64f(it_target->trackPos.x, it_target->trackPos.y);
		CvPoint2D64f p = persp_xform_pt(pt, H);
		it_target->trackPos.x = p.x;
		it_target->trackPos.y = p.y;
		it_target->rt = tg_pre.rt;
		target_set.push_back(*it_target);
		homo_set.push_back(H);
	}
	//save_homo_result(occlusion_target, homo_set);

	for (vector<CvMat*>::iterator it = homo_set.begin(); it != homo_set.end(); it++) {
		cvReleaseMat(&(CvMat*)*it);
	}

	fclose(fp);
	return 0;
}