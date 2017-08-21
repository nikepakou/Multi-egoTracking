#include "sfm.h"
#include "FileIO.h"

vector<Target> target_set;

//�����ֹ���ǵ� ����anchorPoint1 �� anchorPoint2
void loadAnchorPoint()
{
	int Index1 = 0;
	int Index2 = 0;
	Point tmp1;
	Point tmp2;
	char line[512] = { 0 };
	vector<Point2f> p1, p2;

	FILE* fp = fopen("D:\\ouweiqi\\LittlePaper\\data\\MatchPoint.txt", "r+");
	if (!fp){
		printf("�޷���ȡMatchPoint.txt");
	}
	while (1){
		if (fscanf(fp, "\n%[^\n]", line) == EOF)
			break;
		if (line[0] == 'i'){
			//sscanf(line, "img%d.jpg-img%d.jpg", Index1, Index2);
			if (p1.empty() || p2.empty())
				continue;
			anchorPoint1.push_back(p1);
			anchorPoint2.push_back(p2);
			p1.clear();
			p2.clear();
		}
		else{
			sscanf(line, "%d %d %d %d", &tmp1.x, &tmp1.y, &tmp2.x, &tmp2.y);
			p1.push_back(tmp1);
			p2.push_back(tmp2);
		}
	}
	fclose(fp);
}

/********************************������Homograph����õ���2D�켣��***************************/
void export_2DTracklet(char* filename, vector<vector<Point2f> > &tracklet)
{
	FILE* file;
	int i;

	if (tracklet.size() == 0) return;
	file = fopen(filename, "w");

	for (i = 0; i < tracklet.size(); i++)
	{
		vector<Point2f> feat = tracklet.at(i);
		fprintf(file, "%f %f %f %f\n", feat[0].x, feat[0].y,
			feat[1].x, feat[1].y);
	}

	fclose(file);
}

/***************************************************************
* ����Ŀ��Ľŵ���ΪĿ���־��
*   o_x = original.x + weight/2;
*	o_y = original.y + height;
*	���ǳ����д���һ��Ŀ�꣬Ŀ��Ϊ�գ���������Ŀ������
*	
***************************************************************/
int loadObjectPos(char* detect_ret_path, int camera_index)
{
	int index = -1;
	char path[128] = { 0 };
	Target tg;
	//��һ�ӽ�Ŀ��data
	FILE *fp = fopen(detect_ret_path, "r");
	if (!fp){
		ERROR("Cannot read file: data1.txt");
		return -1;
	}
	while (fscanf(fp, "%s %d %d %d %d %d", path, &(tg.rt.x), &(tg.rt.y), &(tg.rt.width), &(tg.rt.height), &(tg.target_index)) != EOF){
		sscanf(path, "img-%d.jpg", &index);
		tg.camera_index = camera_index;
		tg.img_index = index;
		tg.rt.width -= tg.rt.x;
		tg.rt.height -= tg.rt.y;
		tg.trackPos.x = tg.rt.x + tg.rt.width / 2;
		tg.trackPos.y = tg.rt.y + tg.rt.height;

		target_set.push_back(tg);
	}
	fclose(fp);
	return 0;
}

int findOcclusionTarget()
{
	if (target_set.empty())
		return -1;
	FILE* fp = fopen("H:\\Master\\��Ƶ���ݼ�\\Ours\\occlusion.txt", "w+");
	if (!fp){
		ERROR("Cant read file occlusion.txt");
		return -1;
	}
	for (int i = 1; i <= frame_num; i++){
		if (getTargetByIndex(1, i, 1).camera_index == -1)
			fprintf(fp, "1 %d 1\n", i);
		if (getTargetByIndex(1, i, 2).camera_index == -1)
			fprintf(fp, "2 %d 1\n", i);
		if (getTargetByIndex(2, i, 1).camera_index == -1)
			fprintf(fp, "1 %d 2\n", i);
		if (getTargetByIndex(2, i, 2).camera_index == -1)
			fprintf(fp, "2 %d 2\n", i);
	}
	fclose(fp);
}

Target& getTargetByIndex(int camera_index, int img_index, int target_index)
{
	Target tg;
	vector<Target>::iterator it = target_set.begin();
	for (; it != target_set.end(); it++){
		if (it->camera_index == camera_index && it->img_index == img_index && it->target_index == target_index){
			tg = *it;
			break;
		}
	}
	return tg;
}

void save_homo_result(vector<Target>& occlusion_target, vector<CvMat*>& homo_set)
{
	FileStorage fs("H:\\ouweiqi\\LittlePaper\\Code\\Multi-egoTracking\\result\\homo\\occlusion_target.yml", FileStorage::WRITE);
	vector<CvMat*>::iterator it_homo = homo_set.begin();
	vector<Target>::iterator it_target = occlusion_target.begin();
	for (; it_target != occlusion_target.end(); it_target++, it_homo++){
		if (it_target->camera_index == -1){
			it_homo--;
			continue;
		}
		fs << "target_camera" << it_target->camera_index << "target_img" << it_target->img_index << "target_index" << it_target->target_index;
		fs << "target_rect" << it_target->rt << "target_trackPos" << it_target->trackPos;
		Mat homo = cvarrToMat(*it_homo);
		fs << "Homograpy" << homo;
	}
	fs.release();
	
	drawHomoPos(occlusion_target);
}

int save_3D_ret(string file_name, vector<Mat>& rotations, vector<Mat>& motions, vector<Mat>& target3DPos)
{
	int n = (int)rotations.size();
	FileStorage fs(file_name, FileStorage::WRITE);
	
	for (int i = 0; i < n; i++) {
		fs << "Rotations" << rotations[i];
		fs << "Motions" << motions[i];

		Mat_<float> c = target3DPos[i].col(0);
		c /= c(3);	//������꣬��Ҫ�������һ��Ԫ�ز�������������ֵ
		fs << "target1_3D" << Point3f(c(0), c(1), c(2));
		c /= c(2);
		fs << "target1_2D" << K * c;
		
		c = target3DPos[i].col(1);
		c /= c(3);	//������꣬��Ҫ�������һ��Ԫ�ز�������������ֵ
		fs << "target2_3D" << Point3f(c(0), c(1), c(2));
		c /= c(2);
		fs << "target1_2D" << K * c;
	}

	fs.release();
	return 0;
}

int getOcclusionTarget(vector<Target>& occlusion_target)
{
	FILE* fp = fopen("H:\\Master\\��Ƶ���ݼ�\\Ours\\occlusion.txt", "r");
	if (!fp){
		perror("Cant read file Cam2_data2.txt");
		return -1;
	}
	Target tg;
	int index = 0,target=0,view=0;
	while (fscanf(fp, "%d %d %d", &tg.target_index, &tg.img_index, &tg.camera_index) != EOF){
		occlusion_target.push_back(tg);
	}
	fclose(fp);
	return 0;
}

void ShowTracklet()
{
	char rook_window[] = "Drawing 2: Rook";
	Mat rook_image = Mat::zeros(720, 1280, CV_8UC3);

	//for (int i = 0; i < resPos.size(); i++){
	for (int i = 0; i < 100; i++){
		//circle(rook_image, tracklet_Cam1[i + 1].pos[1], 4, Scalar(255, 255, 0));//blue //tracklet_Cam1 �±��1��ʼ
		//line(rook_image, tracklet_Cam1[i + 1].pos[1], tracklet_Cam1[i + 2].pos[1], Scalar(0, 0, 255));
		//addText(rook_image, "")
		//CvFont font = cvFont(3);
		//char tmp[5] = {0};
		//sprintf(tmp, "%d", i + 1);
		//putText(rook_image, tmp, tracklet_Cam1[i + 1].pos[1],FONT_HERSHEY_SCRIPT_SIMPLEX, 0.6, Scalar(255, 0, 0));

		circle(rook_image, resPos.at(i).at(0), 6, Scalar(255, 0, 0));//red(0,0,255)
		line(rook_image, resPos.at(i).at(0), resPos.at(i + 1).at(0), Scalar(255, 0, 0));
		//putText(rook_image, tmp, p[1], FONT_HERSHEY_SCRIPT_SIMPLEX, 0.3, Scalar(0, 0, 255));
	}
	imwrite("D:\\ouweiqi\\LittlePaper\\Code\\Multi-egoTracking\\2DResult\\2d_result_consist_2.jpg", rook_image);
	imshow(rook_window, rook_image);
	moveWindow(rook_window, 1, 1);
	waitKey(0);
}

int drawHomoPos(vector<Target>& target)
{
	int ret = 0;
	char imgpath[128] = { 0 };

	vector<Target>::iterator it_target = target.begin();
	for (; it_target != target.end(); it_target++) {
		if (it_target->camera_index == -1){
			continue;
		}
		sprintf(imgpath, "H:\\Master\\��Ƶ���ݼ�\\Ours\\Camera%d\\img-%d.jpg", it_target->camera_index,it_target->img_index);
		Mat img = imread(imgpath);
		if (img.empty()){
			ERROR("cannot imread %s", imgpath);
			return -1;
		}
		rectangle(img, it_target->rt, Scalar(255, 255, 0), 2.5, 8);
		sprintf(imgpath, "H:\\ouweiqi\\LittlePaper\\Code\\Multi-egoTracking\\result\\homo\\camera_%d_img_%d_target_%d.jpg", it_target->camera_index, it_target->img_index, it_target->target_index);
		imwrite(imgpath,img);
	}
	return ret;
}

int drawFrom3DPos(vector<Target>& target)
{
	int ret = 0;
	char imgpath[128] = { 0 };

	vector<Target>::iterator it_target = target.begin();
	for (; it_target != target.end(); it_target++) {
		if (it_target->camera_index == -1){
			continue;
		}
		sprintf(imgpath, "H:\\Master\\��Ƶ���ݼ�\\Ours\\Camera%d\\img-%d.jpg", it_target->camera_index, it_target->img_index);
		Mat img = imread(imgpath);
		if (img.empty()){
			ERROR("cannot imread %s", imgpath);
			return -1;
		}
		rectangle(img, it_target->rt, Scalar(255, 255, 0), 2.5, 8);
		sprintf(imgpath, "H:\\ouweiqi\\LittlePaper\\Code\\Multi-egoTracking\\result\\3D\\camera_%d_img_%d_target_%d.jpg", it_target->camera_index, it_target->img_index, it_target->target_index);
		imwrite(imgpath, img);
	}
	return ret;
}