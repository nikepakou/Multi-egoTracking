int init(){
	/*
	*  �ռ�켣�ؽ�
	*/
	//int ret = loadObjectPos("H:\\ouweiqi\\LittlePaper\\Code\\Multi-egoTracking\\result\\acf_c1\\data1.txt", 1);
	//ret = loadObjectPos("H:\\ouweiqi\\LittlePaper\\Code\\Multi-egoTracking\\result\\acf_c2\\data2.txt", 2);
	//ret = findOcclusionTarget();

	int ret = loadObjectPos("H:\\Master\\��Ƶ���ݼ�\\Ours\\Camera1\\data1.txt", 1);
	ret = loadObjectPos("H:\\Master\\��Ƶ���ݼ�\\Ours\\Camera2\\data2.txt", 2);
	return ret;
}

void main()
{
	init();
	//calcPosbyH();
	calc2DPosby3D();
	//kalmanTracking();

	//��ȡ���Ĺ켣
	//GetTracklet();
	//ShowTracklet();
	cout << "executive over!!!" << endl;
	system("pause");
}