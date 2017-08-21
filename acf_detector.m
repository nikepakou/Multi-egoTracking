img = 'F:\Master\��Ƶ���ݼ�\Ours\Camera1\img-1.jpg';
detector = peopleDetectorACF;
for i=1:n
	imgname = sprintf('src\\%d.jpg',i);
	I = imread(imgname);
	[bboxes,scores] = detect(detector,I);
	I = insertObjectAnnotation(I,'rectangle',bboxes,scores);
	%figure
	%imshow(I)
	%title('Detected People and Detection Scores');
	imwrite(I,"dest\\%d.jpg",i);
end