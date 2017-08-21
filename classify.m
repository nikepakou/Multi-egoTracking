load 'F:\ouweiqi\LittlePaper\Code\Multi-egoTracking\result\acf_c2\detect_ret.mat';

I = imread(detect_ret(1).imgname);
cellSize = [4 4]; 
trainLabel = [];
trainLabel = [trainLabel; 1];%liu
trainLabel = [trainLabel; 2];%bai

n = length(trainLabel);
for i = 1:n
    img = imresize(imcrop(I,detect_ret(1).bboxes(i,:)), [640 360]);
    img = rgb2gray(img);
    % Apply pre-processing steps
    img = imbinarize(img);
    trainFeatures(i, :) = extractHOGFeatures(img, 'CellSize', cellSize);
end
% fitcecoc uses SVM learners and a 'One-vs-One' encoding scheme.
classify = fitcecoc(trainFeatures, trainLabel);

[n,hogFeatureSize] = size(trainFeatures);

%--------------------------------------------------------

n = length(detect_ret);
testFeatures  = zeros(n-1, hogFeatureSize, 'single');
count = 1;
for i = 2:n
    I = imread(detect_ret(i).imgname);
    [a,b] = size(detect_ret(i).bboxes);
    for j = 1:a
        img = imresize(imcrop(I,detect_ret(i).bboxes(j,:)), [640 360]);
        imwrite(img,['F:\ouweiqi\LittlePaper\Code\Multi-egoTracking\result\acf_c2\testSet\img-' num2str(i) '-' num2str(j) '.jpg']);
        img = rgb2gray(img);
        % Apply pre-processing steps
        img = imbinarize(img);
        testFeatures(count, :) = extractHOGFeatures(img,'CellSize',cellSize);
        count = count + 1;
    end
end

%-----------------------------------------------------------------------------

% Make class predictions using the test features.
predictedLabels = predict(classify, testFeatures);

% Tabulate the results using a confusion matrix.
%confMat = confusionmat(testLabels, predictedLabels);

%helperDisplayConfusionMatrix(confMat)