%only for RGB image homography

tic;
clc;
clear all;
close all
f = 'window_view_';
ext = 'jpg';
img1 = imread([f '1.' ext]);
img2 = imread([f '2.' ext]);

if size(img1,3)==1%to find whether input is RGB image            ��ֵΪ1��Ϊ��ͨ��ͼ��Ϊ3��Ϊ��ͨ��ͼ��
fprintf('error,only for RGB images\n');
end

img1Dup=rgb2gray(img1);%duplicate img1
img1Dup=double(img1Dup);       %     ��matlab����ͼ���������uint8����matlab����ֵһ�����double�ͣ�64λ���洢�����㡿

img2Dup=rgb2gray(img2);%duplicate img2
img2Dup=double(img2Dup);

% use Harris in both images to find corner.              ��Shi-Tomasi�ǵ��⡿
number1=1;
[locs1] = FastShiTomasi(img1Dup,number1);
number2=2;
[locs2] = FastShiTomasi(img2Dup,number2);
toc;
fprintf('FastShiTomasi1  %d points.\n',size(locs1,1) );
fprintf('FastShiTomasi2  %d points.\n',size(locs2,1) );

%using NCC to find coorespondence between two images          ��Ѱ��ƥ�������꡿
[matchLoc1 matchLoc2] =  findCorr(img1Dup,img2Dup,locs1, locs2);
fprintf('NCC points %d matches left.\n',size([matchLoc1 matchLoc2],1) );

% use RANSAC to find homography matrix                        ������RANSAC�㷨�ҵ����ŵĵ�Ӧ�Ծ���
[H inlierIdx] = estHomography(img1Dup,img2Dup,matchLoc2',matchLoc1');
 %H  %#ok
fprintf('RANSAC points %d matches left.\n',size(inlierIdx,2));
 
[imgout]=warpTheImage(H,img1,img2);       %              ��ͼ��ƴ�Ӳ��ںϡ�
%imshow(imgout);title('final image');

figure,imshow(uint8(imgout));       %       ��ֻдimshow����֮ǰ�Ĺ�����������ʾ����figure,imshow���½�һ��������ʾͼ��
toc;
