function [f  inlierIdx] = estHomography( img1,img2,x,y)     %   【输入：两幅图像，第二幅与第一幅图中所有能匹配的点的坐标。输出：最优的单应性矩阵，能匹配的点中的内点编号】
ransacCoef.minPtNum = 4;      %       【类似于c中的结构体】
ransacCoef.iterNum = 1500;
ransacCoef.thDist = 3;
ransacCoef.thInlrRatio = .08;

minPtNum = ransacCoef.minPtNum;
iterNum = ransacCoef.iterNum;
thInlrRatio = ransacCoef.thInlrRatio;
thDist = ransacCoef.thDist;
ptNum = size(x,2);   %            【第二幅图中匹配的点个数】
thInlr = round(thInlrRatio*ptNum);

inlrNum = zeros(1,iterNum);
fLib = cell(1,iterNum);   %              【产生空单元矩阵】

i=1;

for p = 1:iterNum
	% 1. repeated sample  4 points 
	sampleIdx = randIndex(ptNum,minPtNum);     %       【在第二幅图的匹配点中的随机抽取4个点的编号】
	%2 compute a homography from the points
    f1 = solveHomo(x(:,sampleIdx),y(:,sampleIdx));      %           【得到单应性矩阵，第二幅图的点乘以该矩阵得到第一幅图的点（x(:,sampleIdx)表示取x矩阵的对应列）】
	
	% 3 map all points using homography and calculate the distance
	dist =  calcDist(f1,x,y);            %      【求第二幅图的所有匹配点经单应性矩阵变换后与第一幅图匹配点的距离的平方】
	inlier1 = find(dist < thDist);       %      【获得距离平方小于阈值的匹配对的编号】
	inlrNum(p) = length(inlier1);        %      【length求矩阵的行列的最大值】
	if length(inlier1) < thInlr, continue; end     %    【若内点个数小于阈值，结束单次循环（不是结束整个循环，break才是）】
	% 4 compute  a least squares using All the inliers ,if its inlier
	% ration is bigger than threshold Inlier
    
    opp(i)= p;
    i=i+1;
    
    fLib{p} =solveHomo(x(:,inlier1),y(:,inlier1));         %    【否则根据内点重新求单应性矩阵，将结果保存在超矩阵中】
end

%5 find the Homography with the most outlier
[val1,idx] = max(inlrNum);    %       【得到该行矩阵的最大值（内点个数）val1与索引idx】
f = fLib{idx};

dist =  calcDist(f,x,y);
inlierIdx = find(dist < thDist);

% Create a new image showing the two images side by side.
img3 = appendimages(img1,img2);

% Show a figure with lines joining the accepted matches.
figure('Position', [100 100 size(img3,2) size(img3,1)]);
colormap('gray');
imagesc(img3);
hold on;
cols1 = size(img1,2);
%fprintf('after using RANSAC  %d matches left.\n',size(inlierIdx,2) );
for i = 1: size(inlierIdx,2)
line([y(1,inlierIdx(i))  x(1,inlierIdx(i))+cols1], ...
    [y(2,inlierIdx(i)) x(2,inlierIdx(i))], 'Color', 'c'); 
   
end

hold off;

end

