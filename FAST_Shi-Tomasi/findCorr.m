
function [matchLoc1 matchLoc2] = findCorr(img1,img2,locs1, locs2)   %   【输入：两幅图像及两幅图中特征点坐标。输出：两幅图中所有能匹配的点的坐标】




matchTable = zeros(1,size(locs1,1));
 ncc=zeros(1,size(locs2,1));
counter=zeros(1,size(locs2,1));%to make one one pair,because I find different points from locs1,similar to the same points in locs2.
 for i=1:size(locs1,1)
     for j=1:size(locs2,1)

% For each descriptor in the first image, select its match to second image.


    i1Patch=img1(locs1(i,1)-2: locs1(i,1)+2,locs1(i,2)-2: locs1(i,2)+2);
    i2Patch=img2(locs2(j,1)-2: locs2(j,1)+2,locs2(j,2)-2: locs2(j,2)+2);
    i1PatchMean=mean(mean(i1Patch));          %          【mean(A)为求矩阵各列向量的均值，mean(mean(A))为求矩阵中所有元素的均值】
    i2PatchMean=mean(mean(i2Patch));
    i1Patch=i1Patch-i1PatchMean;
    i2Patch=i2Patch-i2PatchMean;%sustract the mean first 
    i1PSumSq=sum(sum(i1Patch.^2))^0.5;
    i2PSumSq=sum(sum(i2Patch.^2))^0.5;
    
    i1PatchNorml=i1Patch/i1PSumSq;
    i2PatchNorml=i2Patch/i2PSumSq;
    ncc(j)= sum(sum(i1PatchNorml.*i2PatchNorml)); % Computes vector of dot products       【归一化互相关法(NCC)，这里的j循环求的是第一幅图中的一个点的5*5邻域对应第二幅图各个点的邻域】
    end
 [vals,index]=sort(ncc)  ;   %           【对ncc的各列进行升序排列,vals为ncc中的元素，index为每个元素在原矩阵中的排位索引】
 

   % Check if nearest neighbor has angle less than distRatio times 2nd.
   
   if (vals(end)>0.9)&&(counter(index(end))==0)    %          【如果ncc最大值大于0.9且j中的该点之前没有与i中对应的点】 
      matchTable(i) = index(end);           %                 【设定此时i匹配的第二幅图中的点】
      counter(index(end))=1;                %                 【将counter至为1，防止之后的i也匹配到该点】
   else
      matchTable(i) = 0;                    %                 【否则i没有匹配的点】
   end
     
 end
% save matchdata matchTable
%}

% Create a new image showing the two images side by side.
img3 = appendimages(img1,img2);      %           【将两幅图像紧挨着摆在一起】

% Show a figure with lines joining the accepted matches.
figure('Position', [100 100 size(img3,2) size(img3,1)]);     %      【“Position”属性指定窗口的大小和位置，属性值为[left, bottom, width, height]，第一、二个参数表示窗口位置，从屏幕的左下角计算】
colormap('gray');     %        【用于控制索引图像的颜色】
imagesc(img3);       %        【显示图像：进行适当的缩放，表示线性映射】
hold on;               %      【使当前轴及图形保持而不被刷新，准备接受此后将绘制】
cols1 = size(img1,2);
for i = 1: size(locs1,1)       %             【将匹配的点之间的连线画出来】
  if (matchTable(i) > 0)
    line([locs1(i,2) locs2(matchTable(i),2)+cols1], ...
         [locs1(i,1) locs2(matchTable(i),1)], 'Color', 'c');    %       【line([1 2],[3 4])绘制点(1,3)与(2,4)之间的连线】
  end
end
hold off;            %          【使当前轴及图形不再具备被刷新的性质】
num = sum(matchTable > 0);       %      【matchTable大于0的个数，即匹配点个数】
%fprintf('NCC Found %d matches.\n', num);     %    【打印出：NCC Found 20 matches.（如果num=20）】

idx1 = find(matchTable);        %       【非零元素的索引（第一幅图哪几个特征点能匹配上）】
idx2 = matchTable(idx1);        %       【非零元素（匹配的是第二幅图的哪个特征点）】
x1 = locs1(idx1,2);
x2 = locs2(idx2,2);
y1 = locs1(idx1,1);
y2 = locs2(idx2,1);

matchLoc1 = [x1,y1];      %    【第一幅图中的点】
matchLoc2 = [x2,y2];      %    【第一幅图中的点】

end