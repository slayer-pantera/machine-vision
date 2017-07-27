
function [matchLoc1 matchLoc2] = findCorr(img1,img2,locs1, locs2)   %   �����룺����ͼ������ͼ�����������ꡣ���������ͼ��������ƥ��ĵ�����꡿




matchTable = zeros(1,size(locs1,1));
 ncc=zeros(1,size(locs2,1));
counter=zeros(1,size(locs2,1));%to make one one pair,because I find different points from locs1,similar to the same points in locs2.
 for i=1:size(locs1,1)
     for j=1:size(locs2,1)

% For each descriptor in the first image, select its match to second image.


    i1Patch=img1(locs1(i,1)-2: locs1(i,1)+2,locs1(i,2)-2: locs1(i,2)+2);
    i2Patch=img2(locs2(j,1)-2: locs2(j,1)+2,locs2(j,2)-2: locs2(j,2)+2);
    i1PatchMean=mean(mean(i1Patch));          %          ��mean(A)Ϊ�������������ľ�ֵ��mean(mean(A))Ϊ�����������Ԫ�صľ�ֵ��
    i2PatchMean=mean(mean(i2Patch));
    i1Patch=i1Patch-i1PatchMean;
    i2Patch=i2Patch-i2PatchMean;%sustract the mean first 
    i1PSumSq=sum(sum(i1Patch.^2))^0.5;
    i2PSumSq=sum(sum(i2Patch.^2))^0.5;
    
    i1PatchNorml=i1Patch/i1PSumSq;
    i2PatchNorml=i2Patch/i2PSumSq;
    ncc(j)= sum(sum(i1PatchNorml.*i2PatchNorml)); % Computes vector of dot products       ����һ������ط�(NCC)�������jѭ������ǵ�һ��ͼ�е�һ�����5*5�����Ӧ�ڶ���ͼ�����������
    end
 [vals,index]=sort(ncc)  ;   %           ����ncc�ĸ��н�����������,valsΪncc�е�Ԫ�أ�indexΪÿ��Ԫ����ԭ�����е���λ������
 

   % Check if nearest neighbor has angle less than distRatio times 2nd.
   
   if (vals(end)>0.9)&&(counter(index(end))==0)    %          �����ncc���ֵ����0.9��j�еĸõ�֮ǰû����i�ж�Ӧ�ĵ㡿 
      matchTable(i) = index(end);           %                 ���趨��ʱiƥ��ĵڶ���ͼ�еĵ㡿
      counter(index(end))=1;                %                 ����counter��Ϊ1����ֹ֮���iҲƥ�䵽�õ㡿
   else
      matchTable(i) = 0;                    %                 ������iû��ƥ��ĵ㡿
   end
     
 end
% save matchdata matchTable
%}

% Create a new image showing the two images side by side.
img3 = appendimages(img1,img2);      %           ��������ͼ������Ű���һ��

% Show a figure with lines joining the accepted matches.
figure('Position', [100 100 size(img3,2) size(img3,1)]);     %      ����Position������ָ�����ڵĴ�С��λ�ã�����ֵΪ[left, bottom, width, height]����һ������������ʾ����λ�ã�����Ļ�����½Ǽ��㡿
colormap('gray');     %        �����ڿ�������ͼ�����ɫ��
imagesc(img3);       %        ����ʾͼ�񣺽����ʵ������ţ���ʾ����ӳ�䡿
hold on;               %      ��ʹ��ǰ�ἰͼ�α��ֶ�����ˢ�£�׼�����ܴ˺󽫻��ơ�
cols1 = size(img1,2);
for i = 1: size(locs1,1)       %             ����ƥ��ĵ�֮������߻�������
  if (matchTable(i) > 0)
    line([locs1(i,2) locs2(matchTable(i),2)+cols1], ...
         [locs1(i,1) locs2(matchTable(i),1)], 'Color', 'c');    %       ��line([1 2],[3 4])���Ƶ�(1,3)��(2,4)֮������ߡ�
  end
end
hold off;            %          ��ʹ��ǰ�ἰͼ�β��پ߱���ˢ�µ����ʡ�
num = sum(matchTable > 0);       %      ��matchTable����0�ĸ�������ƥ��������
%fprintf('NCC Found %d matches.\n', num);     %    ����ӡ����NCC Found 20 matches.�����num=20����

idx1 = find(matchTable);        %       ������Ԫ�ص���������һ��ͼ�ļ�����������ƥ���ϣ���
idx2 = matchTable(idx1);        %       ������Ԫ�أ�ƥ����ǵڶ���ͼ���ĸ������㣩��
x1 = locs1(idx1,2);
x2 = locs2(idx2,2);
y1 = locs1(idx1,1);
y2 = locs2(idx2,1);

matchLoc1 = [x1,y1];      %    ����һ��ͼ�еĵ㡿
matchLoc2 = [x2,y2];      %    ����һ��ͼ�еĵ㡿

end