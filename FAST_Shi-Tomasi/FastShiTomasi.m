% Harris detector
% The code calculates
% the Harris Feature Points(FP) 
% 
% When u execute the code, the test image file opened
% and u have to select by the mouse the region where u
% want to find the Harris points, 
% then the code will print out and display the feature
% points in the selected region.
% You can select the number of FPs by changing the variables 
% max_N & min_N
% A. Ganoun

function [locs] = FastShiTomasi(frame,number)   %            �����룺ͼ�������Harris�ǵ���ͼ������е�λ�á�
% I=rgb2gray(frame);
% I =double(I);
I=frame;
%****************************
% imshow(frame);
% 
% waitforbuttonpress;
% point1 = get(gca,'CurrentPoint');  %button down detected
% rectregion = rbbox;  %%%return figure units
% point2 = get(gca,'CurrentPoint');%%%%button up detected
% point1 = point1(1,1:2); %%% extract col/row min and maxs
% point2 = point2(1,1:2);
% lowerleft = min(point1, point2);
% upperright = max(point1, point2); 
% ymin = round(lowerleft(1)); %%% arrondissement aux nombrs les plus proches
% ymax = round(upperright(1));
% xmin = round(lowerleft(2));
% xmax = round(upperright(2));
% 
% 
% %***********************************
% Aj=6;
% cmin=xmin-Aj; cmax=xmax+Aj; rmin=ymin-Aj; rmax=ymax+Aj;
 min_N=350;max_N=450;
%{
 sigma=1.4;
g = fspecial('gaussian',5*sigma, sigma);
I = conv2(I, g, 'same');
%}
%%%%%%%%%%%%%%Intrest Points %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

mask=[0 0 1 1 1 0 0;...  
      0 1 0 0 0 1 0;...  
      1 0 0 0 0 0 1;...  
      1 0 0 0 0 0 1;...  
      1 0 0 0 0 0 1;...  
      0 1 0 0 0 1 0;...  
      0 0 1 1 1 0 0];   
%for hall
%threshold=55;  
%for window_view_
   threshold=30; 
%figure;imshow(img);title('FAST�ǵ���');hold on;  

R=zeros(size(I,1),size(I,2));
%for window_view_

g = fspecial('gaussian',5, 1.4);
I= conv2(I, g, 'same');

for i=4:size(I,1)-3  
    for j=4:size(I,2)-3%��I1��I9������I0�Ĳ��С����ֵ�����Ǻ�ѡ��  
        delta1=abs(I(i-3,j)-I(i,j))>threshold;  
        delta9=abs(I(i+3,j)-I(i,j))>threshold;  
        if sum([delta1 delta9])==0  
            continue;  
        else  
            delta5=abs(I(i,j+3)-I(i,j))>threshold;  %��I5��I13������I0�Ĳ��С����ֵ�����Ǻ�ѡ��
            delta13=abs(I(i,j-3)-I(i,j))>threshold;  
            if sum([delta1 delta9 delta5 delta13])>=3  
                block=I(i-3:i+3,j-3:j+3);  
                block=block.*mask;%��ȡԲ��16����  
                %pos=find(block);  
                block1=abs(block-I(i,j).*mask)/threshold;  
                block2=floor(block1);  
                res=find(block2);  
                if size(res,1)>=12  
                    R(i,j)=sum(sum(block1));
                end  
            end  
        end  
    end  
end  

[x1,y1]=find(R);
newpoint=[x1,y1];
sigma=1.6; Thrshold=400; 
dx = [-1 0 1; -1 0 1; -1 0 1]; % The Mask 
dy = dx';
g = fspecial('gaussian',3, sigma); %%%%%% Gaussien Filter    ����˹�˲���ģ��ߴ�n=7��
    %%%%%% 
Is=zeros(7,7);
R11=zeros(size(I,1),size(I,2));
for k=1:size(newpoint,1)
    Is=I(x1(k)-3:x1(k)+3,y1(k)-3:y1(k)+3);
    Ix = conv2(Is, dx, 'same');   %         ���ô�ģ�����൱����x������ݶȡ�
    Iy = conv2(Is, dy, 'same');
    
    %%%%% 
    Ix2 = conv2(Ix.^2, g, 'same');  
    Iy2 = conv2(Iy.^2, g, 'same');
    Ixy = conv2(Ix.*Iy, g,'same');
    %%%%%%%%%%%%%%
    %k = 0.04;
    %R11 = (Ix2.*Iy2 - Ixy.^2) - k*(Ix2 + Iy2).^2;         %             ��R=detM-k*(trM)^2��
    %M={Ix2 Ixy;Ixy Iy2};
    
    for i=1:7
        for j=1:7
            [evec,eigv] = eig([Ix2(i,j) Ixy(i,j);Ixy(i,j) Iy2(i,j)]);
            R11(i+x1(k)-4,j+y1(k)-4)=min(min(diag(eigv)));
        end
    end
end
R11=(1000/max(max(R11)))*R11;  %make the largest one to be 1000          ��max(R11)Ϊ�����ÿ�е����ֵ��max(max(R11))Ϊ�����������е����ֵ��
   
 %   R=R11;
    
    sze = 7;                  
    MX = ordfilt2(R11,sze^2,ones(sze));% non-Maximun supression          ��˳��ͳ���˲����˴��൱�ڶ�R����szx*szx�����ֵ�˲����������˷Ǽ���ֵ���ơ�
    R11 = (R11==MX)&(R11>Thrshold);      %              ���õ��ľ���������������Ϊ1���������Ϊ0��
  %R11=R;
%    count=sum(sum(R11(5:size(R11,1)-5,5:size(R11,2)-5)));         %          ��ȥ����Ե5�����غ��Harris�ǵ������
    
%{    
  loop=0;  %use adaptive threshold here            ��������ֵʹ��Harris�ǵ������min_N��max_N֮�䡿
   while (((count<min_N)||(count>max_N))&&(loop<30))
        if count>max_N
            Thrshold=Thrshold*1.5;
        elseif count < min_N
            Thrshold=Thrshold*0.5;
        end
        
        R11 = (R==MX)&(R>Thrshold); 
        count=sum(sum(R11(5:size(R11,1)-5,5:size(R11,2)-5)));
        loop=loop+1;
    end
%}
    
	R=R*0;
    if number==1
        R(5:size(R11,1)-5,0.3*size(R11,2):size(R11,2)-5)=R11(5:size(R11,1)-5,0.3*size(R11,2):size(R11,2)-5);% ignore the corners on the boundary
    else
        if number==2
            R(5:size(R11,1)-5,5:0.7*size(R11,2))=R11(5:size(R11,1)-5,5:0.7*size(R11,2));
        end
    end
	[r1,c1] = find(R);
    PIP=[r1,c1];%% IP 
    locs=PIP;        %           ��������ͼ������еĽǵ�λ�á�

   %%%%%%%%%%%%%%%%%%%% Display             ���ƺ�û�ã���
   %{
   Size_PI=size(PIP,1);
   for r=1: Size_PI
   I(PIP(r,1)-2:PIP(r,1)+2,PIP(r,2))=255;
   I(PIP(r,1)-2:PIP(r,1)+2,PIP(r,2))=255;
   I(PIP(r,1),PIP(r,2)-2:PIP(r,2)+2)=255;
   I(PIP(r,1),PIP(r,2)-2:PIP(r,2)+2)=255;
   
   end
   %}
   
