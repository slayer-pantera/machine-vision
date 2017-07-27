
function d = calcDist(H,pts1,pts2)     %       【输入：单应性矩阵，第二幅图与第一幅图中的匹配点的坐标。输出：第二幅图的匹配点经单应性矩阵变换后与第一幅图匹配点的距离的平方】
%	Project PTS1 to PTS3 using H, then calcultate the distances between
%	PTS2 and PTS3

n = size(pts1,2);
%{
fprintf('[pts1;ones(1,n)]s col is %d .\n',n );
fprintf('[pts1;ones(1,n)]s row is %d .\n',size(pts1,1) );
fprintf('Hs col is %d .\n',size(H,2) );
fprintf('Hs row is %d .\n',size(H,1) );
%}

[a,b]=size(H);
if(a<3||b<3)
    d(1,1:n)=10000;
    return
end

pts3 = H*[pts1;ones(1,n)];         %        【第二幅图的（x，y，1）乘以该矩阵】
pts3 = pts3(1:2,:)./repmat(pts3(3,:),2,1);      %    【归一化并取前两行（repmat将一个矩阵堆叠成大矩阵）】
d = sum((pts2-pts3).^2,1);         %      【sum(x,1)为对每一列求和】

end