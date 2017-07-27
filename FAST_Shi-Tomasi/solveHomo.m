function H = solveHomo(pts1,pts2)         %    【输入：第二幅图与第一幅图中随机抽取的匹配点的坐标。输出：单应性矩阵】
%	H is 3*3, H*[pts1(:,i);1] ~ [pts2(:,i);1], H(3,3) = 1
%	the solving method see "projective-Seitz-UWCSE.ppt"

n = size(pts1,2);
A = zeros(2*n,9);
A(1:2:2*n,1:2) = pts1';
A(1:2:2*n,3) = 1;
A(2:2:2*n,4:5) = pts1';
A(2:2:2*n,6) = 1;
x1 = pts1(1,:)';
y1 = pts1(2,:)';
x2 = pts2(1,:)';
y2 = pts2(2,:)';
A(1:2:2*n,7) = -x2.*x1;
A(2:2:2*n,7) = -y2.*x1;
A(1:2:2*n,8) = -x2.*y1;
A(2:2:2*n,8) = -y2.*y1;
A(1:2:2*n,9) = -x2;
A(2:2:2*n,9) = -y2;      %     【参见Projective.ppt第9页】

[evec,eigv] = eig(A'*A);    %    【返回矩阵的特征值eigv（以对角矩阵形式）和特征向量evec】
H = reshape(evec(:,1),[3,3])';     %   【将evec第一列调整为3*3矩阵再转置】
H = H/H(end); % make H(3,3) = 1             【归一化得到单应性矩阵】

end