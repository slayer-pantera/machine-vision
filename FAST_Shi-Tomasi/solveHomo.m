function H = solveHomo(pts1,pts2)         %    �����룺�ڶ���ͼ���һ��ͼ�������ȡ��ƥ�������ꡣ�������Ӧ�Ծ���
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
A(2:2:2*n,9) = -y2;      %     ���μ�Projective.ppt��9ҳ��

[evec,eigv] = eig(A'*A);    %    �����ؾ��������ֵeigv���ԶԽǾ�����ʽ������������evec��
H = reshape(evec(:,1),[3,3])';     %   ����evec��һ�е���Ϊ3*3������ת�á�
H = H/H(end); % make H(3,3) = 1             ����һ���õ���Ӧ�Ծ���

end