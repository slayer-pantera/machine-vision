
function d = calcDist(H,pts1,pts2)     %       �����룺��Ӧ�Ծ��󣬵ڶ���ͼ���һ��ͼ�е�ƥ�������ꡣ������ڶ���ͼ��ƥ��㾭��Ӧ�Ծ���任�����һ��ͼƥ���ľ����ƽ����
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

pts3 = H*[pts1;ones(1,n)];         %        ���ڶ���ͼ�ģ�x��y��1�����Ըþ���
pts3 = pts3(1:2,:)./repmat(pts3(3,:),2,1);      %    ����һ����ȡǰ���У�repmat��һ������ѵ��ɴ���󣩡�
d = sum((pts2-pts3).^2,1);         %      ��sum(x,1)Ϊ��ÿһ����͡�

end