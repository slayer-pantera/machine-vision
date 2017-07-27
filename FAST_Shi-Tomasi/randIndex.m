function index = randIndex(maxIndex,len)           %   �����룺�ڶ���ͼ��ƥ��ĵ�����������ȡ�ĵ�ĸ������˴�Ϊ4����������ڶ���ͼ��ƥ���������ȡ�ı�š�
%INDEX = RANDINDEX(MAXINDEX,LEN)
%   randomly, non-repeatedly select LEN integers from 1:MAXINDEX

if len > maxIndex
	index = [];
	return
end

index = zeros(1,len);
available = 1:maxIndex;           %            ������һ��maxIndexά���о���
rs = ceil(rand(1,len).*(maxIndex:-1:maxIndex-len+1));     %       ��rand(1,len)����1*4ά������󣨸�Ԫ��ֵΪ0~1����(maxIndex:-1:maxIndex-len+1)����[ptNum ptNum-1 ... ptNum-3]��
for p = 1:len
	while rs(p) == 0
		rs(p) = ceil(rand(1)*(maxIndex-p+1));       %       ����Ϊ0��һ����ѡһ���������
	end
	index(p) = available(rs(p));         %        ����õڶ���ͼ��ƥ���������š�
	while index(p)==[]
        rs(p) = ceil(rand(1)*(maxIndex-p+1));
        index(p) = available(rs(p));
    end
    available(rs(p)) = [];        %            ���Ƿ�Ҫ���޶������Ա�֤�޷�ȡ����ͬ�ı�š�
end