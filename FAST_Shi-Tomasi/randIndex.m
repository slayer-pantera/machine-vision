function index = randIndex(maxIndex,len)           %   【输入：第二幅图中匹配的点个数，随机抽取的点的个数（此处为4）。输出：第二幅图中匹配点的随机抽取的编号】
%INDEX = RANDINDEX(MAXINDEX,LEN)
%   randomly, non-repeatedly select LEN integers from 1:MAXINDEX

if len > maxIndex
	index = [];
	return
end

index = zeros(1,len);
available = 1:maxIndex;           %            【产生一个maxIndex维的行矩阵】
rs = ceil(rand(1,len).*(maxIndex:-1:maxIndex-len+1));     %       【rand(1,len)产生1*4维随机矩阵（各元素值为0~1）；(maxIndex:-1:maxIndex-len+1)产生[ptNum ptNum-1 ... ptNum-3]】
for p = 1:len
	while rs(p) == 0
		rs(p) = ceil(rand(1)*(maxIndex-p+1));       %       【对为0的一项重选一个随机数】
	end
	index(p) = available(rs(p));         %        【获得第二幅图中匹配点的随机编号】
	while index(p)==[]
        rs(p) = ceil(rand(1)*(maxIndex-p+1));
        index(p) = available(rs(p));
    end
    available(rs(p)) = [];        %            【是否要加限定条件以保证无法取到相同的编号】
end