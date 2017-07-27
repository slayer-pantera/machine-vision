% im = appendimages(image1, image2)
%
% Return a new image that appends the two images side-by-side.

function im = appendimages(image1, image2)          %          【输入：两幅图像。输出：两幅图像紧挨着摆在一起合成的新图像】

% Select the image with the fewest rows and fill in enough empty rows
%   to make it the same height as the other image.
rows1 = size(image1,1);
rows2 = size(image2,1);

if (rows1 < rows2)
     image1(rows2,1) = 0;     %            【使第一幅图像矩阵的行数变成row2，即使两幅图像矩阵行数一样】
else
     image2(rows1,1) = 0;
end

% Now append both images side-by-side.
im = [image1 image2];   
