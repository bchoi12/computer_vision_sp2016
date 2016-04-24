function [] = rpca(name, param)

fname = ['freespace_', name, '_out.png'];

img = imread(fname);
if size(img, 3) > 1
    img = rgb2gray(img);
end

img(img > 0) = 1;
% img = 1-img;

img = double(img);

if nargin < 2
	param = 0.7;
end

[A_hat E_hat iter] = exact_alm_rpca(img, param / sqrt(size(img, 2)));

A_hat(A_hat > 0.9) = 255;
A_hat(A_hat <= 0.9) = 0;

% figure;
% imshow(A_hat);
imwrite(A_hat, ['rpca_', fname]);

quit;
