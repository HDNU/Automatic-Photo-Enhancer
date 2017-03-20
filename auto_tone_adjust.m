function auto2=auto_tone_adjust(currentEditedImage)

% currentEditedImage=imread(currentEditedImage);
% currentEditedImage=im2double(currentEditedImage);

% Applying contrast stretching for the given image
auto1=currentEditedImage;
% ycb=rgb2ycbcr(i);
% % gray=rgb2gray(i);
% % gray=ycb(:,:,1);
% % i=ycb;
 
% remap each R, G, B plane
r = remap(currentEditedImage(:,:,1),min(min(currentEditedImage(:,:,1))),max(max(currentEditedImage(:,:,1))));
g = remap(currentEditedImage(:,:,2),min(min(currentEditedImage(:,:,2))),max(max(currentEditedImage(:,:,2))));
b = remap(currentEditedImage(:,:,3), min(min(currentEditedImage(:,:,3))),max(max(currentEditedImage(:,:,3))));

%  r = remap(i(:,:,1),min(min(gray)),max(max(gray)));
%  g = remap(i(:,:,2),min(min(gray)),max(max(gray)));
%  b= remap(i(:,:,3),min(min(gray)),max(max(gray)));

auto1(:,:,1)=r;
auto1(:,:,2)=g;
auto1(:,:,3)=b;

% Applying white balancing for the contrast strecthed image

labImage=rgb2lab(auto1); % convert image into L*a*b color space
[height, width]=size(currentEditedImage(:,:,1));
[avg_a, avg_b ]= get_avg_a_b(labImage,width, height);
auto2=shift_a_b(-avg_a, -avg_b,labImage);
auto2=lab2rgb(auto2);

% imshow(auto2);
% figure
% imshow(currentEditedImage);

end

