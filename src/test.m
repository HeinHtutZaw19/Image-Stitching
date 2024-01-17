clc; clear; close all;
%Global Variables Input
im_src = imread("..\include\Image1.jpg");
im_dest = imread("..\include\Image2.jpg");
im1 = double(im2gray(im_src));
im2 = double(im2gray(im_dest));
[corners1,~] = detectHarrisCorners(im1,1,3,50,300);
[corners2,~] = detectHarrisCorners(im2,1,3,50,300);
colors = rand(10, 3);  
colororder(colors);
hwid = 30;
distT = 8;

%Patch Extraction of source and destination images
patches_src = extractPatches(im1, hwid, corners1);
patches_dest = extractPatches(im2, hwid, corners2);

%Normalization of the patches
patches_src = normalizePatches(patches_src,hwid);
patches_dest = normalizePatches(patches_dest, hwid);

%NCC algorithm
[matched_corners_src, matched_corners_dest] = cornerMatching(corners1, corners2, patches_src, patches_dest);

%Showing Harris Corners in figure 1
figure(1);
imshowpair(uint8(im_src), uint8(im_dest), 'montage')
title("Harris Corners");
%Plotting the corners on the outputted images
hold on;plot(corners1(:,2), corners1(:,1), 'rs', 'MarkerSize', 5, 'LineWidth', 1);drawnow;hold off;
hold on;plot(corners2(:,2)+size(im1,2), corners2(:,1), 'rs', 'MarkerSize', 5, 'LineWidth', 1);drawnow;hold off;


%Showing Corner Correspondence in figure 2  
figure(2);
imshowpair(uint8(im_src), uint8(im_dest), 'montage')
title("Corner Correspondenses with NCC");
for i = 1:size(matched_corners_src, 1)
    hold on;
    plot([matched_corners_src(i, 2), matched_corners_dest(i, 2)+size(im1,2)], ...
         [matched_corners_src(i, 1), matched_corners_dest(i, 1)]);
end
  
%Converting to Homogeneous Coordinates
matched_corners_src_homo= matched_corners_src';
matched_corners_src_homo= [matched_corners_src_homo;ones(1,size(matched_corners_src_homo,2))];
matched_corners_dest_homo= matched_corners_dest';
matched_corners_dest_homo= [matched_corners_dest_homo;ones(1,size(matched_corners_dest_homo,2))];

%Ransac Algorithm
[inlier_H,inlier_index_set] = ransac(matched_corners_src_homo, matched_corners_dest_homo, distT);

%Plotting the inlier correspondences of the corners outputted by RANSAC
figure(3);
imshowpair(uint8(im_src), uint8(im_dest), 'montage')
title("Filterd Correspondenses by RANSAC");
for i = 1:size(inlier_index_set, 2)
    hold on;
    plot([matched_corners_src_homo(2, inlier_index_set(i)), matched_corners_dest_homo(2, inlier_index_set(i))+size(im1,2)], ...
         [matched_corners_src_homo(1, inlier_index_set(i)), matched_corners_dest_homo(1, inlier_index_set(i))]);
end
 
 

%Showing Warped Image Mosaic in figure 4
H = calculateHomography(inlier_index_set,matched_corners_src_homo,matched_corners_dest_homo);
warpIm = backwardWarp(double(im_src),double(im_dest),H);
figure(4);
imshow(uint8(warpIm));title("Image Mosaic");


