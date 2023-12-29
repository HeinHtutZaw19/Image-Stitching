clc; clear; close all;
%Global Variables Input
im1 = double(im2gray(imread("..\include\Image1.jpg")));
im2 = double(im2gray(imread("..\include\Image2.jpg")));
[corners1,~] = detectHarrisCorners(im1,1,3,50,300);
[corners2,~] = detectHarrisCorners(im2,1,3,50,300);
colors = rand(10, 3);  
colororder(colors);
hwid = 30;
distT = 8;

%Patch Extraction of source and destination images
patches_src = ExtractPatches(im1, hwid, corners1);
patches_dest = ExtractPatches(im2, hwid, corners2);

%Normalization of the patches
patches_src = NormalizePatches(patches_src,hwid);
patches_dest = NormalizePatches(patches_dest, hwid);

%NCC algorithm
[matched_corners_src, matched_corners_dest] = cornerMatching(corners1, corners2, patches_src, patches_dest);

%Showing Harris Corners in figure 1
figure(1);
imshowpair(uint8(im1), uint8(im2), 'montage')
title("Harris Corners");
%Plotting the corners on the outputted images
hold on;plot(corners1(:,2), corners1(:,1), 'rs', 'MarkerSize', 5, 'LineWidth', 1);drawnow;hold off;
hold on;plot(corners2(:,2)+size(im1,2), corners2(:,1), 'rs', 'MarkerSize', 5, 'LineWidth', 1);drawnow;hold off;


%Showing Corner Correspondence in figure 2  
figure(2);
imshowpair(uint8(im1), uint8(im2), 'montage')
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
[inlier_H,inlier_index_set] = Ransac(matched_corners_src_homo, matched_corners_dest_homo, distT);

%Plotting the inlier correspondences of the corners outputted by RANSAC
figure(3);
imshowpair(uint8(im1), uint8(im2), 'montage')
title("Filterd Correspondenses by RANSAC");
for i = 1:size(inlier_index_set, 2)
    hold on;
    plot([matched_corners_src_homo(2, inlier_index_set(i)), matched_corners_dest_homo(2, inlier_index_set(i))+size(im1,2)], ...
         [matched_corners_src_homo(1, inlier_index_set(i)), matched_corners_dest_homo(1, inlier_index_set(i))]);
end
 
 

%Showing Warped Image Mosaic in figure 4
H = calculateHomography(inlier_index_set,matched_corners_src_homo,matched_corners_dest_homo);
warpIm = backwardWarp(im1,im2,H);
figure(4);
imshow(uint8(warpIm));title("Image Mosaic");

function [corners, R] = detectHarrisCorners(Image, Sigma, N, D, M)
   corners = zeros(M,2);
   %Gaussian Filter
   hwid = 3*Sigma;
   [xx,yy] = meshgrid(-hwid:hwid, -hwid:hwid);
   tmp = exp(-(xx.^2 + yy.^2)/(2 * Sigma^2));
   tmp = tmp/sum(tmp(:));
   img = imfilter(Image, tmp);

   Gx = imfilter(double(img),[1 0 -1],'conv');
   Gy = imfilter(double(img),[1; 0; -1],'conv');

   %M11
   Sx2 = imfilter(Gx.^2, ones(N)/(N*N));
   %M22
   Sy2 = imfilter(Gy.^2, ones(N)/(N*N));
   %M12,M21
   Sxy = imfilter(Gx.*Gy, ones(N)/(N*N));

   k = 0.05;
   R = ((Sx2 .* Sy2) - (Sxy.*Sxy))-k*(Sx2+Sy2).^2;
   R_tmp = R;
   i = 1;
   while i<=M
      [~,idx] = max(R(:));
      [row,col] = ind2sub(size(R),idx);
      corners(i,1) = row;
      corners(i,2) = col;
      R(max(1,row-D): min(length(R),row+D), max(1,col-D): min(length(R),col+D)) = intmin;
      i=i+1;
   end
   R = R_tmp;
end

function patches = ExtractPatches(im,hwid,corners)
   patches = zeros(hwid*hwid,size(corners,1));
   [nrows,ncols] = size(im);
   for k=1:size(corners,1)
       idx = 1;
       for i = -hwid:hwid
           for j = -hwid:hwid
               qx = corners(k,1) + i;
               qy = corners(k,2) + j;
               if qx >= 1 && qx <= nrows && qy >= 1 && qy <= ncols
                   patches(idx,k) = im(qx,qy);
                   idx = idx+1;
               end
           end
        end
   end
end

function normalized_patches = NormalizePatches(patches,hwid)
   d=(2*hwid+1)^2;
   mean_patches = ones(d,1) .* mean(patches,1);
   std_patches = ones(d,1) .* std(patches, 1);
   normalized_patches = (patches-mean_patches)./std_patches;
end

function [inlier_H,inlier_index_set] = Ransac(matched_corners_src, matched_corners_dest, distT)
   bestVotes = 0;
   for iter = 1:4000
       idx = randperm(size(matched_corners_src,2),4);
       H = calculateHomography(idx,matched_corners_src,matched_corners_dest);
       warpedCorner_src = H*matched_corners_src;
       warped_x = warpedCorner_src(1,:)./warpedCorner_src(3,:);
       warped_y = warpedCorner_src(2,:)./warpedCorner_src(3,:);
       set = [];
       counter = 0;
       
       for k = 1:size(warpedCorner_src,2)
         if(abs(pdist([warped_x(k),warped_y(k);matched_corners_dest(1,k),matched_corners_dest(2,k)]))<distT)
            counter = counter+1;
            set = [set,k];
         end
      end

      if(bestVotes<counter) 
          bestVotes = counter; 
          inlier_H = H;
          inlier_index_set = set;
      end
   end
end

function [matched_corners_src, matched_corners_dest] = cornerMatching(corners1, corners2, patches_src, patches_dest)
    NCC_score = patches_src'*patches_dest;

    %Finding of the bidirectional maximum value in the NCC_score row and column
    [row_max_val,col_idx] = max(NCC_score,[],2);
    [col_max_val,row_idx] = max(NCC_score);
    
    matched_corners_src = [];
    matched_corners_dest = [];
    for i = 1:length(row_max_val)
        if row_max_val(i) == col_max_val(col_idx(i))
            matched_corners_src = [matched_corners_src,  row_idx(col_idx(i))];
            matched_corners_dest = [matched_corners_dest, col_idx(i)];
        end
    end
    matched_corners_src = corners1(matched_corners_src,:);
    matched_corners_dest= corners2(matched_corners_dest,:);
end

function H = calculateHomography(idx,matched_corners_src,matched_corners_dest)
   A=[];
   for i = 1:size(idx,2)
      x_src = matched_corners_src(1,idx(i));
      y_src = matched_corners_src(2,idx(i));
      x_dest = matched_corners_dest(1,idx(i));
      y_dest = matched_corners_dest(2,idx(i));

      A = [A;
          x_src, y_src, 1, 0, 0, 0, -x_src*x_dest, -y_src*x_dest, -x_dest;
          0, 0, 0, x_src, y_src, 1, -x_src*y_dest, -y_src*y_dest, -y_dest];   
    end
       % Computing Homography Matrix using SVD
    [~,~,V2] = svd(A);
    H = V2(:,end);
    H = [H(1),H(2),H(3); H(4),H(5),H(6); H(7),H(8),H(9)];
       
end

function warpIm = backwardWarp(im_src,im_dest,H)
   [nrows_src,ncols_src] = size(im_src);
   [nrows_dest,ncols_dest] = size(im_dest);
   warpIm = zeros(nrows_dest,ncols_dest);
   inv_H = inv(H);
   residual_imsrc_Im = zeros(nrows_src,ncols_dest);
   for x_prime=1:nrows_dest
       for y_prime=1:ncols_dest
           p_prime = [x_prime;y_prime;1];
           p = inv_H*p_prime;
           x = p(1)/p(3);
           y = p(2)/p(3);
           if(x<1 || x>nrows_src|| y<1 || y>ncols_src)
               warpIm(round(x_prime),round(y_prime)) = im_dest(round(x_prime),round(y_prime));
               continue;
           end
           y_imsrc_residual=ncols_src-y+100;
           %Initializing the four points for bilinear interpolation
           x_floor = floor(x);
           x_ceil = ceil(x);
           y_floor = floor(y);
           y_ceil = ceil(y);
           %Warping the intersection segment of the two images
           a = y-y_floor;
           b = x-x_floor;
           warpIm(x_prime,y_prime) = ((1-a)*(1-b)*im_src(x_floor,y_floor) + ...
                                  a*(1-b)*im_src(x_floor,y_ceil) + ...
                                  (1-a)*(b)*im_src(x_ceil,y_floor) + ...
                                  a*b*im_src(x_ceil,y_ceil));
           warpIm(x_prime,y_prime) = warpIm(x_prime,y_prime) + im_dest(round(x_prime),round(y_prime));
           warpIm(x_prime,y_prime) = warpIm(x_prime,y_prime)/2;

           %Warping of the residual pixels left unwarped in the front image
           y_floor = floor(y_imsrc_residual);
           y_ceil = ceil(y_imsrc_residual);

           a = y_imsrc_residual-y_floor;
           b = x-x_floor;
           residual_imsrc_Im(x_prime,y_prime) = ((1-a)*(1-b)*im_src(x_floor,y_floor) + ...
                                  a*(1-b)*im_src(x_floor,y_ceil) + ...
                                  (1-a)*(b)*im_src(x_ceil,y_floor) + ...
                                  a*b*im_src(x_ceil,y_ceil));
       end
   end
   warpIm = cat(2,flip(residual_imsrc_Im,2),warpIm);

   %Deleting the black hole columns in the front
   black_holes_list = find(max(warpIm)==0);
   warpIm = warpIm(:,black_holes_list(end):end);
end

