function warpIm = backwardWarp(im_src,im_dest,H)
   [nrows_src,ncols_src,~] = size(im_src);
   [nrows_dest,ncols_dest,~] = size(im_dest);
   warpIm = zeros(nrows_dest,ncols_dest,3);
   inv_H = inv(H);
   residual_imsrc_Im = zeros(nrows_src,ncols_dest,3);

   for x_prime=1:nrows_dest
       x_prime
       for y_prime=1:ncols_dest
           p_prime = [x_prime;y_prime;1];
           p = inv_H*p_prime;
           x = p(1)/p(3);
           y = p(2)/p(3);
           if(x<1 || x>nrows_src|| y<1 || y>ncols_src)
               warpIm(round(x_prime),round(y_prime),:) = im_dest(round(x_prime),round(y_prime),:);
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
           warpIm(x_prime,y_prime,:) = ((1-a)*(1-b)*im_src(x_floor,y_floor,:) + ...
                                  a*(1-b)*im_src(x_floor,y_ceil,:) + ...
                                  (1-a)*(b)*im_src(x_ceil,y_floor,:) + ...
                                  a*b*im_src(x_ceil,y_ceil,:));
           warpIm(x_prime,y_prime,:) = warpIm(x_prime,y_prime,:) + im_dest(round(x_prime),round(y_prime),:);
           warpIm(x_prime,y_prime,:) = warpIm(x_prime,y_prime,:)/2;

           %Warping of the residual pixels left unwarped in the front image
           y_floor = floor(y_imsrc_residual);
           y_ceil = ceil(y_imsrc_residual);

           a = y_imsrc_residual-y_floor;
           b = x-x_floor;
           residual_imsrc_Im(x_prime,y_prime,:) = ((1-a)*(1-b)*im_src(x_floor,y_floor,:) + ...
                                  a*(1-b)*im_src(x_floor,y_ceil,:) + ...
                                  (1-a)*(b)*im_src(x_ceil,y_floor,:) + ...
                                  a*b*im_src(x_ceil,y_ceil,:));
       end
   end
   warpIm = cat(2,flip(residual_imsrc_Im,2),warpIm);

   %Deleting the black hole columns in the front
   black_holes_list = find(max(sum(warpIm,3))==0);
   warpIm = warpIm(:,black_holes_list(end):end,:);
end