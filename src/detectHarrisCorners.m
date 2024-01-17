
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