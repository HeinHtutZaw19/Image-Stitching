function patches = extractPatches(im,hwid,corners)
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