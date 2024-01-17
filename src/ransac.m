function [inlier_H,inlier_index_set] = ransac(matched_corners_src, matched_corners_dest, distT)
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
