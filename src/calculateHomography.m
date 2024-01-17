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