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