# Image Mosaic

## Motivation
<p align="center">
  <img src="./include/Image1.jpg" width="45%" />
  <img src="./include/Image2.jpg" width="45%" />
</p>


These are the the two input images we want to combine. One is captured in the morning and the other in the evening. There are both rotation and translation differences between the two.

## Harris Corner Detectoin
<img src="./photos/Harris Corners.png" />
300 Harris Corners are computed using the code in Corner Detection. The two images along with the corners are shown side by side to compare the similarities with ease.

## Corner Correspondence
<img src="./photos/Corner Correspondence.png" />
Then the patches with half-wid 30, are extracted from each corner and normalized by subtracting the mean and dividing with standard deviation. It is correlated with each corner in the im_dest and the bilinearly largest correspondences are stored, however this set contains a lot of outliers.

## Ransac Filtering Algorithm
<img src="./photos/filtered correspondence.png" />
The inliers are filtered out by randomly choosing four correspondences, computing homography, applying to the corners and comparing the euclidean distances between the corners with matched correspondences. The maximum inlier set and the relating homography matrix are stored, the new best homography matrix is computed with all the points in the inlier set.

## Backward bilinear warping
<img src="./photos/Image Mosaic.png" />
For the intersected section of the two images, it is blended out by averaging the backward warping of im_src and im_dest and actual values of im_dest. Then, the residual pixels from im_src also undergo a backward warping process with the inverse homography matrix. The resultant residual pixels are concatenated with the blended intersected section and also for the third part, the residual im_dest pixels. Since there will be black holes in front of this image mosaic, the last column index where all the elements in the column contains 0 is searched and truncated from the resultant image.
