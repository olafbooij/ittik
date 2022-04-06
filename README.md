# ittik

ITTIK (I jusT wanT my kittI bacK) gets back the original dense Velodyne LiDAR sensor readings from the sparse point clouds of the KITTI dataset, by reverting the Velodyne calibration.

This dense data is useful as input to convolutional neural networks for object detection, semantic segmentation, etc. In addition, it is useful for geometric processing such as SLAM, because it allows for more accurate timing, and lidar-ray projections.

The following figure shows what is meant with original dense LiDAR sensor readings. On the left the often used "range images", in which point clouds were naively reprojected. On the right the actual dense LiDAR data, acquired by reverting the calibration using ITTIK :

<img src="./doc/plots/sparse_vs_dense/sparse_vs_dense.png">

All scans the Odometry training sets have been processed. The discretized raw scan coordinates are available as <a href='http://www.beteuterd.nl/ittik'>gzipped tar file</a>. Here you can also find images plotted using the raw scan coordinates colored using the <a href='http://semantic-kitti.org/'>semantic kitti</a> labels.

The scans for the 3d object detection sets will follow soon.


Unfortunately, ITTIK cannot deal with motion corrected point-clouds (yet...). This is for example the case for the Odometry test sets.

Btw, the code is not that readable (sorry...) and uses some c++20 features (I used GCC 8.3.0 with `-std=c++2a -fconcepts`).
