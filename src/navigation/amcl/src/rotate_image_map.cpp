

// #include <geometry_msgs/PoseWithCovarianceStamped.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <std_msgs/Bool.h>
// #include <tf/transform_listener.h>

// #include "autoware_msgs/LaneArray.h"

#include <fstream>
#include <iostream>
// #include <pcl_ros/transforms.h>
// #include <pcl_ros/point_cloud.h>
#include <ros/ros.h>

// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/PCLPointCloud2.h>
// #include <pcl/conversions.h>
// #include <pcl_ros/transforms.h>
// #include <pcl_ros/point_cloud.h>
// #include <pcl/common/transforms.h>    
// #include <vector>
// #include <iterator>
// #include <regex>
// #include <pcl/registration/transformation_estimation_svd.h>
// #include <pcl/registration/transformation_estimation_svd_scale.h>
// #include <pcl/registration/icp.h>
// #include <algorithm>
// #include <stdio.h>
// #include <sys/stat.h>
// #include <dirent.h>
// #include<iomanip>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


void HorMirror(IplImage* srcImage,IplImage* Image)
{
	int nHeight = srcImage->height;
	int nWidth = srcImage->width;
	// 1 其实我在在教材算法的基础上修改得来的，结果是正确 ，可是原理却不是太清楚，我自己也不能自圆其说，只好把笔记记下来，为了以后参考，
	// 2 看到这里的同仁如果能明，请告知一二，在此谢过。
	// 3 我在这里描述一下我的想法：
	//    首先，从水平镜像的处理的结果来看。我觉得是：从原始图像到目标图像，每一个像素点坐标的高度是没有任何变化的，即h值保持恒定（从原始图像取值时的h和写入目标图像的h恒定）
	//    其次，变化的只是像素坐标(w,h)的w值，所以从变换式子u=nWidth-w-1 可以理解为原始图像的左边，变换之后 就是目标图片的右边
	int u; 
	for(int h=0;h<nHeight;h++)
	{
		for(int w=0;w<nWidth;w++)
			{
				u=nWidth-w-1;
				//读取原图（u=nWidth-w-1，h）坐标的像素
				uchar gray= (uchar)*((uchar*)(srcImage->imageData+h*srcImage->widthStep)+srcImage->nChannels*u);
				//写入目标目标图片（w,h）坐标   
				*((uchar*)(Image->imageData+h*Image->widthStep)+Image->nChannels*w) = gray;				           
			}
	}
}

void VerticallyMirror(cv::Mat &src,cv::Mat &dst) //实现竖直镜像变换
{

	for (int i = 0; i < src.rows; i++)
	{
		for (int j = 0; j < src.cols; j++)
		{
			dst.at<uchar>(i, j) = src.at<uchar>(src.rows-i-1, j);
		}
	}
}

int main(int argc, char **argv)
{

	///////////////////////////////////////////////////////////////
	//拆分
	//std::string splitfile = "/home/huafy/.ros/generate_map/map_version_3_20190829/0.20_graph_slam_submap_2019-08-29_transformation_transformation_1.pcd";
	//split_pcdfile(splitfile);
	//return 0;
	//////////////////////////////////////////////////////////////

	//根据转移矩阵，转换pcd地图到gps坐标下的地图
	ros::init(argc, argv, "rotate_image");

	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	std::string imagefile_path = "/home/huafy/Desktop/robot/20200709/dayang0709.pgm";
	std::string imagefile_path_write = "/home/huafy/Desktop/robot/20200709/dayang0709_new.pgm";
	std::string imagefile_path_write2 = "/home/huafy/Desktop/robot/20200709/dayang0709_new_new.pgm";
// 	std::string source_points_path = "";
// 	std::string target_points_path = "";
// 	int  transform_type = 0;
	

	// //【1】载入原图
	// IplImage* srcImage = cvLoadImage(imagefile_path.c_str(),0);
	// cvNamedWindow("原图");
	// cvShowImage("原图",srcImage);
	// //【2】为目标图片做准备
	// CvSize srcSize;
	// srcSize.height=srcImage->height;
	// srcSize.width=srcImage->width;
	// IplImage * Image=cvCreateImage(srcSize,srcImage->depth,1);
	// //【3】调用镜像变换函数函数并显示结果
	// HorMirror(srcImage,Image);

	// cvSaveImage(imagefile_path_write.c_str(),Image);


	cv::Mat img2 = cv::imread(imagefile_path_write, 0);
	cv::Mat newImg2 = cv::Mat::zeros(img2.rows, img2.cols, CV_8UC1);
	VerticallyMirror(img2,newImg2);
	cv::imwrite(imagefile_path_write2.c_str(), newImg2);

	// cvNamedWindow("水平镜像变换结果");
	// cvShowImage("水平镜像变换结果",newImg2);
	//【3】释放内存空间
	cv::waitKey();
	// cvDestroyWindow("原图");
	// cvDestroyWindow("水平镜像变换结果");
	// cvReleaseImage(&srcImage);
	// cvReleaseImage(&Image);

	return 0;
}