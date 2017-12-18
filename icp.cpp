#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>  //TicToc

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

void print4x4Matrix (const Eigen::Matrix4d & matrix)
{
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

int main(int argc, char** argv)
{
  PointCloudT::Ptr cloud_model(new PointCloudT);
  PointCloudT::Ptr cloud_sample(new PointCloudT);

  Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();

  pcl::console::TicToc time;
  time.tic();
  if(pcl::io::loadPCDFile<PointT>(argv[1], *cloud_model) < 0)
  {
    PCL_ERROR("Error loading cloud %s. \n", argv[1]);
    return(-1);
  }
  std::cout << "\nLoaded file " << argv[1] << " (" << cloud_model->size() << " points) in " << time.toc() << "ms\n" <<std::endl;

  time.tic();
  if(pcl::io::loadPCDFile<PointT>(argv[2], *cloud_sample) < 0)
  {
    PCL_ERROR("Error loading cloud %s. \n", argv[2]);
    return(-1);
  }
  std::cout << "\nLoaded file " << argv[2] << " (" << cloud_sample->size() << " points) in " << time.toc() << "ms\n" <<std::endl;

  int iterations = 500;
  time.tic();
  pcl::IterativeClosestPoint<PointT, PointT> icp;
  icp.setMaxCorrespondenceDistance(0.01);
  icp.setMaximumIterations(iterations);
  icp.setInputSource(cloud_sample);
  icp.setInputTarget(cloud_model);
  icp.align(*cloud_sample);
  std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc() << " ms" << std::endl;

  if(icp.hasConverged ())
  {
    std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
    std::cout << "\nICP transformation " << iterations << " : cloud_sample -> cloud_model" << std::endl;
    transformation_matrix = icp.getFinalTransformation ().cast<double>();
    print4x4Matrix (transformation_matrix);
  }
  else
  {
    PCL_ERROR ("\nICP has not converged.\n");
    return (-1);
  }

  pcl::visualization::PCLVisualizer viewer("Result of ICP");
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_sample_color_h(cloud_sample, 20, 180, 20);
  viewer.addPointCloud(cloud_model, "model");
  viewer.addPointCloud(cloud_sample, cloud_sample_color_h, "sample");
  while(!viewer.wasStopped())
  {
    viewer.spinOnce();
  }

  return (0);
}
