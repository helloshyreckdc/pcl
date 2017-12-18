#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pass (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read<pcl::PointXYZ> ("./cube5cm2_segmented_0.pcd", *cloud);

  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (-0.1, 0.1);
  pass.filter (*cloud_pass);

  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *cloud_pass << std::endl;

  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud_pass);
  sor.setMeanK (30);
  sor.setStddevMulThresh (1);
  sor.filter (*cloud_filtered);

  std::cerr << "Cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;

  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> ("./cube5cm2_segmented_0_inliers.pcd", *cloud_filtered, false);

  //sor.setNegative (true);
  //sor.filter (*cloud_filtered);
  //writer.write<pcl::PointXYZ> ("../box1_segmented_0_outliers.pcd", *cloud_filtered, false);

  return (0);
}
