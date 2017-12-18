#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

int
main (int argc, char** argv)
{
  pcl::PCLPointCloud2::Ptr cloud2_ini (new pcl::PCLPointCloud2), cloud2_down_sampled (new pcl::PCLPointCloud2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_down_sampled (new pcl::PointCloud<pcl::PointXYZ>), cloud_segmented (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  pcl::PCDReader reader;
  reader.read ("./test_cokecan_scene.pcd", *cloud2_ini);

  std::cerr << "PointCloud before filtering: " << cloud2_ini->width * cloud2_ini->height << " data points." << std::endl;

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud2_ini);
  sor.setLeafSize (0.002f, 0.002f, 0.002f);
  sor.filter (*cloud2_down_sampled);

  // Convert to the templated PointCloud
  pcl::fromPCLPointCloud2 (*cloud2_down_sampled, *cloud_down_sampled);

  std::cerr << "PointCloud after filtering: " << cloud_down_sampled->width * cloud_down_sampled->height << " data points." << std::endl;

  // Write the downsampled version to disk
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> ("./cube5cm2_downsampled.pcd", *cloud_down_sampled, false);

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.01);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  int i = 0, nr_points = (int) cloud_down_sampled->points.size ();
  // While 30% of the original cloud is still there
  while (cloud_down_sampled->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_down_sampled);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the inliers
    extract.setInputCloud (cloud_down_sampled);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*cloud_segmented);
    std::cerr << "PointCloud representing the box segmentation: " << cloud_segmented->width * cloud_segmented->height << " data points." << std::endl;
    std::cerr << "PointCloud representing the cloud_filtered: " << cloud_down_sampled->width * cloud_down_sampled->height << " data points." << std::endl;

    std::stringstream ss;
    ss << "./cube5cm2_segmented_" << i << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_segmented, false);

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud_down_sampled.swap (cloud_f);
    i++;
  }

  return (0);
}
