#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main()
{
  pcl::PointCloud<pcl::PointXYZ> cloud;

  cloud.width = 1951;
  cloud.height = 1;
  cloud.is_dense = false;
  cloud.points.resize(cloud.width * cloud.height);

  int i = 0;

  for(int j=0; j<=25; j++)
    for(int k=0; k<=25; k++)
    {
      cloud.points[i].x = (float)j*0.002;
      cloud.points[i].y = (float)k*0.002;
      cloud.points[i].z = 0.0;

      i++;
    }

  for(int l=0; l<=25; l++)
    for(int m=1; m<=25; m++)
    {
      cloud.points[i].x = (float)l*0.002;
      cloud.points[i].y = 0.0;
      cloud.points[i].z = (float)m*0.002;

      i++;
    }

  for(int n=1; n<=25; n++)
    for(int o=1; o<=25; o++)
    {
      cloud.points[i].x = 0.0;
      cloud.points[i].y = (float)n*0.002;
      cloud.points[i].z = (float)o*0.002;

      i++;
    }

  pcl::io::savePCDFile("./model_draw.pcd", cloud);

  return (0);
}
