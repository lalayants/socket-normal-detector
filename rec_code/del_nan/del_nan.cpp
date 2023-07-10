#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/filters/extract_indices.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

int
main (int argc,
      char* argv[])
{
  // The point clouds we will be using
  PointCloudT::Ptr cloud_in (new PointCloudT);

  pcl::console::TicToc time;
  time.tic ();
  if (pcl::io::loadPCDFile<PointT> (argv[1], *cloud_in) < 0)
  {
    PCL_ERROR ("Error loading cloud %s.\n", argv[1]);
    return (-1);
  }
  std::cout << "\nLoaded file " << argv[1] << " (" << cloud_in->size () << " points) in " << time.toc () << " ms\n" << std::endl;

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::ExtractIndices<PointT> extract;

  for (int i = 0; i < cloud_in->points.size(); i++)
  {
    if (!pcl::isFinite(cloud_in->points[i])){
        inliers->indices.push_back(i);
    } 
  }

  extract.setInputCloud(cloud_in);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cloud_in);
  pcl::io::savePCDFileASCII (argv[2], *cloud_in);

}