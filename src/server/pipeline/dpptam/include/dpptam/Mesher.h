#ifndef MESHER_H
#define MESHER_H

#include <stdlib.h>
#include <stdint.h>
#include <string>

#define _USE_MATH_DEFINES
#include <math.h>

#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>
#include <pcl/pcl_exports.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
<<<<<<< HEAD:src/server/pipeline/DPPTAM/include/dpptam/Mesher.h
#include <pcl/filters/statistical_outlier_removal.h>
=======
>>>>>>> 2eb9c0346d4d7b36aade87f3e607d8d0814d3485:src/server/pipeline/dpptam/include/dpptam/Mesher.h

class Mesher {
public:
    bool reconstruct(const char *inputDir);//, const char *outputFile);
    bool saveMesh(const std::string& outputFile);

    // parameters for greedy triangulation reconstruction
    struct GreedyTriangulationParams {
        float search_radius;
        float mu;
        int max_nearest_neighbors;
        float max_surface_angle;
        float min_angle;
        float max_angle; 
        bool normal_consistency;
        // initialize to pcl-suggested defaults
        GreedyTriangulationParams() : search_radius(0.025), 
                                      mu(2.5), 
                                      max_nearest_neighbors(100), 
                                      max_surface_angle(M_PI/4), 
                                      min_angle(M_PI/18), 
                                      max_angle(2*M_PI/3), 
                                      normal_consistency(false) { }
    };

    // this is where the reconstruction will be saved.
    pcl::PolygonMesh triangles;

protected:
    bool readPointsFromDir(const std::string& inputDir, pcl::PCLPointCloud2::Ptr cloud_blob);
<<<<<<< HEAD:src/server/pipeline/DPPTAM/include/dpptam/Mesher.h
    void estimateNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals, int nn);
    void estimateNormalsMLS(pcl::PointCloud<pcl::PointXYZ>::Ptr inputPoints, pcl::PointCloud<pcl::PointNormal>::Ptr normals, float search_radius);
    void filterCloudVoxels(pcl::PCLPointCloud2::Ptr cloud_blob, pcl::PCLPointCloud2::Ptr cloud_filtered, float leaf_size);
    void filterCloudOutliers(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr filtered, int nn, float stdev);
=======
    void estimateNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals);
    void estimateNormalsMLS(pcl::PointCloud<pcl::PointXYZ>::Ptr inputPoints, pcl::PointCloud<pcl::PointNormal>::Ptr normals);
    void filterCloud(pcl::PCLPointCloud2::Ptr cloud_blob, pcl::PCLPointCloud2::Ptr cloud_filtered, float leaf_size);
>>>>>>> 2eb9c0346d4d7b36aade87f3e607d8d0814d3485:src/server/pipeline/dpptam/include/dpptam/Mesher.h
    void setGreedyTriangulationParams(pcl::GreedyProjectionTriangulation<pcl::PointNormal>::Ptr gp3);

    struct GreedyTriangulationParams _greedyParams;

private:
    std::string file_join(const std::string& dir, const std::string& file);
};

#endif // MESHER_H
