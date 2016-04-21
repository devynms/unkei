/* input: a directory containing .ply files representing point clouds
   output: a .stl file representing the 3D mesh generated from the point clouds
*/

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <stdio.h>
#include <dirent.h>
#include <time.h>

#include <dpptam/Mesher.h>

#define VERBOSE 1
#define DEBUG(say_something) if (VERBOSE) { printf(say_something); }
#define DEBUG1(say_something, what) if (VERBOSE) { printf(say_something, what); }
#define DEBUG2(say_something, what1, what2) if (VERBOSE) { printf(say_something, what1, what2); }
#define DEBUG3(say_something, what1, what2, what3) if (VERBOSE) { printf(say_something, what1, what2, what3); }

std::string Mesher::file_join(const std::string& dir, const std::string& file) {
    const char dir_last = dir.at(dir.size()-1);
    const char file_first = file.at(0);
    std::string join(dir);
    if (dir_last != '/' && file_first != '/')
        return join.append("/").append(file);
    if (dir_last == '/' && file_first == '/')
        return join.append(file.substr(1, file.size()-1));
    return join.append(file);
}

bool Mesher::reconstruct(const char *inputdir, const char *outputfile) {
    std::string inputDir(inputdir);
    std::string outputFile(outputfile);
    if (outputFile.find(".stl") == std::string::npos) {
        printf("output file must be in .stl format!\n");
        return false;
    }
   
    // prepare point cloud structures to store points from all input files
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
     
    // get points from all .pcd files in the input directory
    DIR *dir = opendir(inputDir.c_str());
    if (dir != NULL) {
        struct dirent *ent;
        while ((ent = readdir(dir)) != NULL) {
            std::string inputFile = ent->d_name;
            if (inputFile.find(".pcd") == std::string::npos)
                continue;
            std::string fqfile = file_join(inputDir, inputFile);
            
            temp_cloud->clear();
            pcl::PCLPointCloud2 temp_cloud_blob;
            
            // load temporary point cloud from input file
            pcl::io::loadPCDFile (fqfile, temp_cloud_blob);
            pcl::fromPCLPointCloud2 (temp_cloud_blob, *temp_cloud);

            // add points to big cloud
            cloud->insert(cloud->end(), temp_cloud->begin(), temp_cloud->end());
            DEBUG2("Read %d points from file %s\n", (int)(temp_cloud->size()), inputFile.c_str())
        }
        closedir(dir);
        DEBUG1("Finished reading from files from directory %s\n", inputDir.c_str())
    } else {
        printf ("could not open input directory");
        return false;
    }

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    
    // estimate normals (use PCL's function for now; DPPTAM already estimates normals)
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);
    n.setInputCloud (cloud);
    n.setSearchMethod (tree);
    n.setKSearch (20);
    n.compute (*normals);

    
    // Concatenate the XYZ and normal fields
    pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);

    DEBUG("Finished estimating normals.\n")
    
    // set greedy triangulation parameters
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    gp3.setSearchRadius (0.025);
    gp3.setMu (2.5);
    gp3.setMaximumNearestNeighbors (100);
    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    gp3.setMinimumAngle(M_PI/18); // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3.setNormalConsistency(false);

    // Create search tree
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (cloud_with_normals);

    DEBUG("Finished creating search tree.\n")
    
    // do greedy triangulation
    pcl::PolygonMesh triangles;
    gp3.setInputCloud (cloud_with_normals);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (triangles);

    DEBUG("Finished greedy triangulation.\n")

    // Additional vertex information
    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();

    DEBUG("Finished getting part ids and point states.\n")

    // save result as .stl file
    pcl::io::savePolygonFileSTL(outputFile, triangles);
    DEBUG("Finished saving reconstruction to stl file.\n")    
    
    return true;
}
