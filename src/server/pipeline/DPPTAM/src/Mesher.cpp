/* input: a directory containing .ply files representing point clouds
   output: a .stl file representing the 3D mesh generated from the point clouds
*/

#include <dpptam/Mesher.h>

#include <stdio.h>
#include <dirent.h>
#include <time.h>

#define VERBOSE 1
#define DEBUG(say_something) if(VERBOSE) { printf(say_something); }
#define DEBUG1(say_something, what) if(VERBOSE) { printf(say_something, what); }
#define DEBUG2(say_something, what1, what2) if(VERBOSE) { printf(say_something, what1, what2); }
#define DEBUG3(say_something, what1, what2, what3) if(VERBOSE) { printf(say_something, what1, what2, what3); }

std::string Mesher::file_join(const std::string& dir, const std::string& file) {
    const char dir_last = dir.at(dir.size()-1);
    const char file_first = file.at(0);
    std::string join(dir);
    if(dir_last != '/' && file_first != '/')
        return join.append("/").append(file);
    if(dir_last == '/' && file_first == '/')
        return join.append(file.substr(1, file.size()-1));
    return join.append(file);
}

bool Mesher::readPointsFromDir(const std::string& inputDir, pcl::PCLPointCloud2::Ptr cloud_blob) {
    // get points from all .pcd files in the input directory
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    DIR *dir = opendir(inputDir.c_str());
    if(dir != NULL) {
        struct dirent *ent;
        while((ent = readdir(dir)) != NULL) {
            std::string inputFile = ent->d_name;
            if(inputFile.find(".pcd") == std::string::npos)
                continue;
            std::string fqfile = file_join(inputDir, inputFile);
            
            // load temporary point cloud from input file
            pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PCLPointCloud2::Ptr temp_cloud_blob(new pcl::PCLPointCloud2());
            pcl::io::loadPCDFile(fqfile, *temp_cloud_blob);
            pcl::fromPCLPointCloud2(*temp_cloud_blob, *temp_cloud);

            // add points to big cloud
            cloud->insert(cloud->end(), temp_cloud->begin(), temp_cloud->end());
            DEBUG2("Read %d points from file %s\n",(int)(temp_cloud->size()), inputFile.c_str())
        }
        pcl::toPCLPointCloud2(*cloud, *cloud_blob); 
        closedir(dir);
        return true;
    } else {
        return false;
    }
}

void Mesher::filterCloudVoxels(pcl::PCLPointCloud2::Ptr cloud_blob, pcl::PCLPointCloud2::Ptr cloud_blob_filtered, float leaf_size) {
    pcl::VoxelGrid<pcl::PCLPointCloud2> filter;
    filter.setInputCloud(cloud_blob);
    filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    filter.filter(*cloud_blob_filtered);
}

void Mesher::filterCloudOutliers(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr filtered) {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(input);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.2);
    sor.filter(*filtered);
}

void Mesher::estimateNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals) {
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    n.setKSearch(50);
    n.compute(*normals);

    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
}

void Mesher::estimateNormalsMLS(pcl::PointCloud<pcl::PointXYZ>::Ptr inputPoints, pcl::PointCloud<pcl::PointNormal>::Ptr normals) {

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
    mls.setComputeNormals(true);

    mls.setInputCloud(inputPoints);
    mls.setPolynomialFit(true);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(0.025);

    mls.process(*normals);
}

void Mesher::setGreedyTriangulationParams(pcl::GreedyProjectionTriangulation<pcl::PointNormal>::Ptr gp3) {
    gp3->setSearchRadius(_greedyParams.search_radius);
    gp3->setMu(_greedyParams.mu);
    gp3->setMaximumNearestNeighbors(_greedyParams.max_nearest_neighbors);
    gp3->setMaximumSurfaceAngle(_greedyParams.max_surface_angle);
    gp3->setMinimumAngle(_greedyParams.min_angle);
    gp3->setMaximumAngle(_greedyParams.max_angle);
    gp3->setNormalConsistency(_greedyParams.normal_consistency);
}

bool Mesher::reconstruct(const char *inputdir) {
    std::string inputDir(inputdir);
    
    // read points from file
    pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2());
    if(!readPointsFromDir(inputDir, cloud_blob)) {
        printf("could not open input directory");
        return false;
    }
    DEBUG1("Finished reading from files from directory %s\n", inputDir.c_str())
    DEBUG1("number of points in cloud_blob: %d\n",(int)cloud_blob->data.size())

    // downsample the cloud by taking the centroids of voxels
    pcl::PCLPointCloud2::Ptr cloud_blob_filtered(new pcl::PCLPointCloud2());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_voxels(new pcl::PointCloud<pcl::PointXYZ>);
    filterCloudVoxels(cloud_blob, cloud_blob_filtered, 0.01f);
    pcl::fromPCLPointCloud2(*cloud_blob_filtered, *cloud_filtered_voxels);
    DEBUG1("Finished voxel filtering the point cloud. Cloud now has %d points.\n",(int)(cloud_filtered_voxels->size()))

    // remove point outliers from the cloud 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    filterCloudOutliers(cloud_filtered_voxels, cloud_filtered);
    
    // estimate surface normals 
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    //estimateNormals(cloud_filtered, cloud_with_normals);
    estimateNormalsMLS(cloud_filtered, cloud_with_normals);
    DEBUG("Finished estimating normals.\n")
    
    //create search tree
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);
    DEBUG("Finished creating search tree.\n")
    
    // set greedy triangulation parameters
    _greedyParams.mu = 3;
    _greedyParams.max_nearest_neighbors = 125;
    pcl::GreedyProjectionTriangulation<pcl::PointNormal>::Ptr gp3(new pcl::GreedyProjectionTriangulation<pcl::PointNormal>);
    setGreedyTriangulationParams(gp3);
    DEBUG("Finished setting greedy triangulation parameters.\n")

    // do greedy triangulation
    clock_t t = clock();    
    //pcl::PolygonMesh triangles;
    gp3->setInputCloud(cloud_with_normals);
    gp3->setSearchMethod(tree2);
    gp3->reconstruct(triangles);
    DEBUG1("Finished greedy triangulation in %fs.\n",(float)(clock() - t)/CLOCKS_PER_SEC)
    
    return true;
}

bool Mesher::saveMesh(const std::string& outputFile) {

    if(outputFile.find(".stl") != std::string::npos) {
        // save result as .stl file
        pcl::io::savePolygonFileSTL(outputFile, triangles);
    } else if(outputFile.find(".obj") != std::string::npos) {
        // save result as .obj file
        pcl::io::saveOBJFile(outputFile, triangles);
    } else {
        printf("output file must be in .stl or .obj format!(missing or unrecognized file extension)\n");
        return false;
    }

    DEBUG1("Finished saving reconstruction to file %s.\n", outputFile.c_str())    
    return true;
}
