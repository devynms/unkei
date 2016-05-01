#include <boost/thread.hpp>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>

#include <dpptam/DenseMapping.h>
#include <dpptam/SemiDenseMapping.h>

class PlyListener {
public:
    PlyListener();

    std::string convert_ply2pcd_file(const char *plyfile, const char *pcdfile);
    std::string convert_ply2pcd_dir(const char *plydir, const char *pcdfile);

    std::string pcd_dir;
    std::string mesh_file;
    int pcd_count;
    boost::mutex pcd_count_mutex;
    clock_t tlast;
    boost::mutex tlast_mutex;
};

void ThreadPlyListener(PlyListener *pply_listener, DenseMapping *pdense_mapper, SemiDenseMapping *psemidense_mapper, const std::string& pcd_dir, const std::string& mesh_file);
void thread_ply_listener_convert_ply2pcd_file(const char *ply_file, PlyListener *ply_listener);
void thread_ply_listener_sup(PlyListener *pply_listener, DenseMapping *pdense_mapper);
void thread_ply_listener_map(PlyListener *pply_listener, SemiDenseMapping *psemidense_mapper);
std::string execute(const char *cmd);
