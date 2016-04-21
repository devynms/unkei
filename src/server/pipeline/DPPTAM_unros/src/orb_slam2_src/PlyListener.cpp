#include <string>
#include <iostream>
#include <cstdio>
#include <memory>
#include <ros/package.h>
#include <ros/ros.h>
#include <time.h>

#include <dpptam/PlyListener.h>
#include <dpptam/Mesher.h>

// max seconds to wait on condition variable
#define TIMEOUT 60 

std::string PlyListener::convert_ply2pcd_file(const char *plyfile, const char *pcdfile) {
    char cmd[1024];
    const char *ply2pcd_script = "pcl_ply2pcd";
    snprintf(cmd, 1024, "%s %s %s\n", ply2pcd_script, plyfile, pcdfile);
    return execute(cmd); 
}

std::string PlyListener::convert_ply2pcd_dir(const char *plydir, const char *pcdfile) {
    char cmd[1024];
    const char *ply2pcd_script = "$HOME/projects/eecs395/pcl/scripts/ply2pcd.sh";
    snprintf(cmd, 1024, "%s %s %s\n", ply2pcd_script, plydir, pcdfile);
    return execute(cmd); 
}

PlyListener::PlyListener() : pcd_count(0) { }

void ThreadPlyListener(PlyListener *pply_listener, DenseMapping *pdense_mapper, SemiDenseMapping *psemidense_mapper, const std::string& pcd_dir, const std::string& mesh_file) {
    pply_listener->pcd_dir = pcd_dir;//"/home/acceber/workspaces/ros_catkin/src/dpptam/data/meshes";
    pply_listener->mesh_file = mesh_file;//"/home/acceber/workspaces/ros_catkin/src/dpptam/data/out.stl";
    int pcd_count_save = 0;
    while(ros::ok())
    {
        {
            boost::unique_lock<boost::mutex> lock(pply_listener->pcd_count_mutex);
            pcd_count_save = pply_listener->pcd_count;
            lock.unlock();
        }

        cout << "PlyListener: spawning threads\n";
        boost::thread sup_thread(&thread_ply_listener_sup, pply_listener, pdense_mapper);
        boost::thread map_thread(&thread_ply_listener_map, pply_listener, psemidense_mapper);
        sup_thread.join();
        map_thread.join();
        cout << "PlyListener: joined threads\n";

        {
            boost::unique_lock<boost::mutex> lock(pply_listener->pcd_count_mutex);
            if (pcd_count_save == pply_listener->pcd_count) {
                lock.unlock();
                cout << "PlyListener: Generating mesh from pcd files!\n";
                Mesher mesher;
                mesher.reconstruct(pcd_dir, mesh_file);
                break;
            } else {
                lock.unlock();
            }
        }
        cout << "PlyListener: pcd_count_save = " << pcd_count_save << "\n";
        boost::this_thread::sleep(boost::posix_time::milliseconds(33));
    }
    cout << "PlyListener: Exiting main thread ThreadPlyListener.\n";
}

void thread_ply_listener_map(PlyListener *pply_listener, SemiDenseMapping *psemidense_mapper) {
    cout << "Entered thread_ply_listener_map!\n";
    clock_t t = clock();
    {
        boost::unique_lock<boost::mutex> lock(psemidense_mapper->ply_mutex);
        while (!psemidense_mapper->ply_ready) {
            if (((float)(clock() - t)/CLOCKS_PER_SEC) > TIMEOUT) {
                cout << "PlyListener.thread_ply_listener_map: timeout!\n";
                return;
            }
            cout << "PlyListener.thread_ply_listener_map: waiting for ply_ready.\n";
            psemidense_mapper->ply_cond.wait(lock);
        }
        psemidense_mapper->ply_ready = false;
        lock.unlock();
    }
    
    // convert the new ply files to pcd files
    cout << "PlyListener: converting new ply file\n"; 
    char pcdfile[512];
    {
        boost::unique_lock<boost::mutex> lock(pply_listener->pcd_count_mutex);
        snprintf(pcdfile, 512, "%s/map%d.pcd", pply_listener->pcd_dir.c_str(), pply_listener->pcd_count++);
        lock.unlock();
    }
    std::string foo = pply_listener->convert_ply2pcd_file(psemidense_mapper->ply_file, pcdfile);
}

void thread_ply_listener_sup(PlyListener *pply_listener, DenseMapping *pdense_mapper) {
    cout << "Entered thread_ply_listener_sup!\n";
    clock_t t = clock();
    {
        boost::unique_lock<boost::mutex> lock(pdense_mapper->ply_mutex);
        while (!pdense_mapper->ply_ready) {
            if (((float)(clock() - t)/CLOCKS_PER_SEC) > TIMEOUT) {
                cout << "PlyListener.thread_ply_listener_sup: timeout!\n";
                return;
            }
            pdense_mapper->ply_cond.wait(lock);
        }
        pdense_mapper->ply_ready = false;
        lock.unlock();
    }
    
    // convert the new ply files to pcd files
    cout << "PlyListener: converting new ply file\n"; 
    char pcdfile[512];
    {
        boost::unique_lock<boost::mutex> lock(pply_listener->pcd_count_mutex);
        snprintf(pcdfile, 512, "%s/map%d.pcd", pply_listener->pcd_dir.c_str(), pply_listener->pcd_count++);
        lock.unlock();
    }
    std::string foo = pply_listener->convert_ply2pcd_file(pdense_mapper->ply_file, pcdfile);
}

std::string execute(const char *cmd) {
    cout << "Executing command: " << cmd << "\n";

    std::shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
    
    if (!pipe) return "";
    
    char buffer[128];
    std::string result = "";
    
    while (!feof(pipe.get())) {
        if (fgets(buffer, 128, pipe.get()) != NULL)
            result += buffer;
    }
    
    return result;
}


