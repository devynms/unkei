#include <time.h>
#include <unistd.h>

#include <string>
#include <iostream>
#include <cstdio>
#include <memory>

#include <dpptam/PlyListener.h>
#include <dpptam/vo_system.h>
//#include <dpptam/Mesher.h>

// max seconds to wait on condition variable
#define TIMEOUT 60 
#define PCD_COUNT_MAX 30

PlyListener::PlyListener() : pcd_count(0) { }

void ThreadPlyListener(PlyListener *pply_listener, DenseMapping *pdense_mapper, SemiDenseMapping *psemidense_mapper, vo_system *pSystem, const std::string& pcd_dir, const std::string& mesh_file) {

    boost::filesystem::remove_all(pcd_dir.c_str());
    boost::filesystem::create_directory(pcd_dir.c_str());

    pply_listener->pcd_dir = pcd_dir;
    pply_listener->mesh_file = mesh_file;
    int pcd_count_save = 0;
    
    {boost::unique_lock<boost::mutex> done_lock(pSystem->done_mutex);
    while(!pSystem->done)
    {
        cout << "PlyListener: top of while (acquiring lock on pcd_count_mutex)\n";
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
            //if (pcd_count_save == pply_listener->pcd_count) {
            if (pply_listener->pcd_count > PCD_COUNT_MAX) {
                lock.unlock();
                cout << "PlyListener: Generating mesh from pcd files!\n";
                //Mesher mesher;
                //mesher.reconstruct(pcd_dir.c_str(), mesh_file.c_str());
                std::string mesh_exec_path = "/home/acceber/workspaces/ros_catkin/devel/lib/dpptam/mesher";
                if (execlp(mesh_exec_path.c_str(), mesh_exec_path.c_str(), pcd_dir.c_str(), mesh_file.c_str(), (char *)(NULL)) == -1) {
                    printf("Mesher process failed to execute.\n");
                }
            } else {
                lock.unlock();
            }
        }
        cout << "PlyListener: pcd_count_save = " << pcd_count_save << "\n";
        pSystem->done_cond.wait(done_lock);
        //boost::this_thread::sleep(boost::posix_time::milliseconds(33));
    }
    done_lock.unlock();
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
    cout << "PlyListener.thread_ply_listener_map: converting new ply file\n"; 
    thread_ply_listener_convert_ply2pcd_file(psemidense_mapper->ply_file, pply_listener);
    cout << "PlyListener.thread_ply_listener_map: exiting\n"; 
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
    cout << "PlyListener.thread_ply_listener_sup: converting new ply file\n"; 
    thread_ply_listener_convert_ply2pcd_file(pdense_mapper->ply_file, pply_listener);
    cout << "PlyListener.thread_ply_listener_sup: exiting\n"; 
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

void thread_ply_listener_convert_ply2pcd_file(const char *ply_file, PlyListener *pply_listener) {
    char pcdfile[512];
    {
        boost::unique_lock<boost::mutex> lock(pply_listener->pcd_count_mutex);
        snprintf(pcdfile, 512, "%s/points%d.pcd", pply_listener->pcd_dir.c_str(), pply_listener->pcd_count++);
        lock.unlock();
    }
    std::string foo = pply_listener->convert_ply2pcd_file(ply_file, pcdfile);
}

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




