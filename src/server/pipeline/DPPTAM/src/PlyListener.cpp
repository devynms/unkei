#include <string>
#include <iostream>
#include <cstdio>
#include <memory>
#include <ros/package.h>
#include <ros/ros.h>
#include <time.h>
#include <unistd.h>

#include <dpptam/PlyListener.h>
//#include <dpptam/Mesher.h>

// max seconds to wait on condition variable
#define TIMEOUT 60 
#define PCD_COUNT_MAX 30
#define PLY_TIMEOUT_S 20 

PlyListener::PlyListener() : pcd_count(0) { }

void getMesherExecPath(std::string& path) {
    path.clear();
    
    char *cwd = get_current_dir_name();
    path = std::string(cwd);
    free(cwd);

    printf("Current Working directory is: %s\n", path.c_str());

    if (path.find("DPPTAM") != std::string::npos) {
        //nop
    } else if (path.find("pipeline") != std::string::npos) {
        path += "/DPPTAM";
    } else if (path.find("server") != std::string::npos) {
        path += "/pipeline/DPPTAM";
    } else if (path.find("src") != std::string::npos) {
        path += "/server/pipeline/DPPTAM";
    } else if (path.find("unkei") != std::string::npos) {
        path += "/src/server/pipeline/DPPTAM";
    } else {
        printf("WARNING: current working directory not recognized!\n");
        path = "/home/acceber/projects/eecs395/unkei/src/server/pipeline/DPPTAM";
    }
    path += "/devel/lib/dpptam/mesher";
}

void ThreadPlyListener(PlyListener *pply_listener, DenseMapping *pdense_mapper, SemiDenseMapping *psemidense_mapper, const std::string& pcd_dir, const std::string& mesh_file) {

    boost::filesystem::remove_all(pcd_dir.c_str());
    boost::filesystem::create_directory(pcd_dir.c_str());

    pply_listener->pcd_dir = pcd_dir;
    pply_listener->mesh_file = mesh_file;
    pply_listener->tlast = clock(); 
    pply_listener->pcd_count = 0;
    while(ros::ok())
    {
        cout << "PlyListener: spawning threads\n";
        boost::thread sup_thread(&thread_ply_listener_sup, pply_listener, pdense_mapper);
        boost::thread map_thread(&thread_ply_listener_map, pply_listener, psemidense_mapper);
        sup_thread.join();
        map_thread.join();
        cout << "PlyListener: joined threads\n";

        {
            boost::unique_lock<boost::mutex> pcd_lock(pply_listener->pcd_count_mutex);
            boost::unique_lock<boost::mutex> tlast_lock(pply_listener->tlast_mutex);
            cout << "PlyListener: " << pply_listener->pcd_count << " pcd files\n";
            cout << "PlyListener: " << (float)(clock() - pply_listener->tlast)/CLOCKS_PER_SEC << " seconds since last conversion\n";
            if ((float)(clock() - pply_listener->tlast)/CLOCKS_PER_SEC > PLY_TIMEOUT_S && pply_listener->pcd_count > 0) {
                tlast_lock.unlock();
                pcd_lock.unlock();
                std::string mesh_exec_path;//= str_cwd + "/pipeline/DPPTAM/devel/lib/dpptam/mesher";
                getMesherExecPath(mesh_exec_path);
                if (execlp(mesh_exec_path.c_str(), mesh_exec_path.c_str(), pcd_dir.c_str(), mesh_file.c_str(), (char *)(NULL)) == -1) {
                    printf("PlyListener: Mesher process failed to execute.\n");
                } else {
                    cout << "PlyListener: Generating mesh from pcd files!\n";
                }
            } else {
                tlast_lock.unlock();
                pcd_lock.unlock();
            }
        }
        boost::this_thread::sleep(boost::posix_time::milliseconds(33));
    }
    cout << "PlyListener: Exiting main thread ThreadPlyListener.\n";
}

void thread_ply_listener_map(PlyListener *pply_listener, SemiDenseMapping *psemidense_mapper) {
    //cout << "Entered thread_ply_listener_map!\n";
    //clock_t t = clock();
    //{
    //    boost::unique_lock<boost::mutex> lock(psemidense_mapper->ply_mutex);
    //    while (!psemidense_mapper->ply_ready) {
    //        if (((float)(clock() - t)/CLOCKS_PER_SEC) > TIMEOUT) {
    //            cout << "PlyListener.thread_ply_listener_map: timeout!\n";
    //            return;
    //        }
    //        cout << "PlyListener.thread_ply_listener_map: waiting for ply_ready.\n";
    //        psemidense_mapper->ply_cond.wait(lock);
    //    }
    //    psemidense_mapper->ply_ready = false;
    //    lock.unlock();
    //}
    
    // convert the new ply files to pcd files
    //cout << "PlyListener.thread_ply_listener_map: converting new ply file\n"; 
    //thread_ply_listener_convert_ply2pcd_file(psemidense_mapper->ply_file, pply_listener);
    //cout << "PlyListener.thread_ply_listener_map: exiting\n"; 


    {
        boost::unique_lock<boost::mutex> lock(psemidense_mapper->ply_mutex);
        if (psemidense_mapper->ply_ready) {
            psemidense_mapper->ply_ready = false;
            lock.unlock();
            cout << "PlyListener.thread_ply_listener_map: converting new ply file\n"; 
            thread_ply_listener_convert_ply2pcd_file(psemidense_mapper->ply_file, pply_listener);
        } else {
            lock.unlock();
            cout << "PlyListener.thread_ply_listener_map: nothing to convert.\n"; 
        }
    }
    cout << "PlyListener.thread_ply_listener_map: exiting\n"; 
}

void thread_ply_listener_sup(PlyListener *pply_listener, DenseMapping *pdense_mapper) {
    cout << "Entered thread_ply_listener_sup!\n";
    //clock_t t = clock();
    //{
    //    boost::unique_lock<boost::mutex> lock(pdense_mapper->ply_mutex);
    //    while (!pdense_mapper->ply_ready) {
    //        if (((float)(clock() - t)/CLOCKS_PER_SEC) > TIMEOUT) {
    //            cout << "PlyListener.thread_ply_listener_sup: timeout!\n";
    //            return;
    //        }
    //        pdense_mapper->ply_cond.wait(lock);
    //    }
    //    pdense_mapper->ply_ready = false;
    //    lock.unlock();
    //}

    // convert the new ply files to pcd files
    //cout << "PlyListener.thread_ply_listener_sup: converting new ply file\n"; 
    //thread_ply_listener_convert_ply2pcd_file(pdense_mapper->ply_file, pply_listener);
    //cout << "PlyListener.thread_ply_listener_sup: exiting\n"; 
    
    
    {
        boost::unique_lock<boost::mutex> lock(pdense_mapper->ply_mutex);
        if (pdense_mapper->ply_ready) {
            pdense_mapper->ply_ready = false;
            lock.unlock();
            cout << "PlyListener.thread_ply_listener_sup: converting new ply file\n"; 
            thread_ply_listener_convert_ply2pcd_file(pdense_mapper->ply_file, pply_listener);
        } else {
            lock.unlock();
            cout << "PlyListener.thread_ply_listener_sup: nothing to convert.\n"; 
        }
    }
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

    {
        boost::unique_lock<boost::mutex> lock(pply_listener->tlast_mutex);
        pply_listener->tlast = clock();
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




