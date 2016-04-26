#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <stdio.h>
#include <signal.h>
#include <fcntl.h>


int main(int argc, char *argv[]) {
    if (argc < 2) {
        fprintf(stdout,"Usage: <main> <nth fib to calculate>\n");
        exit(EXIT_FAILURE);
    }

    //create the pipe: pipefd[0]=>read, pipefd[1]=>write
    int pipefd[2];
    if (pipe(pipefd) == -1) {
        fprintf(stdout,"pipe could not be created\n");
        exit(EXIT_FAILURE);
    }
    fprintf(stdout,"Successfully created pipefd: %d, %d\n", pipefd[0], pipefd[1]);
    
    pid_t child1_pid, child2_pid;
    char *path = "./pipefib";
    int buflen=8;
    char pipefdbuf0[buflen], pipefdbuf1[buflen];
    snprintf(pipefdbuf0, buflen, "%d", pipefd[0]);
    snprintf(pipefdbuf1, buflen, "%d", pipefd[1]);
    
    child1_pid = fork();
    if (child1_pid == -1) {
        fprintf(stdout,"fork unsucessful.\n");
        exit(EXIT_FAILURE);
    } else if (child1_pid == 0) {
        fprintf(stdout,"In child process 1! (pid=%d)\n", getpid());
        if (execlp(path, path, "0", pipefdbuf0, argv[1], (char*)NULL) == -1) {
            fprintf(stdout,"failed to execute process from child 1\n");
            exit(EXIT_FAILURE);
        }
    } else {
        child2_pid = fork();
        if (child2_pid == -1) {
            fprintf(stdout,"fork unsucessful.\n");
            exit(EXIT_FAILURE);
        } else if (child2_pid == 0) {
            fprintf(stdout,"In child process 2! (pid=%d)\n", getpid());
            if (execlp(path, path, "1", pipefdbuf1, argv[1], (char*)NULL) == -1) {
                fprintf(stdout,"failed to execute process from child 2\n");
                exit(EXIT_FAILURE);
            }
        } else {
            int status;
            fprintf(stdout,"Waiting for children...\n");
            wait(&status);
            fprintf(stdout,"Finished waiting for children.\n");
        }
    }
    return 0;
}


