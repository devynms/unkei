#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <stdio.h>
#include <signal.h>
#include <fcntl.h>

void print_fib(int fd, int n) {
    int buflen = 16;
    char buf[buflen];
    int last2 = 0; 
    int last1 = 1;
    int thisfib;
    int i;
    for (i = 0; i < n; i++) {
        if (i == 0 || i == 1) {
            snprintf(buf, buflen, "%d", i);
        } else {
            thisfib = last2 + last1;
            snprintf(buf, buflen, "%d", thisfib);
            last2 = last1;
            last1 = thisfib;
        }
        write(fd, buf, buflen);
    }
}

void read_fib(int fd, int n) {
    int buflen = 16;
    char buf[buflen]; 
    int i;
    for (i = 0; i < n; i++) {
        read(fd, buf, buflen);    
    }

    printf("Fibonacci number %d is %d.\n", n, atoi(buf));
}


int main(int argc, char *argv[]) {
    if (argc != 4) {
        printf("Usage: <main> <read/write (0/1)> <fd> <n>\n");
        exit(EXIT_FAILURE);
    }

    int read0 = atoi(argv[1]);
    int fd = atoi(argv[2]);
    int n = atoi(argv[3]);

    if (!read0) {
        printf("Process %d: reading from %d\n", getpid(), fd);
        read_fib(fd, n);
    } else {
        printf("Process %d: writing to %d\n", getpid(), fd);
        print_fib(fd, n);
    }
    
    close(fd);
    return 0;
}
