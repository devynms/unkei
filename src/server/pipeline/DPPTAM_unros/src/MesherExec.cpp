#include <dpptam/Mesher.h>

int main(int argc, char *argv[]) {
    if (argc != 3) {
        printf("Mesher: Usage: <main> <intputdir> <outputfile>\n");
        return EXIT_FAILURE;
    }
    
    Mesher mesher;
    mesher.reconstruct(argv[1], argv[2]);

    return 0;
}
