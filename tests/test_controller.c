#include <string.h>
int main (int argc, char ** argv) {
    // test controller functionality
    if (strcmp(argv[1],"fail") == 0) return -1; // bad!
    else return 0; // correct
}
