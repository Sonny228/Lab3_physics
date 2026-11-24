// Консольная версия экспериментов без SFML: pz3_experiments [файл.txt]
#include "Experiments.h"
int main(int argc, char** argv) {
    runAllExperiments(argc > 1 ? argv[1] : "experiments.txt");
    return 0;
}
