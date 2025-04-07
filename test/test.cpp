

#include "../TW2ILP/PathPattern.h"
#include "CoupledPP.h"

typedef PathPatternOptimized<unsigned int> PP;

using namespace std;

int main(int argc, char** argv) {

    unordered_map<PP,int> testmap;

    PP pp1{{1,2,3}};
    PP pp2{{3,2,1}};

    testmap[pp1]++;
    testmap[pp2]++;

    return 0;
}