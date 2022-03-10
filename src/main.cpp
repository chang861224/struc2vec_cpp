#include <iostream>
#include <vector>
#include <map>
#include <cstdlib>
#include <ctime>
#include "graph.h"
#include "struc2vec.h"
using namespace std;

int main(){
    srand(time(NULL));
    string filename = "barbell.txt";
    Graph G(filename, false);

    cout << "Total edges: " << G.getNumEdges() << endl;

    struc2vec s(G, true, 3);

    s.CreateDistNetwork();
    s.PreprocessParamsRandomWalk();
    vector< vector<long> > walks = s.SimulateWalks(10, 80);
    
    SaveRandomWalks(walks);

    return 0;
}
