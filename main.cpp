#include <iostream>
#include <vector>
#include <map>
#include <cstdlib>
#include <ctime>
#include "src/graph.h"
#include "src/struc2vec.h"
using namespace std;

int main(){
    srand(time(NULL));
    string filename = "graph/PubMed.edgelist";
    Graph G(filename, false);

    cout << "Total edges: " << G.getNumEdges() << endl;

    struc2vec s(G, true, 3);

    s.CreateDistNetwork();
    s.PreprocessParamsRandomWalk();
    vector< vector<long> > walks = s.SimulateWalks(10, 80);
    
    SaveRandomWalks(walks, G);
    word2vec_train("random_walks.txt", "w2v.model", 128);
    SaveEmbedding("w2v.model", "embedding.emb", G);

    return 0;
}
