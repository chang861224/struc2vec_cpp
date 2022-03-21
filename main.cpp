#include <iostream>
#include <vector>
#include <map>
#include <cstdlib>
#include <ctime>
#include <iomanip>
//#include "src/graph.h"
#include "struc2vec.h"
using namespace std;

int main(){
    srand(time(NULL));
    string filename = "graph/PubMed.edgelist";

    bool undirected = false;
    // int layers = 3;
    int num_walks = 10;
    int walk_length = 80;

    clock_t start = clock();

    Graph G(filename, undirected);
    struc2vec model(G, true);

    model.PreprocessNeighborsBFS();
    // model.CalDistVertices();
    model.CalDistAllVertices();
    model.CreateDistNetwork();
    model.PreprocessParamsRandomWalk();
    vector< vector<long> > walks = model.SimulateWalks(num_walks, walk_length);
    
    SaveRandomWalks(walks, G);
    word2vec_train("random_walks.txt", "w2v.model", 128);
    SaveEmbedding("w2v.model", "embedding.emb", G);

    clock_t end = clock();

    double duration = double(end - start) / double(CLOCKS_PER_SEC);

    cout << "Total edges: " << G.getNumEdges() << endl;
    cout << "Execution time: " << fixed << duration << setprecision(4) << " sec" << endl;

    return 0;
}
