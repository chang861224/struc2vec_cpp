#include <iostream>
#include <vector>
#include <unordered_map>
#include "graph.h"
//#include "distance_alg.h"
#include "struc2vec.h"
using namespace std;

int main(){
    string filename = "barbell.txt";
    Graph G(filename, false);

    cout << "Total edges: " << G.getNumEdges() << endl;

    struc2vec s(G, true, 3);

    cout << "PreprocessNeighborsBFS" << endl;
    s.PreprocessNeighborsBFS();
    cout << "PreprocessNeighborsBFS" << endl;
    cout << "PreprocessDegreeLists" << endl;
    s.PreprocessDegreeLists();
    cout << "PreprocessDegreeLists" << endl;
    cout << "CreateVectors" << endl;
    s.CreateVectors();
    cout << "CreateVectors" << endl;
    cout << "CalDistAllVertices" << endl;
    s.CalDistAllVertices();
    cout << "CalDistAllVertices" << endl;
    cout << "CalDistVertices" << endl;
    s.CalDistVertices();
    cout << "CalDistVertices" << endl;
    cout << "CreateDistNetwork" << endl;
    s.CreateDistNetwork();
    cout << "CreateDistNetwork" << endl;
    cout << "PreprocessParamsRandomWalk" << endl;
    s.PreprocessParamsRandomWalk();
    cout << "PreprocessParamsRandomWalk" << endl;
    cout << "SimulateWalks" << endl;
    s.SimulateWalks(10, 80);
    cout << "SimulateWalks" << endl;
/*
    unordered_map< long, vector<long> > m = G.getGraph();
    unordered_map< long, unordered_map< int, vector<double> > > degree_lists = getDegreeListsVertices(G, 3);

    for(auto& v: degree_lists){
        cout << v.first << endl;

        for(auto& l: v.second){
            cout << l.first << ":";

            for(auto e: l.second){
                cout << " " << e;
            }
            cout << endl;
        }

        cout << endl;
    }

    cout << structuralDist(degree_lists[18], degree_lists[19], 1) << endl;
    cout << structuralDist(degree_lists[18], degree_lists[19], 2) << endl;
    cout << structuralDist(degree_lists[18], degree_lists[19], 3) << endl;
    cout << DTW(degree_lists[18][3], degree_lists[19][1]) << endl;
    cout << DTW(degree_lists[18][3], degree_lists[19][2]) << endl;
    cout << DTW(degree_lists[18][3], degree_lists[19][3]) << endl;
*/
    return 0;
}
