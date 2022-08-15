#ifndef _STRUC2VEC_H_
#define _STRUC2VEC_H_

#include <cstdlib>
#include <vector>
#include <set>
#include <map>
#include <algorithm>
#include <utility>
#include <cmath>
#include <climits>
#include "src/graph.h"
#include "src/utils.h"
#include <fstream>
using namespace std;

class struc2vec{
    public:
        struc2vec(Graph input_graph, int input_layers=3);
        ~struc2vec();

        void PreprocessNeighborsBFS();
        // void CalDistAllVertices();
        void CalDistVertices();
        void CreateDistNetwork();
        void PreprocessParamsRandomWalk();
        // vector< vector<long> > SimulateWalks(int num_walks, int walk_length);
        void SimulateWalks(int num_walks, int walk_length);

    private:
        // void PreprocessDegreeLists();
        // void CreateVectors();
        
        map< int, vector<double> > getDegreeLists(long root);
        void ConsolideDist(map< pair<long, long>, map<int, double> >& dists, int start_layer=1);
        double DTW(vector<double>& s, vector<double>& t);
        
        vector< long > ExecuteRandomWalk(long vertex, int walk_length);
        long ChooseNeighbor(long vertex, int layer);
        double ProbMoveup(long num_neighbors);
        long AliasDraw(vector<int> J, vector<double> q);
        
        void GenerateDistNetwork();
        void GenerateDistNetworkPart1();
        void GenerateDistNetworkPart2();
        void GenerateDistNetworkPart3();
        void AliasSetup(vector<double> probs, vector<int>& J, vector<double>& q);

        Graph G;
        int layers;

        map< int, map< string, vector<long> > > degrees;
        map< long, map< int, vector<double> > > degree_list;
        map< long, map< int, vector< vector<double> > > > d_list;

        map< pair<long, long>, map<int, double> > distances;

        map< long, map< int, vector<double> > > weights;
        map< int, double > average_weight;
        map< int, map< long, long > > amount_neighbors;
        map< int, map< pair<long, long>, double> > weights_distances;
        map< int, map< long, vector<double> > > weights_probs;

        map< int, map< long, vector<int> > > alias_method_j;
        map< int, map< long, vector<double> > > alias_method_q;
    
        map< int, map< long, vector<long> > > graphs;
};

#endif
