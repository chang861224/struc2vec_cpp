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
#include "graph.h"
#include "utils.h"
using namespace std;

class struc2vec{
    public:
        struc2vec(Graph input_graph, bool input_is_directed, int input_layers=3);
        ~struc2vec();

        void PreprocessNeighborsBFS();
        void PreprocessDegreeLists();
        void CreateVectors();
        map< pair<long, long>, map<int, double> > CalDistAllVertices();
        map< pair<long, long>, map<int, double> > CalDistVertices();
        void CreateDistNetwork();
        void PreprocessParamsRandomWalk();
        vector< vector<long> > SimulateWalks(int num_walks, int walk_length);

    private:
        map< int, vector<double> > getDegreeLists(long root);
        void ConsolideDist(map< pair<long, long>, map<int, double> >& distances, int start_layer=1);
        double DTW(vector<double>& s, vector<double>& t);
        vector< long > ExecuteRandomWalk(long vertex, int walk_length);
        long ChooseNeighbor(long vertex, int layer);
        double ProbMoveup(long num_neighbors);
        long AliasDraw(vector<int> J, vector<double> q);
        void GenerateDistNetwork();
        void GenerateDistNetworkPart1();
        void GenerateDistNetworkPart2();
        void GenerateDistNetworkPart3();
        void GenerateDistNetworkPart4();
        void GenerateDistNetworkPart5();
        void GenerateDistNetworkPart6();
        void AliasSetup(vector<double> probs, vector<int>& J, vector<double>& q);

        Graph G;
        bool is_directed;
        int layers;

        map< int, map< string, vector<long> > > degrees;

        /*
           degree_list = {
                node_id: {
                    layer: degrees
                }
            }
        */
        map< long, map< int, vector<double> > > degree_list;
        map< long, map< int, vector< vector<double> > > > d_list;

        /*
           weights = {
                node_id: {
                    layer: [double, double, ... ]
                }
            }
        */
        map< long, map< int, vector<double> > > weights;

        /*
           average_weight = {
                layer: avg_weight
            }
        */
        map< int, double > average_weight;

        /*
           amount_neighbors = {
                layer: {
                    node_id: num_neighbors
                }
            }
        */
        map< int, map< long, long > > amount_neighbors;

        map< int, map< long, vector<int> > > alias_method_j;
        map< int, map< long, vector<double> > > alias_method_q;
    
        map< int, map< long, vector<long> > > graphs;
        map< int, map< pair<long, long>, double> > weights_distances;
        map< int, map< long, vector<double> > > weights_probs;
};

#endif
