#ifndef _STRUC2VEC_H_
#define _STRUC2VEC_H_

#include <cstdlib>
#include <vector>
#include <set>
#include <map>
#include <unordered_map>
#include <algorithm>
#include "graph.h"
//#include "distance_alg.h"
#include "utils.h"

class struc2vec{
    public:
        struc2vec(Graph input_graph, bool input_is_directed, int input_layers=3);
        ~struc2vec();

        void PreprocessNeighborsBFS();
        void PreprocessDegreeLists();
        void CreateVectors();
        vector< vector< unordered_map<int, double> > > CalDistAllVertices();
        vector< vector< unordered_map<int, double> > > CalDistVertices();
        void CreateDistNetwork();
        void PreprocessParamsRandomWalk();
        vector< vector<long> > SimulateWalks(int num_walks, int walk_length);

    private:
        unordered_map< int, vector<double> > getDegreeLists(long root);
        void ConsolideDist(vector< vector< unordered_map<int, double> > >& distances, int start_layer=1);
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

        unordered_map< int, unordered_map< string, vector<long> > > degrees;

        /*
           degree_list = {
                node_id: {
                    layer: degrees
                }
            }
        */
        unordered_map< long, unordered_map< int, vector<double> > > degree_list;
        unordered_map< long, unordered_map< int, vector< vector<double> > > > d_list;

        /*
           weights = {
                node_id: {
                    layer: [double, double, ... ]
                }
            }
        */
        unordered_map< long, unordered_map< int, vector<double> > > weights;

        /*
           average_weight = {
                layer: avg_weight
            }
        */
        unordered_map< int, double > average_weight;

        /*
           amount_neighbors = {
                layer: {
                    node_id: num_neighbors
                }
            }
        */
        unordered_map< int, unordered_map< long, long > > amount_neighbors;

        unordered_map< int, unordered_map< long, vector<int> > > alias_method_j;
        unordered_map< int, unordered_map< long, vector<double> > > alias_method_q;
    
        unordered_map< int, unordered_map< long, vector<long> > > graphs;
        vector< vector< unordered_map<int, double> > > weights_distances;
        unordered_map< int, unordered_map< long, vector<double> > > weights_probs;
};

#endif
