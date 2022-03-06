#ifndef _GRAPH_H_
#define _GRAPH_H_

#include <iostream>
#include <fstream>
#include <unordered_map>
#include <vector>
using namespace std;

class Graph{
    public:
        // Constructor
        Graph(string filename, bool undirected);
        ~Graph();

        long searchNodeId(string node);

        // Functions of getting elements
        long getNumNodes() const;
        long getNumEdges() const;
        unordered_map< long, vector<long> > getGraph() const;

    private:
        // Preprocessing
        long Node2Id(string node);
        void LoadEdgeList(string filename, bool undirected);

        // Elements
        unordered_map< string, long > node2id;
        unordered_map< long, vector<long> > graph;
        long num_nodes;
        long num_edges;
};

#endif
