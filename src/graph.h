#ifndef _GRAPH_H_
#define _GRAPH_H_

#include <iostream>
#include <fstream>
#include <map>
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
        map< long, vector<long> > getGraph() const;

    private:
        // Preprocessing
        long Node2Id(string node);
        void LoadEdgeList(string filename, bool undirected);

        // Elements
        map< string, long > node2id;
        map< long, vector<long> > graph;
        long num_nodes;
        long num_edges;
};

#endif
