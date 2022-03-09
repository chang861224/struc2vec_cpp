#include "graph.h"

Graph::Graph(string filename, bool undirected=true){
    num_nodes = 0;
    num_edges = 0;
    LoadEdgeList(filename, undirected);
}

Graph::~Graph(){
}

long Graph::searchNodeId(string node){
    return node2id.count(node) ? node2id[node] : -1;
}

long Graph::getNumNodes() const{
    return num_nodes;
}

long Graph::getNumEdges() const{
    return num_edges;
}

map< long, vector<long> > Graph::getGraph() const{
    return graph;
}

long Graph::Node2Id(string node){
    if(!node2id.count(node)){
        node2id[node] = num_nodes++;
    }

    return node2id[node];
}

void Graph::LoadEdgeList(string filename, bool undirected){
    ifstream file(filename);
    string from_node;
    string to_node;

    if(file.is_open()){
        while(file >> from_node >> to_node){
            graph[Node2Id(from_node)].push_back(Node2Id(to_node));
            num_edges += 1;

            if(undirected){
                graph[Node2Id(to_node)].push_back(Node2Id(to_node));
                num_edges += 1;
            }
        }

        file.close();
    }
    else{
        cout << "Graph file not found!!" << endl;
    }
}

