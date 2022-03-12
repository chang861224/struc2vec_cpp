#include "utils.h"

bool cmp(const vector<double> a, const vector<double> b){
    return a[0] < b[0];
}

double dist_func(double a, double b){
    return ((max(a, b) + 0.5) / (min(a, b) + 0.5) - 1);
}

void SaveRandomWalks(vector< vector<long> > walks, Graph G){
    ofstream f("random_walks.txt");

    for(auto walk: walks){
        string s = "";

        for(auto vertex: walk){
            //s += to_string(vertex);
            s += G.searchNode(vertex);
            s += " ";
        }

        s += "\n";
        f << s;
    }

    f.close();
}
