#ifndef _DISTANCE_ALG_H_
#define _DISTANCE_ALG_H_

#include <algorithm>
#include <unordered_map>
#include <vector>
#include "graph.h"
#include "utils.h"
using namespace std;

unordered_map< long, unordered_map< int, vector<double> > > getDegreeListsVertices(Graph& G, int layers);
unordered_map< int, vector<double> > getDegreeLists(Graph& G, long root, int layers);
double DTW(vector<double>& s, vector<double>& t);
double structuralDist(unordered_map< int, vector<double> >& v1, unordered_map< int, vector<double> >& v2, int layer);
vector< vector< unordered_map<int, double> > > calDistances(Graph G, int layers);
vector< vector< unordered_map<int, double> > > calAllDistances(Graph G, int layers);

#endif
