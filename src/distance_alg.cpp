#include "distance_alg.h"

unordered_map< long, unordered_map< int, vector<double> > > getDegreeListsVertices(Graph& G, int layers){
    unordered_map< long, unordered_map< int, vector<double> > > degree_list;

    for(auto v = 0 ; v < G.getNumNodes() ; v++){
        degree_list[v] = getDegreeLists(G, v, layers);
    }

    return degree_list;
}

unordered_map< int, vector<double> > getDegreeLists(Graph& G, long root, int layers){
    unordered_map< int, vector<double> > listas;
    unordered_map< long, vector<long> > g = G.getGraph();
    vector<bool> vetor_marcacao(G.getNumNodes() + 1, false);

    vector<long> queue;
    queue.push_back(root);
    vetor_marcacao[root] = true;

    vector<double> l;

    int depth = 0;
    int pendingDepthIncrease = 0;
    int timeToDepthIncrease = 1;

    while(!queue.empty()){
        long vertex = queue[0];
        queue.erase(queue.begin());
        timeToDepthIncrease -= 1;

        l.push_back(double(g[vertex].size()));

        for(long v: g[vertex]){
            if(!vetor_marcacao[v]){
                vetor_marcacao[v] = true;
                queue.push_back(v);
                pendingDepthIncrease += 1;
            }
        }

        if(timeToDepthIncrease == 0){
            vector<double> lp = l;
            sort(lp.begin(), lp.end());
            listas[depth] = lp;
            l.clear();

            if(layers == depth){
                break;
            }

            depth += 1;
            timeToDepthIncrease = pendingDepthIncrease;
            pendingDepthIncrease = 0;
        }
    }

    return listas;
}

double DTW(vector<double>& s, vector<double>& t){
    int s_length = s.size();
    int t_length = t.size();
    vector< vector<double> > dtw_matrix(s_length + 1, vector<double>(t_length + 1, 0));

    for(int i = 0 ; i < s_length + 1 ; i++){
        for(int j = 0 ; j < t_length + 1 ; j++){
            dtw_matrix[i][j] = INT_MAX;
        }
    }

    dtw_matrix[0][0] = 0;

    for(int i = 1 ; i < s_length + 1 ; i++){
        for(int j = 1 ; j < t_length + 1 ; j++){
            //double cost = (s[i - 1] - t[j - 1] > 0) ? (s[i - 1] - t[j - 1]) : (t[j - 1] - s[i - 1]);
            double cost = dist_func(s[i - 1], t[j - 1]);

            // Take last min from a square box
            double last_min = min(min(dtw_matrix[i - 1][j], dtw_matrix[i][j - 1]), dtw_matrix[i - 1][j - 1]);
            dtw_matrix[i][j] = cost + last_min;
        }
    }

    return dtw_matrix[s_length][t_length];
}

double structuralDist(unordered_map< int, vector<double> >& v1, unordered_map< int, vector<double> >& v2, int layer){
    if(layer == 0){
        return 0.0;
    }

    return DTW(v1[layer], v2[layer]) + structuralDist(v1, v2, layer - 1);
}

vector< vector< unordered_map<int, double> > > calDistances(Graph G, int layers){
    vector< vector< unordered_map<int, double> > > distances(G.getNumNodes(), vector< unordered_map<int, double> >(G.getNumNodes()));
    unordered_map< long, vector<long> > g = G.getGraph();
    unordered_map< long, unordered_map< int, vector<double> > > degree_list = getDegreeListsVertices(G, layers);

    for(auto iter = g.begin() ; iter != g.end() ; iter++){
        auto v1 = iter->first;
        unordered_map< int, vector<double> > degrees_v1 = degree_list[v1];

        for(auto v2: iter->second){
            unordered_map< int, vector<double> >degrees_v2 = degree_list[v2];
            int max_layer = min(degrees_v1.size(), degrees_v2.size());

            for(auto layer = 0 ; layer < max_layer ; layer++){
                double dist = DTW(degrees_v1[layer], degrees_v2[layer]);
                distances[v1][v2][layer] = dist;
            }
        }
    }

    return distances;
}

vector< vector< unordered_map<int, double> > > calAllDistances(Graph G, int layers){
    vector< vector< unordered_map<int, double> > > distances(G.getNumNodes(), vector< unordered_map<int, double> >(G.getNumNodes()));
    unordered_map< long, vector<long> > g = G.getGraph();
    unordered_map< long, unordered_map< int, vector<double> > > degree_list = getDegreeListsVertices(G, layers);
    
    long cont = 0;

    for(auto iter1 = g.begin() ; iter1 != g.end() ; iter1++){
        auto v1 = iter1->first;
        unordered_map< int, vector<double> > degrees_v1 = degree_list[v1];

        for(auto iter2 = g.begin() ; iter2 != g.end() ; iter2++){
            auto v2 = iter2->first;
            unordered_map< int, vector<double> >degrees_v2 = degree_list[v2];
            int max_layer = min(degrees_v1.size(), degrees_v2.size());

            for(auto layer = 0 ; layer < max_layer ; layer++){
                double dist = DTW(degrees_v1[layer], degrees_v2[layer]);
                distances[v1][v2][layer] = dist;
            }
        }
    }

    return distances;
}

