#include "struc2vec.h"

struc2vec::struc2vec(Graph input_graph, bool input_is_directed, int input_layers): G(input_graph){
    is_directed = input_is_directed;
    layers = input_layers;
}

struc2vec::~struc2vec(){
}

void struc2vec::PreprocessNeighborsBFS(){
    for(auto& v: G.getGraph()){
        degree_list[v.first] = getDegreeLists(v.first);
    }
}

map< int, vector<double> > struc2vec::getDegreeLists(long root){
    map< int, vector<double> > listas;
    map< long, vector<long> > g = G.getGraph();
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

void struc2vec::PreprocessDegreeLists(){
    map< long, map< int, map<double, double> > > d_freq;

    for(auto& degree_v: degree_list){
        for(auto& degree_layer: degree_v.second){
            for(auto& degree: degree_layer.second){
                d_freq[degree_v.first][degree_layer.first][degree] += 1;
            }
        }
    }

    for(auto& degree_v: d_freq){
        for(auto& degree_layer: degree_v.second){
            vector< vector<double> > list_d;

            for(auto& degree: degree_layer.second){
                vector<double> v;
                v.push_back(degree.first);
                v.push_back(degree.second);
                list_d.push_back(v);
            }

            sort(list_d.begin(), list_d.end(), cmp);
            d_list[degree_v.first][degree_layer.first] = list_d;
        }
    }
}

void struc2vec::CreateVectors(){
    set<int> degrees_sorted;
    map< long, vector<long> > g = G.getGraph();

    for(auto& v: g){
        int degree = g[v.first].size();
        degrees_sorted.insert(degree);
        degrees[degree]["vertices"].push_back(v.first);
    }

    int l = degrees_sorted.size();
    vector<int> degrees_sorted_v(degrees_sorted.begin(), degrees_sorted.end());

    for(int i = 0 ; i < degrees_sorted_v.size() ; i++){
        int degree = degrees_sorted_v[i];

        if(i > 0){
            degrees[degree]["before"].clear();
            degrees[degree]["before"].push_back(degrees_sorted_v[i - 1]);
        }

        if(i < l - 1){
            degrees[degree]["after"].clear();
            degrees[degree]["after"].push_back(degrees_sorted_v[i + 1]);
        }
    }
}

map< pair<long, long>, map<int, double> > struc2vec::CalDistAllVertices(){
    map< pair<long, long>, map<int, double> > distances;
    map< long, vector<long> > g = G.getGraph();
    long cont = 0;

    for(auto iter1 = g.begin() ; iter1 != g.end() ; iter1++){
        auto v1 = iter1->first;
        map< int, vector<double> > degrees_v1 = degree_list[v1];

        for(auto iter2 = g.begin() ; iter2 != g.end() ; iter2++){
            auto v2 = iter2->first;
            map< int, vector<double> >degrees_v2 = degree_list[v2];
            int max_layer = min(degrees_v1.size(), degrees_v2.size());

            for(auto layer = 0 ; layer <= max_layer ; layer++){
                double dist = DTW(degrees_v1[layer], degrees_v2[layer]);
                distances[make_pair(v1, v2)][layer] = dist;
            }
        }
    }

    ConsolideDist(distances);

    return distances;
}

map< pair<long, long>, map<int, double> > struc2vec::CalDistVertices(){
    map< pair<long, long>, map<int, double> > distances;
    map< long, vector<long> > g = G.getGraph();

    for(auto iter: g){
        auto v1 = iter.first;
        map< int, vector<double> > lists_v1 = degree_list[v1];

        for(auto v2: iter.second){
            map< int, vector<double> >lists_v2 = degree_list[v2];
            int max_layer = min(lists_v1.size(), lists_v2.size());

            for(auto layer = 0 ; layer <= max_layer ; layer++){
                double dist = DTW(lists_v1[layer], lists_v2[layer]);
                distances[make_pair(v1, v2)][layer] = dist;
            }
        }
    }

    ConsolideDist(distances);

    return distances;
}

void struc2vec::ConsolideDist(map< pair<long, long>, map<int, double> >& distances, int start_layer){
    for(auto distance: distances){
        map<int, double>& layers = distance.second;
        map<int, double> keys_layers(layers.begin(), layers.end());
        start_layer = min(int(keys_layers.size()), start_layer);

        for(int layer = 0 ; layer < start_layer ; layer++){
            keys_layers.erase(keys_layers.begin());
        }

        for(auto& layer: keys_layers){
            layers[layer.first] += layers[layer.first - 1];
        }
    }
}

void struc2vec::CreateDistNetwork(){
    GenerateDistNetwork();
}

void struc2vec::PreprocessParamsRandomWalk(){
    map< int, double > sum_weights;
    map< int, double > amount_edges;

    for(int layer = 0 ; layer <= layers ; layer++){
        for(auto& weight: weights){
            for(auto& w: weight.second[layer]){
                sum_weights[layer] += w;
                amount_edges[layer] += 1;
            }
        }
    }

    for(int layer = 0 ; layer <= layers ; layer++){
        average_weight[layer] = sum_weights[layer] / amount_edges[layer];
    }

    for(int layer = 0 ; layer <= layers ; layer++){
        for(auto& weight: weights){
            long cont_neighbors = 0;

            for(auto& w: weight.second[layer]){
                if(w > average_weight[layer]){
                    cont_neighbors += 1;
                }
            }

            amount_neighbors[layer][weight.first] = cont_neighbors;
        }
    }
}

vector< vector<long> > struc2vec::SimulateWalks(int num_walks, int walk_length){
    vector< vector<long> > walks;
    map< long, vector<long> > g = G.getGraph();

    for(auto& v: g){
        walks.push_back(ExecuteRandomWalk(v.first, walk_length));
    }

    return walks;
}

double struc2vec::DTW(vector<double>& s, vector<double>& t){
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

vector< long > struc2vec::ExecuteRandomWalk(long vertex, int walk_length){
    int init_layer = 0;
    int layer = init_layer;
    vector<long> path;

    path.push_back(vertex);

    while(path.size() < walk_length){
        double r = (double) rand() / RAND_MAX;

        if(r < 0.3){
            vertex = ChooseNeighbor(vertex, layer);
            path.push_back(vertex);
        }
        else{
            r = (double) rand() / RAND_MAX;
            double limiar_moveup = ProbMoveup(amount_neighbors[layer][vertex]);

            if(r > limiar_moveup){
                if(layer > init_layer){
                    layer -= 1;
                }
            }
            else{
                if((layer + 1 <= layers) && (graphs[layer + 1].count(vertex))){
                    layer += 1;
                }
            }
        }
    }

    return path;
}

long struc2vec::ChooseNeighbor(long vertex, int layer){
    vector<long> v_list = graphs[layer][vertex];

    long idx = AliasDraw(alias_method_j[layer][vertex], alias_method_q[layer][vertex]);
    return v_list[idx];
}

double struc2vec::ProbMoveup(long num_neighbors){
    double x = log(num_neighbors + exp(1));
    return x / (x + 1);
}

long struc2vec::AliasDraw(vector<int> J, vector<double> q){
    int K = J.size();

    int kk = int((double)rand() / RAND_MAX * K);

    if((double) rand() / RAND_MAX < q[kk]){
        return kk;
    }

    return J[kk];
}

void struc2vec::GenerateDistNetwork(){
    GenerateDistNetworkPart1();
    GenerateDistNetworkPart2();
    GenerateDistNetworkPart3();
}

void struc2vec::GenerateDistNetworkPart1(){
    map< pair<long, long>, map<int, double> > distances = CalDistVertices();

    for(auto distance: distances){
        long v1 = distance.first.first;
        long v2 = distance.first.second;

        for(auto dist: distance.second){
            int layer = dist.first;
            double d = dist.second;

            weights_distances[layer][make_pair(v1, v2)] = d;
        }
    }
}

void struc2vec::GenerateDistNetworkPart2(){
    map< pair<long, long>, map<int, double> > distances = CalDistVertices();

    for(auto distance: distances){
        long v1 = distance.first.first;
        long v2 = distance.first.second;

        for(auto dist: distance.second){
            int layer = dist.first;
            double d = dist.second;

            graphs[layer][v1].push_back(v2);
            graphs[layer][v2].push_back(v1);
        }
    }
}

void struc2vec::GenerateDistNetworkPart3(){

    for(int layer = 0 ; layer <= layers ; layer++){
        for(auto v: graphs[layer]){
            long v1 = v.first;

            vector<double> e_list;
            double sum_w = 0.0;

            for(long v2: v.second){
                double w = weights_distances[layer].count(make_pair(v1, v2)) ? 
                    exp(-weights_distances[layer][make_pair(v1, v2)]) : -exp(weights_distances[layer][make_pair(v2, v1)]);

                e_list.push_back(w);
                sum_w += w;
            }

            for(auto& x: e_list){
                x /= sum_w;
            }

            weights_probs[layer][v1] = e_list;
            AliasSetup(e_list, alias_method_j[layer][v1], alias_method_q[layer][v1]);
        }
    }
}

void struc2vec::GenerateDistNetworkPart4(){
}

void struc2vec::GenerateDistNetworkPart5(){
}

void struc2vec::GenerateDistNetworkPart6(){
}

void struc2vec::AliasSetup(vector<double> probs, vector<int>& J, vector<double>& q){
    int K = probs.size();
    vector<double> q1(K, 0.0);
    vector<int> J1(K, 0);

    vector<int> smaller;
    vector<int> larger;

    for(int i = 0 ; i < probs.size() ; i++){
        q1[i] = K * probs[i];

        if(q1[i] < 1.0){
            smaller.push_back(i);
        }
        else{
            larger.push_back(i);
        }
    }

    while((smaller.size() > 0) && (larger.size() > 0)){
        int small = smaller.back();
        smaller.pop_back();

        int large = larger.back();
        larger.pop_back();

        J1[small] = large;
        q1[large] = q1[large] + q1[small] - 1.0;

        if(q1[large] < 1.0){
            smaller.push_back(large);
        }
        else{
            larger.push_back(large);
        }
    }

    J.assign(J1.begin(), J1.end());
    q.assign(q1.begin(), q1.end());
}
