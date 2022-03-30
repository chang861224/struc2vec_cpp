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

void word2vec_train(string train_filename, string model_filename, int dimensions) {
    w2v::trainSettings_t trainSettings;

    string stopWordsFile = "";

    trainSettings.size = static_cast<uint16_t>(dimensions);
    trainSettings.withSG = true;


    w2v::w2vModel_t model;
    bool trained = model.train(trainSettings, train_filename, stopWordsFile, nullptr, nullptr, nullptr);

    if (!trained) {
        cerr << "Training failed: " << model.errMsg() << endl;
        return;
    }

    if (!model.save(model_filename)) {
        cerr << "Model file saving failed: " << model.errMsg() << endl;
        return;
    }
}

void SaveEmbedding(string w2v_filename, string embed_filename, Graph G, int dimensions){
    // load pre-trained model
    unique_ptr<w2v::w2vModel_t> w2vModel;
    w2vModel.reset(new w2v::w2vModel_t());
    w2vModel->load(w2v_filename);

    ofstream f(embed_filename);

    f << G.getNumNodes() << " " << dimensions << endl;

    for(auto i = 0 ; i < G.getNumNodes() ; i++){
        string vertex = G.searchNode(i);
        w2v::word2vec_t node(w2vModel, vertex);

        f << vertex;

        for(auto x: node){
            f << " " << x;
        }

        f << endl;
    }

    f.close();
}

void logging(char* s){
    ofstream file("struc2vec.log", ios_base::app);
    file << s << endl;
    file.close();
}
