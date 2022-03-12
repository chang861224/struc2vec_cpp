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

void SaveEmbedding(string w2v_filename, string embed_filename){
    // load pre-trained model
    unique_ptr<w2v::w2vModel_t> w2vModel;
    w2vModel.reset(new w2v::w2vModel_t());
    w2vModel->load(w2v_filename);

    w2v::word2vec_t GGG(w2vModel, "GGG");

    ofstream f(embed_filename);

    if(f.is_open()){
        for(auto x: GGG){
            f << x << " ";
        }

        f << endl;
    }

    f.close();
}

