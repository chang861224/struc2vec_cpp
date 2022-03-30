#ifndef _UTILS_H_
#define _UTILS_H_

#include <fstream>
#include <string>
#include <algorithm>
#include <vector>
#include "graph.h"
#include "word2vec/include/word2vec.hpp"
using namespace std;

bool cmp(const vector<double> a, const vector<double> b);
double dist_func(double a, double b);
void SaveRandomWalks(vector< vector<long> > walks, Graph G);
void word2vec_train(string train_filename, string model_filename, int dimensions);
void SaveEmbedding(string w2v_filename, string embed_filename, Graph G, int dimensions);
void logging(char* s);

#endif
