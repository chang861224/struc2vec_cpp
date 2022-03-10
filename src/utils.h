#ifndef _UTILS_H_
#define _UTILS_H_

#include <fstream>
#include <string>
#include <algorithm>
#include <vector>
using namespace std;

bool cmp(const vector<double> a, const vector<double> b);
double dist_func(double a, double b);
void SaveRandomWalks(vector< vector<long> > walks);

#endif
