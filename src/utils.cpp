#include "utils.h"

bool cmp(const vector<double> a, const vector<double> b){
    return a[0] < b[0];
}

double dist_func(double a, double b){
    return ((max(a, b) + 0.5) / (min(a, b) + 0.5) - 1);
}

