#ifndef PARAMS_H
#define PARAMS_H
#include <iostream>
#include <string>

static int Horizon_SCAN, N_SCAN;
extern void setParam(const int& a,const int& b){
  Horizon_SCAN = a;
  N_SCAN = b;
};


#endif // PARAMS_H