#ifndef CAC_H
#define CAC_H 

#include <vector>

#include "KChinesePostmen.h"

class CAC : public KChinesePostmen
{

public:
    //FIXME: const was removed 
    CAC(const EulerTour& tour, const ReebGraph& g, int k);

    void solve();

//  void solve(EulerTour tour, int k);

//  std::vector<double> solve(int k);
//  void printKTours();

private:
    //private functions
    double computeSMax(EulerTour tour);
    double pathCost(EulerTour tour);

private:

    double m_optimalCost;
};

#endif // CAC_H
