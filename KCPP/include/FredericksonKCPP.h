#ifndef FREDERICKSONKCPP_H
#define FREDERICKSONKCPP_H

#include <vector>

#include "KChinesePostmen.h"

class FredericksonKCPP : public KChinesePostmen
{

public:
    //FIXME: const was removed 
    FredericksonKCPP(const EulerTour& tour, const ReebGraph& g, int k);

		// mod flag indicates if we want to run solve with modified version of cluster computing 
    void solve(bool mod = false);

//    void solve(EulerTour tour, int k);

//    std::vector<double> solve(int k);
//    void printKTours();

private:
    //private functions
    double computeSMax(EulerTour tour);

private:

    double m_optimalCost; // L = c(R)
    EulerTour m_optimalPath; // R = (V1, Ei1, Vi2, ..., Vim, Eim, V1)

    double m_smax; // Smax

    // The last vertices, i.g.v_l_j
    std::vector<Vertex> m_lastVertices;

    std::vector<double> m_results;
};

#endif // FREDERICKSONKCPP_H
