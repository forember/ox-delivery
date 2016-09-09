#ifndef CAC_H
#define CAC_H 

#include <vector>
#include <list>

#include "KChinesePostmen.h"

class CAC : public KChinesePostmen
{

	public:
		CAC(const EulerTour& tour, ReebGraph g, int k); //NOTE: maybe we do not need tour in CAC

		void solve();

	private:
		std::list<ReebEdge> getAdjacentList(Vertex v);


	private:

		EulerTour m_optimalPath; // R = (V1, Ei1, Vi2, ..., Vim, Eim, V1)
		double m_optimalCost;
		std::list<ReebEdge> m_sortedGraphEdges; // Graph edges sorted by X and then by Y axis of corresponding cell
};

#endif // CAC_H
