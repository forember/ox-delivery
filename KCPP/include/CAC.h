#ifndef CAC_H
#define CAC_H 

#include <vector>
#include <list>

#include "KChinesePostmen.h"

class CAC : public KChinesePostmen
{

	public:
		CAC(const EulerTour& tour, RegionData data, ReebGraph g, int k); //NOTE: maybe we do not need tour in CAC

		void solve(bool mod = true);

	private:
		std::list<ReebEdge> getAdjacentList(Vertex v);

		/* this function converts cluster to the reeb graph and runs cpp over it again*/
		EulerTour findEulerTour(kcpp::EulerTour cluster, int i);
		
		double computeSMax(EulerTour tour);


	private:

		EulerTour m_optimalPath; // R = (V1, Ei1, Vi2, ..., Vim, Eim, V1)
		double m_optimalCost;
		std::list<Edge> m_sortedGraphEdges; // Graph edges sorted by X and then by Y axis of corresponding cell
		RegionData m_data;
};

#endif // CAC_H
