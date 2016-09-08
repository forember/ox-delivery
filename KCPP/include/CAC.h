#ifndef CAC_H
#define CAC_H 

#include <vector>
#include <list>

#include "KChinesePostmen.h"

class CAC : public KChinesePostmen
{

public:
    CAC(const EulerTour& tour, const ReebGraph& g, int k);

    void solve();

//  void solve(EulerTour tour, int k);

//  std::vector<double> solve(int k);
//  void printKTours();

private:
		std::list<ReebEdge> CAC::getAdjacentList(Vertex v)

		/* helper predicates for list sort algorithm */
		/* Comparison by X axis*/
		bool compareByXCoords (ReebEdge e1, ReebEdge e2) { 
			unsigned int numPoints;
			double midX1 = 0.0;
			double midX2 = 0.0;
			if (numPoints == 0)
			{
				continue;
			}
			else if (numPoints == 1) 
			{
				midX1 = e1->topBoundary[0].xcoord();
				midX2 = e2->topBoundary[0].xcoord();
			}
			else
			{
				midX1 = e1->topBoundary[numPoints/2-1].xcoord();
				midX2 = e2->topBoundary[numPoints/2-1].xcoord();
			}
			return midX1 < midX2;
		}

		/* Comparison by Y axis*/
		bool compareByXCoords (ReebEdge e1, ReebEdge e2) { 
			unsigned int numPoints;
			double midY1 = 0.0;
			double midY2 = 0.0;
			if (numPoints == 0)
			{
				continue;
			}
			else if (numPoints == 1) 
			{
				midY1 = (e1->topBoundary[0].ycoord()
						+ e1->bottomBoundary[0].ycoord()) / 2;
				midY2 = (e2->topBoundary[0].ycoord()
						+ e2->bottomBoundary[0].ycoord()) / 2;
			}
			else
			{
				midY1 = (e1->topBoundary[numPoints/2-1].ycoord()
						+ e1->bottomBoundary[numPoints/2-1].ycoord()) / 2;
				midY2 = (e2->topBoundary[numPoints/2-1].ycoord()
						+ e2->bottomBoundary[numPoints/2-1].ycoord()) / 2;
			}
			return midX1 < midX2;
		}

private:

		double m_optimalCost;
		std::list<ReebEdge> m_sortedGraphEdges; // Graph edges sorted by X and then by Y axis of corresponding cell
};

#endif // CAC_H
