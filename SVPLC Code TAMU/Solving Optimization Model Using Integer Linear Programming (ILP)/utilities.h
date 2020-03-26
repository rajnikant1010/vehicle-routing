/*
 * utilities.hpp
 *
 *  Created on: Oct 11, 2015
 *      Author: kaarthik
 */

#ifndef UTILITIES_H_
#define UTILITIES_H_

/* Constant Definitions */

#define XTOL 1.0E-6      	// tolerance for the variables
#define CONS_TYPES 2	    // number of different types of constraints

/* Other useful Defines */
#define SMALLEST(a,b) ((a<b) ? (a) : (b))
#define BIGGEST(a,b) ((a<b) ? (b) : (a))

/* Include files */

#include <ilcplex/ilocplex.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <fstream>
#include <cstdlib>
#include <string.h>
#include <math.h>
#include <climits>
#include <ctime>
#include <string>
#include <utility>
#include <algorithm>
#include <numeric>
#include <iterator>
#include <list>
#include <map>
#include <set>
#include <cassert>
#include <stdlib.h>
#include <lemon/list_graph.h>
#include <lemon/connectivity.h>
#include <boost/config.hpp>
#include <boost/version.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/static_assert.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/bimap.hpp>



ILOSTLBEGIN

/* Type Definitions */

/* typedef inherited from BOOST graphs */

typedef std::pair<int, int> edge;
typedef float capacity; // x - 0, y - 1
typedef std::map<edge, capacity> EdgeMap;


typedef struct edgeST {

	int u;
	int v;
	int cost;
	std::set<int> Se;
	edgeST(int u, int v, int cost, std::set<int> Se) : u(u), v(v), cost(cost), Se(Se) {}

}edgeST;

typedef struct coordST {

	float xcoord;
	float ycoord;
	coordST(float xcoord, float ycoord) : xcoord(xcoord), ycoord(ycoord) {}

}coordST;

typedef struct dataST {

	char instancename[50];
	char instancepath[100];
	int numdepots; //For MVPLC
	int numtargets;
	int numsensors;
	std::vector<coordST> depotcoords; //For MVPLC
	std::vector<coordST> targetcoords;
	std::vector<coordST> sensorcoords;
	float threshold;
	float delta;
	std::vector<edgeST> edges;
	int sensorstocover;
	int sensorlimit;

}dataST;

typedef struct numconstraintST {

	std::vector<int> cutcount;
    std::vector<float> separationtime;
	// SEC - 1
	// 4Path - 2
	// Path - 3
	// 2Mat - 4
	// LM-Edge Covering - 0, notice that this one is hard to count time...

}numconstraintST;


/* extern functions */

/* Implementation in utilities.cpp file */

extern void ReadData (dataST *p_dataST);

extern std::set<size_t> GetDelta (dataST *p_dataST, std::set<size_t> &S);

extern std::set<size_t> GetDeltaNew(dataST *p_dataST, std::set<size_t> &S, char inorout);

extern std::set<size_t> GetGamma (dataST *p_dataST, std::set<size_t> &S);

extern void PopulateVariables (dataST *p_dataST,
							   	   IloNumVarArray &x,
							   	   IloNumVarArray &y,
									//IloNumVarArray &w,
							   	   IloNumArray &c,
							   	   int &vartype,
							   	   IloEnv &env);

extern void PopulateConstraints (dataST *p_dataST,
									numconstraintST *p_numconstraintST,
									IloNumVarArray &x,
									IloNumVarArray &y,
									//IloNumVarArray &w,
									IloEnv &env,
									IloModel &model);

/* Implementation in separation.cpp */

/* Implementation in separation.cpp file */

extern void UserSeparation (dataST *p_dataST,
								numconstraintST *p_numconstraintST,
								IloEnv &env,
								IloRangeArray &newcuts,
								IloNumVarArray &xvars,
								IloNumVarArray &yvars,
								IloNumArray &xvals,
								IloNumArray &yvals);

extern void LazySeparation (dataST *p_dataST,
								numconstraintST *p_numconstraintST,
								IloEnv &env,
								IloRangeArray &newcuts,
								IloNumVarArray &xvars,
								IloNumVarArray &yvars,
								IloNumArray &xvals,
								IloNumArray &yvals);

extern void AddSubTourComponentCuts (dataST *p_dataST,
										numconstraintST *p_numconstraintST,
										IloEnv &env,
										IloRangeArray &newcuts,
										IloNumVarArray &xvars,
										IloNumVarArray &yvars,
										IloNumArray &xvals,
										IloNumArray &yvals,
										std::vector<std::set<size_t> > &components);

extern void AddSECLPCuts(dataST *p_dataST,
	numconstraintST *p_numconstraintST,
	IloEnv &env,
	IloRangeArray &newcuts,
	IloNumVarArray &xvars,
	IloNumVarArray &yvars,
	IloNumArray &xvals,
	IloNumArray &yvals,
	std::vector<std::set<size_t> > &components,
	lemon::ListDigraph &supportgraph,
	lemon::ListDigraph::ArcMap<float> &capacity);

extern void AddSubTourDepotLPCuts(dataST *p_dataST,
	numconstraintST *p_numconstraintST,
	IloEnv &env,
	vector<set<size_t> > &componentswithdepot,
	IloNumVarArray &xvars,
	IloNumVarArray &yvars,
	IloNumArray &xvals,
	IloNumArray &yvals,
	IloRangeArray &newcuts,
	lemon::ListGraph &supportgraph);

/*
extern void AddSECComponentCuts2(dataST *p_dataST,
	numconstraintST *p_numconstraintST,
	IloEnv &env,
	IloRangeArray &newcuts,
	IloNumVarArray &x,
	IloNumVarArray &y,
	IloNumArray &xvals,
	IloNumArray &yvals);
	*/

extern void SubTourComponentCutHelper(dataST *p_dataST,
	numconstraintST *p_numconstraintST,
	std::vector<std::set<int> > &componentssansdepot,
	IloNumVarArray &x,
	IloNumVarArray &y,
	IloNumArray &xvals,
	IloNumArray &yvals,
	IloRangeArray &newcuts,
	IloEnv &env);

extern void Add2PathCuts(dataST *p_dataST,
	numconstraintST *p_numconstraintST,
	IloEnv &env,
	vector<set<size_t> > &componentswithdepot,
	IloNumVarArray &xvars,
	IloNumVarArray &yvars,
	IloNumArray &xvals,
	IloNumArray &yvals,
	IloRangeArray &newcuts,
	lemon::ListGraph &supportgraph);

extern void AddPathCuts(dataST *p_dataST,
	numconstraintST *p_numconstraintST,
	IloEnv &env,
	IloRangeArray &newcuts,
	IloNumVarArray &xvars,
	IloNumVarArray &yvars,
	IloNumArray &xvals,
	IloNumArray &yvals,
	std::vector<std::set<size_t> > &components,
	lemon::ListGraph &supportgraph);

extern void AddPathCutsHelper(dataST *p_dataST,
	numconstraintST *p_numconstraintST,
	IloEnv &env,
	std::vector<std::set<size_t> > &componentswithdepot,
	IloNumVarArray &x,
	IloNumVarArray &y,
	IloNumArray &xvals,
	IloNumArray &yvals,
	IloRangeArray &newcuts,
	lemon::ListGraph &supportgraph);

extern void AddPathCutsExplicit(dataST *p_dataST,
	numconstraintST *p_numconstraintST,
	std::vector<std::set<size_t> > &S,
	std::vector<std::set<size_t> > &Sdash,
	std::vector<std::set<size_t> > &Ddash,
	std::vector<std::set<size_t> > &DminusDdash,
	//std::vector<size_t> &i,
	std::vector<size_t> &j,
	std::vector<size_t> &k,
	std::vector<float> &firstpart,
	IloNumVarArray &x,
	IloNumVarArray &y,
	IloNumArray &xvals,
	IloNumArray &yvals,
	IloRangeArray &newcuts,
	IloEnv &env); 


extern void Add2MatCutsGPnew(dataST *p_dataST,
	numconstraintST *p_numconstraintST,
	IloEnv &env,
	IloNumVarArray &x,
	IloNumVarArray &y,
	IloNumArray &xvals,
	IloNumArray &yvals,
	IloRangeArray &newcuts);

extern void Add2MatCutsGPH(dataST *p_dataST,
	numconstraintST *p_numconstraintST,
	IloEnv &env,
	IloNumVarArray &x,
	IloNumVarArray &y,
	IloNumArray &xvals,
	IloNumArray &yvals,
	IloRangeArray &newcuts);

extern void Add2MatCutsGH(dataST *p_dataST,
	numconstraintST *p_numconstraintST,
	IloEnv &env,
	IloNumVarArray &x,
	IloNumVarArray &y,
	IloNumArray &xvals,
	IloNumArray &yvals,
	IloRangeArray &newcuts);

extern void AddTCombCuts(dataST *p_dataST,
	numconstraintST *p_numconstraintST,
	IloEnv &env,
	IloNumVarArray &xvars,
	IloNumVarArray &yvars,
	IloNumArray &xvals,
	IloNumArray &yvals,
	std::vector<std::set<size_t> > &components,
	IloRangeArray &newcuts);

extern float getxdelta(std::set<size_t> &S,
	int vertex,
	IloNumArray &xvals,
	dataST *p_dataST);

extern std::vector<vector<int> > GetDecreasingOrderVertexList(vector<size_t> &depotlist,
	set<size_t> &S,
	IloNumArray &xvals,
	dataST *p_dataST);

extern void AddArcsToSupportGraph(dataST *p_dataST,
	IloNumArray &xvals,
	lemon::ListDigraph &supportgraph);

extern void AddEdgesToSupportGraph(dataST *p_dataST,
	IloNumArray &xvals,
	lemon::ListGraph &supportgraph);

/*
extern void SplitComponentBasedOnDepots2(dataST *p_dataST,
	std::vector<std::set<int> > &componentpartitions,
	std::vector<std::set<int> > &componentswithdepot,
	std::vector<std::set<int> > &componentsansdepot,
	std::vector<int> &component);
	*/

extern void SplitComponentBasedOnDepots(dataST *p_dataST,
	//std::vector<std::set<int> > &componentpartitions,
	std::vector<std::set<size_t> > &componentswithdepot,
	std::vector<std::set<size_t> > &componentsansdepot,
	std::vector<std::set<size_t> > &component);

extern void RemoveSingletonsFromVector(std::vector<std::set<size_t> > &in,
	std::set<int> &cardinality);

extern void RemoveSingleDepotComponents(std::vector<std::set<size_t> > &componentswithdepot, dataST *p_dataST);


/* Inline Functions */

template <class T>
inline std::string ToString (const T& t) {
	std::stringstream ss;
	ss << t;
	return ss.str();
};

inline int ComputeDistance (float x1, float x2, float y1, float y2) {
	float distance = sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
	//The following line is a special, rescaled version, which is only for experimental testing field use.
	//Do not use in general!!!
	//float distance = 15*sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
	//int output = ceil(distance);
	int output = static_cast<int>(distance + 0.5); //New rounding criterion coherent with TSPLIB
	return output;
};

inline int EdgeFromVertices(dataST *p_dataST, int u, int v) {
	std::vector<edgeST>::iterator edgeit;
	int edgeid = 0;
	for (edgeit=p_dataST->edges.begin(), edgeid=0; edgeit!=p_dataST->edges.end(); ++edgeit, ++edgeid) {
		if (u == (*edgeit).u && v == (*edgeit).v)
			return edgeid;
// Modified for ATSP!!!!!!
		//if (v == (*edgeit).u && u == (*edgeit).v)
			//return edgeid;
	}

    return 0;
}

inline size_t SumToN(int n) { return (n*(n + 1)) / 2; }


inline size_t Sub2Ind(int i, int j, dataST *p_dataST) {
	size_t small = SMALLEST(i, j);
	size_t big = BIGGEST(i, j);
	size_t v = p_dataST->numdepots + p_dataST->numtargets;
	size_t t = p_dataST->numtargets;
	size_t d = p_dataST->numdepots;
	if (small == big) {
		std::cerr << "Program exited - self edge call" << std::endl;
		exit(1);
	}
	if (small < d) return ((small*t) + (big - d));
	else {
		size_t depotedgeoffset = d*t;
		size_t position = (big - small - 1);
		size_t targetedgeoffset = SumToN(t - 1) - SumToN(t - 1 - (small - d));
		return (depotedgeoffset + targetedgeoffset + position);
	}
}

inline void GetDdash(IloNumArray &xvals, std::set<size_t> &Ddash, std::set<size_t> &D, const int &j, const int &k, dataST *p_dataST) {
	for (std::set<size_t>::iterator sit = D.begin(); sit != D.end(); ++sit) {
		float xjd = xvals[Sub2Ind(j, *sit, p_dataST)];
		float xkd = xvals[Sub2Ind(k, *sit, p_dataST)];
		if (xjd > xkd)
			Ddash.insert(*sit);
	}
	return;
};


#endif
