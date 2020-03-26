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
//#include <lemon/list_graph.h>
//#include <lemon/connectivity.h>
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
	std::vector<int> targetseq; //For SVPLC_Heu
	std::vector<coordST> sensorcoordstrans; //For SVPLC_Heu
	std::vector<float> sensorcoordsts; //For SVPLC_Heu
	std::vector<float> sensorcoordstd; //For SVPLC_Heu
	std::vector<std::set<int>> LMonedge; //For SVPLC_Heu

	float d; //For SVPLC_Heu
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

/*
extern void ReadData (dataST *p_dataST);
extern void CoordTrans(int seq, dataST *p_dataST);
extern void LMPlace(int seq, dataST *p_dataST);
*/
#endif
