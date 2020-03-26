/*
 * separation.cpp
 *
 *  Created on: Aug 22, 2014
 *      Author: kaarthik
 */


#include "utilities.h"
#include <lemon/nagamochi_ibaraki.h> // Kaarthik said not robust enough; hence the following
#include <lemon/hao_orlin.h> // for a standard mincut in SEC-LP
#include <lemon/preflow.h> // for PR and min s-t cut

bool IsLengthLessThanThree(std::set<size_t> a) { return (a.size() <= 2); } // For Grotschel and Holland separation

typedef vector<float>::const_iterator vecfloatiter;
struct ordering {
	bool operator ()(pair<size_t, vecfloatiter> const& a, pair<size_t, vecfloatiter> const& b) {
		return *(a.second) > *(b.second);
	}
};

template <typename T>
vector<T> sort_from_ref(vector<T> const& in, vector<pair<size_t, vecfloatiter> > const& reference) {
	vector<T> ret(in.size());
	size_t const size = in.size();
	for (size_t i = 0; i<size; ++i)
		ret[i] = in[reference[i].first];
	return ret;
}


///*
void UserSeparation (dataST *p_dataST,
					numconstraintST *p_numconstraintST,
					IloEnv &env,
					IloRangeArray &newcuts,
					IloNumVarArray &xvars,
					IloNumVarArray &yvars,
					IloNumArray &xvals,
					IloNumArray &yvals) {

	lemon::ListDigraph supportgraph;
	AddArcsToSupportGraph(p_dataST, xvals, supportgraph);

	lemon::ListDigraph::ArcMap<float> capacity(supportgraph);
	for (lemon::ListDigraph::ArcIt e(supportgraph); e!=lemon::INVALID; ++e) {
		int index = EdgeFromVertices(p_dataST, supportgraph.id(supportgraph.source(e)), supportgraph.id(supportgraph.target(e)));
		capacity[e] = xvals[index];
	}

	lemon::ListDigraph::NodeMap<int> componentmap(supportgraph);
	int noofcomponents = stronglyConnectedComponents (supportgraph, componentmap);

	std::vector<std::set<size_t> > components(noofcomponents);
	for (lemon::ListDigraph::NodeIt n(supportgraph); n!=lemon::INVALID; ++n)
		components[componentmap[n]].insert(supportgraph.id(n));

	std::clock_t start;
	std::clock_t stop;

	if (noofcomponents > 1){
		//start = std::clock();
		AddSubTourComponentCuts(p_dataST, p_numconstraintST, env, newcuts, xvars, yvars, xvals, yvals, components);
		//p_numconstraintST->separationtime[1] += (std::clock() - start) / (double) CLOCKS_PER_SEC;
	}
	/*
	if (noofcomponents == 1) { //Important: This may or may not happen in the optimal solution for my MVPLC problem, depending on instance.
	//So, it might be favorable if I always call AddSubTourLPCuts...
		return;
		cout << "In" << endl;
		lemon::NagamochiIbaraki<lemon::ListGraph, lemon::ListGraph::EdgeMap<float> > mc(supportgraph, capacity);
		mc.run();
		cout << "Out" << endl;

		lemon::ListGraph::NodeMap<bool> cutmap(supportgraph);
		float cutvalue = mc.minCutMap(cutmap);
		if (cutvalue < 2-XTOL)
			AddSubTourLPCuts(p_dataST, p_numconstraintST, env, newcuts, xvars, yvars, xvals, yvals, supportgraph, cutmap);

	}
	*/

	if (noofcomponents == 1)
	AddSECLPCuts(p_dataST, p_numconstraintST, env, newcuts, xvars, yvars, xvals, yvals, components, supportgraph, capacity);
	//I almost forgot to call this vital, crucial function!!! Only checked it out on Oct 25 at dusk...
	//This is almost a horrible crime...
	//Timer called in interior

	//start = std::clock();
	//AddPathCuts(p_dataST, p_numconstraintST, env, newcuts, xvars, yvars, xvals, yvals, components, supportgraph);
	//p_numconstraintST->separationtime[3] += (std::clock() - start) / (double) CLOCKS_PER_SEC;

	start = std::clock();
	//Add2MatCutsGPnew(p_dataST, p_numconstraintST, env, xvars, yvars, xvals, yvals, newcuts);
	//Add2MatCutsGH (p_dataST, p_numconstraintST, env, xvars, yvars, xvals, yvals, newcuts);
	//AddTCombCuts (p_dataST, p_numconstraintST, env, xvars, yvars, xvals, yvals, components, newcuts);
	stop = std::clock();
	//p_numconstraintST->separationtime[4] += (stop - start);
	// / (double) CLOCKS_PER_SEC;

	return;
};
//*/

void LazySeparation (dataST *p_dataST,
					numconstraintST *p_numconstraintST,
					IloEnv &env,
					IloRangeArray &newcuts,
					IloNumVarArray &xvars,
					IloNumVarArray &yvars,
					IloNumArray &xvals,
					IloNumArray &yvals) {

	lemon::ListDigraph supportgraph;
	AddArcsToSupportGraph(p_dataST, xvals, supportgraph);

	lemon::ListDigraph::ArcMap<float> capacity(supportgraph);
	for (lemon::ListDigraph::ArcIt e(supportgraph); e != lemon::INVALID; ++e) {
		int index = EdgeFromVertices(p_dataST, supportgraph.id(supportgraph.source(e)), supportgraph.id(supportgraph.target(e)));
		capacity[e] = xvals[index];
	}

	lemon::ListDigraph::NodeMap<int> componentmap(supportgraph);

	int noofcomponents = stronglyConnectedComponents (supportgraph, componentmap);

	if (noofcomponents == 1) return; //Don't do this for my MVPP problem! Still need path cuts!

	std::vector<std::set<size_t> > components(noofcomponents);
	for (lemon::ListDigraph::NodeIt n(supportgraph); n!=lemon::INVALID; ++n)
		components[componentmap[n]].insert(supportgraph.id(n));

	/*
	vector<set<size_t> > componentswithdepot;
	vector<set<size_t> > componentssansdepot;
	SplitComponentBasedOnDepots(p_dataST, componentswithdepot, componentssansdepot, components);
	//*/

	if (noofcomponents > 1)
		AddSubTourComponentCuts(p_dataST, p_numconstraintST, env, newcuts, xvars, yvars, xvals, yvals, components);
		//AddSECComponentCuts2(p_dataST, p_numconstraintST, env, newcuts, xvars, yvars, xvals, yvals);

	//AddPathCuts(p_dataST, p_numconstraintST, env, newcuts, xvars, yvars, xvals, yvals, components, supportgraph);

	return;
};


//Remember to define this new function in "utilities.h" accordingly!!!!!! 

void AddArcsToSupportGraph(dataST *p_dataST,
	IloNumArray &xvals,
	lemon::ListDigraph &supportgraph) {
	for (int i = 0; i<p_dataST->numtargets; ++i) supportgraph.addNode();
	// For MVPP, please remember to add "p_dataST->numdepots + " !!!!!!

	for (size_t i = 0; i<xvals.getSize(); ++i)
		if (xvals[i] > XTOL)
			supportgraph.addArc(supportgraph.nodeFromId(p_dataST->edges[i].u),
				supportgraph.nodeFromId(p_dataST->edges[i].v));
}


// The following implementation should be sufficient for ATSP as well, i.e. equivalent to Cutset form
void AddSubTourComponentCuts (dataST *p_dataST,
								numconstraintST *p_numconstraintST,
								IloEnv &env,
								IloRangeArray &newcuts,
								IloNumVarArray &xvars,
								IloNumVarArray &yvars,
								IloNumArray &xvals,
								IloNumArray &yvals,
								std::vector<std::set<size_t> > &components) {

	for (size_t i = 0; i < components.size(); ++i) {
		std::set<size_t> S = components[i];
		std::set<size_t>::iterator sit;
		// Uncomment the following lines for MVPP!!!!!!
		/*
		bool sansdepot = true;
		for (int d = 0; d < p_dataST->numdepots; ++d) {
			if (S.find(d) != S.end())
				sansdepot = false;
		}
		if (sansdepot)	 //find this component S sans depot, add a new cut
			{
		*/
				std::set<size_t> edgelist = GetGamma(p_dataST, S);
				IloExpr expr(env);
				for (sit = edgelist.begin(); sit != edgelist.end(); sit++)
					expr += xvars[*sit];

				p_numconstraintST->cutcount[1]++;
				char name[200];
				strcpy(name, ("SEC_" + ToString(p_numconstraintST->cutcount[1])).c_str());
				IloRange cut(env, 0.0, expr, S.size() - 1, name);
				newcuts.add(cut);
				//cout << "Adding a SubTour Component Cut" << endl;
				S.clear(); expr.clear();
			//}
	}

	return;
}

void AddSECLPCuts(dataST *p_dataST,
	numconstraintST *p_numconstraintST,
	IloEnv &env,
	IloRangeArray &newcuts,
	IloNumVarArray &xvars,
	IloNumVarArray &yvars,
	IloNumArray &xvals,
	IloNumArray &yvals,
	std::vector<std::set<size_t> > &components,
	lemon::ListDigraph &supportgraph,
	lemon::ListDigraph::ArcMap<float> &capacity) {

	// This is a highly simplified version purely for the sake of SVPP......
	std::vector<size_t> vertexmap;
	std::set<size_t>::iterator sit;
	for (sit = components[0].begin(); sit != components[0].end(); ++sit)
		vertexmap.push_back(*sit);


	lemon::HaoOrlin<lemon::ListDigraph, lemon::ListDigraph::ArcMap<float> > haoorlin(supportgraph, capacity);

	haoorlin.run();

	lemon::ListDigraph::NodeMap<bool> cutmap(supportgraph);
	float cutvalue = haoorlin.minCutMap(cutmap);

	std::set<size_t> A, B, S;
	for (lemon::ListDigraph::NodeIt n(supportgraph); n != lemon::INVALID; ++n) {
		if (cutmap[n] == true)
			A.insert(vertexmap[supportgraph.id(n)]);
		else
			B.insert(vertexmap[supportgraph.id(n)]);
	}

	assert(A.find(p_dataST->numtargets) != A.end() || B.find(p_dataST->numtargets) != B.end());
	if (A.find(p_dataST->numtargets) == A.end())
		S.insert(A.begin(), A.end());
	else
		S.insert(B.begin(), B.end());

	std::set<size_t> edgelist = GetDeltaNew(p_dataST, S, 'o');

	IloExpr xdeltapS(env); // , yi(env);
	IloNum xdeltapSvalue = 0; // , yivalue = 0;
	for (sit = edgelist.begin(); sit != edgelist.end(); sit++) {
		xdeltapS += xvars[*sit];
		xdeltapSvalue += xvals[*sit];
	}

	// This latest modification: Noticed by Bingyu 08.31, tested on 09.01
	//for (sit = S.begin(); sit != S.end(); ++sit) {
		if (xdeltapSvalue + XTOL < 1) {
			p_numconstraintST->cutcount[0]++;
			char name[200];
			strcpy(name, ("SEC_" + ToString(p_numconstraintST->cutcount[0])).c_str());
			IloRange cut(env, 1.0, xdeltapS, INFINITY, name);
			newcuts.add(cut);
			//cout << "Adding a SubTour LP Cut" << endl;
		}
	//}

	S.clear(); xdeltapS.clear();

return;

}