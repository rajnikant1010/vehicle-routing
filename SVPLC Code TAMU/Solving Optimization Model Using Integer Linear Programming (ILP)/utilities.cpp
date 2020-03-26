/*
 * utilities.cpp
 *
 *  Created on: Oct 11, 2015
 *      Author: kaarthik
 */


#include "utilities.h"

/* Function Definitions in the same order declared */

/* void ReadData */

void ReadData (dataST *p_dataST) {

	char filename[200];
	strcpy(filename, p_dataST->instancepath);
	strcat(filename, p_dataST->instancename);

	ifstream infile;
	string str(filename);
	infile.open(str.c_str());
	std::cout << filename << std::endl;
	if (!infile) {
		std::cerr << "Error: Instance file could not be opened" << std::endl;
		exit(1);
	}

	std::cout << ">> Instance: " << filename << std::endl;
	infile >> p_dataST->numtargets >> p_dataST->numsensors >> p_dataST->delta
	 >> p_dataST->threshold;
	//Do not use ">> p_dataST->numdepots" for SVPP; use for MVPP

	/*
	for (int d = 0; d<p_dataST->numdepots; ++d) {
		float x, y;
		infile >> x >> y;
		p_dataST->depotcoords.push_back(coordST(x, y));
	}
	*/

	for (int t=0; t<p_dataST->numtargets; ++t) {
		float x,y;
		infile >> x >> y;
		p_dataST->targetcoords.push_back(coordST(x,y));
	}

	for (int s=0; s<p_dataST->numsensors; ++s) {
		float x,y;
		infile >> x >> y;
		p_dataST->sensorcoords.push_back(coordST(x,y));
	}

	std::vector<std::set<int> > Si(p_dataST->numtargets);

	for (int t=0; t<p_dataST->numtargets; ++t) {
		for (int s=0; s<p_dataST->numsensors; ++s) {
			int D = ComputeDistance(p_dataST->targetcoords[t].xcoord, p_dataST->sensorcoords[s].xcoord,
									p_dataST->targetcoords[t].ycoord, p_dataST->sensorcoords[s].ycoord);
			if (D < p_dataST->threshold)
				Si[t].insert(s);
		}
	}

	/*
	std::vector<std::set<int> > Sd(p_dataST->numdepots);

	for (int t = 0; t<p_dataST->numdepots; ++t) {
		for (int s = 0; s<p_dataST->numsensors; ++s) {
			int D = ComputeDistance(p_dataST->depotcoords[t].xcoord, p_dataST->sensorcoords[s].xcoord,
				p_dataST->depotcoords[t].ycoord, p_dataST->sensorcoords[s].ycoord);
			if (D < p_dataST->threshold)
				Sd[t].insert(s);
		}
	}

	for (int i = 0; i<p_dataST->numdepots; ++i) {
		for (int j = 0; j<p_dataST->numtargets; ++j) {
			int cost = ComputeDistance(p_dataST->depotcoords[i].xcoord, p_dataST->targetcoords[j].xcoord,
				p_dataST->depotcoords[i].ycoord, p_dataST->targetcoords[j].ycoord);
			set<int> Se;
			set_intersection(Sd[i].begin(), Sd[i].end(), Si[j].begin(), Si[j].end(),
				std::inserter(Se, Se.begin()));
			//if (Se.size() >= p_dataST->sensorstocover)
				p_dataST->edges.push_back(edgeST(i, j + p_dataST->numdepots, cost, Se));
		}
	}
	*/

	for (int i=0; i<p_dataST->numtargets; ++i) {
		for (int j=0; j<p_dataST->numtargets; ++j) {
//			for (int j = i + 1; j<p_dataST->numtargets; ++j) {
			if (i == j) continue; // This is very important for the asymmetric version!!!
			int cost = ComputeDistance(p_dataST->targetcoords[i].xcoord, p_dataST->targetcoords[j].xcoord,
										p_dataST->targetcoords[i].ycoord, p_dataST->targetcoords[j].ycoord);
			set<int> Se;
			set_intersection(Si[i].begin(), Si[i].end(), Si[j].begin(), Si[j].end(),
								std::inserter(Se, Se.begin()));
			//if (Se.size() >= p_dataST->sensorstocover)
				//p_dataST->edges.push_back(edgeST(i + p_dataST->numdepots,j + p_dataST->numdepots,cost,Se));
				p_dataST->edges.push_back(edgeST(i, j, cost, Se));
		}
	}
	// I think doing this is still sufficient; however, another important thing is that, modify the "EdgeFromVertices" function
	// in "utilities.h" accordingly!!!!!!
	
	return;

};

std::set<size_t> GetDelta (dataST *p_dataST, std::set<size_t> &S) {

	std::vector<edgeST>::iterator edgeit;
	std::set<size_t> edgelist;

	size_t edgeid;
	for (edgeit=p_dataST->edges.begin(),edgeid=0; edgeit!=p_dataST->edges.end(); ++edgeit, ++edgeid) {
		size_t u = (*edgeit).u;
		size_t v = (*edgeit).v;
		if (S.find(u) != S.end() || S.find(v) != S.end()) {
			edgelist.insert(edgeid); continue;
		}
	}

	return edgelist;
};

//Remember to define this new function in "utilities.h" accordingly!!!!!! 
std::set<size_t> GetDeltaNew (dataST *p_dataST,
	std::set<size_t> &S,
	//size_t vehicle,
	char inorout) {

	std::vector<edgeST>::iterator edgeit;
	std::set<size_t> edgelist;
	//size_t v = vehicle;

	size_t edgeid;
	for (edgeit = p_dataST->edges.begin(), edgeid = 0; edgeit != p_dataST->edges.end(); ++edgeit, ++edgeid) {
		size_t from = (*edgeit).u;
		size_t to = (*edgeit).v;
		if (S.find(from) != S.end() && S.find(to) == S.end() && inorout == 'o') {
			edgelist.insert(edgeid); continue;
		}
		if (S.find(to) != S.end() && S.find(from) == S.end() && inorout == 'i') {
			edgelist.insert(edgeid); continue;
		}
		if (inorout == 'b') {
			if (S.find(from) != S.end() && S.find(to) == S.end())
				edgelist.insert(edgeid);
			if (S.find(to) != S.end() && S.find(from) == S.end())
				edgelist.insert(edgeid);
		}
	}

	return edgelist;
};



std::set<size_t> GetGamma (dataST *p_dataST, std::set<size_t> &S) {

	std::vector<edgeST>::iterator edgeit;
	std::set<size_t> edgelist;

	size_t edgeid;
	for (edgeit=p_dataST->edges.begin(),edgeid=0; edgeit!=p_dataST->edges.end(); ++edgeit, ++edgeid) {
		size_t u = (*edgeit).u;
		size_t v = (*edgeit).v;
		if (S.find(u) != S.end() && S.find(v) != S.end())
			edgelist.insert(edgeid);
	}

	return edgelist;

};

void PopulateVariables (dataST *p_dataST,
						IloNumVarArray &x,
						IloNumVarArray &y,
						IloNumArray &c,
						int &vartype,
						IloEnv &env) {


	char varname[100];

	int DTcount = 0; //No depot in SVPP
/*
	int DTcount = p_dataST->numdepots*p_dataST->numtargets;

	for (int i=0; i<DTcount; ++i) {
		strcpy(varname, "x_");
		strcat(varname, (ToString(p_dataST->edges[i].u)).c_str());
		strcat(varname, "_");
		strcat(varname, (ToString(p_dataST->edges[i].v)).c_str());

		c.add(p_dataST->edges[i].cost);
		if (vartype == 0) { IloNumVar var(env, 0.0, 2.0, ILOFLOAT, varname); x.add(var); }
		else { IloNumVar var(env, 0.0, 2.0, ILOINT, varname); x.add(var); }

	}
	*/

	for (int i = DTcount; i<p_dataST->edges.size(); ++i) {
		strcpy(varname, "x_");
		strcat(varname, (ToString(p_dataST->edges[i].u)).c_str());
		strcat(varname, "_");
		strcat(varname, (ToString(p_dataST->edges[i].v)).c_str());

		c.add(p_dataST->edges[i].cost);
		if (vartype == 0) { IloNumVar var(env, 0.0, 1.0, ILOFLOAT, varname); x.add(var); }
		else { IloNumVar var(env, 0.0, 1.0, ILOINT, varname); x.add(var); }

	}

	for (int i=0; i<p_dataST->numsensors; ++i) {
		strcpy(varname, "y_");
		strcat(varname, (ToString(i)).c_str());

		if (vartype == 0) { IloNumVar var(env, 0.0, 1.0, ILOFLOAT, varname); y.add(var); }
		else { IloNumVar var(env, 0.0, 1.0, ILOINT, varname); y.add(var); }
	}

	return;

};

void PopulateConstraints (dataST *p_dataST,
						numconstraintST *p_numconstraintST,
						IloNumVarArray &x,
						IloNumVarArray &y,
						IloEnv &env,
						IloModel &model) {

	char consname[100];

	std::set<size_t> S;

	// Degree Constraints for the targets
	/*
	for (size_t i= p_dataST->numdepots; i<p_dataST->numdepots + p_dataST->numtargets; ++i) {

		IloExpr expr(env);
		S.clear(); S.insert(i);
		std::set<size_t> edgelist = GetDelta(p_dataST, S);

		std::set<size_t>::iterator sit;
		for (sit=edgelist.begin(); sit!=edgelist.end(); ++sit)
			expr += x[*sit];

		strcpy(consname, ("Degree_"+ToString(i)).c_str());
		model.add(expr == 2).setName(consname);

		expr.clear();

	}
	*/

	for (size_t i = 0; i < p_dataST->numtargets; ++i) {

		IloExpr exprin(env), exprout(env);
		S.clear(); S.insert(i);
		std::set<size_t> edgelistin = GetDeltaNew(p_dataST, S, 'i');
		std::set<size_t> edgelistout = GetDeltaNew(p_dataST, S, 'o');

		std::set<size_t>::iterator  sitin, sitout;
		for (sitin = edgelistin.begin(); sitin != edgelistin.end(); sitin++)
			exprin += x[*sitin];
		for (sitout = edgelistout.begin(); sitout != edgelistout.end(); sitout++)
			exprout += x[*sitout];

		strcpy(consname, ("InDegree_" + ToString(i)).c_str());
		model.add(exprin == 1).setName(consname);
		strcpy(consname, ("OutDegree_" + ToString(i)).c_str());
		model.add(exprout == 1).setName(consname);

		exprin.clear(); exprout.clear();

	}


///*
	// Set Cover Constraints

	std::vector<edgeST>::iterator edgesit;
	int edgeid = 0;
	for (edgesit=p_dataST->edges.begin(), edgeid=0; edgesit!=p_dataST->edges.end(); ++edgesit, ++edgeid) {

		//First things first: When calling x-y coordinate data,
		//Take care of the depot vs. target index! Make use of  p_dataST->numdepots
		float x1, y1, x2, y2, hori, vert, distance;

	/*
		if ((*edgesit).u < p_dataST->numdepots) {
			x1 = p_dataST->depotcoords[(*edgesit).u].xcoord;
			y1 = p_dataST->depotcoords[(*edgesit).u].ycoord;
		}
		else {
			x1 = p_dataST->targetcoords[(*edgesit).u - p_dataST->numdepots].xcoord;
			y1 = p_dataST->targetcoords[(*edgesit).u - p_dataST->numdepots].ycoord;
		}

		if ((*edgesit).v < p_dataST->numdepots) {
			x2 = p_dataST->depotcoords[(*edgesit).v].xcoord;
			y2 = p_dataST->depotcoords[(*edgesit).v].ycoord;
		}
		else {
			x2 = p_dataST->targetcoords[(*edgesit).v - p_dataST->numdepots].xcoord;
			y2 = p_dataST->targetcoords[(*edgesit).v - p_dataST->numdepots].ycoord;
		}
	*/

		x1 = p_dataST->targetcoords[(*edgesit).u].xcoord;
		y1 = p_dataST->targetcoords[(*edgesit).u].ycoord;

		x2 = p_dataST->targetcoords[(*edgesit).v].xcoord;
		y2 = p_dataST->targetcoords[(*edgesit).v].ycoord;

		distance = sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1)); //use the exact distance to compute sin and cos values
		hori = (x2-x1)/ distance;
		vert = (y2 - y1) / distance;

		//float delta = 100; //Manual setting according to the grid size and sensing range. Extremely important!!!

		float xstart = x1;
		float ystart = y1;
		float xend, yend;
		int segcount = 1;

		//When a new partition point is necessary:
		while ((x2 - xstart - p_dataST->delta*hori)*(x1 - xstart - p_dataST->delta*hori) < 0) {
			//New partition point
			xend = xstart + p_dataST->delta*hori;
			yend = ystart + p_dataST->delta*vert;

			std::set<int> Sstart, Send, Sseg;
			for (int s = 0; s<p_dataST->numsensors; ++s) {
				//int D = ComputeDistance(xstart, p_dataST->sensorcoords[s].xcoord,
					//ystart, p_dataST->sensorcoords[s].ycoord);
				float D = sqrt((xstart - p_dataST->sensorcoords[s].xcoord)*(xstart - p_dataST->sensorcoords[s].xcoord)
					+ (ystart - p_dataST->sensorcoords[s].ycoord)*(ystart - p_dataST->sensorcoords[s].ycoord));
				if (D < p_dataST->threshold)
					Sstart.insert(s);
			}

			///*
			//New for ATSP
			for (int s = 0; s<p_dataST->numsensors; ++s) {
				//int D = ComputeDistance(xend, p_dataST->sensorcoords[s].xcoord,
					//yend, p_dataST->sensorcoords[s].ycoord);
				float D1 = (p_dataST->sensorcoords[s].xcoord - xend)*(xend - xstart);
				float D2 = (p_dataST->sensorcoords[s].ycoord - yend)*(yend - ystart);

				float D3 = sqrt((xend - p_dataST->sensorcoords[s].xcoord)*(xend - p_dataST->sensorcoords[s].xcoord)
					+ (yend - p_dataST->sensorcoords[s].ycoord)*(yend - p_dataST->sensorcoords[s].ycoord));
				float D4 = sqrt((xstart - xend)*(xstart - xend)
					+ (ystart - yend)*(ystart - yend));
				if (D1+D2 >= 0.707*D3*D4)
				// Extremely Important: Change the Numerical value above when FOV changes!!!!!!
					// Use cosine(FOV)
					Send.insert(s);
			}
			//*/

			/*
			//New for STSP
			for (int s = 0; s<p_dataST->numsensors; ++s) {
				float D = sqrt((xend - p_dataST->sensorcoords[s].xcoord)*(xend - p_dataST->sensorcoords[s].xcoord)
					+ (yend - p_dataST->sensorcoords[s].ycoord)*(yend - p_dataST->sensorcoords[s].ycoord));
				if (D < p_dataST->threshold)
					Send.insert(s);
			}
			*/

			set_intersection(Sstart.begin(), Sstart.end(), Send.begin(), Send.end(),
				std::inserter(Sseg, Sseg.begin()));
			//Sensors in Sseg will be able to cover (both ends of, hence) this segment.

			std::set<int>::iterator sit;
			IloExpr expr(env);
			for (sit = Sseg.begin(); sit != Sseg.end(); ++sit)
				expr += y[*sit];

			expr -= (p_dataST->sensorstocover*x[edgeid]);
			strcpy(consname, ("SC_" + ToString(edgeid) + "_" + ToString(segcount)).c_str());
			model.add(expr >= 0).setName(consname);
			p_numconstraintST->cutcount[0]++;

			//Prepare for the evaluation of the following segment (still on this edge)
			expr.clear();
			//Added on 04.18 19:40pm:
			Sstart.clear(); Send.clear(); Sseg.clear();
			xstart = xend;
			ystart = yend;
			segcount++;
		}

		float xlast = xstart;
		float ylast = ystart;

		//When coming to the last partition on this edge:
		if ((x2 - xstart - p_dataST->delta*hori)*(x1 - xstart - p_dataST->delta*hori) >= 0) {
			
			std::set<int> Sstart, Send, Sseg;
			for (int s = 0; s<p_dataST->numsensors; ++s) {
				float D = sqrt((xstart - p_dataST->sensorcoords[s].xcoord)*(xstart - p_dataST->sensorcoords[s].xcoord)
					+ (ystart - p_dataST->sensorcoords[s].ycoord)*(ystart - p_dataST->sensorcoords[s].ycoord));
				if (D < p_dataST->threshold)
					Sstart.insert(s);
			}

			///*
			//New for ATSP
			for (int s = 0; s<p_dataST->numsensors; ++s) {
				float D1 = (p_dataST->sensorcoords[s].xcoord - x2)*(x2 - xstart);
				float D2 = (p_dataST->sensorcoords[s].ycoord - y2)*(y2 - ystart);

				float D3 = sqrt((x2 - p_dataST->sensorcoords[s].xcoord)*(x2 - p_dataST->sensorcoords[s].xcoord)
					+ (y2 - p_dataST->sensorcoords[s].ycoord)*(y2 - p_dataST->sensorcoords[s].ycoord));
				float D4 = sqrt((xstart - x2)*(xstart - x2)
					+ (ystart - y2)*(ystart - y2));
				if (D1+D2 >= 0.707*D3*D4)
					// Extremely Important: Change the Numerical value above when FOV changes!!!!!!
					// Use cosine(FOV)
					Send.insert(s);
			}
			//*/

			/*
			//New for STSP
			for (int s = 0; s<p_dataST->numsensors; ++s) {
				float D = sqrt((x2 - p_dataST->sensorcoords[s].xcoord)*(x2 - p_dataST->sensorcoords[s].xcoord)
					+ (y2 - p_dataST->sensorcoords[s].ycoord)*(y2 - p_dataST->sensorcoords[s].ycoord));
				if (D < p_dataST->threshold)
					Send.insert(s);
			}
			*/

				set_intersection(Sstart.begin(), Sstart.end(), Send.begin(), Send.end(),
				std::inserter(Sseg, Sseg.begin()));
			//Sensors in Sseg will be able to cover (both ends of, hence) this segment.

			std::set<int>::iterator sit;
			IloExpr expr(env);
			for (sit = Sseg.begin(); sit != Sseg.end(); ++sit)
				expr += y[*sit];

			expr -= (p_dataST->sensorstocover*x[edgeid]);
			strcpy(consname, ("SC_" + ToString(edgeid) + "_" + ToString(segcount)).c_str());
			model.add(expr >= 0).setName(consname);
			p_numconstraintST->cutcount[0]++;

			expr.clear();			
			//Added on 04.18 19:40pm:
			Sstart.clear(); Send.clear(); Sseg.clear();
		}
	}
//*/

	// Sensor limit

	// model.add(IloSum(y) <= p_dataST->sensorlimit).setName("Sensorbound");
	// model.add(IloSum(y) <= 13).setName("Sensorbound");

	/*
	// Clearance Constraint I
	for (int s = 0; s<p_dataST->numsensors; ++s) {
		for (int t = s+1; t<p_dataST->numsensors; ++t) {
			float Dst = sqrt((p_dataST->sensorcoords[t].xcoord - p_dataST->sensorcoords[s].xcoord)*(p_dataST->sensorcoords[t].xcoord - p_dataST->sensorcoords[s].xcoord)
				+ (p_dataST->sensorcoords[t].ycoord - p_dataST->sensorcoords[s].ycoord)*(p_dataST->sensorcoords[t].ycoord - p_dataST->sensorcoords[s].ycoord));
			if (Dst < 3)
				// Extremely Important: Change the Numerical value above when Clearance changes!!!!!!
				model.add(y[s] + y[t] <= 1);
		}
	}

	// Clearance Constraint II
	for (int s = 0; s<p_dataST->numsensors; ++s) {
		for (int t = 0; t<p_dataST->numtargets; ++t) {
			float Dst = sqrt((p_dataST->targetcoords[t].xcoord - p_dataST->sensorcoords[s].xcoord)*(p_dataST->targetcoords[t].xcoord - p_dataST->sensorcoords[s].xcoord)
				+ (p_dataST->targetcoords[t].ycoord - p_dataST->sensorcoords[s].ycoord)*(p_dataST->targetcoords[t].ycoord - p_dataST->sensorcoords[s].ycoord));
			if (Dst < 1)
				// Extremely Important: Change the Numerical value above when Clearance changes!!!!!!
				model.add(y[s] == 0);
		}
	}
	*/

	return;
};

