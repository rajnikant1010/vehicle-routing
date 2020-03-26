#include "utilities.h"

using namespace std;

ILOUSERCUTCALLBACK4(UserCuts, IloNumVarArray, xvars,
							IloNumVarArray, yvars,
							dataST*, p_dataST,
							numconstraintST*, p_numconstraintST) {
	try {

		IloEnv env = getEnv();

		IloNumArray xvals(env, xvars.getSize()), yvals(env, yvars.getSize());
		getValues(xvals, xvars);
		getValues(yvals, yvars);

		double itime, ftime;
		double lowerbound = getObjValue();
		int nodes = getNnodes();

		IloRangeArray newcuts(env);
		itime = env.getTime();

		UserSeparation (p_dataST, p_numconstraintST, env, newcuts, xvars, yvars, xvals, yvals);

		ftime = env.getTime();

		for (size_t i=0; i<newcuts.getSize(); ++i)
			add(newcuts[i]);

		newcuts.end(); xvals.end(); yvals.end();
	}

	catch(IloException& e) {
		cout << "Error" << e;
	}
	catch (...) {
		cerr << "Error in UserCutCallback" << endl;
	}

}

ILOLAZYCONSTRAINTCALLBACK4(LazyConstraints, IloNumVarArray, xvars,
							IloNumVarArray, yvars,
							dataST*, p_dataST,
							numconstraintST*, p_numconstraintST) {
	try {

		IloEnv env = getEnv();

		IloNumArray xvals(env, xvars.getSize()), yvals(env, yvars.getSize());
		getValues(xvals, xvars);
		getValues(yvals, yvars);

		double itime, ftime;
		double lowerbound = getObjValue();
		int nodes = getNnodes();

		IloRangeArray newcuts(env);
		itime = env.getTime();

		LazySeparation (p_dataST, p_numconstraintST, env, newcuts, xvars, yvars, xvals, yvals);

		ftime = env.getTime();

		for (size_t i=0; i<newcuts.getSize(); ++i)
			add(newcuts[i]);

		newcuts.end(); xvals.end(); yvals.end();
	}

	catch(IloException& e) {
		cout << "Error" << e;
	}
	catch (...) {
		cerr << "Error in LazyConstraintCallback" << endl;
	}

}




int main(int argc, char *argv[]) {

	dataST *p_dataST = new dataST;
	numconstraintST *p_numconstraintST = new numconstraintST;
	p_numconstraintST->cutcount.resize(1,0);

	int vartype = 0; // cutting plane - all variables continuous
	int cplexcuts = 0; // to use or not to use cplex cuts
	int modeltype = 0; // has to be 1/2/3/4
    int writeoutput = 0; // has to be 0/1

	if (argc < 4) { //was 5 originally, with threshold input in line.
		cout << "Format: ./main instancename instancepath vartype(0/1)" << endl;
		exit(1);
	}

	strcpy(p_dataST->instancename,argv[1]);
	strcpy(p_dataST->instancepath,argv[2]);
	vartype = atoi(argv[3]); // if vartype 1 - Branch and cut
	//cplexcuts = atoi(argv[4]); // if 1 - use cplex cuts

	//p_dataST->threshold = (float)atoi(argv[4]);
	p_dataST->sensorstocover = 2;
	p_dataST->sensorlimit = 100;

	ReadData(p_dataST);


	// Build Model

	IloEnv env;

	try {

		// Initial model

		IloModel model(env);
		model.setName(p_dataST->instancename);

		IloNumVarArray xvars(env);
		IloNumVarArray yvars(env);
		IloNumArray c(env);

		PopulateVariables(p_dataST, xvars, yvars, c, vartype, env);

		model.add(xvars); model.add(yvars);

		IloExpr obj(env);
		obj += IloScalProd(xvars,c); //Comment this if just solving LM Covering
        obj += 20*IloSum(yvars); //Comment this if just solving MDTSP
		model.add(IloMinimize(env,obj)).setName("Objective");

		PopulateConstraints(p_dataST, p_numconstraintST, xvars, yvars, env, model);
		// model.add(IloScalProd(xvars,c) <= 296).setName("TSPPathbound");
		//ALWAYS Comment this whenever solving joint optimization!!!!!!

		IloCplex cplex(model);
		cplex.extract(model);
		cplex.exportModel(("LPFiles/"+ToString(p_dataST->instancename)+".lp").c_str());
		//Uncomment it if needed sometimes

		//cplex.setParam(cplex.HeurFreq, -1);
		cplex.setParam(cplex.PreInd,0);
		//cplex.setParam(cplex.CutsFactor,1);
		cplex.setParam(IloCplex::MIPInterval,5);
		cplex.setParam(IloCplex::MIPDisplay,3);
		cplex.setParam(IloCplex::VarSel,3);
		cplex.setParam(IloCplex::TiLim,5400); //This may be changed at any time
		cplex.setParam(IloCplex::LBHeur,1);

		cplex.use(UserCuts(env, xvars, yvars, p_dataST, p_numconstraintST));
		cplex.use(LazyConstraints(env, xvars, yvars, p_dataST, p_numconstraintST));
		char logfile[100];
		ofstream outfile;
		strcpy(logfile, "logs-a20/");
		strcat(logfile, p_dataST->instancename);
		outfile.open(logfile);
		cplex.setOut(outfile);
		cplex.solve();
		outfile.close();

		char resultsfilename[100];
		strcpy(resultsfilename, "Results-a20/");
        //strcat(resultsfilename, ToString(p_dataST->numtargets).c_str());
        //strcat(resultsfilename, "/");
		strcat(resultsfilename, p_dataST->instancename);

		ofstream resultsfilestream;
		resultsfilestream.open(resultsfilename);
        
        resultsfilestream << "number of targets : " << p_dataST->numtargets << endl;
        resultsfilestream << "grid size : 100 by 100 " << endl;
        resultsfilestream << "radius of the circular range for the sensor : " << p_dataST->threshold << endl;
        resultsfilestream << endl;
        resultsfilestream << "target coordinates (format i x y) " << endl;

        for (int i=0; i<p_dataST->numtargets; ++i)
            resultsfilestream << i << " " << 
                p_dataST->targetcoords[i].xcoord << " " << 
                p_dataST->targetcoords[i].ycoord << endl;
        
        resultsfilestream << endl; 
        resultsfilestream << "sensor coordinates (format x y) " << endl; 
        
        int finalsensorcount = 0;
		for (int i=0; i<yvars.getSize(); ++i)
			if (cplex.getValue(yvars[i]) > 0.9) {
                finalsensorcount += 1;
                resultsfilestream << p_dataST->sensorcoords[i].xcoord << " " << 
                    p_dataST->sensorcoords[i].ycoord << endl;
            }

        resultsfilestream << endl;

        vector<int> edge_indexes;
		for (int i=0; i<p_dataST->edges.size(); ++i)
			if (cplex.getValue(xvars[i]) > 0.9) {
                cout << p_dataST->edges[i].u + 1 << " -- " << p_dataST->edges[i].v + 1 << endl;
                edge_indexes.push_back(i);
            }
				
       /*
		vector<int> sequence; 
        sequence.push_back(0);
        for (int i=0; i<edge_indexes.size(); ++i) {
            int edge_index = edge_indexes[i];
            if (p_dataST->edges[edge_index].u == 0) {
                sequence.push_back(p_dataST->edges[edge_index].v);
                edge_indexes.erase(edge_indexes.begin() + i);
                break;
            }

            if (p_dataST->edges[edge_index].v == 0) {
                sequence.push_back(p_dataST->edges[edge_index].u);
                edge_indexes.erase(edge_indexes.begin() + i);
                break;
            }

        }

        while (sequence.back() != 0) {
            int j = sequence.back();
            for (int i=0; i<edge_indexes.size(); ++i) {
                int edge_index = edge_indexes[i];
                if (p_dataST->edges[edge_index].u == j) {
                    sequence.push_back(p_dataST->edges[edge_index].v);
                    edge_indexes.erase(edge_indexes.begin() + i);
                    break;
                }

                if (p_dataST->edges[edge_index].v == j) {
                    sequence.push_back(p_dataST->edges[edge_index].u);
                    edge_indexes.erase(edge_indexes.begin() + i);
                    break;
                }

            }

        }
        
        assert(sequence.size() == p_dataST->numtargets + 1);

        resultsfilestream << "target visit sequence " << endl;
        for (int i=0; i<sequence.size(); ++i)
            resultsfilestream << sequence[i] << endl;
        */
        
        resultsfilestream << endl;
        resultsfilestream << "sensor count : " << finalsensorcount << endl;
        resultsfilestream << "travel cost : " << (cplex.getObjValue() - 20*finalsensorcount); 
		resultsfilestream.close();
		
		/*
		resultsfilestream.open("Results-n20/outresults-n20.csv");
		resultsfilestream << p_dataST->instancename << "," << p_dataST->numdepots << "," << 
				p_dataST->numtargets << "," << p_dataST->numsensors << "," << p_dataST->delta << "," << 
				p_dataST->threshold << "," << finalsensorcount << "," <<  
				cplex.getObjValue() << "," <<
                p_numconstraintST->cutcount[0] << "," << p_numconstraintST->cutcount[1] << "," <<
                p_numconstraintST->cutcount[2] << "," << p_numconstraintST->cutcount[3] << "," <<
                p_numconstraintST->cutcount[4] << "," << cplex.getNnodes() << endl;
		resultsfilestream.close();
		*/
        		
		/*
		resultsfilestream.open("Results-n20/outtimes-n20.csv");
		resultsfilestream << p_dataST->instancename << "," << cplex.getTime() << "," <<
                std::accumulate(p_numconstraintST->separationtime.begin(), p_numconstraintST->separationtime.end(),0) << "," <<
                p_numconstraintST->separationtime[1] << "," <<
                p_numconstraintST->separationtime[2] << "," <<
                p_numconstraintST->separationtime[3] << "," <<
                p_numconstraintST->separationtime[4] << endl;
		resultsfilestream.close();
		*/

        
		cout << "ObjValue: " << cplex.getObjValue() << endl;


		for (int i=0; i<yvars.getSize(); ++i)
			if (cplex.getValue(yvars[i]) > 0.9)
				cout << i+1 << ", ";
		cout << endl;
		}

		catch (IloException& ex) {
			cerr << "Error: " << ex << endl;
		}
		catch (...) {
			cerr << "Error" << endl;
		}

		env.end();

	return 0;
}
