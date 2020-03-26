/*
* main.cpp
*
*  Last checked and modified: Dec 17, 2019
*      Author: Bingyu
*
*  This code implements Dr. Kaarthik Sundar's proposed LPP algorithm in the 2018 ACC paper.
*/

#include "utilities.h"

using namespace std;

void ReadData(dataST *p_dataST) {

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
	infile >> p_dataST->numtargets >> p_dataST->numsensors >> p_dataST->threshold;

	for (int t = 0; t < p_dataST->numtargets; ++t) {
		float x, y;
		infile >> x >> y;
		p_dataST->targetcoords.push_back(coordST(x, y));
	}
	
	//The current coding fashion assumes we write the sequence as a closed loop 
	for (int t = 0; t < p_dataST->numtargets + 1; ++t) {
		int seq;
		infile >> seq;
		p_dataST->targetseq.push_back(seq);
	}

	for (int s = 0; s < p_dataST->numsensors; ++s) {
		float x, y;
		infile >> x >> y;
		p_dataST->sensorcoords.push_back(coordST(x, y));
	}
	
	//The following lines are for initialization. Look ugly, but might be necessary...
	for (int s = 0; s < p_dataST->numsensors; ++s) {
		p_dataST->sensorcoordstrans.push_back(p_dataST->sensorcoords[s]);
		p_dataST->sensorcoordsts.push_back(-1000);
		p_dataST->sensorcoordstd.push_back(1000);
	}

	set<int> st0;
	for (int t = 0; t < p_dataST->numtargets; ++t) {
		p_dataST->LMonedge.push_back(st0);
	}

	return;
}

void CoordTrans(int seq, dataST *p_dataST) {
	//Determine the current line segment 
	//The current coding fashion assumes the sequence of Waypoints (determined by the LKH heuristic) 
	//has natural indices of 1~n, not 0~(n-1)
	//Also assumes we write the sequence as a closed loop

	int start, end;
	start = p_dataST->targetseq[seq] - 1; end = p_dataST->targetseq[seq + 1] - 1;
	
	/*
	//If not written as a closed loop:
	if (seq < p_dataST->numtargets - 1) {
		start = p_dataST->targetseq[seq] - 1; end = p_dataST->targetseq[seq + 1] - 1;
	}

	if (seq == p_dataST->numtargets - 1) {
		start = p_dataST->targetseq[seq] - 1; end = p_dataST->targetseq[0] - 1;
	}
	*/

	//Compute length of the current line segment 
	p_dataST->d = sqrt((p_dataST->targetcoords[end].ycoord - p_dataST->targetcoords[start].ycoord)*(p_dataST->targetcoords[end].ycoord - p_dataST->targetcoords[start].ycoord) + (p_dataST->targetcoords[end].xcoord - p_dataST->targetcoords[start].xcoord)*(p_dataST->targetcoords[end].xcoord - p_dataST->targetcoords[start].xcoord));

	//Compute axis rotation angle theta (Counter-clockwise !!!)
	//Notice: For correctness, it is nontrivial to use the function atan2 to determine the quadrant as well !!!
	float theta = atan2((p_dataST->targetcoords[end].ycoord - p_dataST->targetcoords[start].ycoord),(p_dataST->targetcoords[end].xcoord - p_dataST->targetcoords[start].xcoord));
	
	//Compute new coordinates of LMs
	//Notice: Translation first, Followed by rotation !!!
	for (int i = 0; i < p_dataST->numsensors; i++)
	{
		p_dataST->sensorcoordstrans[i].xcoord = (p_dataST->sensorcoords[i].xcoord - p_dataST->targetcoords[start].xcoord) * cos(theta) + (p_dataST->sensorcoords[i].ycoord - p_dataST->targetcoords[start].ycoord) * sin(theta);
		p_dataST->sensorcoordstrans[i].ycoord = - (p_dataST->sensorcoords[i].xcoord - p_dataST->targetcoords[start].xcoord) * sin(theta) + (p_dataST->sensorcoords[i].ycoord - p_dataST->targetcoords[start].ycoord) * cos(theta);
	}
	
	//Compute ts & td coordinates of LMs
	for (int i = 0; i < p_dataST->numsensors; i++)
	{
		float ts2 = (p_dataST->threshold * p_dataST->threshold) - (p_dataST->sensorcoordstrans[i].ycoord * p_dataST->sensorcoordstrans[i].ycoord);
		float checky = 0.7071 * p_dataST->threshold - abs(p_dataST->sensorcoordstrans[i].ycoord);
		if (checky < 0)
			//LM[i] will be useless for the current line segment
		{
			p_dataST->sensorcoordsts[i] = 1000; p_dataST->sensorcoordstd[i] = -1000;
		}
		if (checky >= 0) {
			float ts3 = p_dataST->sensorcoordstrans[i].xcoord - sqrt(ts2);
			if (ts3 >= 0) 
				p_dataST->sensorcoordsts[i] = ts3;
			else if (ts3 < 0) 
				p_dataST->sensorcoordsts[i] = 0;
			float td2 = p_dataST->sensorcoordstrans[i].xcoord - abs(p_dataST->sensorcoordstrans[i].ycoord);
			if (td2 >= p_dataST->d)
				p_dataST->sensorcoordstd[i] = p_dataST->d;
			else if(td2 < p_dataST->d)
				p_dataST->sensorcoordstd[i] = td2;
		}
	}
	
	return;
}

void LMPlace(int seq, dataST *p_dataST) {

	int feas = 0;
	for (int i = 0; i < p_dataST->numsensors; i++)
	{
		if (p_dataST->sensorcoordstd[i] >= p_dataST->d - XTOL)
			feas++;
	}
	if (feas == 0) {
		cout << "The incumbent line segment " << seq << " cannot be 2-covered by the given LMs. Break!" << endl;
		cout << "The length of the incumbent line segment is " << p_dataST->d << endl;
		cout << "The respective intervals that can be covered by the given LMs are:" << endl;
		for (int i = 0; i < p_dataST->numsensors; i++)
			cout << p_dataST->sensorcoordsts[i] << " " << p_dataST->sensorcoordstd[i] << endl;
	}
		
	float ExmPt1 = 0; // The farthest point of examination (on the right)
	float ExmPt2 = 0; // The second farthest point of examination (on the left)

	int indmax = 0; // The index corresponding to a good LM which can extend incumbent valid coverage to the farthest 

	// Choosing the initial two LMs needs more care
	int indmax2 = 0;
	for (int i = 0; i < p_dataST->numsensors; i++)
	{
		if ((p_dataST->sensorcoordsts[i] <= ExmPt2) && (p_dataST->sensorcoordstd[i] > p_dataST->sensorcoordstd[indmax]))
			indmax = i;
	}

	for (int i = 0; i < p_dataST->numsensors; i++)
	{
		if ((i != indmax) && (p_dataST->sensorcoordsts[i] <= ExmPt2) && (p_dataST->sensorcoordstd[i] > p_dataST->sensorcoordstd[indmax2]))
			indmax2 = i;
	}

	//After the two loops above, we get the best two LMs to be added to placement
	p_dataST->LMonedge[seq].insert(indmax);
	p_dataST->LMonedge[seq].insert(indmax2);

	//Update the two points of examination 
	ExmPt2 = p_dataST->sensorcoordstd[indmax2];
	ExmPt1 = p_dataST->sensorcoordstd[indmax];

	while (ExmPt2 < p_dataST->d - XTOL)
	{
		if (feas == 0)
			break;

		for (int i = 0; i < p_dataST->numsensors; i++)
		{
			if ((p_dataST->sensorcoordsts[i] <= ExmPt2) && (p_dataST->sensorcoordstd[i] > p_dataST->sensorcoordstd[indmax]))
				indmax = i;
		}

		//After the loop above, we get the best LM to be added to placement
		p_dataST->LMonedge[seq].insert(indmax);
		//Update the two points of examination 
		ExmPt2 = ExmPt1;
		ExmPt1 = p_dataST->sensorcoordstd[indmax];
	}

	return;
}


int main(int argc, char *argv[]) {

	std::clock_t start;
	double duration;

	start = std::clock();

	dataST *p_dataST = new dataST;

	if (argc < 3) { //might be changed.
		cout << "Format: ./main instancename instancepath" << endl;
		exit(1);
	}

	strcpy(p_dataST->instancename, argv[1]);
	strcpy(p_dataST->instancepath, argv[2]);

	ReadData(p_dataST);

	for (int t = 0; t < p_dataST->numtargets; ++t) {
		CoordTrans(t, p_dataST);
		//cout << "Transformed edge " << t << endl;
		LMPlace(t, p_dataST);
		//cout << "Placed edge " << t << endl;
	}
	
	//Add a set_union function here to obtain LMs to cover all line segments in the path
	set<int> Se;
	std::set<int>::iterator LMit;
	//int LMid;
	for (int t = 0; t < p_dataST->numtargets; ++t) {
		//for (int s = 0; s < p_dataST->LMonedge[t].size(); ++s) {
		for (LMit = p_dataST->LMonedge[t].begin(); LMit != p_dataST->LMonedge[t].end(); ++LMit){
			if (Se.find(*LMit) == Se.end())
				Se.insert(*LMit);
		}
	}

	duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
	cout << "Computation time for this instance: " << duration << " second." << endl;

	//Output its Cardinality!
	cout << "The path will use " << Se.size() << " LMs to cover. Their indices are the following:" << endl;
	//Display it!
	for (LMit = Se.begin(); LMit != Se.end(); ++LMit) {
		cout << (*LMit) << " ";
	}

	return 0;
}




