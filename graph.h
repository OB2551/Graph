#include <vector>
#include <set>
#include<iostream>
#include "Polynomial.h"
#include <map>

class graph {
public:
	std::map<int, std::vector<int > > adj;
    int size;
	int edge_count;
	graph();
	graph(std::map<int, std::vector<int > > adjacencies, int count);
	graph(int n);
	void traverse(int u, std::map<int, bool>& visited);
	void traverse2(int u, std::map<int, bool>& visited, std::map<int, std::vector<int>>& new_adjs, int& edges, std::map<int, bool>& visited2);
	bool isConnected();
	bool isTree();
	bool isTotallyDisconnected();
	bool isComplete();
	void addEdge(int V1, int V2);
	void addNode(int V);
	int edgeFinder(int V1, int V2);
	bool isNeighbour(int V1, int V2);
	std::vector<graph> connectedComponents();
	graph contraction(int V1, int V2);
	void removeEdge(int V1, int V2);
	void removeNode(int V);
	graph edgeRemoved(int V1, int V2);
	Polynomial completePoly();
	Polynomial treePoly();
	Polynomial cyclePoly();
	Polynomial getChromaticPolynomial();
	void toString();
	bool adjacent(int u, int v);
	int chromaticNumber(Polynomial chromPol);
	std::map<int,int> greedyColouring(int& coloursneeded);



};