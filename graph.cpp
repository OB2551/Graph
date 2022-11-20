#include "graph.h"
#include<vector>
#include <map>
#include <list>


/*Uses GMP library for large integers, possibility of integer overflow with standard integer class
when taking repeated powers of larger matrices*/
graph::graph() 
{
	size = 0;
	edge_count = 0;
	adj = {};
}
graph::graph(std::map<int, std::vector<int > > adjacencies, int count) 
{
	edge_count = count;
	adj = adjacencies;
	size = adj.size();

}
graph::graph(int m) 
{
		size = m;
		adj = std::map<int, std::vector<int > >{};
		for (int i = 0; i < size; i++)
		{
			adj.insert({ i, {} });
		}
		edge_count = 0;

}

void graph::traverse(int u, std::map<int, bool>& visited) 
{
	visited[u] = true; //mark v as visited
	for (auto& v : adj[u]) 
	{
		if (!visited[v])
		{
			traverse(v, visited);
		}
	}
}
void graph::traverse2(int u, std::map<int, bool>& visited, std::map<int, std::vector<int>>& new_adjs, int& edges, std::map<int, bool>& visited2)
{
	visited[u] = true; //mark v as visited
	visited2[u] = true;
	new_adjs.insert({ u,{} });
	for (auto& v : adj[u]) 
	{
		new_adjs[u].push_back(v);
		edges++;
		if (!visited[v]) 
		{
			
			traverse2(v, visited, new_adjs, edges, visited2);
		}
	}
}
bool graph::isConnected() {
	//bool* vis = new bool[size];
	std::map<int, bool> vis;
	//for all vertex u as start point, check whether all nodes are visible or not
	for (auto& v : adj) {
		for (auto& i : adj) { vis[i.first] = false; } //initialize as no node is visited
		traverse(v.first, vis);
		for (auto& i : adj) {
			if (!vis[i.first]) //if there is a node, not visited by traversal, graph is not connected
				return false;
		}
	}
	return true;
}
bool graph::isTree() {
	/*checks if graph is a tree. A tree by definition is connected
	and has n-1 edges if it has n nodes*/
	if (this->isConnected() && edge_count == size - 1) { return true; }
	else { return false; }
}
bool graph::isTotallyDisconnected() {
	//checks if graph has no edges
	if (edge_count == 0) {
		return true;
	}
	return false;
}
bool graph::isComplete() {
	//checks for completeness, a complete graph with n nodes has nC2 = n(n-1)/2 edges
	if (edge_count == (size * (size - 1))/2) {
		return true;
	}
	return false;
}
void graph::addEdge(int V1, int V2) {
	if (!isNeighbour(V1, V2))
	{
		adj[V1].push_back(V2);
		adj[V2].push_back(V1);
		edge_count++;
	}
}
void graph::addNode(int V) 
{
	adj[V] = {};
	size++;
}
int graph::edgeFinder(int V1, int V2)
{
	for (int i = 0; i < adj[V1].size(); i++)
	{
		if (adj[V1][i] == V2)
		{
			return i;
		}
	}
	return size;
}
bool graph::isNeighbour(int V1, int V2) 
{
	std::vector<int>::iterator position = std::find(adj[V1].begin(), adj[V1].end(), V2);
	if (position == adj[V1].end()) 
	{
		return false;
	}
	return true;
}
std::vector<graph> graph::connectedComponents() {
	std::vector<graph> components = {};
	std::map<int, bool> vis, VIS;
	
	//for all vertex u as start point, check whether all nodes are visible or not
	for (auto& i : adj) { VIS[i.first] = false; }
	for (auto& u : adj) {
		if (VIS[u.first]) { continue; }
		
		for (auto& i : adj) { vis[i.first] = false; } //initialize as no node is visited
		int edges = 0;
		std::map<int, std::vector<int> > new_adjs;
	
			traverse2(u.first, vis, new_adjs, edges, VIS);
		
		components.push_back(graph(new_adjs, edges / 2));
		
	}
	return components;
}
void graph::removeEdge(int V1, int V2) 
{
	int i1 = edgeFinder(V1, V2), i2 = edgeFinder(V2, V1);
	adj[V1].erase(adj[V1].begin()+i1);
	adj[V2].erase(adj[V2].begin() + i2);		
	edge_count--;
}
void graph::removeNode(int V) 
{
	for (auto& node : adj) {
		int position = edgeFinder(node.first, V);
		if (position < size)
		{
			node.second.erase(node.second.begin() + position);
			edge_count--;
		}
	}
	adj.erase(V);
	size--;
}
graph graph::contraction(int V1, int V2) {
	/*Forms a new graph by contracting vertex V2 to vertex V1.
	If there is an edge from V1 to V2 then it is non-existant in the new graph.
	*/
	std::map<int, std::vector<int>> new_adj = adj;
	int new_edge_count = edge_count;
	for (auto &neighbour : new_adj[V2]) 
	{
		//for neighbours of V2, delete V2 from their list of neighbours:
		for (std::vector<int>::iterator iter = new_adj[neighbour].begin(); iter != new_adj[neighbour].end(); iter++) 
		{
			if (*iter == V2) 
			{
				new_adj[neighbour].erase(iter);
				break;
			}
		}
		//if neighbour is not already a neighbour of V1, make it a neihgbour of V1
		//exclude case where possibly V1 is a neighbour of V2
		if (isNeighbour(V2, neighbour) and neighbour != V1)
		{
			new_adj[neighbour].push_back(V1);
			new_adj[V1].push_back(neighbour);
		}
		else 
		{
			//If neighbour of V2 is already a neighbour of V1, do nothing, except
			// we now get a duplicate edge, so edgecount of contracted graph decreases by one
			new_edge_count--;
		}
	}
	new_adj.erase(V2);
	return graph(new_adj, new_edge_count);
}
graph graph::edgeRemoved(int V1, int V2) 
{
	//returns graph with specified edge E removed
	std::map<int, std::vector<int> > new_adj = adj;
	int i1 = edgeFinder(V1, V2), i2 = edgeFinder(V2, V1);

	new_adj[V1].erase(new_adj[V1].begin() + i1);
	new_adj[V2].erase(new_adj[V2].begin() + i2);

	return graph(new_adj, edge_count - 1);
}
Polynomial graph::completePoly() 
{
	//if graph is complete with n nodes, every vertex is adjacent
	//so n colours need. Chromatic polynomial is 0 for all k<n
	//That is , P_G(x) = (x-(n-1))....(x-1)(x-0)
	Polynomial P({ 1 });
	for (int k = 0; k < size; k++) 
	{
		P = Polynomial({ -k,1 }) * P;
	}
	return P;
}
Polynomial graph::treePoly() 
{
	//tree chromatic polynomial is of form x(x-1)^(n-1) for tree with n nodes
	Polynomial P({ 0,1 });
	P = Polynomial({ -1,1 }).pow(size-1) * P;
	return P;
}
Polynomial graph::cyclePoly() 
{
	Polynomial fac = Polynomial({ -1,1 });
	Polynomial res = fac.pow(size);
	int x = pow(-1, size);
	Polynomial C = Polynomial({ x}) * fac;
	return res+C;
}
Polynomial graph::getChromaticPolynomial()
{   /*Returns the chromatic polynomial of graph
	using recursive contraction method*/
	//first check edge cases:
	if (this->isTotallyDisconnected()) 
	{
		Polynomial P = Polynomial({ 0,1 }).shift(size-1);
		return P;
	}
	if (this->isComplete()) 
	{
		return completePoly();
	}
	if (this->isConnected()) 
	{
		if (size - 1 == edge_count)
		{
			return treePoly();
		}
		//otherwise contract graph. choose any vertex (not optimal), it has an edge since connected
		int e1 = adj.begin()->first;
		int e2 = adj.begin()->second[0];
		Polynomial g1 = -this->contraction(e1, e2).getChromaticPolynomial(),
		g2 = this->edgeRemoved(e1, e2).getChromaticPolynomial();
		return g1+g2;
	}

	else
	{
    //Split into connected components and take product of their chromatic polynomials
	std::vector<graph> comps = this->connectedComponents();
	Polynomial res = Polynomial({ 1 });
	for (auto g : comps)
	{
		res = g.getChromaticPolynomial() * res;
	}
	return res;
	}
	
	
} 
void graph::toString() 
{
	//prints nodes and edges to console
	std::cout << "Vertices: ";
	for (auto v : adj) 
	{
		std::cout << std::to_string(v.first) + ", ";
		for (auto e : v.second) 
		{
			std::cout << e << std::endl;
		}
	}
}
bool graph::adjacent(int u, int v) 
{
	if (std::find(adj[u].begin(), adj[u].end(), v) != adj[u].end()) 
	{
		return true;
	}  
return false;
}
int graph::chromaticNumber(Polynomial chromPol)
{
	//finds minimal k such that P_g(k) !=0
	for (int k = 1; k < size+2; k++) 
	{
		if (chromPol.evaluate(k) >0) 
		{ 
			return k; 
		}
		
	}
	return size;
}
std::map<int, int> graph::greedyColouring(int &coloursneeded) 
{
	/*greedy algorthm for colouring graph.may not always return optimal result*/
	std::map<int,int> coloured_vertices;
	std::set<int> coloursused{};
	
	for (auto &v:adj) 
	{
		int node = v.first;
		std::set<int> possible_colours = coloursused;
		possible_colours.insert(coloursused.size());
		for (auto &vert_colour : coloured_vertices) 
		{
			if (adjacent(vert_colour.first, node)) 
			{
				int vertex_colour = vert_colour.second;
				possible_colours.erase(vertex_colour);
			}
			
		}
		coloured_vertices[node] = *possible_colours.begin();
		coloursused.insert(*possible_colours.begin());


	}
	coloursneeded = coloursused.size();
	return coloured_vertices;
}
