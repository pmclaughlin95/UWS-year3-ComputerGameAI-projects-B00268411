

//
//=======================================================================
// Copyright (c) 2004 Kristopher Beevers
//
// Distributed under the Boost Software License, Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//=======================================================================
//


#include <boost/graph/astar_search.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/random.hpp>
#include <boost/random.hpp>
#include <boost/graph/graphviz.hpp>
#ifndef _WIN32
#include <sys/time.h>
#endif
#include <vector>
#include <list>
#include <iostream>
#include <fstream>
#include <math.h>    // for sqrt

using namespace boost;
using namespace std;


// auxiliary types
struct location
{
	float y, x; // lat, long
};
typedef float cost;

template <class Number, class LocMap>
class node_writer {
public:
	node_writer(Number n, LocMap l, float _minx, float _maxx,
		float _miny, float _maxy,
		unsigned int _ptx, unsigned int _pty)
		: number(n), loc(l), minx(_minx), maxx(_maxx), miny(_miny),
		maxy(_maxy), ptx(_ptx), pty(_pty) {}
	template <class Vertex>
	void operator()(ostream& out, const Vertex& v) const {
		float px = 1 - (loc[v].x - minx) / (maxx - minx);
		float py = (loc[v].y - miny) / (maxy - miny);
		out << "[label=\"" << number[v] << "\", pos=\""
			<< static_cast<unsigned int>(ptx * px) << ","
			<< static_cast<unsigned int>(pty * py)
			<< "\", fontsize=\"11\"]";
	}
private:
	Number number;
	LocMap loc;
	float minx, maxx, miny, maxy;
	unsigned int ptx, pty;
};

template <class WeightMap>
class time_writer {
public:
	time_writer(WeightMap w) : wm(w) {}
	template <class Edge>
	void operator()(ostream &out, const Edge& e) const {
		out << "[label=\"" << wm[e] << "\", fontsize=\"11\"]";
	}
private:
	WeightMap wm;
};


// euclidean distance heuristic
template <class Graph, class CostType, class LocMap>
class distance_heuristic : public astar_heuristic<Graph, CostType>
{
public:
	typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
	distance_heuristic(LocMap l, Vertex goal)
		: m_location(l), m_goal(goal) {}
	CostType operator()(Vertex u)
	{
		CostType dx = m_location[m_goal].x - m_location[u].x;
		CostType dy = m_location[m_goal].y - m_location[u].y;
		return ::sqrt(dx * dx + dy * dy);
	}
private:
	LocMap m_location;
	Vertex m_goal;
};


struct found_goal {}; // exception for termination

					  // visitor that terminates when we find the goal
template <class Vertex>
class astar_goal_visitor : public boost::default_astar_visitor
{
public:
	astar_goal_visitor(Vertex goal) : m_goal(goal) {}
	template <class Graph>
	void examine_vertex(Vertex u, Graph& g) {
		if (u == m_goal)
			throw found_goal();
	}
private:
	Vertex m_goal;
};


int main(int argc, char **argv)
{

	// specify some types
	typedef adjacency_list<listS, vecS, undirectedS, no_property,
		property<edge_weight_t, cost> > mygraph_t;
	typedef property_map<mygraph_t, edge_weight_t>::type WeightMap;
	typedef mygraph_t::vertex_descriptor vertex;
	typedef mygraph_t::edge_descriptor edge_descriptor;
	typedef std::pair<int, int> edge;

	// specify data
	enum nodes {
		n0, n1, n2, n3, n4, n5, n6, n7, n8, n9, n10, n11, n12, n13, n14, n15, n16, n17, n18, n19, n20, n21, n22, n23, n24, n25, n26, n27, n28, n29, n30, n31, n32, n33, n34, n35, n36, n37, n38, n39, n40,
		n41, n42, n43, n44, n45, n46, n47, n48, n49, n50, n51, n52, n53, n54, n55, n56, n57, n58, n59, n60, n61, n62, n63, N

	};
	const char *numbers[] = {
		"0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13", "14", "15", "16", "17", "18", "19", "20", "21", "22", "23", "24", "25", "26", "27", "28", "29", "30",
		"31", "32", "33", "34", "35", "36", "37", "38", "39", "40",
		"41", "42", "43", "44", "45", "46", "47", "48", "49", "50", "51", "52", "53", "54", "55", "56", "57", "58", "59", "60", "61", "62","63"
	};
	location locations[] = { // lat/long
		{ 45, 0 },{ 330, 0 },{ 150, 30 },{ 120, 45 },{ 345, 45 },{ 0, 60 },{ 465, 60 },{ 60,105 },{ 120, 105 },{ 210, 120 } ,{ 300, 120 },{ 165, 135 },{ 450, 135 },{ 45, 150 },{ 570, 150 },{ 45, 165 },
	{ 315, 180 },{ 90, 210 },{ 555, 210 },{ 180, 225 },{ 120, 255 },{ 375, 255 },{ 240, 27 },{ 300, 285 },{ 360, 285 },{ 45, 300 },{ 540, 300 },{ 270, 330 },{ 225, 345 },{ 345, 345 },{ 555, 345 },
	{ 90, 375 },{ 390, 375 },{ 240, 405 },{ 345, 405 },{ 390, 405 },{ 120, 420 },{ 360, 420 },{ 375, 420 },{ 285, 450 },{ 420, 450 },{ 180, 465 },{ 210, 465 },{ 420, 465 },{ 495, 465 },{ 30, 480 },
	{ 150, 480 },{ 15, 495 },{ 300, 495 },{ 495, 495 },{ 45, 510 },{ 105, 525 },{ 255, 25 },{ 450, 525 },{ 480, 525 },{ 90, 540 },{ 225, 540 },{ 570, 555 },{ 195, 570 },{ 450, 570 },{ 480, 570 },
	{ 195, 585 },{ 450, 585 },{ 585, 585 }
	};
	edge edge_array[] = {
		edge(n0, n5), edge(n0, n3), edge(n0, n7), edge(n0, n2),
		edge(n1, n4), edge(n1, n10), edge(n1, n6), edge(n1, n9),
		edge(n2, n3), edge(n2, n8), edge(n2, n11),
		edge(n3, n8), edge(n3, n7),
		edge(n4, n10), edge(n4, n6), edge(n4, n12),
		edge(n5, n7), edge(n5, n13), edge(n5, n15),
		edge(n6, n12), edge(n6, n14),
		edge(n7, n13),
		edge(n8, n11), edge(n8, n13),
		edge(n9, n11), edge(n9, n10), edge(n9, n19),
		edge(n10, n16),
		edge(n11, n19),
		edge(n12, n14),edge(n12, n18),
		edge(n13, n15),
		edge(n14, n18), edge(n14, n26),
		edge(n15, n17), edge(n15, n20),
		edge(n16, n21), edge(n16, n23), edge(n16, n24),
		edge(n17, n20), edge(n17, n19), edge(n17, n25),
		edge(n18, n26), edge(n18, n30),
		edge(n19, n20),
		edge(n20, n25),
		edge(n21, n24), edge(n21, n23), edge(n21, n29),
		edge(n22, n23), edge(n22, n27), edge(n22, n28), edge(n22, n24),
		edge(n23, n27),
		edge(n24, n29),
		edge(n25, n31), edge(n25, n36),
		edge(n26, n30), edge(n26, n32),
		edge(n27, n28), edge(n27, n29),
		edge(n28, n33), edge(n28, n29),
		edge(n30, n44), edge(n30, n49),
		edge(n31, n36), edge(n31, n45), edge(n31, n46),
		edge(n32, n35), edge(n32, n38), edge(n32, n34),
		edge(n33, n39), edge(n33, n42), edge(n33, n41),
		edge(n34, n37), edge(n34, n38), edge(n34, n35),
		edge(n35, n38), edge(n35, n37),
		edge(n36, n46), edge(n36, n41),
		edge(n37, n38), edge(n37, n40),
		edge(n39, n48), edge(n39, n42), edge(n39, n52),
		edge(n40, n43), edge(n40, n44), edge(n40, n53),
		edge(n41, n42), edge(n41, n46),
		edge(n42, n46),
		edge(n43, n53), edge(n43, n44), edge(n43, n49),
		edge(n44, n49),
		edge(n45, n47), edge(n45, n50), edge(n45, n55),
		edge(n47, n50), edge(n47, n55), edge(n47, n51),
		edge(n48, n52), edge(n48, n56), edge(n48, n58),
		edge(n49, n54),
		edge(n50, n55), edge(n50, n51),
		edge(n51, n55), edge(n51, n58),
		edge(n52, n56), edge(n52, n58),
		edge(n53, n54), edge(n53, n59),
		edge(n54, n60), edge(n54, n59),
		edge(n56, n58), edge(n56, n61),
		edge(n57, n63), edge(n57, n60), edge(n57, n59), edge(n57, n62),
		edge(n59, n62),
		edge(n60, n62), edge(n60, n63),
		edge(n61, n62), edge(n61, n63)

	};
	unsigned int num_edges = sizeof(edge_array) / sizeof(edge);
	cost weights[] = { // estimated travel time (mins)
		75, 87, 106, 109, 47, 123, 147, 169, 33, 80, 106, 60, 84, 87, 120, 138, 75, 100, 114, 76, 138, 47, 54, 87, 47, 90, 109, 61, 91, 120, 129, 15, 61, 152, 63, 117, 96, 106, 114, 54,
		91, 100, 91, 135, 67, 87, 33, 80, 94, 61, 67, 76, 120, 54, 61, 87, 141, 47, 167, 47, 76, 61, 120, 134, 161, 54, 120, 120, 30, 47, 54, 63, 67, 84, 21, 33, 45, 21, 33, 67, 75, 15,
		67, 47, 76, 80, 15, 76, 80, 30, 33, 61, 67, 75, 80, 30, 21, 33, 84, 33, 87, 94, 54, 87, 129, 33, 54, 61, 21, 100, 33, 75, 30, 45, 45, 54, 42, 54, 33, 91, 120, 123, 15, 33, 106, 255, 390
	};


	// create graph
	mygraph_t g(N);
	WeightMap weightmap = get(edge_weight, g);
	for (std::size_t j = 0; j < num_edges; ++j) {
		edge_descriptor e; bool inserted;
		boost::tie(e, inserted) = add_edge(edge_array[j].first,
			edge_array[j].second, g);
		weightmap[e] = weights[j];
	}


	// pick random start/goal
	//  boost::mt19937 gen(time(0));
	vertex start = n0;
	vertex goal = n63;

	cout << "Start vertex: " << numbers[start] << endl;
	cout << "Goal vertex: " << numbers[goal] << endl;



	vector<mygraph_t::vertex_descriptor> p(num_vertices(g));
	vector<cost> d(num_vertices(g));
	try {
		// call astar named parameter interface
		astar_search_tree
		(g, start,
			distance_heuristic<mygraph_t, cost, location*>
			(locations, goal),
			predecessor_map(make_iterator_property_map(p.begin(), get(vertex_index, g))).
			distance_map(make_iterator_property_map(d.begin(), get(vertex_index, g))).
			visitor(astar_goal_visitor<vertex>(goal)));


	}
	catch (found_goal fg) { // found a path to the goal
		list<vertex> shortest_path;
		for (vertex v = goal;; v = p[v]) {
			shortest_path.push_front(v);
			if (p[v] == v)
				break;
		}
		cout << "Shortest path from " << numbers[start] << " to "
			<< numbers[goal] << ": ";
		list<vertex>::iterator spi = shortest_path.begin();
		cout << numbers[start];
		for (++spi; spi != shortest_path.end(); ++spi)
			cout << " -> " << numbers[*spi];
		cout << endl << "Total travel time: " << d[goal] << endl;
		return 0;
	}

	cout << "Didn't find a path from " << numbers[start] << "to"
		<< numbers[goal] << "!" << endl;
	return 0;

}
