//Author: Sivert Andresen Cubedo

#pragma once

#ifndef SimpleClipper_HEADER
#define SimpleClipper_HEADER

#include <memory>
#include <vector>
#include <list>
#include <unordered_map>
#include <algorithm>

#include <wykobi.hpp>

namespace SimpleClipper {
	//Forward
	//struct Graph;
	//struct Graph::Node;

	/*
	Graph container
	*/
	struct Graph {
		struct Node {
			std::vector<Node*> edges;				//out edges
			std::vector<Node*> in_edges;			//in edges
			wykobi::point2d<float> point;			//point
			bool is_subject_path = false;			//is node part of subject_path
			bool is_clip_path = false;				//is node part of clip_path
			bool visited = false;					//visited
			void addEdge(Node* e);					//add edge to node
			void removeEdge(Node* e);				//remove edge from node
		};
		std::vector<std::unique_ptr<Node>> nodes;	//node container
		typedef std::vector<Node*> Path;
		Path subject_path;							//subject path
		Path clip_path;								//clip path
		Graph();
		Graph(const Graph & graph);					//copy constuctor	GIVES ME WIERD RESULTS :'(
		Graph(wykobi::polygon<float, 2> subject_poly, wykobi::polygon<float, 2> clip_polygon);	//make graph from a subject_poly and a clip_poly
		Node* makeNode(wykobi::point2d<float> & p);	//make node in graph
		void removeNode(Node* n);					//delete node (and all related out/in edges)
		void clearVisited();						//set all visited vars to false
		void clearEdges();							//remove all edges from nodes
	};

	/*
	Container for path of nodes
	*/
	typedef std::vector<Graph::Node*> Path;

	/*
	Perpendicular or someshit?!?!?
	http://mathworld.wolfram.com/PerpendicularVector.html
	*/
	wykobi::vector2d<float> perp(wykobi::vector2d<float> & vector);
	
	/*
	Check if poly1 and poly2 are overlapping
	*/
	bool isPolygonOverlapping(wykobi::polygon<float, 2> & poly1, wykobi::polygon<float, 2> & poly2);

	/*
	Check if inner is inside outer
	*/
	bool isPolygonInsidePolygon(wykobi::polygon<float, 2> & inner, wykobi::polygon<float, 2> & outer);

	/*
	Remove hull from path
	*/
	std::vector<Path> removeHull(Path & path, Path & hull);
	std::vector<Path> removeHull(Path & path, std::vector<Path> & hull_vec);
	std::vector<wykobi::polygon<float, 2>> removeHull(wykobi::polygon<float, 2> & poly, wykobi::polygon<float, 2> & hull);
	std::vector<wykobi::polygon<float, 2>> removeHull(wykobi::polygon<float, 2> & path, std::vector<wykobi::polygon<float, 2>> & hull_vec);

	/*
	Find hull
	*/
	std::vector<Path> findHull(Graph & graph);

	/*
	Traverse outside of graph
	*/
	Path traverseUnion(Graph & graph);

	/*
	Return wykobi polygon of path
	*/
	wykobi::polygon<float, 2> pathToPolygon(Path & path);

	/*
	Return path with nodes inserted
	*/
	Path insertNodesOnPath(Path & path, std::vector<std::vector<Graph::Node*>> & nodes);

	/*
	Insert path to graph
	*/
	void insertPathToGraph(Path & path);

	/*
	Clip away clip_path and return paths of remaining paths
	*/
	std::vector<Path> clipDifference(Graph & graph);

	/*
	Return paths from intersections
	*/
	std::vector<Path> clipIntersection(Graph & graph);

	/*
	Clip intersection
	*/
	std::vector<wykobi::polygon<float, 2>> clipIntersection(wykobi::polygon<float, 2> & subject_poly, wykobi::polygon<float, 2> & clip_poly);

	/*
	Clip diffrence
	*/
	std::vector<wykobi::polygon<float, 2>> clipDifference(wykobi::polygon<float, 2> & subject_poly, wykobi::polygon<float, 2> & clip_poly);

	/*
	Clip union
	*/
	std::vector<wykobi::polygon<float, 2>> clipUnion(wykobi::polygon<float, 2> & subject_poly, wykobi::polygon<float, 2> & clip_poly);

	/*
	Get most clockwise node
	*/
	Graph::Node* getClockwiseMost(Graph::Node* prev_node, Graph::Node* current_node);

	/*
	Get most counterclockwise node
	*/
	Graph::Node* getCounterClockwiseMost(Graph::Node* prev_node, Graph::Node* current_node);

}


#endif // !SimpleClipper_HEADER
