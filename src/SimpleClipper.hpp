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
    /*
    Node
    */
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

    /*
    Path container
    */
    struct Path : public std::vector<Node*> {
        std::vector<Path> hulls;	//vector containing paths to hulls
        std::pair<Node*, Node*> edge(std::size_t index);	//return 2 nodes makeing a segment at index
    };

    /*
    Graph container
    */
    struct Graph {
        std::vector<std::unique_ptr<Node>> nodes;	//node container
        Path subject_path;							//subject path
        Path clip_path;								//clip path
        Graph();									//empty constructor
        Graph(const Graph & graph);					//copy constuctor	GIVES ME WIERD RESULTS :'(
        Graph(wykobi::polygon<float, 2> subject_poly, wykobi::polygon<float, 2> clip_polygon);	//make graph from a subject_poly and a clip_poly
        Node* makeNode(wykobi::point2d<float> & p);	//make node in graph
        void removeNode(Node* n);					//delete node (and all related out/in edges)
        void clearVisited();						//set all visited vars to false
        void clearEdges();							//remove all edges from nodes
    };

    /*
    Perpendicular or someshit?!?!?
    http://mathworld.wolfram.com/PerpendicularVector.html
    */
    wykobi::vector2d<float> perp(wykobi::vector2d<float> & vector);

    /*
    Get segment from Path
    */
    wykobi::segment<float, 2> edge(Path & path, std::size_t index);

    /*
    Check if poly1 and poly2 are overlapping
    */
    bool isPolygonOverlapping(wykobi::polygon<float, 2> & poly1, wykobi::polygon<float, 2> & poly2);

    /*
    Check if inner is inside outer
    */
    bool isPolygonInsidePolygon(wykobi::polygon<float, 2> & subject, wykobi::polygon<float, 2> & outer);

    /*
    Check if point in path
    */
    bool isPointInsidePath(wykobi::point2d<float> & point, Path & path);
    
    /*
    Check if Node is inside path
    This cannot be true if node is part of path
    */
    bool isNodeInsidePath(Node* n, Path & path);

    /*
    Check if paths are overlapping
    */
    bool isPathOverlapping(Path & path1, Path & path2);

    /*
    Check if path is inside path.
    This can only be true if subject does not share any node with outer
    */
    bool isPathInsidePath(Path & subject, Path & outer);

    /*
    Remove hull from polygon
    */
    std::vector<wykobi::polygon<float, 2>> removeHull(wykobi::polygon<float, 2> & poly, wykobi::polygon<float, 2> & hull);
    std::vector<wykobi::polygon<float, 2>> removeHull(wykobi::polygon<float, 2> & path, std::vector<wykobi::polygon<float, 2>> & hull_vec);

    /*
    Segment helper func
    */
    bool isSegmentFromPolygonSelfIntersecting(wykobi::polygon<float, 2> & polygon, std::size_t index, wykobi::point2d<float> point);
    bool isSegmentPointsEqual(wykobi::segment<float, 2> & seg1, wykobi::segment<float, 2> & seg2);

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
    Path insertNodesOnPath(Path & path, std::vector<std::vector<Node*>> & nodes);

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
    Node* getClockwiseMost(Node* prev_node, Node* current_node);

    /*
    Get most counterclockwise node
    */
    Node* getCounterClockwiseMost(Node* prev_node, Node* current_node);

}


#endif // !SimpleClipper_HEADER
