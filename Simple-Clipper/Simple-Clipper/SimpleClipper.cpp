//Author: Sivert Andresen Cubedo

#include "SimpleClipper.hpp"

SimpleClipper::Graph::Graph() {

}
SimpleClipper::Graph::Graph(const Graph & graph) {
	//copy nodes
	std::unordered_map<Node*, Node*> node_map;
	//node_map.reserve(graph.nodes.size());
	for (auto & ptr : graph.nodes) {
		Node* origin_node = ptr.get();
		Node* clone_node = this->makeNode(origin_node->point);
		clone_node->is_subject_path = origin_node->is_subject_path;
		clone_node->is_clip_path = origin_node->is_clip_path;
		node_map.emplace(std::pair<Node*, Node*>(origin_node, clone_node));
	}
	//copy edges
	//for (auto & pair : node_map) {
	for (auto & ptr : graph.nodes) {
		//Node* origin_node = pair.first;
		Node* origin_node = ptr.get();
		//Node* clone_node = pair.second;
		Node* clone_node = node_map.find(origin_node)->second;
		for (Node* origin_edge : origin_node->edges) {
			Node* clone_edge = node_map.find(origin_edge)->second;
			clone_node->addEdge(clone_edge);
		}
	}
	//copy paths
	for (Node* origin_node : graph.subject_path) {
		Node* clone_node = node_map.find(origin_node)->second;
		this->subject_path.push_back(clone_node);
	}
	for (Node* origin_node : graph.clip_path) {
		Node* clone_node = node_map.find(origin_node)->second;
		this->clip_path.push_back(clone_node);
	}
}

SimpleClipper::Graph::Graph(wykobi::polygon<float, 2> subject_poly, wykobi::polygon<float, 2> clip_poly) {
	//make sure polys has same orientation
	if (wykobi::polygon_orientation(subject_poly) != wykobi::CounterClockwise) {
		subject_poly.reverse();
	}
	if (wykobi::polygon_orientation(clip_poly) != wykobi::CounterClockwise) {
		clip_poly.reverse();
	}
	Path poly1_path;
	Path poly2_path;
	for (std::size_t i = 0; i < subject_poly.size(); ++i) {
		poly1_path.push_back(this->makeNode(subject_poly[i]));
	}
	for (std::size_t i = 0; i < clip_poly.size(); ++i) {
		auto it = std::find_if(poly1_path.begin(), poly1_path.end(), [&](Graph::Node* n1) { return n1->point == clip_poly[i]; });
		if (it != poly1_path.end()) {
			poly2_path.push_back((*it));
		}
		else {
			poly2_path.push_back(this->makeNode(clip_poly[i]));
		}
	}
	std::vector<std::vector<Graph::Node*>> poly1_intersections(poly1_path.size());
	std::vector<std::vector<Graph::Node*>> poly2_intersections(poly2_path.size());
	//find
	for (std::size_t i = 0; i < poly1_path.size(); ++i) {
		Graph::Node* outer_node_a = poly1_path[i];
		Graph::Node* outer_node_b = (i + 1 < poly1_path.size()) ? poly1_path[i + 1] : poly1_path[0];
		wykobi::segment<float, 2> outer_segment = wykobi::make_segment(outer_node_a->point, outer_node_b->point);
		for (std::size_t j = 0; j < poly2_path.size(); ++j) {
			Graph::Node* inner_node_a = poly2_path[j];
			Graph::Node* inner_node_b = (j + 1 < poly2_path.size()) ? poly2_path[j + 1] : poly2_path[0];
			wykobi::segment<float, 2> inner_segment = wykobi::make_segment(inner_node_a->point, inner_node_b->point);
			if (wykobi::intersect(outer_segment, inner_segment)) {
				wykobi::point2d<float> intersection = wykobi::intersection_point(outer_segment, inner_segment);
				if (
					intersection == outer_node_a->point ||
					intersection == outer_node_b->point ||
					intersection == inner_node_a->point ||
					intersection == inner_node_b->point
					)
				{
					//???
				}
				else {
					//add to intersecton vectors
					Graph::Node* n = this->makeNode(intersection);
					poly1_intersections[i].push_back(n);
					poly2_intersections[j].push_back(n);
				}
			}
		}
	}
	//insert intersections
	poly1_path = insertNodesOnPath(poly1_path, poly1_intersections);
	poly2_path = insertNodesOnPath(poly2_path, poly2_intersections);
	//insert potential nodes that are "on" existing edges
	std::vector<std::vector<Graph::Node*>> poly1_on_edge(poly1_path.size());
	std::vector<std::vector<Graph::Node*>> poly2_on_edge(poly2_path.size());
	for (std::size_t i = 0; i < poly1_path.size(); ++i) {
		Graph::Node* outer_node_a = poly1_path[i];
		Graph::Node* outer_node_b = (i + 1 < poly1_path.size()) ? poly1_path[i + 1] : poly1_path[0];
		wykobi::segment<float, 2> outer_segment = wykobi::make_segment(outer_node_a->point, outer_node_b->point);
		for (std::size_t j = 0; j < poly2_path.size(); ++j) {
			Graph::Node* inner_node = poly2_path[j];
			if ((inner_node != outer_node_a) && (inner_node != outer_node_b) && wykobi::point_on_segment(inner_node->point, outer_segment)) {
				poly1_on_edge[i].push_back(inner_node);
			}
		}
	}
	for (std::size_t i = 0; i < poly2_path.size(); ++i) {
		Graph::Node* outer_node_a = poly2_path[i];
		Graph::Node* outer_node_b = (i + 1 < poly2_path.size()) ? poly2_path[i + 1] : poly2_path[0];
		wykobi::segment<float, 2> outer_segment = wykobi::make_segment(outer_node_a->point, outer_node_b->point);
		for (std::size_t j = 0; j < poly1_path.size(); ++j) {
			Graph::Node* inner_node = poly1_path[j];
			if ((inner_node != outer_node_a) && (inner_node != outer_node_b) && wykobi::point_on_segment(inner_node->point, outer_segment)) {
				poly2_on_edge[i].push_back(inner_node);
			}
		}
	}
	//insert points on edges
	poly1_path = insertNodesOnPath(poly1_path, poly1_on_edge);
	poly2_path = insertNodesOnPath(poly2_path, poly2_on_edge);
	//add edges to graph
	insertPathToGraph(poly1_path);
	insertPathToGraph(poly2_path);
	this->subject_path = poly1_path;
	this->clip_path = poly2_path;
	//mark nodes after what path they're part of
	for (Graph::Node* n : this->subject_path) {
		n->is_subject_path = true;
	}
	for (Graph::Node* n : this->clip_path) {
		n->is_clip_path = true;
	}
}

SimpleClipper::Graph::Node* SimpleClipper::Graph::makeNode(wykobi::point2d<float> & p) {
	Node n;
	n.point = p;
	nodes.push_back(std::unique_ptr<Node>(new Node(n)));
	return nodes.back().get();
}

void SimpleClipper::Graph::removeNode(Node* n) {
	for (Node* e : n->edges) {
		n->removeEdge(e);
	}
	for (Node* i_e : n->in_edges) {
		i_e->removeEdge(n);
	}
	auto it = std::find_if(nodes.begin(), nodes.end(), [&](std::unique_ptr<Node> & ptr) {return ptr.get() == n; });
	nodes.erase(it);
}

void SimpleClipper::Graph::clearVisited() {
	for (auto & n : nodes) {
		n->visited = false;
	}
}

void SimpleClipper::Graph::clearEdges() {
	for (auto & n : nodes) {
		for (Graph::Node* e : n->edges) {
			n->removeEdge(e);
		}
	}
}

void SimpleClipper::Graph::Node::removeEdge(Node* e) {
	auto it = std::find(edges.begin(), edges.end(), e);
	if (it != edges.end()) {
		edges.erase(it);
		e->in_edges.erase(std::find(e->in_edges.begin(), e->in_edges.end(), this));
	}
}

void SimpleClipper::Graph::Node::addEdge(Node* e) {
	if (std::find(edges.begin(), edges.end(), e) == edges.end()) {
		edges.push_back(e);
		e->in_edges.push_back(this);
	}
}


SimpleClipper::Path SimpleClipper::insertNodesOnPath(Path & path, std::vector<std::vector<Graph::Node*>> & nodes) {
	Path out_path;
	for (std::size_t i = 0; i < path.size(); ++i) {
		out_path.push_back(path[i]);
		if (!nodes[i].empty()) {
			if (nodes[i].size() > 1) {
				std::sort(nodes[i].begin(), nodes[i].end(), [&](Graph::Node* n1, Graph::Node* n2)
				{ return (wykobi::distance(path[i]->point, n1->point) < wykobi::distance(path[i]->point, n2->point)); }
				);
			}
			for (Graph::Node* n : nodes[i]) {
				out_path.push_back(n);
			}
		}
	}
	return out_path;
}

void SimpleClipper::insertPathToGraph(Path & path) {
	Path::iterator it = path.begin();
	while (it != path.end()) {
		Graph::Node* outer_node_a = (*it);
		Graph::Node* outer_node_b = (std::next(it) != path.end()) ? (*std::next(it)) : (*path.begin());
		outer_node_a->addEdge(outer_node_b);
		++it;
	}
}

std::vector<SimpleClipper::Path> SimpleClipper::findHull(Graph & graph) {
	//find all nodes with more then one edge
	std::vector<Graph::Node*> outer_nodes = traverseUnion(graph);
	std::vector<Graph::Node*> pending_nodes;
	for (std::unique_ptr<Graph::Node> & n : graph.nodes) {
		if (n->edges.size() > 1 && std::find(outer_nodes.begin(), outer_nodes.end(), n.get()) == outer_nodes.end()) {
			pending_nodes.push_back(n.get());
		}
	}
	std::vector<std::vector<Graph::Node*>> hull_vec;
	//for each edge of start node, go most clockwise until start node is reached (then we found hull), or until an already visited node is reached (then no hull)
	for (Graph::Node* start_node : pending_nodes) {
		if (start_node->visited) continue;
		for (Graph::Node* current_branch : start_node->edges) {
			Path current_path;
			Graph::Node* last_node = start_node;
			Graph::Node* current_node = current_branch;
			current_path.push_back(start_node);
			while (true) {
				current_path.push_back(current_node);
				current_node->visited = true;
				Graph::Node* next_node = getClockwiseMost(last_node, current_node);
				if (next_node->visited) {
					for (Graph::Node* n : current_path) {
						n->visited = false;
					}
					break;
				}
				else if (next_node == start_node) {
					//for (Graph::Node* n : current_path) {
					//
					//}
					hull_vec.push_back(current_path);
					break;
				}
				last_node = current_node;
				current_node = next_node;
			}
		}
	}
	graph.clearVisited();
	return hull_vec;
}

SimpleClipper::Path SimpleClipper::traverseUnion(Graph & graph) {
	Path node_path;
	Graph::Node* current_node = nullptr;
	Graph::Node* next_node = nullptr;
	Graph::Node* start_node = nullptr;
	Graph::Node* prev_node = nullptr;
	wykobi::point2d<float> edge_point = wykobi::make_point<float>(wykobi::infinity<float>(), wykobi::infinity<float>());
	float low_dist = wykobi::infinity<float>();
	for (std::size_t i = 0; i < graph.nodes.size(); ++i) {
		Graph::Node* n = graph.nodes[i].get();
		if (n->point.x < edge_point.x) edge_point.x = n->point.x;
		if (n->point.y < edge_point.y) edge_point.y = n->point.y;
	}
	for (std::size_t i = 0; i < graph.nodes.size(); ++i) {
		Graph::Node* n = graph.nodes[i].get();
		float test_dist = wykobi::distance<float>(edge_point, n->point);
		if (test_dist < low_dist) {
			low_dist = test_dist;
			start_node = n;
		}
	}
	Graph::Node fake_node;
	fake_node.point = start_node->point - wykobi::make_vector<float>(1.f, 0.f);
	current_node = getClockwiseMost(&fake_node, start_node);
	prev_node = start_node;
	node_path.push_back(start_node);
	while (current_node != start_node) {
		node_path.push_back(current_node);
		next_node = getClockwiseMost(prev_node, current_node);
		prev_node = current_node;
		current_node = next_node;
	}
	return node_path;
}

SimpleClipper::Graph::Node* SimpleClipper::getClockwiseMost(Graph::Node* prev_node, Graph::Node* current_node) {
	std::vector<Graph::Node*> edges = current_node->edges;
	auto it = std::find(edges.begin(), edges.end(), prev_node);
	if (it != edges.end()) edges.erase(it);
	if (edges.empty()) return nullptr;
	it = edges.begin();
	wykobi::vector2d<float> current_dir = current_node->point - prev_node->point;
	Graph::Node* next_node = (*it);
	++it;
	wykobi::vector2d<float> next_dir = next_node->point - current_node->point;
	bool is_current_convex = (wykobi::dot_product(next_dir, perp(current_dir)) <= 0);
	while (it != edges.end()) {
		Graph::Node* test_node = (*it);
		wykobi::vector2d<float> test_dir = test_node->point - current_node->point;
		if (is_current_convex) {
			if (wykobi::dot_product(current_dir, perp(test_dir)) < 0 || wykobi::dot_product(next_dir, perp(test_dir)) < 0) {
				next_dir = test_dir;
				next_node = test_node;
				is_current_convex = (wykobi::dot_product(next_dir, perp(current_dir)) <= 0);
			}
		}
		else {
			if (wykobi::dot_product(current_dir, perp(test_dir)) < 0 && wykobi::dot_product(next_dir, perp(test_dir)) < 0) {
				next_dir = test_dir;
				next_node = test_node;
				is_current_convex = (wykobi::dot_product(next_dir, perp(current_dir)) <= 0);
			}
		}
		++it;
	}
	return next_node;
}

SimpleClipper::Graph::Node* SimpleClipper::getCounterClockwiseMost(Graph::Node* prev_node, Graph::Node* current_node) {
	std::vector<Graph::Node*> edges = current_node->edges;
	auto it = std::find(edges.begin(), edges.end(), prev_node);
	if (it != edges.end()) edges.erase(it);
	if (edges.empty()) return nullptr;
	it = edges.begin();
	wykobi::vector2d<float> current_dir = current_node->point - prev_node->point;
	Graph::Node* next_node = (*it);
	++it;
	wykobi::vector2d<float> next_dir = next_node->point - current_node->point;
	bool is_current_convex = (wykobi::dot_product(next_dir, perp(current_dir)) <= 0);
	while (it != edges.end()) {
		Graph::Node* test_node = (*it);
		wykobi::vector2d<float> test_dir = test_node->point - current_node->point;
		if (is_current_convex) {
			if (wykobi::dot_product(current_dir, perp(test_dir)) > 0 && wykobi::dot_product(next_dir, perp(test_dir)) > 0) {
				next_dir = test_dir;
				next_node = test_node;
				is_current_convex = (wykobi::dot_product(next_dir, perp(current_dir)) <= 0);
			}
		}
		else {
			if (wykobi::dot_product(current_dir, perp(test_dir)) > 0 || wykobi::dot_product(next_dir, perp(test_dir)) > 0) {
				next_dir = test_dir;
				next_node = test_node;
				is_current_convex = (wykobi::dot_product(next_dir, perp(current_dir)) <= 0);
			}
		}
		++it;
	}
	return next_node;
}

wykobi::vector2d<float> SimpleClipper::perp(wykobi::vector2d<float> & vector) {
	return wykobi::make_vector<float>(vector.y, -vector.x);
}

std::vector<SimpleClipper::Path> SimpleClipper::clipDifference(Graph & graph) {
	Path & subject_path = graph.subject_path;
	Path & clip_path = graph.clip_path;
	wykobi::polygon<float, 2> subject_path_poly = pathToPolygon(subject_path);
	wykobi::polygon<float, 2> clip_path_poly = pathToPolygon(clip_path);
	//remove nodes from subject_path that are inside clip_path, but not part of clip_path
	auto it = subject_path.begin();
	while (it != subject_path.end()) {
		Graph::Node* n = (*it);
		if (!n->is_clip_path && wykobi::point_in_polygon(n->point, clip_path_poly)) {
			it = subject_path.erase(it);
			graph.removeNode(n);
		}
		else {
			++it;
		}
	}
	//remove nodes from clip_path that are not inside subject_path, and are not part of subject_path
	it = clip_path.begin();
	while (it != clip_path.end()) {
		Graph::Node* n = (*it);
		if (!n->is_subject_path && !wykobi::point_in_polygon(n->point, subject_path_poly)) {
			it = clip_path.erase(it);
			graph.removeNode(n);
		}
		else {
			++it;
		}
	}
	//(at this point, subject_path and clip_path are both invalid)
	//create new independent paths
	std::vector<Path> path_vec;
	while (true) {
		//find node with more then 2 out edges, and is not visited
		Graph::Node* start_node = nullptr;
		for (Graph::Node* n : clip_path) {
			if (n->edges.size() > 1 && !n->visited) {
				start_node = n;
				break;
			}
		}
		if (start_node == nullptr) break;
		start_node->visited = true;
		Path search_path;
		search_path.push_back(start_node);
		//search until in_edges.size() > 1 then go in_edges back the other "way"
		Graph::Node* current_node = start_node->edges.front();
		Graph::Node* last_node = start_node;
		Graph::Node* next_node = nullptr;
		while (current_node->in_edges.size() == 1) {
			search_path.push_back(current_node);
			last_node = current_node;
			current_node = current_node->edges.front();
		}
		search_path.push_back(current_node);
		next_node = (current_node->in_edges.front() != last_node) ? current_node->in_edges.front() : current_node->in_edges.back();
		current_node = next_node;
		while (current_node != start_node) {
			search_path.push_back(current_node);
			last_node = current_node;
			current_node = current_node->in_edges.front();
		}
		path_vec.push_back(search_path);
	}
	return path_vec;
}

std::vector<SimpleClipper::Path> SimpleClipper::clipIntersection(Graph & graph) {
	std::vector<Path> out_vec;
	Path & subject_path = graph.subject_path;
	Path & clip_path = graph.clip_path;
	wykobi::polygon<float, 2> subject_path_poly = pathToPolygon(subject_path);
	wykobi::polygon<float, 2> clip_path_poly = pathToPolygon(clip_path);
	//remove nodes in subject_path that are not part of clip_path and are not inside clip_polygon
	auto it = subject_path.begin();
	while (it != subject_path.end()) {
		Graph::Node* n = (*it);
		if (!n->is_clip_path && !wykobi::point_in_polygon(n->point, clip_path_poly)) {
			it = subject_path.erase(it);
			graph.removeNode(n);
		}
		else {
			++it;
		}
	}
	//remove nodes in clip_path that are not part of subject_path and are not inside subject_polygon
	it = clip_path.begin();
	while (it != clip_path.end()) {
		Graph::Node* n = (*it);
		if (!n->is_subject_path && !wykobi::point_in_polygon(n->point, subject_path_poly)) {
			it = clip_path.erase(it);
			graph.removeNode(n);
		}
		else {
			++it;
		}
	}
	//from nodes that has >1 out_edges and 1 in_edge, remove all "outer edges(most counterclockwise or some shit)"
	std::list<Graph::Node*> pending_nodes;
	for (auto & ptr : graph.nodes) {
		if (ptr->edges.size() > 1) {
			pending_nodes.push_back(ptr.get());
		}
	}
	while (!pending_nodes.empty()) {
		Graph::Node* current_node = pending_nodes.front();
		pending_nodes.pop_front();
		if (current_node->in_edges.size() > 1) {
			pending_nodes.push_back(current_node);
		}
		else {
			while (current_node->edges.size() > 1) {
				Graph::Node* e = getClockwiseMost(current_node->in_edges.front(), current_node);
				current_node->removeEdge(e);
			}
		}
	}
	//at this point all nodes should only have 1 out edge, and one in edge
	//traverse all seperate paths (mark visited and stuff)
	while (true) {
		Graph::Node* start_node = nullptr;
		for (auto & ptr : graph.nodes) {
			if (!ptr->visited) {
				start_node = ptr.get();
				break;
			}
		}
		if (start_node == nullptr) break;
		Path current_path;
		Graph::Node* current_node = start_node->edges.front();
		start_node->visited = true;
		current_path.push_back(start_node);
		while (current_node != start_node) {
			current_path.push_back(current_node);
			current_node->visited = true;
			current_node = current_node->edges.front();
		}
		out_vec.push_back(current_path);
	}
	return out_vec;
}

std::vector<wykobi::polygon<float, 2>> SimpleClipper::clipIntersection(wykobi::polygon<float, 2> & subject_poly, wykobi::polygon<float, 2> & clip_poly) {
	std::vector<wykobi::polygon<float, 2>> out_vec;
	if (isPolygonInsidePolygon(subject_poly, clip_poly)) {
		out_vec.push_back(subject_poly);
		return out_vec;
	}
	else if (isPolygonInsidePolygon(clip_poly, subject_poly)) {
		out_vec.push_back(clip_poly);
		return out_vec;
	}
	else if (!isPolygonOverlapping(subject_poly, clip_poly)) {
		return out_vec;
	}
	Graph graph(subject_poly, clip_poly);
	std::vector<std::vector<Graph::Node*>> path_vec = clipIntersection(graph);
	for (auto & p : path_vec) {
		out_vec.push_back(pathToPolygon(p));
	}
	return out_vec;
}

std::vector<wykobi::polygon<float, 2>> SimpleClipper::clipDifference(wykobi::polygon<float, 2> & subject_poly, wykobi::polygon<float, 2> & clip_poly) {
	std::vector<wykobi::polygon<float, 2>> out_vec;
	if (isPolygonInsidePolygon(subject_poly, clip_poly)) {
		return out_vec;
	}
	else if (isPolygonInsidePolygon(clip_poly, subject_poly)) {
		return removeHull(subject_poly, clip_poly);
	}
	else if (!isPolygonOverlapping(subject_poly, clip_poly)) {
		out_vec.push_back(subject_poly);
		return out_vec;
	}
	Graph graph(subject_poly, clip_poly);
	std::vector<std::vector<Graph::Node*>> path_vec = clipDifference(graph);
	for (auto & p : path_vec) {
		out_vec.push_back(pathToPolygon(p));
	}
	return out_vec;
}

std::vector<wykobi::polygon<float, 2>> SimpleClipper::clipUnion(wykobi::polygon<float, 2> & subject_poly, wykobi::polygon<float, 2> & clip_poly) {
	std::vector<wykobi::polygon<float, 2>> poly_vec;
	if (isPolygonInsidePolygon(subject_poly, clip_poly)) {
		poly_vec.push_back(clip_poly);
		return poly_vec;
	}
	else if (isPolygonInsidePolygon(clip_poly, subject_poly)) {
		poly_vec.push_back(subject_poly);
		return poly_vec;
	}
	else if (!isPolygonOverlapping(subject_poly, clip_poly)) {
		poly_vec.push_back(subject_poly);
		poly_vec.push_back(clip_poly);
		return poly_vec;
	}
	//make sure both polygons are clockwise
	if (wykobi::polygon_orientation(subject_poly) != wykobi::CounterClockwise) {
		subject_poly.reverse();
	}
	if (wykobi::polygon_orientation(clip_poly) != wykobi::CounterClockwise) {
		clip_poly.reverse();
	}
	Graph graph(subject_poly, clip_poly);
	Path node_path = traverseUnion(graph);
	std::vector<Path> hull_vec = findHull(graph);
	wykobi::polygon<float, 2> out_poly = pathToPolygon(node_path);
	std::vector<wykobi::polygon<float, 2>> temp_vec;
	if (!hull_vec.empty()) {
		for (auto p : hull_vec) {
			temp_vec.push_back(pathToPolygon(p));
		}
		poly_vec = removeHull(out_poly, temp_vec);
		return poly_vec;
	}
	else {
		poly_vec.push_back(out_poly);
		return poly_vec;
	}
}

wykobi::polygon<float, 2> SimpleClipper::pathToPolygon(Path & path) {
	wykobi::polygon<float, 2> poly;
	poly.reserve(path.size());
	for (Graph::Node* n : path) {
		poly.push_back(n->point);
	}
	return poly;
}

//end