//Author: Sivert Andresen Cubedo

#include <iostream>

#include <wykobi.hpp>

#include "SimpleClipper.hpp"

int main() {

	wykobi::polygon<float, 2> p1 = wykobi::make_polygon<float>(wykobi::make_triangle<float>(0, 0, 100, 0, 100, 100));
	wykobi::polygon<float, 2> p2 = wykobi::make_polygon<float>(wykobi::make_rectangle<float>(0, 0, 100, 100));



	SimpleClipper::Graph graph(p1, p2);

	std::cout << graph.nodes.size();
	



	return EXIT_SUCCESS;
}

