//Author: Sivert Andresen Cubedo

#include <iostream>

#include <wykobi.hpp>

#include "SimpleClipper.hpp"

int main() {

    wykobi::polygon<float, 2> p1 = wykobi::make_polygon<float>(wykobi::make_triangle<float>(0, 0, 100, 0, 100, 100));
    wykobi::polygon<float, 2> p2 = wykobi::make_polygon<float>(wykobi::make_rectangle<float>(0, 0, 100, 100));



    auto polygons = SimpleClipper::clipUnion(p1, p2);



    std::cout << polygons.size() << "\n";
    std::cout << polygons.front().size() << "\n";



    return EXIT_SUCCESS;
}

