//Author: Sivert Andresen Cubedo

#include <iostream>

#include <wykobi.hpp>

#include "SimpleClipper.hpp"

template <typename T, size_t D>
std::ostream& writePolygonAsPythonList(std::ostream& os, const wykobi::polygon<T, D>& polygon) {
    os << "[";
    for (auto& point : polygon) {
        os << "(" << point.x << ", " << point.y << "), ";
    }
    os << "]";
    return os;
}

template <typename T, size_t D>
std::ostream& writePolygonsAsPythonList(std::ostream& os, const std::vector<wykobi::polygon<T, D>>& polygons) {
    os << "[";
    for (auto& polygon : polygons) {
        writePolygonAsPythonList(os, polygon);
        os << ", ";
    }
    os << "]";
    return os;
}

int main() {

    wykobi::polygon<float, 2> p1 = wykobi::make_polygon<float>(wykobi::make_triangle<float>(0, 0, 100, 0, 100, 100));
    wykobi::polygon<float, 2> p2 = wykobi::make_polygon<float>(wykobi::make_rectangle<float>(0, 0, 100, 100));



    auto polygons = SimpleClipper::clipUnion(p1, p2);

    writePolygonsAsPythonList(std::cout, polygons);

    return EXIT_SUCCESS;
}

