#! /bin/python3

import matplotlib.pyplot as plt
import argparse

def parseLineToPolygons(line: str) -> list:
    polygons = eval(line)
    # if polygons is not a list or tuple, error
    if not isinstance(polygons, list) or isinstance(polygons, tuple):
        raise ValueError("polygons is not a list")
    # check all polygons in polygons is a list and has at least 3 vertices
    for polygon in polygons:
        if not isinstance(polygon, list) or isinstance(polygons, tuple):
            raise ValueError("polygon is not a list")
        if len(polygon) < 3:
            raise ValueError("polygon size is less than 3")
    return polygons

def plotPolygons(ax, polygons: list):
    for polygon in polygons:
        # copy and close polygon
        p = polygon.copy()
        p.append(p[0])
        # plot edges of polygon
        ax.plot(*zip(*p))
        # plot vertices of polygon
        ax.scatter(*zip(*p))

def main():
    # parse command line arguments
    parser = argparse.ArgumentParser(description="Display polygons from pipe input")
    args = parser.parse_args()

    ax_polygons = []
    # read polygons from pipe input
    while True:
        try:
            # read polygon from pipe input
            line = input()
            
            # convert string to list
            ax_polygons.append(parseLineToPolygons(line))
        except EOFError:
            break

    # use matplotlib to display polygons
    # create subplot for each polygon collection
    fig, axs = plt.subplots(len(ax_polygons))
    # plot each polygon collection
    # handle if len is 1
    if len(ax_polygons) == 1:
        plotPolygons(axs, ax_polygons[0])
    else:
        for i in range(len(ax_polygons)):
            plotPolygons(axs[i], ax_polygons[i])
    
    # show plot
    plt.show()


if __name__ == "__main__":
    main()