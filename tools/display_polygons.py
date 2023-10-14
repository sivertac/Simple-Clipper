#! /bin/python3

import matplotlib.pyplot as plt

def main():
    # read polygons from pipe input
    polygons = []
    while True:
        try:
            # read polygon from pipe input
            line = input()
            # convert string to list
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
        except EOFError:
            break

    # use matplotlib to display polygons
    fig, ax = plt.subplots()
    for polygon in polygons:
        # copy and close polygon
        p = polygon.copy()
        p.append(p[0])
        # plot edges of polygon
        ax.plot(*zip(*p))
        # plot vertices of polygon
        ax.scatter(*zip(*p))

    print(polygons)
    ax.autoscale()
    plt.show()


if __name__ == "__main__":
    main()