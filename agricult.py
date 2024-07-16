import matplotlib.pyplot as plt
import numpy as np
import shapely.geometry as sg
from shapely.geometry import Polygon, MultiPoint
from shapely.ops import unary_union
from scipy.spatial.distance import pdist, squareform
from shapely.geometry import LineString
from shapely.affinity import rotate, translate, scale
import math
from abc import ABC, abstractmethod
from typing import TypeVar, Type

class Plotable(ABC):
    @abstractmethod
    def plot(self, plt, *args, **kwargs):
        pass


class Point(Plotable):
    def __init__(self, x, y):
        self.x = x
        self.y = y
    def __str__(self):
        return f'(x: {self.x}, y: {self.y})'

    def plot(self, plt, *args, **kwargs):
        plt.scatter(self.x, self.y, *args, **kwargs)

    def raw(self) -> list:
        return [self.x, self.y]


class Segment(Plotable):
    def __init__(self, start: Point, end: Point):
        self.start = start
        self.end = end

    def __str__(self):
        return f'Start: {self.start}, End: {self.end}'

    def plot(self, plt, *args, **kwargs):
        plt.plot([self.start.x, self.end.x], [self.start.y, self.end.y], *args, **kwargs)


class Shape(Plotable):
    segments = []
    points = []
    diameter = 0
    def __init__(self, points: list):
        self.points = points
        self.min_x = np.min(np.array([point.raw() for point in points])[:, 0])
        self.max_x = np.max(np.array([point.raw() for point in points])[:, 0])
        self.min_y = np.min(np.array([point.raw() for point in points])[:, 1])
        self.max_y = np.max(np.array([point.raw() for point in points])[:, 1])

        self.diameter = self.__find_diameter()

        for i in range(len(points)):
            self.segments.append(Segment(points[i], points[(i+1) % len(points)]))  
    
    def __find_diameter(self):
        inner_points = [point.raw() for point in self.points]
        multipoint = MultiPoint(inner_points)
        convex_hull = multipoint.convex_hull
        hull_coords = list(convex_hull.exterior.coords)
        distances = pdist(hull_coords)
        distances = squareform(distances)
        max_index = np.unravel_index(np.argmax(distances, axis=None), distances.shape)
        return distances[max_index[0]][max_index[1]] #distance between farthests points


    def raw(self):
        return [[point.x, point.y] for point in self.points]

    def __str__(self):
        return f"{', '.join(map(str, self.segments))}"
    
    def plot(self, plt, *args, **kwargs):
        for seg in self.segments:
            seg.plot(plt, *args, **kwargs)


class Field(Shape):
    def __init__(self, points: list, widthOfOverlap: float):
        Shape.__init__(self, points)
        self.widthOfOverlap = widthOfOverlap
        self.__innerField() 

    def __innerField(self):
        points = [point.raw() for point in self.points]
        poly = sg.Polygon(points)
        inner = poly.buffer(-self.widthOfOverlap, join_style=2)
        inner_points = [Point(coord[0], coord[1]) for coord in list(inner.boundary.coords)]
        self.inner_field = Shape(inner_points)


class Grid(Plotable):
    intersect_points = []
    
    def __init__(self, field: Field, start_point: Point, bearing: float = 0):
        self.field  = field
        self.start_point  = start_point
        self.bearing = bearing
        self.__generate()

    def __generate(self):
        base_line = LineString([(0, 0), (0, 1)])
        rotated_line = rotate(base_line, self.bearing, use_radians=True)
        rotated_line_offset = np.array([rotated_line.xy[0][0], rotated_line.xy[1][0]]) - np.array(self.start_point.raw())
        rotated_line = translate(rotated_line, -rotated_line_offset[0], -rotated_line_offset[1])
        factor = 5*self.field.diameter/rotated_line.length, 5*self.field.diameter/rotated_line.length
        rotated_line = scale(rotated_line, xfact=factor[0], yfact=factor[1])

        lines = []
        num_lines = int(field.diameter/field.widthOfOverlap)
        for i in range(num_lines):
            dx = field.widthOfOverlap * i * math.cos(self.bearing)
            dy = field.widthOfOverlap * i * math.sin(self.bearing)
            translated_line = translate(rotated_line, dx, dy)
            lines.append(translated_line)
        self.lines = lines
        self.__intersection(np.array(self.field.raw()), lines[0])
        intersects = []
        for line in lines:
            intersects.append(self.__intersection(np.array(self.field.raw()), line))
        intersects = [point for point in intersects if len(point) == 2]
        self.intersect_points = intersects

    def __intersection(self, polygon, line):
        polygon = Polygon(polygon)
        intersection_points = []
        intersection = polygon.intersection(line)
        intersection_points.append(intersection)
        points = list(intersection.coords)
        return points

    def plot(self, plt, *args, **kwargs):
        # for i in range(len(self.lines)):
        #     seg = Segment(Point(self.lines[i].xy[0][0], self.lines[i].xy[1][0]), Point(self.lines[i].xy[0][1], self.lines[i].xy[1][1]))
        #     seg.plot(plt, *args, **kwargs)
        assert(len(self.intersect_points) > 0)
        for point in self.intersect_points:
            plt.scatter(point[0][0], point[0][1], *args, **kwargs)
            plt.scatter(point[1][0], point[1][1], *args, **kwargs)

start = Point(0,0)
end = Point(2,2)

field = Field([
    Point(0,0), 
    Point(0,200),
    Point(200,200), 
    Point(200,0)
            ], 10)

grid = Grid(field.inner_field, Point(15, 10), np.deg2rad(45))

fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot()
field.plot(ax, color = 'red')
grid.plot(ax, color = 'blue')
# field.innerField.plot(ax, color = 'blue')
plt.show()