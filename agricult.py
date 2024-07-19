import matplotlib.pyplot as plt
import numpy as np
import shapely.geometry as sg
from scipy.spatial.distance import pdist, squareform
from shapely.geometry import LineString
from shapely.affinity import rotate, translate, scale
import math
from abc import ABC, abstractmethod
from dubins import Dubins
from environment import StaticEnvironment
from rrt import RRT
from rrt_star_dubins import RRTStarDubins
import itertools

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
        multipoint = sg.MultiPoint(inner_points)
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
    def __init__(self, points: list, width):
        Shape.__init__(self, points)
        self.width = width
        # self.__innerField() 


    def __innerField(self):
        points = [point.raw() for point in self.points]
        poly = sg.Polygon(points)
        inner = poly.buffer(-self.width, join_style=2) #create inner bound field
        inner_points = [Point(coord[0], coord[1]) for coord in list(inner.boundary.coords)][:-1]
        # self.inner_field = Shape(inner_points)
        

class Grid(Plotable):
    intersect_points = []

    def __init__(self, field: Field, start: Point, width, bearing: float = 0, fromCenter = False):
        self.field  = field
        self.start_point  = start
        self.width = width
        self.bearing = bearing
        self.fromCenter = fromCenter
        self.__generate()

    def __generate(self):
        base_line = LineString([(0, 0), (0, 1)])
        rotated_line = rotate(base_line, self.bearing, use_radians=True)
        rotated_line_offset = np.array([rotated_line.xy[0][0], rotated_line.xy[1][0]]) - np.array(self.start_point.raw())
        rotated_line = translate(rotated_line, -rotated_line_offset[0], -rotated_line_offset[1])
        factor = 5*self.field.diameter/rotated_line.length, 5*self.field.diameter/rotated_line.length
        rotated_line = scale(rotated_line, xfact=factor[0], yfact=factor[1])

        self.__lineMultiply(rotated_line, self.fromCenter)

        intersects = [self.__intersection(np.array(self.field.raw()), line) for line in self.lines]
        intersects = [point for point in intersects if len(point) == 2]
        
        self.intersect_points = intersects

    def __lineMultiply(self, line, fromCenter):
        lines = []
        if not fromCenter:
            num_lines = int(self.field.diameter/self.width)
            for i in range(num_lines):
                print(i)
                dx = self.width * i * math.cos(self.bearing)
                dy = self.width * i * math.sin(self.bearing)
                translated_line = translate(line, dx, dy)
                lines.append(translated_line)
        else:
            num_lines = int(self.field.diameter/self.width)
            for i in range(num_lines):
                dx = self.width * i * math.cos(self.bearing)
                dy = self.width * i * math.sin(self.bearing)
                translated_line = translate(line, dx, dy)
                lines.append(translated_line)
                if i > 0:
                    dx = self.width * -i * math.cos(self.bearing)
                    dy = self.width * -i * math.sin(self.bearing)
                    translated_line = translate(line, dx, dy)
                    lines.append(translated_line)

        self.lines = lines

    def __intersection(self, polygon, line):
        polygon = sg.Polygon(polygon)
        intersection_points = []
        intersection = polygon.intersection(line)
        intersection_points.append(intersection)
        points = list(intersection.coords)
        return points

    def plot(self, plt, *args, **kwargs):
        assert(len(self.intersect_points) > 0)
        for point in self.intersect_points:
            plt.scatter(point[0][0], point[0][1], *args, **kwargs)
            plt.scatter(point[1][0], point[1][1], *args, **kwargs)

        assert(len(self.lines) > 0)
        for i in range(len(self.lines)):
            x1 = self.lines[i].xy[0][0] 
            y1 = self.lines[i].xy[1][0]
            x2 = self.lines[i].xy[0][1] 
            y2 = self.lines[i].xy[1][1]
            plt.plot([x1, x2], [y1, y2], color = 'b')

class RoundTrajectory:

    def __init__(self, zone: list, start, width, bearing, radius, fromCenter = False):
        
        self.zone = Field([Point(point[0], point[1]) for point in zone], width)
        self.start = Point(start[0], start[1])
        self.width = width
        self.bearing = bearing
        self.fromCenter = fromCenter
        self.radius = radius
        self.__assertions()
        self.grid = Grid(field=self.zone, start=self.start, width=self.width, bearing=np.deg2rad(bearing), fromCenter=self.fromCenter)

    def __assertions(self):
        polygon = sg.Polygon(self.zone.raw())
        point = sg.Point(self.start.raw())
        assert(point.within(polygon)) #Starting point inside the zone?

    def __centerSort(self, lines):
        lines = np.array(lines)
        assert(lines.shape[0] > 0)
        output = np.zeros((lines.shape[0]*2, lines.shape[1]))
        cnt = 0
        for i in range(lines.shape[0]):
            output[cnt] = lines[i][i % 2]
            cnt = cnt + 1 
            output[cnt] = lines[i][(i+1) % 2]
            cnt = cnt + 1 
        return output

    def __outerEndSort(self, lines):
        lines = np.array(lines)
        assert(lines.shape[0] > 0)
        output = np.zeros((lines.shape[0]*2, lines.shape[1]))
        cnt = 0
        size = int(lines.shape[0]/2) if lines.shape[0] % 2 == 0 else int(lines.shape[0]/2)+1
        for i in range(size):
            first = lines[i]
            output[cnt] = first[0]
            cnt = cnt + 1 
            output[cnt] = first[1]
            cnt = cnt + 1 
            if not (lines.shape[0] % 2 == 1 and i == size-1):
                second = lines[lines.shape[0]-i-1]
                output[cnt] = second[1]
                cnt = cnt + 1 
                output[cnt] = second[0]
                cnt = cnt + 1 

        return output

    def generateTrajectory(self):
        
        if self.fromCenter:
            self.sorted_points = self.__centerSort(self.grid.intersect_points)
        else:
            self.sorted_points = self.__outerEndSort(self.grid.intersect_points)

        local_planner = Dubins(self.radius, point_separation=.5)
        # env = StaticEnvironment((250, 250), None, rt.zone.inner_field.raw())
        # rrt = RRT(env)
        paths = []
        cnt = 0
        for i in range(len(self.sorted_points)-1):
            if i%2 != 0:
                start = (self.sorted_points[i][0], self.sorted_points[i][1], 
                         np.deg2rad(np.rad2deg(self.bearing)+270 if cnt % 2 else np.rad2deg(self.bearing)+90))
                end = (self.sorted_points[i+1][0], self.sorted_points[i+1][1], 
                       np.deg2rad(np.rad2deg(self.bearing)+90 if cnt % 2 else np.rad2deg(self.bearing)+270))
                path = local_planner.dubins_path(start, end)
                paths.append(path)
                cnt = cnt + 1
        self.path = paths
        return paths

    def plot(self, plt):
        self.zone.plot(ax, color = 'red')
        for i in range(len(self.sorted_points)-1): #path ploting
            if i % 2 == 0:
                plt.plot([self.sorted_points[i][0], self.sorted_points[i+1][0]], 
                         [self.sorted_points[i][1], self.sorted_points[i+1][1]], color='b')

        for point in self.path: #dubins ploting
            ax.plot(point[:, 0], point[:, 1], color='b')

        for i in range(len(self.sorted_points)-1): #points ploting
            x1 = self.sorted_points[i][0]
            y1 = self.sorted_points[i][1]
            x2 = self.sorted_points[i+1][0]
            y2 = self.sorted_points[i+1][1]
            ax.scatter(x1, y1, color = 'black', linewidths=1)
            ax.scatter(x2, y2, color = 'black', linewidths=1)
            ax.annotate(str(i), (x1, y1))
            ax.annotate(str(i+1), (x2, y2))

        ax.scatter(self.sorted_points[0][0], self.sorted_points[0][1], color = 'g', linewidths=5)
        ax.scatter(self.sorted_points[len(self.sorted_points)-1][0], self.sorted_points[len(self.sorted_points)-1][1], color = 'r', linewidths=5)



points = [[0,0], [0,200], [200,200], [200,0]]

rt = RoundTrajectory(zone=points, start=(100,100), width=40, bearing=0, radius=10, fromCenter=True)

paths = rt.generateTrajectory()


fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot()
rt.plot(ax)
# rt.grid.plot(ax)

plt.show()