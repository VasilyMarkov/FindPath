import matplotlib.pyplot as plt
import numpy as np
import shapely.geometry as sg

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    def __str__(self):
        return f'(x: {self.x}, y: {self.y})'

    def plot(self, plt, *args, **kwargs):
        plt.scatter(self.x, self.y, *args, **kwargs)


class Segment:
    def __init__(self, start: Point, end: Point):
        self.start = start
        self.end = end

    def __str__(self):
        return f'Start: {self.start}, End: {self.end}'

    def plot(self, plt, *args, **kwargs):
        plt.plot([self.start.x, self.end.x], [self.start.y, self.end.y], *args, **kwargs)

class Polygon:
    segments = []
    points = []

    def __init__(self, points: list):
        self.points = points
        for i in range(len(points)):
            self.segments.append(Segment(points[i], points[(i+1) % len(points)]))
        
    
    def __str__(self):
        return f"{', '.join(map(str, self.segments))}"
    
    def plot(self, plt, *args, **kwargs):
        for seg in self.segments:
            seg.plot(plt, *args, **kwargs)


class Grid:
    def __init__(self, field: Polygon, start_point: Point, bearing: float = 0):
        self.fields  = field
        self.start_point  = start_point
        self.bearing = bearing


start = Point(0,0)
end = Point(2,2)

field = Polygon([
    Point(0,0), 
    Point(0,200),
    Point(200,200), 
    Point(200,0)
                ])

fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot()

field.plot(ax, color = 'red')
print(field)
plt.show()