import math
import time
from abc import abstractmethod

import pygame
import pygame.math as pm

class AbstractObject:
    @abstractmethod
    def step(self, timestep):
        pass

objects: list[AbstractObject] = list()
objects_group = pygame.sprite.Group()

def physics():
    timestep = 1 / 60
    for phys_object in objects:
        phys_object.step(timestep)

def nearestPoint(line_a: pm.Vector2, line_b: pm.Vector2, point: pm.Vector2):
    ab = line_b - line_a
    ap = point - line_a
    projection = ap.dot(ab)
    length = ab.length_squared()
    dist = projection / length

    if dist <= 0:
        contact_point = line_a
    elif dist >= 1:
        contact_point = line_b
    else:
        contact_point = ab * dist

    square_distance = (contact_point - point).length_squared()
    return square_distance, contact_point

def approxEqual(a: float, b: float, epsilon: float=0.0001):
    return abs(a - b) < epsilon

class Object(pygame.sprite.Sprite, AbstractObject):
    def __init__(self, position: pm.Vector2, rotation: float, mass: float, restitution: float, color: tuple[int, int, int], vertices: list[pm.Vector2]):
        super().__init__()

        # record object's existence
        objects.append(self)
        self.add(objects_group)

        # set a bunch of default values
        self.position: pm.Vector2 = position
        self.velocity: pm.Vector2 = pm.Vector2()
        self.force: pm.Vector2 = pm.Vector2()
        self.rotation: float = rotation
        self.angular_velocity: float = 0.0
        self.mass: float = mass
        self.restitution = restitution
        self.vertices: list[pm.Vector2] = vertices
        self.translated_vertices: list[pm.Vector2] = list(vertices)
        self.forces: list[tuple[pm.Vector2, pm.Vector2]] = list()

        # get vertices as list of tuple so we can draw it
        points = list()

        # record max x and max y
        max_x = 0
        max_y = 0

        # find the center of mass, assuming constant density
        centroid_x = 0
        centroid_y = 0
        signed_area = 0

        # also find the moment of inertia
        inertia_x = 0
        inertia_y = 0

        for i in range(len(vertices)):
            vertex = vertices[i]

            # append to list of points
            points.append((vertex.x, vertex.y))

            # update maximum x/y
            max_x = max(vertex.x, max_x)
            max_y = max(vertex.y, max_y)

            # signed area = .5 * sum(xi * yi+1 - xi+1 * yi)
            curr_area = vertices[i - 1].x * vertices[i].y - vertices[i].x * vertices[i - 1].y
            signed_area += curr_area

            # centroid = 1/(6A) * sum((x/yi + x/yi+1)(x/yi * x/yi+1 - x/yi+1 * x/yi))
            centroid_x += (vertices[i - 1].x + vertices[i].x) * curr_area
            centroid_y += (vertices[i - 1].y + vertices[i].y) * curr_area

            # moment of inertia = 1/12 * sum((x/yi + x/yi+1)(x/yi^2 + x/yi*x/yi+1 + x/yi+1^2))
            # using gauss shoelace formula
            inertia_x += curr_area * (vertices[i - 1].x ** 2 + vertices[i - 1].x * vertices[i].x + vertices[i].x ** 2)
            inertia_y += curr_area * (vertices[i - 1].y ** 2 + vertices[i - 1].y * vertices[i].y + vertices[i].y ** 2)

        signed_area *= 0.5

        density = mass / signed_area

        centroid_x /= 6 * signed_area
        centroid_y /= 6 * signed_area

        inertia_x /= 12
        inertia_y /= 12
        inertia_x -= signed_area * centroid_y ** 2
        inertia_y -= signed_area * centroid_x ** 2

        self.mass_center: pm.Vector2 = pm.Vector2(centroid_x, centroid_y)
        self.inertia = (inertia_x + inertia_y) * density

        width = max_x
        height = max_y

        self.surface = pygame.Surface((width, height), pygame.SRCALPHA, 32)
        self.image = self.surface
        pygame.draw.polygon(self.image, color, points)
        self.rect = self.image.get_rect(center=self.position)

    def step(self, timestep):
        for force, point in self.forces:
            self.__applyForce(force, point, timestep)
        self.forces.clear()
        self.position += self.velocity * timestep
        self.rotation += self.angular_velocity * timestep

        for i in range(len(self.vertices)):
            vertex = pm.Vector2()
            vertex.x = self.vertices[i].x
            vertex.y = self.vertices[i].y
            vertex.x += 1
            vertex -= self.mass_center
            x = vertex.x * math.cos(self.rotation) - vertex.y * math.sin(self.rotation)
            y = vertex.y * math.cos(self.rotation) + vertex.x * math.sin(self.rotation)
            vertex.x = x
            vertex.y = y
            vertex += self.position
            self.translated_vertices[i] = vertex

    def update(self, *args, **kwargs):
        self.image = pygame.transform.rotate(self.surface, math.degrees(-self.rotation))
        self.rect = self.image.get_rect(center=self.position)

    def __applyForce(self, force: pm.Vector2, point: pm.Vector2, timestep: float):
        self.velocity += force * timestep * (1.0 / self.mass)
        torque = point.x * force.y - point.y * force.x
        self.angular_velocity += (torque / self.inertia) * timestep

    def applyForce(self, force: pm.Vector2, point: pm.Vector2):
        self.forces.append((force, point))

    def applyImpulse(self, force: pm.Vector2, point: pm.Vector2):
        self.__applyForce(force, point, 1.0)

    def getAxes(self):
        axes: list[pm.Vector2] = list()
        for i in range(len(self.translated_vertices)):
            edge = self.translated_vertices[i] - self.translated_vertices[i - 1]
            axes.append(pm.Vector2(-edge.y, edge.x).normalize())
        return axes

    def project(self, axis):
        min_p = axis.dot(self.translated_vertices[0])
        max_p = min_p
        for i in range(1, len(self.translated_vertices)):
            proj = axis.dot(self.translated_vertices[i])
            min_p = min(proj, min_p)
            max_p = max(proj, max_p)
        return pm.Vector2(min_p, max_p)

    def findContactPoints(self, other):
        point_a = pm.Vector2()
        point_b = pm.Vector2()
        num_points = 0
        min_distance = float("inf")

        for i in range(len(self.translated_vertices)):
            point = self.translated_vertices[i]
            for j in range(len(other.translated_vertices)):
                vertex_a = other.translated_vertices[j - 1]
                vertex_b = other.translated_vertices[j]
                distance, contact_point = nearestPoint(vertex_a, vertex_b, point)

                if approxEqual(distance, min_distance):
                    if not approxEqual(contact_point.x, point_a.x) and approxEqual(contact_point.y, point_a.y):
                        num_points = 2
                        point_b = contact_point
                elif distance < min_distance:
                    min_distance = distance
                    num_points = 1
                    point_a = contact_point

        for i in range(len(other.translated_vertices)):
            point = other.translated_vertices[i]
            for j in range(len(self.translated_vertices)):
                vertex_a = self.translated_vertices[j - 1]
                vertex_b = self.translated_vertices[j]
                distance, contact_point = nearestPoint(vertex_a, vertex_b, point)

                if approxEqual(distance, min_distance):
                    if not approxEqual(contact_point.x, point_a.x) and approxEqual(contact_point.y, point_a.y):
                        num_points = 2
                        point_b = contact_point
                elif distance < min_distance:
                    min_distance = distance
                    num_points = 1
                    point_a = contact_point

        if num_points == 1:
            return [point_a]
        elif num_points == 2:
            return [point_a, point_b]
        else:
            return None

    def getCollisionNormal(self, other):
        axes = self.getAxes()
        axes.extend(other.getAxes())
        overlap = float("inf")
        closest_axis = None
        for axis in axes:
            proj_s = self.project(axis)
            proj_o = other.project(axis)
            if not ((proj_s.x < proj_o.x < proj_s.y) or (proj_o.x < proj_s.x < proj_o.y)):
                return None
            else:
                cur_overlap = min(proj_s.y, proj_o.y) - max(proj_s.x, proj_o.x)
                if cur_overlap < overlap:
                    overlap = cur_overlap
                    closest_axis = axis

        return closest_axis

    def collide(self, other):
        normal = self.getCollisionNormal(other)
        if normal is None: return

        contact_points = self.findContactPoints(other)
        if contact_points is None: return

        restitution = min(self.restitution, other.restitution)

        impulses = []
        r_as = []
        r_bs = []

        for point in contact_points:
            r_a = point - self.position
            r_b = point - other.position

            r_as.append(r_a)
            r_bs.append(r_b)

            rp_a = pm.Vector2(-r_a.y, r_a.x)
            rp_b = pm.Vector2(-r_b.y, r_b.x)

            a_velocity = rp_a * self.angular_velocity + self.velocity
            b_velocity = rp_b * other.angular_velocity + other.velocity

            relative_velocity = b_velocity - a_velocity
            contact_speed = relative_velocity.dot(normal)

            if contact_speed > 0.0:
                impulses.append(None)
                continue

            inv_mass_a = 1.0 / self.mass
            inv_mass_b = 1.0 / other.mass

            r_adj_a = (rp_a.dot(normal)) ** 2 / self.inertia
            r_adj_b = (rp_b.dot(normal)) ** 2 / other.inertia

            j = (-(1.0 + restitution) * contact_speed) / (inv_mass_a + inv_mass_b + r_adj_a + r_adj_b)
            j /= len(contact_points)

            impulse = normal * j
            impulses.append(impulse)

        for i in range(len(contact_points)):
            impulse = impulses[i]
            if impulse is None:
                continue
            self.applyImpulse(-impulse, r_as[i])
            other.applyImpulse(impulse, r_bs[i])


def main():
    pygame.init()
    screen = pygame.display.set_mode((800, 600))
    clock = pygame.time.Clock()
    running = True

    pentagon = [
        pm.Vector2(100, 0),
        pm.Vector2(200, 80),
        pm.Vector2(140, 200),
        pm.Vector2(60, 200),
        pm.Vector2(0, 80)
    ]

    stick = [
        pm.Vector2(0, 0),
        pm.Vector2(50, 0),
        pm.Vector2(50, 300),
        pm.Vector2(0, 300)
    ]

    _object = Object(pm.Vector2(0, 0), 0, 5, 0.5, (255, 50, 50), stick)
    _object.update()

    ob2 = Object(pm.Vector2(300, 300), 0, 1, 0.5, (255, 50, 50), pentagon)

    floor_pos = pm.Vector2(100, 100)
    floor = Object(floor_pos, 0, 1, 1, (100, 100, 100), stick)

    _object.applyImpulse(pm.Vector2(250, 250), pm.Vector2(200, 0))
    _object.applyImpulse(pm.Vector2(-10, -10), pm.Vector2(0, 0))

    start_time = time.perf_counter()

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        #_object.applyForce(pm.Vector2(0, 98), pm.Vector2(0, 0))

        #if time.perf_counter() < start_time + 1:
        #    _object.applyForce(pm.Vector2(100, 10), pm.Vector2(1000, 0))

        screen.fill((255, 255, 255))
        objects_group.update()
        objects_group.draw(screen)

        ob2.collide(_object)
        _object.collide(floor)
        floor.collide(ob2)

        floor.position = floor_pos
        floor.velocity = pm.Vector2()
        floor.angular_velocity = 0

        physics()

        pygame.display.flip()
        clock.tick(60)

if __name__ == '__main__':
    main()