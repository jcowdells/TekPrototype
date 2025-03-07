import math
import time
from abc import abstractmethod

import pygame
import pygame.math as pm

# abstract physics object with just a step method
class AbstractObject:
    @abstractmethod
    def step(self, timestep):
        pass

# lists of objects, and their object group
objects: list[AbstractObject] = list()
objects_group = pygame.sprite.Group()

# method to run physics step
def physics():
    timestep = 1 / 60
    for phys_object in objects:
        phys_object.step(timestep)

# method to find nearest point on a line segment
def nearestPoint(line_a: pm.Vector2, line_b: pm.Vector2, point: pm.Vector2):
    # B
    # X==P
    # | /
    # |/
    # A

    # given A, B and P, find X

    # get vectors AB and AP
    ab = line_b - line_a
    ap = point - line_a
    # project AP onto AB
    projection = ap.dot(ab)
    length = ab.length_squared()

    # find proportion 0.0 - 1.0 that AP takes up of AB
    dist = projection / length

    # before point A
    if dist <= 0:
        contact_point = line_a
    # after point B
    elif dist >= 1:
        contact_point = line_b
    # somewhere between
    else:
        contact_point = ab * dist

    # get the distance from original point to the closest point
    square_distance = (contact_point - point).length_squared()
    return square_distance, contact_point

# check for floating point equality, may have very similar value and == will return False
def approxEqual(a: float, b: float, epsilon: float=0.0001):
    return abs(a - b) < epsilon

# Physics object class (sprite + rigidbody)
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
        self.restitution = restitution # object's "bounciness"
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

        # iterate over vertices
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

        # finish up some calculations, e.g. CoM = 1/6A * (prev calculations)
        signed_area *= 0.5

        density = mass / signed_area

        centroid_x /= 6 * signed_area
        centroid_y /= 6 * signed_area

        inertia_x /= 12
        inertia_y /= 12
        inertia_x -= signed_area * centroid_y ** 2
        inertia_y -= signed_area * centroid_x ** 2

        # use the values we just found to set the center of mass and moment of inertia
        self.mass_center: pm.Vector2 = pm.Vector2(centroid_x, centroid_y)
        self.inertia = (inertia_x + inertia_y) * density

        # minimum size of pygame surface to fit the shape onto
        width = max_x
        height = max_y

        # create transparent surface
        self.surface = pygame.Surface((width, height), pygame.SRCALPHA, 32)
        self.image = self.surface

        # draw polygon onto it
        pygame.draw.polygon(self.image, color, points)

        # get bounding rectangle of image
        self.rect = self.image.get_rect(center=self.position)

    def step(self, timestep):
        # apply all forces that occured since last update
        for force, point in self.forces:
            self.__applyForce(force, point, timestep)
        self.forces.clear()

        # update position and rotation based on velocities
        self.position += self.velocity * timestep
        self.rotation += self.angular_velocity * timestep

        # update the positions of each vertex based on new position and rotation
        for i in range(len(self.vertices)):
            # make a copy of vertex so we don't change original
            vertex = pm.Vector2()
            vertex.x = self.vertices[i].x
            vertex.y = self.vertices[i].y

            # center vertices around (0, 0)
            vertex -= self.mass_center

            # using formula to rotate points
            x = vertex.x * math.cos(self.rotation) - vertex.y * math.sin(self.rotation)
            y = vertex.y * math.cos(self.rotation) + vertex.x * math.sin(self.rotation)
            vertex.x = x
            vertex.y = y

            # translate to world position and save new vertex
            vertex += self.position
            self.translated_vertices[i] = vertex

    def update(self, *args, **kwargs):
        # redraw polygon image
        self.image = pygame.transform.rotate(self.surface, math.degrees(-self.rotation))

        # update bounding box position
        self.rect = self.image.get_rect(center=self.position)

    def __applyForce(self, force: pm.Vector2, point: pm.Vector2, timestep: float):
        # F = ma
        # a = F/m
        # v = at = (F/m)t
        self.velocity += force * timestep * (1.0 / self.mass)

        # torque = cross product of force and displacement
        torque = point.x * force.y - point.y * force.x

        # angular acceleration = torque / moment of inertia
        # angular velocity = angular acceleration * time
        self.angular_velocity += (torque / self.inertia) * timestep

    def applyForce(self, force: pm.Vector2, point: pm.Vector2):
        self.forces.append((force, point))

    def applyImpulse(self, force: pm.Vector2, point: pm.Vector2):
        # impulse = instantaneous force (apply the whole force instantly, not over time)
        self.__applyForce(force, point, 1.0)

    def getAxes(self):
        # get axes for SAT collisions
        # an axis is just a perpendicular vector from each face
        axes: list[pm.Vector2] = list()
        for i in range(len(self.translated_vertices)):
            # iterate vertices 2 at a time, gives faces
            edge = self.translated_vertices[i] - self.translated_vertices[i - 1]

            # rotate face vector and normalise to get axis
            axes.append(pm.Vector2(-edge.y, edge.x).normalize())
        return axes

    def project(self, axis):
        # project each face onto an axis, and get minimum and maxium displacements
        # pick a projection to start with (vertex 0)
        # projection is found using dot product
        min_p = axis.dot(self.translated_vertices[0])
        max_p = min_p

        # iterate over the remaining vertices
        for i in range(1, len(self.translated_vertices)):
            # project each one, and store new minimum and maximum projection
            proj = axis.dot(self.translated_vertices[i])
            min_p = min(proj, min_p)
            max_p = max(proj, max_p)
        return pm.Vector2(min_p, max_p)

    def findContactPoints(self, other):
        # find the points of contact between two polygons
        # maximum of two contact points, currently we dont know any (0 points right now)
        point_a = pm.Vector2()
        point_b = pm.Vector2()
        num_points = 0

        # store minimum distance, we don't know right now, so just assume the worst (infinite)
        min_distance = float("inf")

        # iterate over vertices of object
        for i in range(len(self.translated_vertices)):
            point = self.translated_vertices[i]

            # iterate over other object
            for j in range(len(other.translated_vertices)):
                # get a face of the other object
                vertex_a = other.translated_vertices[j - 1]
                vertex_b = other.translated_vertices[j]

                # compute closest point of contact between this shape's point and other shape's face
                distance, contact_point = nearestPoint(vertex_a, vertex_b, point)

                # if we already have a similar distance, possibly another contact point
                if approxEqual(distance, min_distance):
                    # make sure it isn't just the same point found in reverse
                    if not approxEqual(contact_point.x, point_a.x) and approxEqual(contact_point.y, point_a.y):
                        # if a different point, then this is the second contact point
                        num_points = 2
                        point_b = contact_point
                # if this is a new smallest distance, this must be the contact point
                elif distance < min_distance:
                    min_distance = distance
                    num_points = 1
                    point_a = contact_point

        # repeat this process, check the other shape's points over this shape's faces
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

        # return a list of contact points, based on number of contact points
        if num_points == 1:
            return [point_a]
        elif num_points == 2:
            return [point_a, point_b]
        else:
            return None

    def getCollisionNormal(self, other):
        # find the collision normal
        # firstly get a list of all axes of both shapes
        axes = self.getAxes()
        axes.extend(other.getAxes())

        # some variables to track inside the loop
        overlap = float("inf")
        closest_axis = None
        for axis in axes:
            # project each shape along an axis
            proj_s = self.project(axis)
            proj_o = other.project(axis)

            # check if the projections overlap
            if not ((proj_s.x < proj_o.x < proj_s.y) or (proj_o.x < proj_s.x < proj_o.y)):
                # if there is no overlap, there is no collision
                return None
            else:
                # if there is an overlap, compute the size of the overlap
                cur_overlap = min(proj_s.y, proj_o.y) - max(proj_s.x, proj_o.x)
                # if overlap is smaller, this is a closer axis, so update values
                if cur_overlap < overlap:
                    overlap = cur_overlap
                    closest_axis = axis

        return closest_axis

    def collide(self, other):
        # find the collision normal, if no normal then there's no collision
        normal = self.getCollisionNormal(other)
        if normal is None: return

        # find contact points, if there are no contact points then there's no collision
        contact_points = self.findContactPoints(other)
        if contact_points is None: return

        # assume restitution to be the minimum of both objects
        restitution = min(self.restitution, other.restitution)

        impulses = []
        r_as = []
        r_bs = []

        # loop over the contact points
        for point in contact_points:
            # calculate the vectors from contact point to centers of mass
            r_a = point - self.position
            r_b = point - other.position

            # store the vectors
            r_as.append(r_a)
            r_bs.append(r_b)

            # calculate perpendicular vectors (used in angular caclulations)
            rp_a = pm.Vector2(-r_a.y, r_a.x)
            rp_b = pm.Vector2(-r_b.y, r_b.x)

            # relative velocity = linear velocity + angular velocity difference of both objects
            a_velocity = rp_a * self.angular_velocity + self.velocity
            b_velocity = rp_b * other.angular_velocity + other.velocity

            relative_velocity = b_velocity - a_velocity

            # contact speed is the relative velocity in the direction of the collision
            contact_speed = relative_velocity.dot(normal)

            # if contact speed is positive, objects are already moving away from each other
            if contact_speed > 0.0:
                impulses.append(None)
                continue

            # calculate inverse mass
            inv_mass_a = 1.0 / self.mass
            inv_mass_b = 1.0 / other.mass

            # adjustment values to account for rotational velocities
            r_adj_a = (rp_a.dot(normal)) ** 2 / self.inertia
            r_adj_b = (rp_b.dot(normal)) ** 2 / other.inertia

            # calculate impulse J using formula
            j = (-(1.0 + restitution) * contact_speed) / (inv_mass_a + inv_mass_b + r_adj_a + r_adj_b)

            # share J between contact points
            j /= len(contact_points)

            # apply impulse along direction of collision
            impulse = normal * j

            # store the impulse
            impulses.append(impulse)

        for i in range(len(contact_points)):
            impulse = impulses[i]
            if impulse is None:
                # None impulse means objects were moving away
                continue

            # apply impulse to objects, in opposite directions
            self.applyImpulse(-impulse, r_as[i])
            other.applyImpulse(impulse, r_bs[i])


def main():
    # pygame shenanigans
    pygame.init()
    screen = pygame.display.set_mode((800, 600))
    clock = pygame.time.Clock()
    running = True

    # points to describe a pentagon
    pentagon = [
        pm.Vector2(100, 0),
        pm.Vector2(200, 80),
        pm.Vector2(140, 200),
        pm.Vector2(60, 200),
        pm.Vector2(0, 80)
    ]

    # points to describe a long stick
    stick = [
        pm.Vector2(0, 0),
        pm.Vector2(50, 0),
        pm.Vector2(50, 300),
        pm.Vector2(0, 300)
    ]

    # create a rectangular object
    object_1 = Object(pm.Vector2(0, 0), 0, 5, 0.5, (255, 50, 50), stick)
    object_1.update()

    # create a pentagonal object
    object_2 = Object(pm.Vector2(300, 300), 0, 1, 0.5, (255, 50, 50), pentagon)

    # apply some acceleration to the objects
    object_1.applyImpulse(pm.Vector2(250, 250), pm.Vector2(200, 0))
    object_2.applyImpulse(pm.Vector2(-10, -10), pm.Vector2(0, 0))

    while running:
        # pygame updates
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # clear screen
        screen.fill((255, 255, 255))

        # update and draw objects
        objects_group.update()
        objects_group.draw(screen)

        # check collisions
        object_2.collide(object_1)

        # update physics
        physics()

        # pygame stuff
        pygame.display.flip()
        clock.tick(60)

if __name__ == '__main__':
    main()