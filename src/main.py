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

class Object(pygame.sprite.Sprite, AbstractObject):
    def __init__(self, position: pm.Vector2, rotation: float, mass: float, color: tuple[int, int, int], vertices: list[pm.Vector2]):
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
        self.vertices: list[pm.Vector2] = vertices
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
        self.__inertia = (inertia_x + inertia_y) * density

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

    def update(self, *args, **kwargs):
        self.image = pygame.transform.rotate(self.surface, math.degrees(-self.rotation))
        self.rect = self.image.get_rect(center=self.position)

    def __applyForce(self, force: pm.Vector2, point: pm.Vector2, timestep: float):
        self.velocity += force * timestep * (1.0 / self.mass)
        torque = point.x * force.y - point.y * force.x
        self.angular_velocity += (torque / self.__inertia) * timestep

    def applyForce(self, force: pm.Vector2, point: pm.Vector2):
        self.forces.append((force, point))

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
    
    _object = Object(pm.Vector2(0, 0), 0, 1, (255, 50, 50), pentagon)
    _object.update()

    start_time = time.perf_counter()

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        if time.perf_counter() < start_time + 1:
            _object.applyForce(pm.Vector2(100, 10), pm.Vector2(1000, 0))

        screen.fill((255, 255, 255))
        objects_group.update()
        objects_group.draw(screen)

        physics()

        pygame.display.flip()
        clock.tick(60)

if __name__ == '__main__':
    main()