"""
world_manager.py

Stores every obstacle in the scene.
"""

from world.obstacle import Obstacle


class WorldManager:

    def __init__(self):

        self.obstacles = {}

    def add(self, obstacle: Obstacle):

        self.obstacles[obstacle.name] = obstacle

    def remove(self, name):

        if name in self.obstacles:
            del self.obstacles[name]

    def clear(self):

        self.obstacles.clear()

    def get(self, name):

        return self.obstacles[name]

    def all(self):

        return list(self.obstacles.values())

    def count(self):

        return len(self.obstacles)

    def print_summary(self):

        print()
        print("=" * 60)
        print("World")
        print("=" * 60)

        print("Total Obstacles :", self.count())
        print()

        for obstacle in self.all():

            print(
                obstacle.name,
                obstacle.shape,
                obstacle.position,
            )