#!/usr/bin/env python3

print('hello world')


from ast import Num
import math
from tokenize import Number

class Robot:

    def __init__(self,
        wheelBase, 
        maxVelocity,
        acceleration,
        startX = 0.0,
        startY = 0.0,
        startTheta = 0.0
    ) -> None:

        self.wheelBase = wheelBase
        self.maxVelocity = maxVelocity
        self.acceleration = acceleration

        self.x = startX
        self.y = startY
        self.theta = startTheta

        self.velocity = 0.0
        self.dtheta = 0.0

        self.leftWheel = 0.0
        self.rightWheel = 0.0
        self.dt = 0.05

        self.leftDist = 0.0
        self.rightDist = 0.0

    def coerceIn(self, val, bot, top) -> Number: return max(min(val, top), bot)

    def setLeft(self, left):
        self.leftWheel = self.coerceIn(left, -1.0, 1.0)
    def setRight(self, right):
        self.rightWheel = self.coerceIn(right, -1.0, 1.0)

    def update(self):
        targetVelocity = (self.leftWheel + self.rightWheel) / 2 * self.maxVelocity
        dtheta = 0.0

        if(self.rightWheel == -self.leftWheel):
            dtheta = self.leftWheel * self.maxVelocity / self.wheelBase * self.dt
        elif(self.leftWheel != self.rightWheel):
            r = self.wheelBase * (self.leftWheel + self.rightWheel) / 2 / abs(self.leftWheel - self.rightWheel)
            dist = targetVelocity * self.dt
            h = math.sqrt(r*r - dist*dist/4)
            dtheta = 2 * math.atan(dist/2/h) * (1 if self.leftWheel > self.rightWheel else -1)

        self.theta = (self.theta + dtheta) % (math.pi * 2)

        if self.acceleration > abs(targetVelocity - self.velocity):
            self.velocity = targetVelocity
        else:
            self.velocity += self.acceleration if targetVelocity > self.velocity else -self.acceleration

        self.velocity = self.coerceIn(self.velocity, -self.maxVelocity, self.maxVelocity)

        self.x += math.sin(self.theta) * self.velocity * self.dt
        self.y += math.cos(self.theta) * self.velocity * self.dt

        self.leftDist += self.velocity * self.dt
        self.rightDist += self.velocity * self.dt

    def getLeftDistance(self) -> Number: return self.leftDist
    def getRightDistance(self) -> Number: return self.rightDist
    def getHeading(self) -> Number: return self.theta
    def getHeadingInDegrees(self) -> Number: return self.theta / math.pi * 180