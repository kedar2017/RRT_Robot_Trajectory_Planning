import random
from Geometry import Space
from Geometry import Node
from Geometry import Tree
from Geometry import Point
from Geometry import Obstacle

from utils.trajectory_generator import TrajectoryGenerator
from robots.rrr_robot import RRRRobot
import math
import numpy as np
import matplotlib.pyplot as plt

def generateRandomNode(space):
    return space.getRandomNode()

def radialPosCheck(nodeA,nodeB,threshold):
    radialPos=math.sqrt(pow(nodeA.point.posX-nodeB.point.posX,2)
        +pow(nodeA.point.posY-nodeB.point.posY,2)
        +pow(nodeA.point.posZ-nodeB.point.posZ,2))
    if radialPos>threshold:
        return False
    return True

def nearestNodeTree(tree,sampledNode):
    return tree.findNearest(sampledNode)

def createNewNodeNearest(tree,startNode,endNode):
    return tree.createNewNodetoNearest(startNode,endNode)

def expandTree(tree,startNode,endNode):
    return tree.expandToRandom(startNode,endNode)

def checkGoalToTree(tree,goalNode,threshold):
    nearestNode=nearestNodeTree(tree,goalNode)
    if radialPosCheck(goalNode,nearestNode,threshold):
        return True
    return False

def updateFreeSpace(space,removeNodefromSpace):
    return space.removeNodeFromSpace(removeNodefromSpace)

def radialPointtoObstacle(point,center,radius):
    radialPos=math.sqrt(pow(point.posX-center.posX,2)
        +pow(point.posY-center.posY,2)
        +pow(point.posZ-center.posZ,2))
    if radialPos<radius:
        return True
    return False

def insideObstacle(obstacle,point):
    if radialPointtoObstacle(point,obstacle.center,obstacle.radius):
        return True
    return False

def distCalcSquared(pointA,pointB):
    return pow(pointA.posX-pointB.posX,2)+pow(pointA.posY-pointB.posY,2)+pow(pointA.posZ-pointB.posZ,2)

def dotProduct(vec1,vec2):
    return vec1.posX*vec2.posX+vec1.posY*vec2.posY+vec1.posZ*vec2.posZ

def distToLine(obstacle,start,end):
    distX1X0=distCalcSquared(start,obstacle.center)
    distX2X1=distCalcSquared(end,start)
    dotProd1=Point(start.posX-obstacle.center.posX,start.posY-obstacle.center.posY,start.posZ-obstacle.center.posZ)
    dotProd2=Point(end.posX-start.posX,end.posY-start.posY,end.posZ-start.posZ)
    dotProds=dotProduct(dotProd1,dotProd2)
    nume = distX1X0*distX2X1-pow(dotProds,2)
    deno = distX2X1
    return math.sqrt(abs(nume/deno))

def linePassesObstacle(obstacle,start,end):
    if insideObstacle(obstacle,start) or insideObstacle(obstacle,end):
        return True
    if distToLine(obstacle,start,end)<obstacle.radius:
        return True
    return False

def checkLineCollision(space,start,end):
    for obst in space.obst:
        if linePassesObstacle(obst,start,end):
            return True
    return False

def checkPointCollision(space,point):
    for obst in space.obst:
        if insideObstacle(obst,point):
            return True
    return False

def createSpheres(ax,obstacles):
    r = 10
    pi = np.pi
    cos = np.cos
    sin = np.sin
    phi, theta = np.mgrid[0.0:pi:100j, 0.0:2.0*pi:100j]
    for obst in obstacles:
        r=obst.radius
        x = obst.center.posX+r*sin(phi)*cos(theta)
        y = obst.center.posY+r*sin(phi)*sin(theta)
        z = obst.center.posZ+r*cos(phi)
        ax.plot_surface(
        x, y, z,  rstride=1, cstride=1, color='Red', alpha=0.6, linewidth=10)

def plotEveryThing(space,tree):
    vecX=[]
    vecY=[]
    vecZ=[]
    for node in tree.treeNS:
        vecX.append(node.getPos().posX)
        vecY.append(node.getPos().posY)
        vecZ.append(node.getPos().posZ)
    ax = plt.axes(projection='3d')
    ax.scatter3D(vecX, vecY, vecZ, c=vecZ, cmap='Greens')
    #createSpheres(ax,space.obst)
    plt.plot(space.start[0],space.start[1],space.start[2], color='Red', marker='o', markersize=12)
    plt.plot(space.goal[0],space.goal[1],space.goal[2], color='Red', marker='o', markersize=12)
    plt.show()
    return 

def run(space):

    rootNode=Node(Point(space.start[0],space.start[1],space.start[2]))
    tree=Tree(rootNode)
    space.removeNodeFromSpace(rootNode)
    goalNode=Node(Point(space.goal[0],space.goal[1],space.goal[2]))
    iterations=0
    thresh=0.2
    while not checkGoalToTree(tree,goalNode,thresh):
        iterations=iterations+1
        randomNode=generateRandomNode(space)
        if checkPointCollision(space,randomNode.point):
            continue
        nearestNode=nearestNodeTree(tree,randomNode)
        removeNodefromSpace=createNewNodeNearest(tree,nearestNode,randomNode)
        if checkLineCollision(space,nearestNode.getPos(),removeNodefromSpace.getPos()):
            continue
        removeNodefromSpace=expandTree(tree,nearestNode,randomNode)
        updateFreeSpace(space,removeNodefromSpace)    
    plotEveryThing(space,tree)
    
    dq_max = 100
    ddq_max = 200
    dx_max = 100
    ddx_max = 100
    n = 5
    cf = 10
    p_1 = [4, 0.0, 1.0]
    p_2 = [3, 1, 2]
    tg = TrajectoryGenerator(dq_max, ddq_max, dx_max, ddx_max, control_freq=cf)
    ps, dps, ts = tg.generate_lin_trajectory(p_1, p_2, n=n,plot=True)
    
    robot=RRRRobot()
    zs=robot.move_via_points(ps)
    plt.show()
 
if __name__ == "__main__":
    space=Space(0,0,0,2,2,2,(1,1,1),(2,2,1.5),[(10,60,60,60)])
    run(space)
    #[(10,60,60,60),(10,30,60,30),(10,40,50,85),(10,10,50,25)]