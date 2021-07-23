import random
from re import L

from numpy.core.numeric import cross
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

def crossProduct(vec1,vec2):
    return Point(vec1.posY*vec2.posZ - vec1.posZ*vec2.posY,
         vec1.posZ*vec2.posX - vec1.posX*vec2.posZ,
         vec1.posX*vec2.posY - vec1.posY*vec2.posX)

def lambdaCalc(start,end,obstacle):
    nume = dotProduct(Point(end.posX-obstacle.posX,end.posY-obstacle.posY,end.posZ-obstacle.posZ), Point(end.posX-start.posX,end.posY-start.posY,end.posZ-start.posZ))
    deno = dotProduct(Point(end.posX-start.posX,end.posY-start.posY,end.posZ-start.posZ),Point(end.posX-start.posX,end.posY-start.posY,end.posZ-start.posZ))
    return nume/deno

def distToLine(obstacle,start,end):
    distX1X0=distCalcSquared(start,obstacle.center)
    distX2X1=distCalcSquared(end,start)
    dotProd1=Point(start.posX-obstacle.center.posX,start.posY-obstacle.center.posY,start.posZ-obstacle.center.posZ)
    dotProd2=Point(end.posX-start.posX,end.posY-start.posY,end.posZ-start.posZ)
    dotProds=dotProduct(dotProd1,dotProd2)
    nume = distX1X0*distX2X1-pow(dotProds,2)
    deno = distX2X1
    dist = math.sqrt(abs(nume/deno))
    return dist

def linePassesObstacle(obstacle,start,end):
    if insideObstacle(obstacle,start) or insideObstacle(obstacle,end):
        return True
    if distToLine(obstacle,start,end)<obstacle.radius:
        lambd = lambdaCalc(start,end,obstacle.center)
        pointOnLine = Point(lambd*start.posX+(1-lambd)*end.posX,lambd*start.posY+(1-lambd)*end.posY,lambd*start.posZ+(1-lambd)*end.posZ)
        vecComm = Point(pointOnLine.posX-obstacle.center.posX,pointOnLine.posY-obstacle.center.posY,pointOnLine.posZ-obstacle.center.posZ)
        vecDiffA= Point(start.posX-obstacle.center.posX,start.posY-obstacle.center.posY,start.posZ-obstacle.center.posZ)
        vecDiffB= Point(end.posX-obstacle.center.posX,end.posY-obstacle.center.posY,end.posZ-obstacle.center.posZ)
        if dotProduct(crossProduct(vecDiffA,vecComm),crossProduct(vecDiffB,vecComm))>0:
            return False
        else:
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

def checkRobotConfiguration(treeNode,newNode):
    dq_max = 100
    ddq_max = 200
    dx_max = 100
    ddx_max = 100
    n = 3
    cf = 10
    p_1 = [treeNode.getPos().posX,treeNode.getPos().posY,treeNode.getPos().posZ]
    p_2 = [newNode.getPos().posX,newNode.getPos().posY,newNode.getPos().posZ]
    tg = TrajectoryGenerator(dq_max, ddq_max, dx_max, ddx_max, control_freq=cf)
    ps, dps, ts = tg.generate_lin_trajectory(p_1, p_2, n=n,plot=False)
    robot = RRRRobot()
    jointPosToCheck = robot.move_via_points(ps)
    for i in range(len(jointPosToCheck)):
        for j in range(len(jointPosToCheck[i])-1):
            pointA = Point(jointPosToCheck[i][j][0],jointPosToCheck[i][j][1],jointPosToCheck[i][j][2])
            pointB = Point(jointPosToCheck[i][j+1][0],jointPosToCheck[i][j+1][1],jointPosToCheck[i][j+1][2])
            if checkPointCollision(space,pointA) or checkPointCollision(space,pointB):
                '''
                print("This configuration is a problem with point A")
                print(pointA.posX)
                print(pointA.posY)
                print(pointA.posZ)
                print("This configuration is a problem with point B")
                print(pointB.posX)
                print(pointB.posY)
                print(pointB.posZ)
                '''
                return True
            if checkLineCollision(space,pointA,pointB):
                '''
                print("This LINK is a problem")
                print("Point A")
                print(pointA.posX)
                print(pointA.posY)
                print(pointA.posZ)
                print("Point B")
                print(pointB.posX)
                print(pointB.posY)
                print(pointB.posZ)
                print("How's that possible?")
                print(space.obst[0].radius)
                print(space.obst[0].center.posX)
                print(space.obst[0].center.posY)
                print(space.obst[0].center.posZ)
                print(distToLine(space.obst[0],pointA,pointB))
                print("^^^^")
                '''
                return True
    return False

def robotMapper(treeNode,newNode,ax,tg):
    
    p_1 = [treeNode.getPos().posX,treeNode.getPos().posY,treeNode.getPos().posZ]
    p_2 = [newNode.getPos().posX,newNode.getPos().posY,newNode.getPos().posZ]

    ps, dxs, ts = tg.generate_lin_trajectory(p_1, p_2, n=8, plot=True)
    robot = RRRRobot()
    jointPosToCheck = robot.move_via_points(ps)
    xJoints,yJoints,zJoints=[],[],[]
    ax.set_xlim(0,10)
    ax.set_ylim(0,10)
    ax.set_zlim(0,10)
    for k in range(len(jointPosToCheck)):
        for j in range(len(jointPosToCheck[k])):
            if j<len(jointPosToCheck[k])-1:
                ax.plot([jointPosToCheck[k][j][0],jointPosToCheck[k][j+1][0]],[jointPosToCheck[k][j][1],jointPosToCheck[k][j+1][1]],[jointPosToCheck[k][j][2],jointPosToCheck[k][j+1][2]], 'Black')
        plt.pause(0.01)
    ax.scatter3D(xJoints, yJoints, zJoints, marker='X')
    return 

def plotEveryThing(space,tree,ax):
    vecX=[]
    vecY=[]
    vecZ=[]
    for node in tree.treeNS:
        vecX.append(node.getPos().posX)
        vecY.append(node.getPos().posY)
        vecZ.append(node.getPos().posZ)
    ax.scatter3D(vecX, vecY, vecZ, c=vecZ, cmap='Greens')
    createSpheres(ax,space.obst)
    ax.plot(space.start[0],space.start[1],space.start[2], color='Red', marker='o', markersize=12)
    ax.plot(space.goal[0],space.goal[1],space.goal[2], color='Red', marker='o', markersize=12)
    plt.pause(0.05)
    return 

def run(space):
    dq_max = 100
    ddq_max = 200
    dx_max = 100
    ddx_max = 100
    n = 3
    cf = 10
    
    tg = TrajectoryGenerator(dq_max, ddq_max, dx_max, ddx_max, control_freq=cf)
    rootNode=Node(Point(space.start[0],space.start[1],space.start[2]))
    tree=Tree(rootNode)
    space.removeNodeFromSpace(rootNode)
    goalNode=Node(Point(space.goal[0],space.goal[1],space.goal[2]))
    iterations=0
    thresh=1
    while not checkGoalToTree(tree,goalNode,thresh):
        iterations=iterations+1
        randomNode=generateRandomNode(space)
        if checkPointCollision(space,randomNode.point):
            continue
        nearestNode=nearestNodeTree(tree,randomNode)
        removeNodefromSpace=createNewNodeNearest(tree,nearestNode,randomNode)
        if checkLineCollision(space,nearestNode.getPos(),removeNodefromSpace.getPos()):
            continue
        if checkRobotConfiguration(nearestNode,removeNodefromSpace):
            continue
        removeNodefromSpace=expandTree(tree,nearestNode,randomNode)
        updateFreeSpace(space,removeNodefromSpace)

    nearestNode=nearestNodeTree(tree,goalNode)
    fig= plt.figure()
    ax=fig.add_subplot(111,projection='3d')
    plotEveryThing(space,tree,ax)
    while nearestNode!=rootNode:
        robotMapper(nearestNode,nearestNode.parent,ax,tg)
        nearestNode=nearestNode.parent
    plt.show()

if __name__ == "__main__":
    space=Space(0,0,0,10,10,10,(3,3,3),(9,6,8),[(1,6,6,4),(1,2,4,6),(1,5,6,6),(1,5.5,5.5,5.5),(1,3,8,8)])
    run(space)
