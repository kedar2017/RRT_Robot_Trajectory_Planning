import random
from Geometry import Space
from Geometry import Node
from Geometry import Tree
from Geometry import Point
from Geometry import Obstacle

import math

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
    return space.removeNodeFreeSpace(removeNodefromSpace)

def run(space):

    rootNode=Node(Point(space.start[0],space.start[1],space.start[2]))
    tree=Tree(rootNode)
    space.removeNodeFromSpace(rootNode)
    goalNode=Node(Point(space.goal[0],space.goal[1],space.goal[2]))
    iterations=0
    thresh=8
    while not checkGoalToTree(tree,goalNode,thresh):
        iterations=iterations+1
        randomNode=generateRandomNode(space)
        
        nearestNode=nearestNodeTree(tree,randomNode)
        #removeNodefromSpace=createNewNodeNearest(tree,nearestNode,randomNode)
        removeNodefromSpace=expandTree(tree,nearestNode,randomNode)


    for node in tree.treeNS:
        print(node.getPos().posX)
        print(node.getPos().posY)
        print(node.getPos().posZ)
        print("FIRST MILESTONE--------------------")

if __name__ == "__main__":
    space=Space(0,0,0,200,200,200,(100,40,100),(100,40,120),[(0.2,2,2,2)])
    run(space)
