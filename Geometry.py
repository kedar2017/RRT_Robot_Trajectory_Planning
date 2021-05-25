import math
import random

class Point:
    def __init__(self,x,y,z):
        self.posX=x
        self.posY=y
        self.posZ=z

class Node:
    def __init__(self,point):
        self.point=point
        self.parent=None
        self.children=[]
    
    def getPos(self):
        return self.point
    
    def setPos(self,point):
        self.point=point
        return
    
    def setParent(self,parent):
        self.parent=parent
        return
    
    def getParent(self):
        return self.parent

    def addChild(self,child):
        self.children.append(child)
        return

class Tree:

    def __init__(self,root):
        self.root=root
        self.root.parent=None
        self.treeNS=[]
        self.treeNS.append(root)
    
    def addNode(self,addEnd,addER):
        addEnd.children.append(addER)
        addER.parent=addEnd
        self.treeNS.append(addER)
        return
    
    def findNearest(self,randomNode):
        minDist=math.inf
        nearestNode=None
        for node in self.treeNS:
            dist = self.estDist(node,randomNode)
            if dist<minDist:
                minDist=dist
                nearestNode=node
        return nearestNode
    
    def estDist(self,node,randomNode):
        return math.sqrt(pow(node.point.posX-randomNode.point.posX,2)
        +pow(node.point.posY-randomNode.point.posY,2)
        +pow(node.point.posZ-randomNode.point.posZ,2))
    
    def createNewNodetoNearest(self,nearestNode,randomNode):
        DELTA=0.5
        addedNodeFromSpace=None
        radialDist=self.estDist(nearestNode,randomNode)
        if radialDist<DELTA:
            addedNodeFromSpace=randomNode
        else:
            theta=math.atan2(abs(nearestNode.point.posY-randomNode.point.posY),
            abs(nearestNode.point.posX-randomNode.point.posX))
            phi=math.atan2(math.sqrt(pow(nearestNode.point.posY-randomNode.point.posY,2)
            +pow(nearestNode.point.posX-randomNode.point.posX,2)),
            abs(nearestNode.point.posZ-randomNode.point.posZ))
            #expandTo=Node(Point(int(nearestNode.point.posX+DELTA*math.sin(phi)*math.cos(theta)),
            #int(nearestNode.point.posY+DELTA*math.sin(phi)*math.sin(theta)),
            #int(nearestNode.point.posZ+DELTA*math.cos(phi))))
            expandTo=Node(Point(nearestNode.point.posX+DELTA*math.sin(phi)*math.cos(theta),
            nearestNode.point.posY+DELTA*math.sin(phi)*math.sin(theta),
            nearestNode.point.posZ+DELTA*math.cos(phi)))

            addedNodeFromSpace=expandTo
        return addedNodeFromSpace
    
    def expandToRandom(self,expandFrom,randomNode):
        DELTA=0.5
        addedNodeFromSpace=None
        radialDist=self.estDist(expandFrom,randomNode)
        if radialDist<DELTA:
            self.addNode(expandFrom,randomNode)
            addedNodeFromSpace=randomNode
        else:
            theta=math.atan2(abs(expandFrom.point.posY-randomNode.point.posY),
            abs(expandFrom.point.posX-randomNode.point.posX))
            phi=math.atan2(math.sqrt(pow(expandFrom.point.posY-randomNode.point.posY,2)
            +pow(expandFrom.point.posX-randomNode.point.posX,2)),
            abs(expandFrom.point.posZ-randomNode.point.posZ))
            #expandTo=Node(Point(int(expandFrom.point.posX+DELTA*math.sin(phi)*math.cos(theta)),
            #int(expandFrom.point.posY+DELTA*math.sin(phi)*math.sin(theta)),
            #int(expandFrom.point.posZ+DELTA*math.cos(phi))))
            expandTo=Node(Point(expandFrom.point.posX+DELTA*math.sin(phi)*math.cos(theta),
            expandFrom.point.posY+DELTA*math.sin(phi)*math.sin(theta),
            expandFrom.point.posZ+DELTA*math.cos(phi)))
            self.addNode(expandFrom,expandTo)
            addedNodeFromSpace=expandTo
        return addedNodeFromSpace

class Obstacle:
    
    def __init__(self,center,radius):
        self.center=center
        self.radius=radius
        return

class Space:

    def __init__(self,xS,yS,zS,xE,yE,zE,start,goal,obst):
        self.winXStart=xS
        self.winYStart=yS
        self.winZStart=zS
        self.winXEnd=xE
        self.winYEnd=yE
        self.winZEnd=zE
        self.start=(start[0],start[1],start[2])
        self.goal=(goal[0],goal[1],goal[2])
        self.obst=[]
        self.mapDonePoints={}
        self.initObstacles(obst)

    def initObstacles(self,obst):
        for i in range(len(obst)):
            centerPos=Point(obst[i][1],obst[i][2],obst[i][3])
            obstacleNew = Obstacle(centerPos,obst[i][0])
            self.obst.append(obstacleNew)
    
    def getRandomPoint(self):
        randomIndexX = random.randint(self.winXStart,self.winXEnd)
        randomIndexY = random.randint(self.winYStart,self.winYEnd)
        randomIndexZ = random.randint(self.winZStart,self.winZEnd)
        
        return (randomIndexX,randomIndexY,randomIndexZ)
    
    def getRandomNode(self):
        randomPoint=self.getRandomPoint()
        while (randomPoint[0],randomPoint[1],randomPoint[2]) in self.mapDonePoints:
            randomPoint=self.getRandomPoint()
        return Node(Point(randomPoint[0],randomPoint[1],randomPoint[2]))

    def removeNodeFromSpace(self,node):
        posX=int(node.getPos().posX)
        posY=int(node.getPos().posY)
        posZ=int(node.getPos().posZ)
        self.mapDonePoints[(posX,posY,posZ)]=1
        return


'''
    u = np.random.rand()
    v = np.random.rand()
    theta = u * 2.0 * np.pi
    phi = np.arccos(2.0 * v - 1.0)
    sinTheta = np.sin(theta);
    cosTheta = np.cos(theta);
    sinPhi = np.sin(phi);
    cosPhi = np.cos(phi);
    rx = a * sinPhi * cosTheta;
    ry = b * sinPhi * sinTheta;
    rz = c * cosPhi;
    return rx, ry, rz
'''