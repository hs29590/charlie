#!/usr/bin/env python
from findPath import FindPathInGrid
class Planner:

    def __init__(self):

        self.grid = [[1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
               [1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1],
               [1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1],
               [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
               [1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1],
               [1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1],
               [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]]


        self.station_coordinates = {}
        self.station_coordinates['A'] = [1, 9]; # 0
        self.station_coordinates['B'] = [5, 9]; # 1
        self.station_coordinates['C'] = [1, 7]; # 2
        self.station_coordinates['D'] = [5, 7]; # 3
        self.station_coordinates['E'] = [1, 5]; # 4
        self.station_coordinates['F'] = [5, 5]; # 5
        self.station_coordinates['G'] = [3, 1]; # 6
        self.station_coordinates['H'] = [1, 1]; # 7
        self.station_coordinates['I'] = [3, 3]; # 8
        self.station_coordinates['J'] = [3, 5]; # 9
        self.station_coordinates['K'] = [3, 7]; # 10
        self.station_coordinates['L'] = [3, 9]; # 11
        self.station_coordinates['M'] = [1, 3]; # 12

        self.findPathObject = FindPathInGrid(self.grid, self.station_coordinates);

        self.startNode = None;
        self.endNode = None;
        self.pathExecuted = False;
        
    def setStartNode(self,val):
        self.startNode = val;
        
    def setEndNode(self,val):
        self.endNode = val;

    def calculatePath(self): 
        if(self.startNode is not None and self.endNode is not None):
            self.findPathObject.search(self.startNode, self.endNode)
            self.pathExecuted = True;
        else:
            print("Set Start and/or End node first");
    
    def getLeftRightTurnList(self):
        if(self.pathExecuted):
            return self.findPathObject.left_right_turn_list;
        else:
            print("Calculate Path First");
            return None;
      
    def getNodeList(self):
        if(self.pathExecuted):
            return self.findPathObject.node_list;
        else:
            print("Calculate Path First");
            return None;

        
    def getTurnList(self):
        if(self.pathExecuted):
            return self.findPathObject.turn_list;
        else:
            print("Calculate Path First");
            return None;
        
        
#planner = Planner();
#planner.setStartNode('B');
#planner.setEndNode('F');
#planner.calculatePath();
#print planner.getLeftRightTurnList();
#print planner.getTurnList();
#print planner.getNodeList();    
