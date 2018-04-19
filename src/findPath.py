# -----------
# User Instructions:
#
# Modify the the search function so that it returns
# a shortest path as follows:
# 
# [['>', 'v', ' ', ' ', ' ', ' '],
#  [' ', '>', '>', '>', '>', 'v'],
#  [' ', ' ', ' ', ' ', ' ', 'v'],
#  [' ', ' ', ' ', ' ', ' ', 'v'],
#  [' ', ' ', ' ', ' ', ' ', '*']]
#
# Where '>', '<', '^', and 'v' refer to right, left, 
# up, and down motions. Note that the 'v' should be 
# lowercase. '*' should mark the goal cell.
#
# You may assume that all test cases for this function
# will have a path from init to goal.
# ----------

#grid = [[0, 0, 1, 0, 0, 0],
#        [0, 0, 0, 0, 0, 0],
#        [0, 0, 1, 0, 1, 0],
#        [0, 0, 1, 0, 1, 0],
#        [0, 0, 1, 0, 1, 0]]


class FindPathInGrid(object):
    
    def __init__(self, _grid, _coordinates):

        self.grid = _grid;
        self.station_coordinates = _coordinates;

        self.delta = [[-1, 0 ], # go up
                 [ 0, -1], # go left
                 [ 1, 0 ], # go down
                 [ 0, 1 ]] # go right

        self.delta_name = ['^', '<', 'v', '>']
        self.cost = 1
        self.final_path = None;
        self.final_expand = None;
        self.turn_list = None;
        self.node_list = None;
        self.left_right_turn_list = None;

    def search(self,start_node,end_node):
        self.final_path = None;
        self.final_expand = None;

        init = self.station_coordinates[start_node];
        goal = self.station_coordinates[end_node];

        closed = [[0 for row in range(len(self.grid[0]))] for col in range(len(self.grid))]
        closed[init[0]][init[1]] = 1
        expand = [[' ' for row in range(len(self.grid[0]))] for col in range(len(self.grid))]
        

        x = init[0]
        y = init[1]
        g = 0

        open = [[g, x, y, [[x,y]]]]
        

        found = False  # flag that is set when search is complete
        resign = False # flag set if we can't find expand

        while not found and not resign:
            if len(open) == 0:
                resign = True
                return 'fail'
            else:
                open.sort()
                open.reverse()
                next = open.pop()
               
                #print next
                x = next[1]
                y = next[2]
                g = next[0]
                thisPath = next[3]
                #print "open len"
                #print len(thisPath)
                
                if x == goal[0] and y == goal[1]:
                    found = True
                    path = next[3];
                    #print "path len"
                    #print len(path)
                    self.final_path = path;
                    #for ind in range(len(path)):
                    #    self.final_path.append(path[ind]);
                    lenPath = len(path);
                    #print "len path"
                    #print lenPath
                    if(lenPath == 1):
                        expand[0][0] = '*';
                    for step in range(lenPath):
                        if step == 0:
                            continue;
                        #print step
                        diff1 = [0,0];
                        diff1[0] = path[step][0] - path[step-1][0];
                        diff1[1] = path[step][1] - path[step-1][1];
                        #print diff1
                        #print self.delta.index(diff1)
                        expand[path[step-1][0]][path[step-1][1]] = self.delta_name[self.delta.index(diff1)];
                        expand[goal[0]][goal[1]] = '*';
                else:
                    for i in range(len(self.delta)):
                        x2 = x + self.delta[i][0]
                        y2 = y + self.delta[i][1]
                        if x2 >= 0 and x2 < len(self.grid) and y2 >=0 and y2 < len(self.grid[0]):
                            if closed[x2][y2] == 0 and self.grid[x2][y2] == 0:
                                g2 = g + self.cost
                                newPath = thisPath + [[x2,y2]];
                                open.append([g2, x2, y2, newPath])
                                closed[x2][y2] = 1

        self.final_expand = expand;
        self.getNodeAndTurnList();
        return expand # make sure you return the shortest path
        
    def getNodeAndTurnList(self):
        self.node_list = []
        self.turn_list = []
        self.left_right_turn_list = []
        if(self.final_expand is not None):
            if(self.final_path is not None):
                for ind in range(len(self.final_path)):
                    try:
                        self.node_list.append(self.station_coordinates.keys()[self.station_coordinates.values().index(self.final_path[ind])]);
                        self.turn_list.append(self.final_expand[self.final_path[ind][0]][self.final_path[ind][1]])
                    except:
                        continue;
                self.left_right_turn_list = ['S']
                for step in range(len(self.turn_list)):
                    if(step == 0):
                        continue;

                    if(self.turn_list[step] == '*'):
                        self.left_right_turn_list.append('E');
                        continue;

                    if(self.turn_list[step-1] == self.turn_list[step]):
                        self.left_right_turn_list.append('S');
                    elif(self.turn_list[step-1] == '>'): #going right
                        if(self.turn_list[step] == '^'):
                            self.left_right_turn_list.append('L');
                        elif(self.turn_list[step] == 'v'):
                            self.left_right_turn_list.append('R');
                    elif(self.turn_list[step-1] == '<'): #going left
                        if(self.turn_list[step] == '^'):
                            self.left_right_turn_list.append('R');
                        elif(self.turn_list[step] == 'v'):
                            self.left_right_turn_list.append('L');
                    elif(self.turn_list[step-1] == 'v'): #going down
                        if(self.turn_list[step] == '<'):
                            self.left_right_turn_list.append('R');
                        elif(self.turn_list[step] == '>'):
                            self.left_right_turn_list.append('L');
                    elif(self.turn_list[step-1] == '^'): #going up
                        if(self.turn_list[step] == '<'):
                            self.left_right_turn_list.append('L');
                        elif(self.turn_list[step] == '>'):
                            self.left_right_turn_list.append('R');
                    
