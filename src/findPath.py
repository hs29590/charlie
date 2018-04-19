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
        return expand # make sure you return the shortest path
        
    def getNodeAndTurnList(self, node_list, turn_list):
        if(self.final_expand is not None):
            if(self.final_path is not None):
                for ind in range(len(self.final_path)):
                    try:
                        node_list.append(station_coordinates.keys()[station_coordinates.values().index(self.final_path[ind])]);
                        turn_list.append(self.final_expand[self.final_path[ind][0]][self.final_path[ind][1]])
                    except:
                        continue;
                left_right_turn_list = ['S']
                for step in range(len(turn_list)):
                    if(step == 0):
                        continue;

                    if(turn_list[step] == '*'):
                        left_right_turn_list.append('E');
                        continue;

                    if(turn_list[step-1] == turn_list[step]):
                        left_right_turn_list.append('S');
                    elif(turn_list[step-1] == '>'): #going right
                        if(turn_list[step] == '^'):
                            left_right_turn_list.append('L');
                        elif(turn_list[step] == 'v'):
                            left_right_turn_list.append('R');
                    elif(turn_list[step-1] == '<'): #going left
                        if(turn_list[step] == '^'):
                            left_right_turn_list.append('R');
                        elif(turn_list[step] == 'v'):
                            left_right_turn_list.append('L');
                    elif(turn_list[step-1] == 'v'): #going down
                        if(turn_list[step] == '<'):
                            left_right_turn_list.append('R');
                        elif(turn_list[step] == '>'):
                            left_right_turn_list.append('L');
                    elif(turn_list[step-1] == '^'): #going up
                        if(turn_list[step] == '<'):
                            left_right_turn_list.append('L');
                        elif(turn_list[step] == '>'):
                            left_right_turn_list.append('R');

        return left_right_turn_list;
                    


grid = [[1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
       [1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1],
       [1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1],
       [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
       [1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1],
       [1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1],
       [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]]


station_coordinates = {}
station_coordinates['A'] = [1, 9]; # 0
station_coordinates['B'] = [5, 9]; # 1
station_coordinates['C'] = [1, 7]; # 2
station_coordinates['D'] = [5, 7]; # 3
station_coordinates['E'] = [1, 5]; # 4
station_coordinates['F'] = [5, 5]; # 5
station_coordinates['G'] = [3, 1]; # 6
station_coordinates['H'] = [1, 1]; # 7
station_coordinates['I'] = [3, 3]; # 8
station_coordinates['J'] = [3, 5]; # 9
station_coordinates['K'] = [3, 7]; # 10
station_coordinates['L'] = [3, 9]; # 11
station_coordinates['M'] = [1, 3]; # 12

findPathObject = FindPathInGrid(grid, station_coordinates);

findPathObject.search('H', 'B')

#if(findPathObject.final_expand is not None):
#    for i in findPathObject.final_expand:
#        print i
        
#print findPathObject.final_path;


node_list = []
turn_list = []

print findPathObject.getNodeAndTurnList(node_list, turn_list);
print node_list
#print turn_list



