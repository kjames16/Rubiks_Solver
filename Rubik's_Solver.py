""" Rubik's Solver
This program will generate and sovle a Rubik's cube using A-Star Search
"""

import random
import heapq
import copy

# Simulate a goal state of having 6 sides and 9 spaces for each side

goal_front = [["W", "W", "W"],
              ["W", "W", "W"],
              ["W", "W", "W"]]

goal_left = [["B", "B", "B"],
             ["B", "B", "B"],
             ["B", "B", "B"]]

goal_back = [["Y", "Y", "Y"],
             ["Y", "Y", "Y"],
             ["Y", "Y", "Y"]]

goal_right = [["G", "G", "G"],
             ["G", "G", "G"],
             ["G", "G", "G"]]

goal_top = [["R", "R", "R"],
            ["R", "R", "R"],
            ["R", "R", "R"]]

goal_bottom = [["O", "O", "O"],
               ["O", "O", "O"],
               ["O", "O", "O"]]

goal_state = [goal_front, goal_left, goal_back, goal_right, goal_top, goal_bottom]

def Goal_Test(state):
    #Test to see if the current state matches the goal state
    if state == goal_state:
        return True
    else:
        return False
    
def Print_State(state):
    # Prints the current state
    names = ["Front: ", "Left: ", "Back: ", "Right: ", "Top: ", "Bottom: "]
    i = 0
    for side in state:
        print(f"{names[i]}\n{state[i][0][0]}|{state[i][0][1]}|{state[i][0][2]}")
        print("- - -")
        print(f"{state[i][1][0]}|{state[i][1][1]}|{state[i][1][2]}")
        print("- - -")
        print(f"{state[i][2][0]}|{state[i][2][1]}|{state[i][2][2]}\n")

        i+=1

def Get_Actions():
    #Returns the actions
    actions = ["topL", "topR", "bottomL", "bottomR", "frontL", "frontR", "backL", "backR", "rightU", "rightD", "leftU", "leftD"]
    return actions

def Transition_Model(state, action):

    #Given a state and an action, return state that would result from taking this action

    next_state = copy.deepcopy(state)

    match action:
        case "topL":
            #Rotate the top row
            for i in range(4):                                #Each side
                for j in range(3):                            #Each space
                    next_state[(i+1)%4][0][j] = state[i][0][j]
            #Rotate the top side
                #Corners
            next_state[4][0][0] = state[4][0][2]
            next_state[4][2][0] = state[4][0][0]
            next_state[4][2][2] = state[4][2][0]
            next_state[4][0][2] = state[4][2][2]
                #Sides
            next_state[4][1][0] = state[4][0][1]
            next_state[4][2][1] = state[4][1][0]
            next_state[4][1][2] = state[4][2][1]
            next_state[4][0][1] = state[4][1][2]

        case "topR":
            #Rotate the top row
            for i in range(3,-1,-1):                          #Each side
                for j in range(3):                            #Each space
                    next_state[i][0][j] = state[(i+1)%4][0][j]
            #Rotate the top side
                #Corners
            next_state[4][0][2] = state[4][0][0]
            next_state[4][2][2] = state[4][0][2]
            next_state[4][2][0] = state[4][2][2]
            next_state[4][0][0] = state[4][2][0]
                #Sides
            next_state[4][1][2] = state[4][0][1]
            next_state[4][2][1] = state[4][1][2]
            next_state[4][1][0] = state[4][2][1]
            next_state[4][0][1] = state[4][1][0]

        case "bottomL":
            #Rotate the bottom row
            for i in range(4):                                #Each side
                for j in range(3):                            #Each space
                    next_state[(i+1)%4][2][j] = state[i][2][j]
            #Rotate the bottom side
                #Corners
            next_state[5][0][0] = state[5][0][2]
            next_state[5][2][0] = state[5][0][0]
            next_state[5][2][2] = state[5][2][0]
            next_state[5][0][2] = state[5][2][2]
                #Sides
            next_state[5][1][0] = state[5][0][1]
            next_state[5][2][1] = state[5][1][0]
            next_state[5][1][2] = state[5][2][1]
            next_state[5][0][1] = state[5][1][2]

        case "bottomR":
             #Rotate the bottom row
            for i in range(3,-1,-1):                          #Each side
                for j in range(3):                            #Each space
                    next_state[i][2][j] = state[(i+1)%4][2][j]
            #Rotate the bottom side
                #Corners
            next_state[5][0][2] = state[5][0][0]
            next_state[5][2][2] = state[5][0][2]
            next_state[5][2][0] = state[5][2][2]
            next_state[5][0][0] = state[5][2][0]
                #Sides
            next_state[5][1][2] = state[5][0][1]
            next_state[5][2][1] = state[5][1][2]
            next_state[5][1][0] = state[5][2][1]
            next_state[5][0][1] = state[5][1][0]

        case "frontL":
            #Rotate the front column
            for i, j in zip(range(3),range(2,-1,-1)):
                next_state[1][j][2] = state[4][2][i]
                next_state[5][0][j] = state[1][j][2]
                next_state[3][i][0] = state[5][0][j]
                next_state[4][2][i] = state[3][i][0]
            #Rotate the front side
               #Corners
            next_state[0][0][0] = state[0][0][2]
            next_state[0][2][0] = state[0][0][0]
            next_state[0][2][2] = state[0][2][0]
            next_state[0][0][2] = state[0][2][2]
                #Sides
            next_state[0][1][0] = state[0][0][1]
            next_state[0][2][1] = state[0][1][0]
            next_state[0][1][2] = state[0][2][1]
            next_state[0][0][1] = state[0][1][2]

        case "frontR":
            #Rotate the front column
            for i, j in zip(range(3),range(2,-1,-1)):
                next_state[3][j][0] = state[4][2][j]
                next_state[5][0][i] = state[3][j][0]
                next_state[1][i][2] = state[5][0][i]
                next_state[4][2][j] = state[1][i][2]
            #Rotate the front side
               #Corners
            next_state[0][0][2] = state[0][0][0]
            next_state[0][2][2] = state[0][0][2]
            next_state[0][2][0] = state[0][2][2]
            next_state[0][0][0] = state[0][2][0]
                #Sides
            next_state[0][1][2] = state[0][0][1]
            next_state[0][2][1] = state[0][1][2]
            next_state[0][1][0] = state[0][2][1]
            next_state[0][0][1] = state[0][1][0]

        case "backL":
            #Rotate the back column
            for i, j in zip(range(3),range(2,-1,-1)):
                next_state[1][j][0] = state[4][0][i]
                next_state[5][2][j] = state[1][j][0]
                next_state[3][i][2] = state[5][2][j]
                next_state[4][0][i] = state[3][i][2]
            #Rotate the back side
               #Corners
            next_state[2][0][2] = state[2][0][0]
            next_state[2][2][2] = state[2][0][2]
            next_state[2][2][0] = state[2][2][2]
            next_state[2][0][0] = state[2][2][0]
                #Sides
            next_state[2][1][2] = state[2][0][1]
            next_state[2][2][1] = state[2][1][2]
            next_state[2][1][0] = state[2][2][1]
            next_state[2][0][1] = state[2][1][0]

        case "backR":
            #Rotate the front column
            for i, j in zip(range(3),range(2,-1,-1)):
                next_state[3][j][2] = state[4][0][j]
                next_state[5][2][i] = state[3][j][2]
                next_state[1][i][0] = state[5][2][i]
                next_state[4][0][j] = state[1][i][0]
            #Rotate the back side
               #Corners
            next_state[2][0][0] = state[2][0][2]
            next_state[2][2][0] = state[2][0][0]
            next_state[2][2][2] = state[2][2][0]
            next_state[2][0][2] = state[2][2][2]
                #Sides
            next_state[2][1][0] = state[2][0][1]
            next_state[2][2][1] = state[2][1][0]
            next_state[2][1][2] = state[2][2][1]
            next_state[2][0][1] = state[2][1][2]

        case "rightU":
            #Rotate the right column
            for i, j in zip(range(3), range(2,-1,-1)):
                next_state[4][j][2] = state[0][j][2]
                next_state[2][i][0] = state[4][j][2]
                next_state[5][j][2] = state[2][i][0]
                next_state[0][j][2] = state[5][j][2]
            #Rotate the right side
               #Corners
            next_state[3][0][2] = state[3][0][0]
            next_state[3][2][2] = state[3][0][2]
            next_state[3][2][0] = state[3][2][2]
            next_state[3][0][0] = state[3][2][0]
                #Sides
            next_state[3][1][2] = state[3][0][1]
            next_state[3][2][1] = state[3][1][2]
            next_state[3][1][0] = state[3][2][1]
            next_state[3][0][1] = state[3][1][0]

        case "rightD":
            #Rotate the right column
            for i,j in zip(range(3),range(2,-1,-1)):
                next_state[5][j][2] = state[0][j][2]
                next_state[2][i][0] = state[5][j][2]
                next_state[4][j][2] = state[2][i][0]
                next_state[0][j][2] = state[4][j][2]
            #Rotate the right side
               #Corners
            next_state[3][0][0] = state[3][0][2]
            next_state[3][2][0] = state[3][0][0]
            next_state[3][2][2] = state[3][2][0]
            next_state[3][0][2] = state[3][2][2]
                #Sides
            next_state[3][1][0] = state[3][0][1]
            next_state[3][2][1] = state[3][1][0]
            next_state[3][1][2] = state[3][2][1]
            next_state[3][0][1] = state[3][1][2]

        case "leftU":
            #Rotate the left column
            for i, j in zip(range(3), range(2,-1,-1)):
                next_state[4][j][0] = state[0][j][0]
                next_state[2][i][2] = state[4][j][0]
                next_state[5][i][0] = state[2][i][2]
                next_state[0][j][0] = state[5][i][0]
            #Rotate the left side
               #Corners
            next_state[1][0][0] = state[1][0][2]
            next_state[1][2][0] = state[1][0][0]
            next_state[1][2][2] = state[1][2][0]
            next_state[1][0][2] = state[1][2][2]
                #Sides
            next_state[1][1][0] = state[1][0][1]
            next_state[1][2][1] = state[1][1][0]
            next_state[1][1][2] = state[1][2][1]
            next_state[1][0][1] = state[1][1][2]

        case "leftD":
            #Rotatae the left column
             for i,j in zip(range(3),range(2,-1,-1)):
                next_state[5][j][0] = state[0][j][0]
                next_state[2][i][2] = state[5][j][0]
                next_state[4][j][0] = state[2][i][2]
                next_state[0][j][0] = state[4][j][0]
             #Rotate the left side
               #Corners
             next_state[1][0][2] = state[1][0][0]
             next_state[1][2][2] = state[1][0][2]
             next_state[1][2][0] = state[1][2][2]
             next_state[1][0][0] = state[1][2][0]
                #Sides
             next_state[1][1][2] = state[1][0][1]
             next_state[1][2][1] = state[1][1][2]
             next_state[1][1][0] = state[1][2][1]
             next_state[1][0][1] = state[1][1][0]


    return next_state

def generate_random_start_w_fixed_depth(depth):
  
  start_state = copy.deepcopy(goal_state)
  action_log = []

  for i in range(depth):
    actions = Get_Actions()

    # The following is to help avoid taking an action which simply reverses the
    # immediately preceding action.
    if len(action_log) > 0:
      last_action = action_log[-1] # This returns the last action done.
      for action in actions:
         if action == last_action:
            if action[-1] == "L":
               actions.remove(action[:-1] + "R")
            elif action[-1] == "R":
               actions.remove(action[:-1] + "L")
            elif action[-1] == "U":
               actions.remove(action[:-1] + "D")
            elif action[-1] == "D":
               actions.remove(action[:-1] + "U")
               

    random_action_index = random.randint(0, len(actions) - 1)

    next_action = actions[random_action_index]

    start_state = Transition_Model(start_state, next_action)

    action_log.append(next_action)
    print(next_action)
    

  return start_state

def breadth_first_search(start_state):
  frontier = []
  expansion_count = 0

  heapq.heappush(frontier,
                (0, (start_state, [])))

  while frontier:
    curr_value, curr_node = heapq.heappop(frontier)

    curr_state = curr_node[0]
    curr_path = curr_node[1]

    expansion_count += 1  # x = x + 1

    if curr_state == goal_state:
      return curr_path, expansion_count

    # If it doesn't pass the goal test, we move on to generating child nodes.
    # Get available actions from current state
    curr_actions = Get_Actions()

    if len(curr_path) > 0:
        last_action = curr_path[-1] # This returns the last element in curr_path

        for action in curr_actions:
         if action == last_action:
            if action[-1] == "L":
               curr_actions.remove(action[:-1] + "R")
            elif action[-1] == "R":
               curr_actions.remove(action[:-1] + "L")
            elif action[-1] == "U":
               curr_actions.remove(action[:-1] + "D")
            elif action[-1] == "D":
               curr_actions.remove(action[:-1] + "U")

    # Generate child node from each action
    for action in curr_actions:

      next_state = Transition_Model(curr_state, action)
      next_path = copy.deepcopy(curr_path)
      next_path.append(action)
      next_value = len(next_path)

      # Add child to priority queue (our frontier)
      heapq.heappush(frontier, (next_value, (next_state, next_path)))

def h1(state):
    corners_out_of_place = 0
    goal_corners = [{"W","B","R"},{"W","R","G"},{"W","G","O"},{"W","O","B"},{"Y","G","R"},{"Y","R","B"},{"Y","B","O"},{"Y","O","G"}]
        #         F-T_L          F-T-R        F-B-R.        F-B-L          B-T-L         B-T-R         B-B-R         B-B-L
    state_corners = [[state[0][0][0],state[1][0][2],state[4][2][0]], [state[0][0][2],state[4][2][2],state[3][0][0]], [state[0][2][2],state[3][2][0],state[5][0][2]], [state[0][2][0],state[5][0][0],state[1][2][2]],
                     [state[2][0][0],state[3][0][2],state[4][0][2]], [state[2][0][2],state[4][0][0],state[1][0][0]], [state[2][2][2],state[1][2][0],state[5][2][0]], [state[2][2][0],state[5][2][2],state[3][2][2]]]
    
    for corner_i, corner in enumerate(state_corners):
       for side_i, side in enumerate(corner):
          if side in goal_corners[corner_i]:
             corners_out_of_place +=0
          else:
             corners_out_of_place +=1
             

    return corners_out_of_place

def h2(state):
   estimated_distance = 0

   for side_i, side in enumerate(state):
        for row_i, row in enumerate(side):
            for col_i, tile in enumerate(row):
               
               if tile == goal_state[side_i][row_i][col_i]:
                  estimated_distance += 0
               else:
                  for goal_side_i, goal_side in enumerate(goal_state):
                     for goal_row_i, goal_row in enumerate(goal_side):
                        for goal_col_i, goal_tile in enumerate(goal_row):
                           
                           if tile == goal_tile:
                              if (side_i == 0 and goal_side_i == 2) or (side_i == 2 and goal_side_i == 0):
                                 estimated_distance += 2
                              elif (side_i == 1 and goal_side_i == 3) or (side_i == 3 and goal_side_i == 1):
                                 estimated_distance += 2
                              elif (side_i == 4 and goal_side_i == 5) or (side_i == 5 and goal_side_i == 4):
                                 estimated_distance += 2
                              else:
                                 estimated_distance += 1
                            
                              #row_dist = row_i - goal_row_i
                              #col_dist = col_i - goal_col_i
                              #estimated_distance += side_dist
                              #estimated_distance += abs(row_dist)
                              #estimated_distance += abs(col_dist)

   return estimated_distance
                                 
def h3(state):
   rows_out_of_place = 0

   for side_i, side in enumerate(goal_state):
     for row_i, correct_row in enumerate(side):
        if correct_row[1] != state[side_i][row_i][1]:
            rows_out_of_place += 1
                
   return rows_out_of_place

def h4(state):
   estimated_distance = 0

   colors = {"W": 0, "B": 1, "Y": 2, "G": 3, "R": 4, "O": 5}

   for side_i, side in enumerate(state):
        for row_i, row in enumerate(side):
            for col_i, tile in enumerate(row):
               if tile == goal_state[side_i][row_i][col_i]:
                  estimated_distance += 0
               else:
                   if (side_i == 0 and colors[tile] == 2) or (side_i == 2 and colors[tile] == 0):
                       estimated_distance += 2
                   elif (side_i == 1 and colors[tile] == 3) or (side_i == 3 and colors[tile] == 1):
                       estimated_distance += 2
                   elif (side_i == 4 and colors[tile] == 5) or (side_i == 5 and colors[tile] == 4):
                       estimated_distance += 2
                   else:
                       estimated_distance += 1

   return estimated_distance

def A_star_search(start_state, heuristic1, heruistic2):
  """
  # A* search algorithm: choose the cheapest node from the frontier based on
  # heuristic and the backward cost, and expand that node next.
  """
  expanded_states = set()
  frontier = []

  expansion_count = 0

  # First, we generate the root node of the search tree: start_state.
  heapq.heappush(frontier,
                 (max(heuristic1(start_state),heruistic2(start_state)), (start_state, [])))
  expanded_states.add(str(start_state))

  # As long as the frontier is not empty, this loop will continue.
  while frontier:

    # Expand the node with the smallest estimated cost according to the heuristic
    # curr_node = (curr_state, path_to_curr_state_from_start)
    curr_value, curr_node = heapq.heappop(frontier)
    curr_state = curr_node[0]
    curr_path = curr_node[1]

    expansion_count += 1

    # Check to see if we just expanded a goal node
    if Goal_Test(curr_state):
      return curr_path, expansion_count

    # If it doesn't pass the goal test, we move on to generating child nodes.
    # Get available actions from current state
    curr_actions = Get_Actions()

    # To help mitigate expanding nodes which have themselves as grandparent,
    # let's make sure to not consider any action which simply reverses whatever
    # the last action was. For example, if we just moved the blank up a space,
    # then it would be kind of silly to generate a node by moving the blank back
    # down, right? Note that we shouldn't do this if we are expanding the start
    # node, since no prior actions have been taken yet, which is the case when
    # len(curr_path) == 0.
    if len(curr_path) > 0:
        last_action = curr_path[-1] # This returns the last element in curr_path

        for action in curr_actions:
         if action == last_action:
            if action[-1] == "L":
               curr_actions.remove(action[:-1] + "R")
            elif action[-1] == "R":
               curr_actions.remove(action[:-1] + "L")
            elif action[-1] == "U":
               curr_actions.remove(action[:-1] + "D")
            elif action[-1] == "D":
               curr_actions.remove(action[:-1] + "U")

    # Generate child node from each action
    for action in curr_actions:

      next_state = Transition_Model(curr_state, action)
      next_path = copy.deepcopy(curr_path)
      next_path.append(action)
      next_value = max(heuristic1(next_state),heruistic2(next_state)) + len(next_path)

      if str(next_state) in expanded_states:
        continue

      # Add child to priority queue (our frontier)
      heapq.heappush(frontier, (next_value, (next_state, next_path)))

      expanded_states.add(str(next_state))

  return ['boo'], -1


if __name__ == '__main__':
    """
    start_state = [[["W","B","B"],
                    ["G","W","R"],
                    ["O","B","R"]], 
                    
                    [["O","O","G"],
                     ["Y","B","Y"],
                     ["R","W","W"]],
                     
                     [["Y","W","G"],
                      ["O","Y","R"],
                      ["G","R","B"]],
                      
                      [["Y","G","R"],
                       ["G","G","W"],
                       ["Y","Y","R"]],
                       
                       [["Y","B","B"],
                        ["Y","R","O"],
                        ["O","R","O"]],
                        
                        [["B","O","G"],
                         ["G","O","B"],
                         ["W","W","W"]]]
                        """
    start_state = generate_random_start_w_fixed_depth(5)
    #Print_State(start_state)

    #print("h1: ")
    #AStar_h1_solution, AStar_h1_expansions = A_star_search(start_state, h1)
    #print(AStar_h1_solution)
    #print(len(AStar_h1_solution))
    #print('num expansions: ' + str(AStar_h1_expansions))

    #print("h2: ")
    #AStar_h2_solution, AStar_h2_expansions = A_star_search(start_state, h2)
    #print(AStar_h2_solution)
    #print(len(AStar_h2_solution))
    #print('num expansions: ' + str(AStar_h2_expansions))

    print("h3: ")
    AStar_h3_solution, AStar_h3_expansions = A_star_search(start_state, h3, h1)
    print(AStar_h3_solution)
    print(len(AStar_h3_solution))
    print('num expansions: ' + str(AStar_h3_expansions))

    #print("h4: ")
    #AStar_h4_solution, AStar_h4_expansions = A_star_search(start_state, h4)
    #print(AStar_h4_solution)
    #print(len(AStar_h4_solution))
   #print('num expansions: ' + str(AStar_h4_expansions))
   
