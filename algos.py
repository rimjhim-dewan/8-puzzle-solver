
import numpy as np
import time
class Node():
    def __init__(self,state,parent,action,depth,step_cost,path_cost, heuristic_cost):
        self.state=state
        self.parent=parent
        self.action=action
        self.depth=depth
        self.step_cost=step_cost
        self.path_cost=path_cost
        self.heuristic_cost=heuristic_cost

        self.move_up=None
        self.move_left=None
        self.move_down=None
        self.move_right=None

    def try_move_down(self):
        zero_index=[i[0] for i in np.where(self.state==0)] 
        if zero_index[0] == 0:
            return False
        else:
            up_value = self.state[zero_index[0]-1,zero_index[1]] 
            new_state = self.state.copy()
            new_state[zero_index[0],zero_index[1]] = up_value
            new_state[zero_index[0]-1,zero_index[1]] = 0
            return new_state,up_value
        
    def try_move_right(self):
        zero_index=[i[0] for i in np.where(self.state==0)] 
        if zero_index[1] == 0:
            return False
        else:
            left_value = self.state[zero_index[0],zero_index[1]-1] 
            new_state = self.state.copy()
            new_state[zero_index[0],zero_index[1]] = left_value
            new_state[zero_index[0],zero_index[1]-1] = 0
            return new_state,left_value
        
    def try_move_up(self):
        zero_index=[i[0] for i in np.where(self.state==0)] 
        if zero_index[0] == 2:
            return False
        else:
            lower_value = self.state[zero_index[0]+1,zero_index[1]] 
            new_state = self.state.copy()
            new_state[zero_index[0],zero_index[1]] = lower_value
            new_state[zero_index[0]+1,zero_index[1]] = 0
            return new_state,lower_value
        
    def try_move_left(self):
        zero_index=[i[0] for i in np.where(self.state==0)] 
        if zero_index[1] == 2:
            return False
        else:
            right_value = self.state[zero_index[0],zero_index[1]+1] 
            new_state = self.state.copy()
            new_state[zero_index[0],zero_index[1]] = right_value
            new_state[zero_index[0],zero_index[1]+1] = 0
            return new_state,right_value

    def h_manhattan_cost(self,new_state,goal_state):
        current = new_state
        # digit and coordinates they are supposed to be
        goal_position_dic = {1:(0,0),2:(0,1),3:(0,2),4:(1,0),5:(1,1),6:(1,2),7:(2,0),8:(2,1),0:(2,2)} 
        sum_manhattan = 0
        for i in range(3):
            for j in range(3):
                if current[i,j] != 0:
                    sum_manhattan += sum(abs(a-b) for a,b in zip((i,j), goal_position_dic[current[i,j]]))
        return sum_manhattan

    def h_misplaced_cost(self,new_state,goal_state):
        cost = np.sum(new_state != goal_state)-1 # minus 1 to exclude the empty tile
        if cost > 0:
            return cost
        else:
            return 0

    def get_h_cost(self,new_state,goal_state,heuristic_function,path_cost,depth):
        if heuristic_function == 'num_misplaced':
            return self.h_misplaced_cost(new_state,goal_state)
        elif heuristic_function == 'manhattan':
            return self.h_manhattan_cost(new_state,goal_state)
        # since this game is made unfair by setting the step cost as the value of the tile being moved
        # to make it fair, I made all the step cost as 1
        # made it a best-first-search with manhattan heuristic function
        elif heuristic_function == 'fair_manhattan':
            return self.h_manhattan_cost(new_state,goal_state) - path_cost + depth
        
    def print_path(self):
        state_trace=[self.state]
        action_trace=[self.action]
        depth_trace=[self.depth]
        step_cost_trace=[self.step_cost]
        path_cost_trace=[self.path_cost]
        heuristic_cost_trace=[self.heuristic_cost]
        #refactor with one stack of objects
        while self.parent:
            self=self.parent
            state_trace.append(self.state)
            action_trace.append(self.action)
            depth_trace.append(self.depth)
            step_cost_trace.append(self.step_cost)
            path_cost_trace.append(self.path_cost)
            heuristic_cost_trace.append(self.heuristic_cost)
        result =[]
        dep=[]
        heur=[]
        pat=[]
        step_counter = 0
        while state_trace:
            #print('step',step_counter)
            result.append(state_trace.pop())
            dep.append(depth_trace.pop())
            pat.append(path_cost_trace.pop())
            heur.append(heuristic_cost_trace.pop())
##            #print('action=',action_trace.pop(),', depth=',str(depth_trace.pop()),\
##            ', step cost=',str(step_cost_trace.pop()),', total_cost=',\
##            str(path_cost_trace.pop() + heuristic_cost_trace.pop()),'\n')
            
            step_counter += 1
        data=[dep,heur,pat]
        return result,data

    def breadth_first_search(self, goal_state):
        start = time.time()
        
        queue = [self] # queue of found but unvisited nodes, FIFO
        queue_num_nodes_popped = 0 # number of nodes popped off the queue, measuring time performance
        queue_max_length = 1 # max number of nodes in the queue, measuring space performance
        
        depth_queue = [0] # queue of node depth
        path_cost_queue = [0] # queue for path cost
        visited = set([]) # record visited states
        count=0
        while queue:
            count+=1
            if count==8000:
                        print("too hard a problem for this approach")
                        return False, False, False, False
            #print(queue[-1].state)
            # update maximum length of the queue
            if len(queue) > queue_max_length:
                queue_max_length = len(queue)
                
            current_node = queue.pop(0) # select and remove the first node in the queue
            queue_num_nodes_popped += 1 
            
            current_depth = depth_queue.pop(0) # select and remove the depth for current node
            current_path_cost = path_cost_queue.pop(0) # # select and remove the path cost for reaching current node
            visited.add(tuple(current_node.state.reshape(1,9)[0])) # avoid repeated state, which is represented as a tuple
            
            # when the goal state is found, trace back to the root node and print out the path
            if np.array_equal(current_node.state,goal_state):
                states,data=(current_node.print_path())
                end_t=time.time()-start
                print("count",count)
                return states,data,end_t,count
            
##                print('Time performance:',str(queue_num_nodes_popped),'nodes popped off the queue.')
##                print('Space performance:', str(queue_max_length),'nodes in the queue at its max.')
##                print('Time spent: %0.2fs' % (time.time()-start))
##                return True
            else:                
                # see if moving upper tile down is a valid move
                if current_node.try_move_down():
                    new_state,up_value = current_node.try_move_down()
                    # check if the resulting node is already visited
                    if tuple(new_state.reshape(1,9)[0]) not in visited:
                        # create a new child node
                        current_node.move_down = Node(state=new_state,parent=current_node,action='down',depth=current_depth+1,\
                                              step_cost=up_value,path_cost=current_path_cost+up_value,heuristic_cost=0)
                        queue.append(current_node.move_down)
                        depth_queue.append(current_depth+1)
                        path_cost_queue.append(current_path_cost+up_value)
                    
                # see if moving left tile to the right is a valid move
                if current_node.try_move_right():
                    new_state,left_value = current_node.try_move_right()
                    # check if the resulting node is already visited
                    if tuple(new_state.reshape(1,9)[0]) not in visited:
                        # create a new child node
                        current_node.move_right = Node(state=new_state,parent=current_node,action='right',depth=current_depth+1,\
                                              step_cost=left_value,path_cost=current_path_cost+left_value,heuristic_cost=0)
                        queue.append(current_node.move_right)
                        depth_queue.append(current_depth+1)
                        path_cost_queue.append(current_path_cost+left_value)
                 
                # see if moving lower tile up is a valid move
                if current_node.try_move_up():
                    new_state,lower_value = current_node.try_move_up()
                    # check if the resulting node is already visited
                    if tuple(new_state.reshape(1,9)[0]) not in visited:
                        # create a new child node
                        current_node.move_up = Node(state=new_state,parent=current_node,action='up',depth=current_depth+1,\
                                              step_cost=lower_value,path_cost=current_path_cost+lower_value,heuristic_cost=0)
                        queue.append(current_node.move_up)
                        depth_queue.append(current_depth+1)
                        path_cost_queue.append(current_path_cost+lower_value)

                if current_node.try_move_left():
                    new_state,right_value = current_node.try_move_left()
                # check if the resulting node is already visited
                    if tuple(new_state.reshape(1,9)[0]) not in visited:
                        # create a new child node
                        current_node.move_left = Node(state=new_state,parent=current_node,action='left',depth=current_depth+1,\
                                              step_cost=right_value,path_cost=current_path_cost+right_value,heuristic_cost=0)
                        queue.append(current_node.move_left)
                        depth_queue.append(current_depth+1)
                        path_cost_queue.append(current_path_cost+right_value)

    def best_first_search(self, goal_state):
        start = time.time()
        
        queue = [(self,0)] # queue of (found but unvisited nodes, heuristic cost), ordered by heuristic cost
        queue_num_nodes_popped = 0 # number of nodes popped off the queue, measuring time performance
        queue_max_length = 1 # max number of nodes in the queue, measuring space performance
        
        depth_queue = [(0,0)] # queue of node depth, (depth, heuristic cost)
        path_cost_queue = [(0,0)] # queue for path cost, (path_cost, heuristic cost)
        visited = set([]) # record visited states
        count=0
        while queue:
            count+=1
            if count==8000:
                        print("too hard a problem for this approach")
                        return False, False, False, False
            print(queue[-1][0].state)
            # sort queue based on heuristic cost, in ascending order
            queue = sorted(queue, key=lambda x: x[1])
            depth_queue = sorted(depth_queue, key=lambda x: x[1])
            path_cost_queue = sorted(path_cost_queue, key=lambda x: x[1])
            
            # update maximum length of the queue
            if len(queue) > queue_max_length:
                queue_max_length = len(queue)
                
            current_node = queue.pop(0)[0] # select and remove the first node in the queue
            #print 'pop'
            #print current_node.state
            #print 'heuristic_cost',current_node.heuristic_cost,'\n'
            
            queue_num_nodes_popped += 1 
            current_depth = depth_queue.pop(0)[0] # select and remove the depth for current node
            current_path_cost = path_cost_queue.pop(0)[0] # # select and remove the path cost for reaching current node
            visited.add(tuple(current_node.state.reshape(1,9)[0])) # avoid repeated state, which is represented as a tuple
            
            # when the goal state is found, trace back to the root node and print out the path
            if np.array_equal(current_node.state,goal_state):
                states,data=(current_node.print_path())
                end_t=time.time()-start
                print("count",count)
                return states,data,end_t,count
                
##                print('Time performance:',str(queue_num_nodes_popped),'nodes popped off the queue.')
##                print('Space performance:', str(queue_max_length),'nodes in the queue at its max.')
##                print('Time spent: %0.2fs' % (time.time()-start))
##                return True
            else:     
                # see if moving upper tile down is a valid move
                if current_node.try_move_down():
                    new_state,up_value = current_node.try_move_down()
                    # check if the resulting node is already visited
                    if tuple(new_state.reshape(1,9)[0]) not in visited:
                        # get heuristic cost
                        h_cost = self.h_misplaced_cost(new_state,goal_state)
                        # create a new child node
                        current_node.move_down = Node(state=new_state,parent=current_node,action='down',depth=current_depth+1,\
                                              step_cost=up_value,path_cost=current_path_cost+up_value,heuristic_cost=h_cost)
                        queue.append((current_node.move_down,h_cost))
                        depth_queue.append((current_depth+1,h_cost))
                        path_cost_queue.append((current_path_cost+up_value,h_cost))
                    
                # see if moving left tile to the right is a valid move
                if current_node.try_move_right():
                    new_state,left_value = current_node.try_move_right()
                    # check if the resulting node is already visited
                    if tuple(new_state.reshape(1,9)[0]) not in visited:
                        # get heuristic cost
                        h_cost = self.h_misplaced_cost(new_state,goal_state)
                        # create a new child node
                        current_node.move_right = Node(state=new_state,parent=current_node,action='right',depth=current_depth+1,\
                                              step_cost=left_value,path_cost=current_path_cost+left_value,heuristic_cost=h_cost)
                        queue.append((current_node.move_right,h_cost))
                        depth_queue.append((current_depth+1,h_cost))
                        path_cost_queue.append((current_path_cost+left_value,h_cost))
                    
                # see if moving lower tile up is a valid move
                if current_node.try_move_up():
                    new_state,lower_value = current_node.try_move_up()
                    # check if the resulting node is already visited
                    if tuple(new_state.reshape(1,9)[0]) not in visited:
                        # get heuristic cost
                        h_cost = self.h_misplaced_cost(new_state,goal_state)
                        # create a new child node
                        current_node.move_up = Node(state=new_state,parent=current_node,action='up',depth=current_depth+1,\
                                              step_cost=lower_value,path_cost=current_path_cost+lower_value,heuristic_cost=h_cost)
                        queue.append((current_node.move_up,h_cost))
                        depth_queue.append((current_depth+1,h_cost))
                        path_cost_queue.append((current_path_cost+lower_value,h_cost))
                        
                if current_node.try_move_left():
                    new_state,right_value = current_node.try_move_left()
                    # check if the resulting node is already visited
                    if tuple(new_state.reshape(1,9)[0]) not in visited:
                        # get heuristic cost
                        h_cost = self.h_misplaced_cost(new_state,goal_state)
                        # create a new child node
                        current_node.move_left = Node(state=new_state,parent=current_node,action='left',depth=current_depth+1,\
                                              step_cost=right_value,path_cost=current_path_cost+right_value,heuristic_cost=h_cost)
                        queue.append((current_node.move_left,h_cost))
                        depth_queue.append((current_depth+1,h_cost))
                        path_cost_queue.append((current_path_cost+right_value,h_cost))

    def depth_first_search(self, goal_state):
        start = time.time()
        
        queue = [self] # queue of found but unvisited nodes, FILO
        queue_num_nodes_popped = 0 # number of nodes popped off the queue, measuring time performance
        queue_max_length = 1 # max number of nodes in the queue, measuring space performance
        
        depth_queue = [0] # queue of node depth
        path_cost_queue = [0] # queue for path cost
        visited = set([]) # record visited states

        count=1
        while queue:
            count+=1
            if count==8000:
                print("too hard a problem for this approach")
                return False, False, False, False
            # update maximum length of the queue
            if len(queue) > queue_max_length:
                queue_max_length = len(queue)
                
            current_node = queue.pop(0) # select and remove the first node in the queue
            queue_num_nodes_popped += 1 
            
            current_depth = depth_queue.pop(0) # select and remove the depth for current node
            current_path_cost = path_cost_queue.pop(0) # # select and remove the path cost for reaching current node
            visited.add(tuple(current_node.state.reshape(1,9)[0])) # add state, which is represented as a tuple
            
            # when the goal state is found, trace back to the root node and print out the path
            if np.array_equal(current_node.state,goal_state):
                #current_node.print_path()
                states,data=(current_node.print_path())
                end_t=time.time()-start
                print("count",count)
                return states,data,end_t,count
                
##                print('Time performance:',str(queue_num_nodes_popped),'nodes popped off the queue.')
##                print('Space performance:', str(queue_max_length),'nodes in the queue at its max.')
##                print('Time spent: %0.2fs' % (time.time()-start))
##                return True
            else:                
                # see if moving upper tile down is a valid move
                if current_node.try_move_down():
                    new_state,up_value = current_node.try_move_down()
                    # check if the resulting node is already visited
                    if tuple(new_state.reshape(1,9)[0]) not in visited:
                        # create a new child node
                        current_node.move_down = Node(state=new_state,parent=current_node,action='down',depth=current_depth+1,\
                                              step_cost=up_value,path_cost=current_path_cost+up_value,heuristic_cost=0)
                        queue.insert(0,current_node.move_down)
                        depth_queue.insert(0,current_depth+1)
                        path_cost_queue.insert(0,current_path_cost+up_value)
                    
                # see if moving left tile to the right is a valid move
                if current_node.try_move_right():
                    new_state,left_value = current_node.try_move_right()
                    # check if the resulting node is already visited
                    if tuple(new_state.reshape(1,9)[0]) not in visited:
                        # create a new child node
                        current_node.move_right = Node(state=new_state,parent=current_node,action='right',depth=current_depth+1,\
                                              step_cost=left_value,path_cost=current_path_cost+left_value,heuristic_cost=0)
                        queue.insert(0,current_node.move_right)
                        depth_queue.insert(0,current_depth+1)
                        path_cost_queue.insert(0,current_path_cost+left_value)
                 
                # see if moving lower tile up is a valid move
                if current_node.try_move_up():
                    new_state,lower_value = current_node.try_move_up()
                    # check if the resulting node is already visited
                    if tuple(new_state.reshape(1,9)[0]) not in visited:
                        # create a new child node
                        current_node.move_up = Node(state=new_state,parent=current_node,action='up',depth=current_depth+1,\
                                              step_cost=lower_value,path_cost=current_path_cost+lower_value,heuristic_cost=0)
                        queue.insert(0,current_node.move_up)
                        depth_queue.insert(0,current_depth+1)
                        path_cost_queue.insert(0,current_path_cost+lower_value)

                # see if moving right tile to the left is a valid move
                if current_node.try_move_left():
                    new_state,right_value = current_node.try_move_left()
                    # check if the resulting node is already visited
                    if tuple(new_state.reshape(1,9)[0]) not in visited:
                        # create a new child node
                        current_node.move_left = Node(state=new_state,parent=current_node,action='left',depth=current_depth+1,\
                                              step_cost=right_value,path_cost=current_path_cost+right_value,heuristic_cost=0)
                        queue.insert(0,current_node.move_left)
                        depth_queue.insert(0,current_depth+1)
                        path_cost_queue.insert(0,current_path_cost+right_value)

    def uniform_cost_search(self, goal_state):
        start = time.time()
        
        queue = [(self,0)] # queue of (found but unvisited nodes, path cost), ordered by path cost(accumulated step cost)
        queue_num_nodes_popped = 0 # number of nodes popped off the queue, measuring time performance
        queue_max_length = 1 # max number of nodes in the queue, measuring space performance
        
        depth_queue = [(0,0)] # queue of node depth, (depth, path cost)
        path_cost_queue = [0] # queue for path cost
        visited = set([]) # record visited states
        count=0
        while queue:
            count+=1
            if count==8000:
                print("too hard a problem for this approach")
                return False, False, False, False
            # sort queue based on path cost, in ascending order
            queue = sorted(queue, key=lambda x: x[1])
            depth_queue = sorted(depth_queue, key=lambda x: x[1])
            path_cost_queue = sorted(path_cost_queue, key=lambda x: x)
            
            # update maximum length of the queue
            if len(queue) > queue_max_length:
                queue_max_length = len(queue)
                
            current_node = queue.pop(0)[0] # select and remove the first node in the queue
            #print 'pop'
            #print current_node.state,'\n'
            queue_num_nodes_popped += 1 
            current_depth = depth_queue.pop(0)[0] # select and remove the depth for current node
            current_path_cost = path_cost_queue.pop(0) # # select and remove the path cost for reaching current node
            visited.add(tuple(current_node.state.reshape(1,9)[0])) # avoid repeated state, which is represented as a tuple
            
            # when the goal state is found, trace back to the root node and print out the path
            if np.array_equal(current_node.state,goal_state):
                #current_node.print_path()
                states,data=(current_node.print_path())
                end_t=time.time()-start
                print("count",count)
                return states,data,end_t,count
##                print 'Time performance:',str(queue_num_nodes_popped),'nodes popped off the queue.'
##                print 'Space performance:', str(queue_max_length),'nodes in the queue at its max.'
##                print 'Time spent: %0.2fs' % (time.time()-start)
##                return True
            
            else:     
                # see if moving upper tile down is a valid move
                if current_node.try_move_down():
                    new_state,up_value = current_node.try_move_down()
                    # check if the resulting node is already visited
                    if tuple(new_state.reshape(1,9)[0]) not in visited:
                        # create a new child node
                        current_node.move_down = Node(state=new_state,parent=current_node,action='down',depth=current_depth+1,\
                                              step_cost=up_value,path_cost=current_path_cost+up_value,heuristic_cost=0)
                        queue.append((current_node.move_down,current_path_cost+up_value))
                        depth_queue.append((current_depth+1,current_path_cost+up_value))
                        path_cost_queue.append(current_path_cost+up_value)
                    
                # see if moving left tile to the right is a valid move
                if current_node.try_move_right():
                    new_state,left_value = current_node.try_move_right()
                    # check if the resulting node is already visited
                    if tuple(new_state.reshape(1,9)[0]) not in visited:
                        # create a new child node
                        current_node.move_right = Node(state=new_state,parent=current_node,action='right',depth=current_depth+1,\
                                              step_cost=left_value,path_cost=current_path_cost+left_value,heuristic_cost=0)
                        queue.append((current_node.move_right,current_path_cost+left_value))
                        depth_queue.append((current_depth+1,current_path_cost+left_value))
                        path_cost_queue.append(current_path_cost+left_value)
                    
                # see if moving lower tile up is a valid move
                if current_node.try_move_up():
                    new_state,lower_value = current_node.try_move_up()
                    # check if the resulting node is already visited
                    if tuple(new_state.reshape(1,9)[0]) not in visited:
                        # create a new child node
                        current_node.move_up = Node(state=new_state,parent=current_node,action='up',depth=current_depth+1,\
                                              step_cost=lower_value,path_cost=current_path_cost+lower_value,heuristic_cost=0)
                        queue.append((current_node.move_up,current_path_cost+lower_value))
                        depth_queue.append((current_depth+1,current_path_cost+lower_value))
                        path_cost_queue.append(current_path_cost+lower_value)

                # see if moving right tile to the left is a valid move
                if current_node.try_move_left():
                    new_state,right_value = current_node.try_move_left()
                    # check if the resulting node is already visited
                    if tuple(new_state.reshape(1,9)[0]) not in visited:
                        # create a new child node
                        current_node.move_left = Node(state=new_state,parent=current_node,action='left',depth=current_depth+1,\
                                              step_cost=right_value,path_cost=current_path_cost+right_value,heuristic_cost=0)
                        queue.append((current_node.move_left,current_path_cost+right_value))
                        depth_queue.append((current_depth+1,current_path_cost+right_value))
                        path_cost_queue.append(current_path_cost+right_value)

    def iterative_deepening_DFS(self, goal_state):
        start = time.time()
        
        queue_num_nodes_popped = 0 # number of nodes popped off the queue, measuring time performance
        queue_max_length = 1 # max number of nodes in the queue, measuring space performance
        
        # search the tree that's 40 levels in depth
        count=0
        for depth_limit in range(40):
            #print 'depth limit',depth_limit
        
            queue = [self] # queue of found but unvisited nodes, FILO
            depth_queue = [0] # queue of node depth
            path_cost_queue = [0] # queue for path cost
            visited = set([]) # record visited states

            while queue:
                count+=1
                if count==50000:
                    print(count)
                    print("too hard a problem for this approach")
                    return False, False, False, False
                # update maximum length of the queue
                if len(queue) > queue_max_length:
                    queue_max_length = len(queue)

                current_node = queue.pop(0) # select and remove the first node in the queue
                #print 'pop'
                #print current_node.state
                queue_num_nodes_popped += 1 

                current_depth = depth_queue.pop(0) # select and remove the depth for current node
                #print 'depth:',current_depth,'\n'
                current_path_cost = path_cost_queue.pop(0) # # select and remove the path cost for reaching current node
                visited.add(tuple(current_node.state.reshape(1,9)[0])) # add state, which is represented as a tuple

                # when the goal state is found, trace back to the root node and print out the path
                if np.array_equal(current_node.state,goal_state):
                    #current_node.print_path()
                    states,data=(current_node.print_path())
                    end_t=time.time()-start
                    print("count",count)
                    return states,data,end_t,count

##                    print 'Time performance:',str(queue_num_nodes_popped),'nodes popped off the queue.'
##                    print 'Space performance:', str(queue_max_length),'nodes in the queue at its max.'
##                    print 'Time spent: %0.2fs' % (time.time()-start)
##                    return True
                else:              
                    if current_depth < depth_limit:
                        
                        # see if moving upper tile down is a valid move
                        if current_node.try_move_down():
                            new_state,up_value = current_node.try_move_down()
                            # check if the resulting node is already visited
                            if tuple(new_state.reshape(1,9)[0]) not in visited:
                                # create a new child node
                                current_node.move_down = Node(state=new_state,parent=current_node,action='down',depth=current_depth+1,\
                                                      step_cost=up_value,path_cost=current_path_cost+up_value,heuristic_cost=0)
                                queue.insert(0,current_node.move_down)
                                depth_queue.insert(0,current_depth+1)
                                path_cost_queue.insert(0,current_path_cost+up_value)

                        # see if moving left tile to the right is a valid move
                        if current_node.try_move_right():
                            new_state,left_value = current_node.try_move_right()
                            # check if the resulting node is already visited
                            if tuple(new_state.reshape(1,9)[0]) not in visited:
                                # create a new child node
                                current_node.move_right = Node(state=new_state,parent=current_node,action='right',depth=current_depth+1,\
                                                      step_cost=left_value,path_cost=current_path_cost+left_value,heuristic_cost=0)
                                queue.insert(0,current_node.move_right)
                                depth_queue.insert(0,current_depth+1)
                                path_cost_queue.insert(0,current_path_cost+left_value)

                        # see if moving lower tile up is a valid move
                        if current_node.try_move_up():
                            new_state,lower_value = current_node.try_move_up()
                            # check if the resulting node is already visited
                            if tuple(new_state.reshape(1,9)[0]) not in visited:
                                # create a new child node
                                current_node.move_up = Node(state=new_state,parent=current_node,action='up',depth=current_depth+1,\
                                                      step_cost=lower_value,path_cost=current_path_cost+lower_value,heuristic_cost=0)
                                queue.insert(0,current_node.move_up)
                                depth_queue.insert(0,current_depth+1)
                                path_cost_queue.insert(0,current_path_cost+lower_value)
                         # see if moving right tile to the left is a valid move
                        if current_node.try_move_left():
                            new_state,right_value = current_node.try_move_left()
                            # check if the resulting node is already visited
                            if tuple(new_state.reshape(1,9)[0]) not in visited:
                                # create a new child node
                                current_node.move_left = Node(state=new_state,parent=current_node,action='left',depth=current_depth+1,\
                                                      step_cost=right_value,path_cost=current_path_cost+right_value,heuristic_cost=0)
                                queue.insert(0,current_node.move_left)
                                depth_queue.insert(0,current_depth+1)
                                path_cost_queue.insert(0,current_path_cost+right_value)

    def a_star_search(self,goal_state,heuristic_function):
        start = time.time()
        
        queue = [(self,0)] # queue of (found but unvisited nodes, path cost+heuristic cost), ordered by the second element
        queue_num_nodes_popped = 0 # number of nodes popped off the queue, measuring time performance
        queue_max_length = 1 # max number of nodes in the queue, measuring space performance
        
        depth_queue = [(0,0)] # queue of node depth, (depth, path_cost+heuristic cost)
        path_cost_queue = [(0,0)] # queue for path cost, (path_cost, path_cost+heuristic cost)
        visited = set([]) # record visited states
        count=0
        while queue:
            count+=1
            if count==50000:
                print(count)
                print("too hard a problem for this approach")
                return False, False, False, False
            count+=1
            # sort queue based on path_cost+heuristic cost, in ascending order
            queue = sorted(queue, key=lambda x: x[1])
            depth_queue = sorted(depth_queue, key=lambda x: x[1])
            path_cost_queue = sorted(path_cost_queue, key=lambda x: x[1])
            
            # update maximum length of the queue
            if len(queue) > queue_max_length:
                queue_max_length = len(queue)
                
            current_node = queue.pop(0)[0] # select and remove the first node in the queue
            #print 'pop'
            #print current_node.state
            #print 'path_cost',current_node.path_cost
            #print 'heuristic_cost',current_node.heuristic_cost
            #print 'total_cost',current_node.path_cost+current_node.heuristic_cost,'\n'
            
            queue_num_nodes_popped += 1 
            current_depth = depth_queue.pop(0)[0] # select and remove the depth for current node
            current_path_cost = path_cost_queue.pop(0)[0] # # select and remove the path cost for reaching current node
            visited.add(tuple(current_node.state.reshape(1,9)[0])) # avoid repeated state, which is represented as a tuple
            
            # when the goal state is found, trace back to the root node and print out the path
            if np.array_equal(current_node.state,goal_state):
                #current_node.print_path()
                states,data=(current_node.print_path())
                end_t=time.time()-start
                print("count",count)
                return states,data,end_t,count
##                print 'Time performance:',str(queue_num_nodes_popped),'nodes popped off the queue.'
##                print 'Space performance:', str(queue_max_length),'nodes in the queue at its max.'
##                print 'Time spent: %0.2fs' % (time.time()-start)
##                return True
            else:     
                # see if moving upper tile down is a valid move
                if current_node.try_move_down():
                    new_state,up_value = current_node.try_move_down()
                    # check if the resulting node is already visited
                    if tuple(new_state.reshape(1,9)[0]) not in visited:
                        path_cost=current_path_cost+up_value
                        depth = current_depth+1
                        # get heuristic cost
                        h_cost = self.get_h_cost(new_state,goal_state,heuristic_function,path_cost,depth)
                        # create a new child node
                        total_cost = path_cost+h_cost
                        current_node.move_down = Node(state=new_state,parent=current_node,action='down',depth=depth,\
                                              step_cost=up_value,path_cost=path_cost,heuristic_cost=h_cost)
                        queue.append((current_node.move_down, total_cost))
                        depth_queue.append((depth, total_cost))
                        path_cost_queue.append((path_cost, total_cost))
                    
                # see if moving left tile to the right is a valid move
                if current_node.try_move_right():
                    new_state,left_value = current_node.try_move_right()
                    # check if the resulting node is already visited
                    if tuple(new_state.reshape(1,9)[0]) not in visited:
                        path_cost=current_path_cost+left_value
                        depth = current_depth+1
                        # get heuristic cost
                        h_cost = self.get_h_cost(new_state,goal_state,heuristic_function,path_cost,depth)
                        # create a new child node
                        total_cost = path_cost+h_cost
                        current_node.move_right = Node(state=new_state,parent=current_node,action='right',depth=depth,\
                                              step_cost=left_value,path_cost=path_cost,heuristic_cost=h_cost)
                        queue.append((current_node.move_right, total_cost))
                        depth_queue.append((depth, total_cost))
                        path_cost_queue.append((path_cost, total_cost))
                    
                # see if moving lower tile up is a valid move
                if current_node.try_move_up():
                    new_state,lower_value = current_node.try_move_up()
                    # check if the resulting node is already visited
                    if tuple(new_state.reshape(1,9)[0]) not in visited:
                        path_cost=current_path_cost+lower_value
                        depth = current_depth+1
                        # get heuristic cost
                        h_cost = self.get_h_cost(new_state,goal_state,heuristic_function,path_cost,depth)
                        # create a new child node
                        total_cost = path_cost+h_cost
                        current_node.move_up = Node(state=new_state,parent=current_node,action='up',depth=depth,\
                                              step_cost=lower_value,path_cost=path_cost,heuristic_cost=h_cost)
                        queue.append((current_node.move_up, total_cost))
                        depth_queue.append((depth, total_cost))
                        path_cost_queue.append((path_cost, total_cost))

                # see if moving right tile to the left is a valid move
                if current_node.try_move_left():
                    new_state,right_value = current_node.try_move_left()
                    # check if the resulting node is already visited
                    if tuple(new_state.reshape(1,9)[0]) not in visited:
                        path_cost=current_path_cost+right_value
                        depth = current_depth+1
                        # get heuristic cost
                        h_cost = self.get_h_cost(new_state,goal_state,heuristic_function,path_cost,depth)
                        # create a new child node
                        total_cost = path_cost+h_cost
                        current_node.move_left = Node(state=new_state,parent=current_node,action='left',depth=depth,\
                                              step_cost=right_value,path_cost=path_cost,heuristic_cost=h_cost)
                        queue.append((current_node.move_left, total_cost))
                        depth_queue.append((depth, total_cost))
                        path_cost_queue.append((path_cost, total_cost))


    

                




