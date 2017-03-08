class DepthFirstPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.nodes = dict()

    def Plan(self, start_config, goal_config):
        
        plan = []

        # TODO: Here you will implement the breadth first planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space

        #Build a dict to record vaild path
        path = dict()
        #Get the start and end node
        start_node = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        goal_node = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)
        #Build the queue to store nodes
        stack = []
        #Add the start node into queue
        stack.append(start_node)
        #Use the nodes dict to store node and visited state
        self.nodes[start_node]=True
        while len(stack)!=0:
            x = stack.pop()
            if x == goal_node:
            # This if only for start_node is goal_node initially
                plan = self.find_path(path,start_node,goal_node)
                return plan
            else:
                nodes_nearby = self.planning_env.GetValidSuccessors(x)
                for node in nodes_nearby:
                    if node in self.nodes and self.nodes[node]==True:
                        continue
                    else:
                        self.nodes[node]=True
                        stack.append(node)
                        #Add the valid edge into path
                        path[node]= x
                        #If this node reach goal_node, get the whole plan
                        if node == goal_node:
                            plan = self.find_path(path,start_node,goal_node)
                            break

        return plan


    def find_path(self, path, start_id, end_id): #[start_id, end_id)
        id_next_v = end_id
        path2 = []
        node_config = self.planning_env.discrete_env.NodeIdToConfiguration(id_next_v)
        path2.append(node_config)
        while(id_next_v != start_id):
            id_next_v = path[id_next_v]
            node_config = self.planning_env.discrete_env.NodeIdToConfiguration(id_next_v)
            path2.append(node_config)
        path2.reverse()
        return path2
