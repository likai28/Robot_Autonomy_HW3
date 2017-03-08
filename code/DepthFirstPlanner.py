import time
class DepthFirstPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.nodes = dict()

    def Plan(self, start_config, goal_config):
        start_time = time.time()
        path_length = 0
        nodes_num = 0    

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
        nodes_num += 1
        while len(stack)!=0:
            x = stack.pop()
            if x == goal_node:
            # This if only for start_node is goal_node initially
                plan, path_length = self.find_path(path,start_node,goal_node)
                total_time = time.time() - start_time;
                return (plan,total_time, path_length, nodes_num)
            else:
                nodes_nearby = self.planning_env.GetValidSuccessors(x)
                for node in nodes_nearby:
                    if node in self.nodes and self.nodes[node]==True:
                        continue
                    else:
                        self.nodes[node]=True
                        nodes_num += 1
                        stack.append(node)
                        #Add the valid edge into path
                        path[node]= x
                        #If this node reach goal_node, get the whole plan
                        if node == goal_node:
                            plan, path_length = self.find_path(path,start_node,goal_node)
                            break
        total_time = time.time() - start_time;

        return (plan, total_time, path_length, nodes_num)


    def find_path(self, path, start_id, end_id): #[start_id, end_id)
        path_length =0
        id_next_v = end_id
        path2 = []
        node_config = self.planning_env.discrete_env.NodeIdToConfiguration(id_next_v)
        path2.append(node_config)
        while(id_next_v != start_id):
            path_length += self.planning_env.ComputeDistance(id_next_v,path[id_next_v])
            id_next_v = path[id_next_v]
            node_config = self.planning_env.discrete_env.NodeIdToConfiguration(id_next_v)
            path2.append(node_config)
        path2.reverse()
        return path2, path_length
