import numpy
from DiscreteEnvironment import DiscreteEnvironment

class HerbEnvironment(object):
    
    def __init__(self, herb, resolution):
        
        self.robot = herb.robot
        self.lower_limits, self.upper_limits = self.robot.GetActiveDOFLimits()
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)

        # account for the fact that snapping to the middle of the grid cell may put us over our
        #  upper limit
        upper_coord = numpy.array([x - 1 for x in self.discrete_env.num_cells])
        upper_config = self.discrete_env.GridCoordToConfiguration(upper_coord)
        for idx in range(len(upper_config)):
            self.discrete_env.num_cells[idx] -= 1

        # add a table and move the robot into place
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        
        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 0.7], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)
        
        # set the camera
        camera_pose = numpy.array([[ 0.3259757 ,  0.31990565, -0.88960678,  2.84039211],
                                   [ 0.94516159, -0.0901412 ,  0.31391738, -0.87847549],
                                   [ 0.02023372, -0.9431516 , -0.33174637,  1.61502194],
                                   [ 0.        ,  0.        ,  0.        ,  1.        ]])
        self.robot.GetEnv().GetViewer().SetCamera(camera_pose)
    
    def GetSuccessors(self, node_id):
        successors = []
	config = self.discrete_env.NodeIdToGridCoord(node_id)
	for i in range(0,self.discrete_env.dimension):
		config[i] += 1
		if config[i] < self.discrete_env.num_cells[i]:
			successors.append(self.discrete_env.GridCoordToNodeId(config))
		config[i] -= 2
		if config[i] >= 0:
			successors.append(self.discrete_env.GridCoordToNodeId(config))
		config[i] += 1
        return successors

    def ComputeDistance(self, start_id, end_id):

	epsilon = 0.1
	start = self.discrete_env.NodeIdToConfiguration(start_id) 
	end = self.discrete_env.NodeIdToConfiguration(end_id) 
	with self.robot:
		cfg = self.robot.GetActiveDOFValues()
		vec = end-start
		veclen = numpy.linalg.norm(vec)
		vec = vec/veclen
		travel = 0
		while travel < veclen:
			val = start + vec*travel
			self.robot.SetActiveDOFValues(val)	
			if self.robot.GetEnv().CheckCollision(self.robot) or self.robot.CheckSelfCollision():
				return float("inf") 
			travel += epsilon
	return self.ComputeHeuristicCost(start_id, end_id)	

    def ComputeHeuristicCost(self, start_id, goal_id):
        
        cost = 0
	start = numpy.array(self.discrete_env.NodeIdToConfiguration(start_id))
	end = numpy.array(self.discrete_env.NodeIdToConfiguration(goal_id)) 	
	return numpy.linalg.norm(end-start)
