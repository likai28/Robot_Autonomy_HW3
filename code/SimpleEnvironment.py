import numpy
import pylab as pl
from DiscreteEnvironment import DiscreteEnvironment

class SimpleEnvironment(object):
    
    def __init__(self, herb, resolution):
        self.robot = herb.robot
        self.lower_limits = [-5., -5.]
        self.upper_limits = [5., 5.]
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)

        # add an obstacle
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 1.5], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)

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

    def GetValidSuccessors(self, node_id):
	successors = self.GetSuccessors(node_id)
	successors = [x for x in successors if self.ComputeDistance(node_id,x) != float("inf")]
	return successors

    def ComputeDistance(self, start_id, end_id):
	epsilon = 0.01
	start = self.discrete_env.NodeIdToConfiguration(start_id) 
	end = self.discrete_env.NodeIdToConfiguration(end_id) 
	with self.robot:
		tform = self.robot.GetTransform()
		vec = end-start
		veclen = numpy.linalg.norm(vec)
		vec = vec/veclen
		travel = 0
		while travel < veclen:
			val = start + vec*travel
			tform[0:2,3] = val 
			self.robot.SetTransform(tform)
			if self.robot.GetEnv().CheckCollision(self.robot) == True:
				return float("inf") 
			travel += epsilon
	return self.ComputeHeuristicCost(start_id, end_id)	

    def ComputeHeuristicCost(self, start_id, goal_id):
        
        cost = 0
	start = numpy.array(self.discrete_env.NodeIdToConfiguration(start_id))
	end = numpy.array(self.discrete_env.NodeIdToConfiguration(goal_id)) 	
	return numpy.linalg.norm(end-start)

    def ComputeManhattan(self, start_id, goal_id):
      start = numpy.array(self.discrete_env.NodeIdToGridCoord(start_id))
      end = numpy.array(self.discrete_env.NodeIdToGridCoord(goal_id))
      return abs(start[0]-end[0])+abs(start[1]-end[1])


    def InitializePlot(self, goal_config):
        self.fig = pl.figure()
        pl.xlim([self.lower_limits[0], self.upper_limits[0]])
        pl.ylim([self.lower_limits[1], self.upper_limits[1]])
        pl.plot(goal_config[0], goal_config[1], 'gx')

        # Show all obstacles in environment
        for b in self.robot.GetEnv().GetBodies():
            if b.GetName() == self.robot.GetName():
                continue
            bb = b.ComputeAABB()
            pl.plot([bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0]],
                    [bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1]], 'r')
                    
                     
        pl.ion()
        pl.show()
        
    def PlotEdge(self, sconfig, econfig):
        pl.plot([sconfig[0], econfig[0]],
                [sconfig[1], econfig[1]],
                'k.-', linewidth=2.5)
        pl.draw()

        
