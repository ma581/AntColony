% This will carry out OBSTACLE AVOIDANCE to output the wheel speeds omega_l
% and omega_R
% MA Kurien (ma581)


%ANT STRUCTURE
% ant.Position = (x,y)
% ant.Orientation = theta
% ant.GoalGain
% ant.PheremoneGain
% ant.ObstacleAvoidanceGain
% ant.RandomMotionGain

%MAZE STRUCTURE
% maze.Walls = n by m matrix (boolean)
% maze.EdgeList = x by 4 matrix of connected points
% maze.Goal = exit/food position

%PHEREMONE STRUCTURE
% pher.DecayRate
% pher.Matrix = n by m matrix with pheremone strengths

function [omega_l,omega_r] = omegas(ant,maze,pher)


    %Function to look at mazeEdgeList and identify relevant mazeEdges
    
    
    %Function to calculate distances from sensors to mazeEdges
    
    
    %Use the results and antParameters to calculate omega_l and omega_r
    %If centre sensor is below threshold, turn left/right

end