% This will carry out OBSTACLE AVOIDANCE to output the wheel speeds omega_l
% and omega_R
% MA Kurien (ma581)





function [omega_l,omega_r] = omegas(ant,maze,pher)


    %1. Function to look at mazeEdgeList and identify relevant mazeEdges
        % Output relevantEdgePoints = [ Q1 ; Q2 ; Q3 ; Q4];
    
        
    %2. Function to calculate distances from sensors to mazeEdges
    [r_ls1,r_rs1] = sensortowall(relevantEdgePoints,[p_s1;p_s2]);
    
    
    %3. Function to calculate angle to strongest pheremone trail in the 3x3
    %grid the ant is on
        % Output angleToPher = angle; % If>1 more gain for left wheel else more gain for right weel
        
    %4. Ant to goal
    distanceFromGoal = abs(maze.Goal - ant.p_c(1,1:2)); %distance from ant to maze exit
    bearingToGoal = acos(dot([ant.p_c(1,1),1000]-ant.p_c(1,1:2))/(norm([ant.p_c(1,1),1000])*norm(ant.p_c(1,1:2))));
    angleToGoal = bearingToGoal - ant.p_c(1,3); % angle from ant orientation to maze exit. If>1 more gain for left wheel else more gain for right weel    
        
        
    %Use the results and antParameters to calculate omega_l and omega_r
    %If both distances to a wall is below threshold, turn left/right
    

                                                
    
    
    
    %Ant to walls - if condition if heading straight to wall
    
    
    
    
end