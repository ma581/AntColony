% This is OLD and NOT USED

% This will carry out OBSTACLE AVOIDANCE to output the wheel speeds omega_l
% and omega_R
% This is the ANT ROBOT CONTROLLER
% MA Kurien (ma581)



function [omega_l,omega_r] = omegas(ant,maze,pher)


     % Random noise generation
        rand_omega_r = rand(1);
        rand_omega_l = rand(1);

    %1. Function to look at mazeEdgeList and identify relevant mazeEdges
        % Output relevantEdgePoints = [ Q1 ; Q2 ; Q3 ; Q4];
    
        
    %2. Function to calculate distances from sensors to mazeEdges
    [r_ls1,r_rs1] = sensortowall(relevantEdgePoints,[p_s1;p_s2]);
        
        % Controller from Braitenberg
            obstacle_omega_l = 1/r_ls1^2*ant.Rho +1;
            obstacle_omega_r = 1/r_rs1^2*ant.Rho +1;

    %3. Function to calculate angle to strongest pheremone trail in the 3x3
    %grid the ant is on
        % Output angleToPher = angle; % If>1 more gain for left wheel else more gain for right weel
            [r_lp1,r_rp1] = sensortopher(pher,[p_s1;p_s2]);

      % Controller from Braitenberg
        pher_omega_l = 1/r_lp1^2*ant.Rho +1;
        pher_omega_r = 1/r_rp1^2*ant.Rho +1;      
        
        
    %4. Ant to goal
%     distanceFromGoal = abs(maze.Goal - ant.p_c(1,1:2)); %distance from ant to maze exit
%     bearingToGoal = acos(dot([ant.p_c(1,1),1000]-ant.p_c(1,1:2))/(norm([ant.p_c(1,1),1000])*norm(ant.p_c(1,1:2))));
%     angleToGoal = bearingToGoal - ant.p_c(1,3); % angle from ant orientation to maze exit. If>1 more gain for left wheel else more gain for right weel    
        
       [r_lg1,r_rg1] = sensortogoal(pher,[p_s1;p_s2]);

      % Controller from Braitenberg
        goal_omega_l = 1/r_lg1^2*ant.Rho +1;
        goal_omega_r = 1/r_rg1^2*ant.Rho +1;   

        
    %Use the results and antParameters to calculate omega_l and omega_r
    %If both distances to a wall is below threshold, turn left/right
    
    omega_l = ant.GoalGain * goal_omega_l + ...
              ant.PheremoneGain * pher_omega_l + ...
              ant.ObstacleAvoidanceGain * obstacle_omega_l + ...
              ant.RandomMotionGain * rand_omega_l;
     
      omega_r = ant.GoalGain * goal_omega_r + ...
                ant.PheremoneGain * pher_omega_r + ...
                ant.ObstacleAvoidanceGain * obstacle_omega_r + ...
                ant.RandomMotionGain * rand_omega_r;
          
                     
                                                
    
    
    
    %Ant to walls - if condition if heading straight to wall
    
    
    
    
end