%MAIN function


%Initialize structures:
% ant
%ANT STRUCTURE
% ant.p_c = [0.6;0.6;pi/2];  %initial robot position and orientation
% ant.p_c_old = p_c;        %save old state for trajectory
% ant.GoalGain = 0.5;
% ant.PheremoneGain = 0.3;
% ant.ObstacleAvoidanceGain = 0.1;
% ant.RandomMotionGain = 0.1;


% maze
% pheremone


%Generate maze (JW's function)


%Calculate maze.EdgeList (John's function) 


%Optimization and solving:

%Repeat over eg 100 iterations{
    
    % Read pheremone matrix
    % Initialize eg 10 ants
    
        %Iterate over time steps till eg 1000 time steps{
        
            %Iterate over each ant{
                % Calculate wheel omega (Manoj's function)
                % Update ant position
            %}
                
            % Update pheremone matrix (Sarah's function)
        %}
            
  %}
  
  % Visualizing results and animation
            