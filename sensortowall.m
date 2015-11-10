% This function returns the perpendicular distance from the sensors to the relavant walls
% MA Kurien ma581


function [r_ls1,r_rs1] = sensortowall(relevantEdgePoints,sensorPosition)
  
%Need to calculate distance from line (wall) to point:    
%       Let Q1 and Q2 be endpoints (or any two distinct points) of the line segment 
%       and P the point in question, then
%               d = abs(cross(Q2-Q1,P-Q1))/abs(Q2-Q1);
    
%     r_ls1 = norm(p_s1-p_ls1); %Left sensor to light source
%     r_rs1 = norm(p_s2-p_ls1); %Right sensor to light source

%Left
    Q2 = relevantEdgePoints(2,:);
    Q1 = relevantEdgePoints(1,:);
    P =  sensorPosition(1,:);
    r_ls1 = norm(cross(Q2-Q1,P-Q1))/norm(Q2-Q1); %Left sensor to wall
    
 %Right
    Q2 = relevantEdgePoints(4,:);
    Q1 = relevantEdgePoints(3,:);
    P =  sensorPosition(2,:);
    r_rs1 = norm(cross(Q2-Q1,P-Q1))/norm(Q2-Q1); %Right sensor to wall
    


end