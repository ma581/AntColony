% This defines the properties of an ANT and its CONTROLLER
% MA Kurien (ma581)

classdef ant < handle
    properties
        %Braitenberg robot properties  
        p_c;            %Array to store x,y,theta position
        p_c_old;        %save old state for trajectory
        p_c_prev;       %Previous position (UNUSED)
        rho = 10;       %light intensity to rotational speed constant
        l_s = 0.1;      %shaft length vehicle
        r_w = 0.02;     %radius wheel
        dt = 1e-3;      %time increment
        d_s = 0.1;      %sensor distance

        
        % Additional properties
        randomMotionGain = 0.1; % Gain for random motion
%         testValue = zeros(1,2);
        straightMotionGain = 5; % Gain to keep driving straight
    end
    
    methods
        function obj = antController(obj,surface,timestep)
            % This will carry out OBSTACLE AVOIDANCE with GRADIENT DECENT for a
            % potential field. This is the ANT CONTROLLER and will update
            % ant.p_c and ant.p_c_old
            % MA Kurien (ma581)

            % Random noise generation
            rand_omega_r = rand(1);
            rand_omega_l = rand(1);
            
            % Find discrete position (ie the index) 
            if timestep>1
                index_y = floor(obj.p_c(1,timestep-1))+1; % +1 to account for indexing starting at 1
                index_x = floor(obj.p_c(2,timestep-1))+1;
                
%                 y = round(obj.p_c(1,1))+1; % +1 to account for indexing starting at 1
%                 x = round(obj.p_c(2,1))+1;

            else
                index_y = floor(obj.p_c(1,timestep))+1; % +1 to account for indexing starting at 1
                index_x = floor(obj.p_c(2,timestep))+1;
                
            end
%                 
            %Reading in neighbours in 3x3 grid
            listOfNearbyPot = []; %init
            
            %Considering which of three cases we are in a 3x3 grid
            if ((index_x-1)>0 && (index_x+1)<=size(surface,2) && (index_y-1)>0 && (index_y+1)<=size(surface,1))
                s = 1;  %Case 1 (middle of 3x3 grid)
                %disp('s= 1');
            elseif((index_y-1)==0 && (index_x-1)>0 && (index_x+1)<size(surface,2) ||...
                    (index_y+1)>size(surface,1) && (index_x-1)>0 && (index_x+1)<size(surface,2) ||...
                    (index_x-1)==0 && (index_y-1)>0 && (index_y+1)<size(surface,1)||...
                    (index_x+1)>size(surface,2) && (index_y-1)>0 && (index_y+1)<size(surface,1))
                s = 2; %Case 2 (edge of 3x3 grid)
                %disp('s= 2');
%             elseif((index_y-1)==0 && (index_x-1)==0 && (index_x+1)<size(surface,2) && (index_y+1)<size(surface,1)||...
%                     (index_y-1)==0 && (index_x-1)>0 && (index_x+1)>size(surface,2) && (index_y+1)<size(surface,1)||...
%                     (index_y-1)>0 && (index_x-1)==0 && (index_x+1)<size(surface,2) && (index_y+1)>size(surface,1)||...
%                     (index_y-1)>0 && (index_x-1)>0 && (index_x+1)>size(surface,2) && (index_y+1)>size(surface,1))
%                 s = 3; %Case 3 (corner)
            else
                s=3;
                disp('s= 3');
            end
            
            
            stepArgDirections = [3*pi/4, pi/2, pi/4, pi, 0, 5*pi/4,3*pi/2, 7*pi/4]; %Argand plot angles
            relArgDirections = []; % the relevant directions for each case
            
            switch s
                case 1 %Case 1 (middle of 3x3 grid)
                    %disp('Case 1');
                    for i = -1:1
                        for j = -1:1
                            listOfNearbyPot = [listOfNearbyPot;surface(index_y+i,index_x+j)];
                        end
                    end
                    
                    listOfNearbyPot(5) = []; %ignoring current position
                    relArgDirections = stepArgDirections; %relevant
                    
                case 2 %Case 2 (edge of 3x3 grid)
                    %disp('Case 2')
                    if (index_x-1)==0 %Middle left
                        for i = -1:1
                            for j = 0:1
                                listOfNearbyPot = [listOfNearbyPot;surface(index_y+i,index_x+j)];
                            end
                        end
                        listOfNearbyPot(3) = []; %ignoring current position
                        relArgDirections = [stepArgDirections(2:3),...
                                            stepArgDirections(5),...
                                            stepArgDirections(7:8)];
                                        
                    elseif (index_x+1)>size(surface,2) %Middle right
                        for i = -1:1
                            for j = -1:0
                                listOfNearbyPot = [listOfNearbyPot;surface(index_y+i,index_x+j)];
                            end
                        end
                        listOfNearbyPot(4) = []; %ignoring current position
                        relArgDirections = [stepArgDirections(1:2),...
                                            stepArgDirections(4),...    
                                            stepArgDirections(6:7)];                                
                                        
                    elseif (index_y-1)==0 %Middle top
                          for i = 0:1
                            for j = -1:1
                                listOfNearbyPot = [listOfNearbyPot;surface(index_y+i,index_x+j)];
                            end
                          end
                        listOfNearbyPot(2) = []; %ignoring current position
                        relArgDirections = [stepArgDirections(4:8)];
                          
                    elseif (index_y+1)>size(surface,1) %Middle bottom
                          for i = -1:0
                            for j = -1:1
                                listOfNearbyPot = [listOfNearbyPot;surface(index_y+i,index_x+j)];
                            end
                          end
                          listOfNearbyPot(5) = []; %ignoring current position
                          relArgDirections = stepArgDirections(1:5);
                    end
  
                case 3
                    %disp('Case 3')
                    if ((index_x-1)==0 && (index_y-1) ==0 )%Top left
                         for i = 0:1
                            for j = 0:1
                                listOfNearbyPot = [listOfNearbyPot;surface(index_y+i,index_x+j)];
                            end
                         end
                         listOfNearbyPot(1) = []; %ignoring current position 
                         relArgDirections = [stepArgDirections(5),stepArgDirections(7),stepArgDirections(8)];
                    
                    elseif ((index_x+1)>size(surface,2)&& (index_y-1)==0) %Top right
                        for i = 0:1
                            for j = -1:0
                                listOfNearbyPot = [listOfNearbyPot;surface(index_y+i,index_x+j)];
                            end
                        end
                        listOfNearbyPot(2) = []; %ignoring current position 
                        relArgDirections = [stepArgDirections(4),stepArgDirections(6),stepArgDirections(7)];
                    
                    elseif ((index_x-1)==0 && (index_y+1)>size(surface,1)) %Bottom left
                        for i = -1:0
                            for j = 0:1
                                listOfNearbyPot = [listOfNearbyPot;surface(index_y+i,index_x+j)];
                            end
                        end
                        listOfNearbyPot(3) = []; %ignoring current position 
                        relArgDirections = [stepArgDirections(2),stepArgDirections(3),stepArgDirections(5)];        
                   
                    elseif ((index_x+1)>size(surface,2) && (index_y+1)>size(surface,1)) %Bottom right
                        for i = -1:0
                            for j = -1:0
                                listOfNearbyPot = [listOfNearbyPot;surface(index_y+i,index_x+j)];
                            end
                        end
                        listOfNearbyPot(4) = []; %ignoring current position 
                        relArgDirections = [stepArgDirections(1),stepArgDirections(2),stepArgDirections(4)];
                    end
                    
                    otherwise
                    disp('other value')
            end
            
            
            
                    % Need to take into account multiple available
                    % lowestSteps                    
                    lowestStep = find(listOfNearbyPot == min(listOfNearbyPot)); %index
                    randomNumbers = rand(size(lowestStep));
                    randomIndex = find(randomNumbers==min(randomNumbers));
                    
                     singleLowestStep = lowestStep(randomIndex); %Picks a random direction 
%                     desiredArgDirection = relArgDirections(singleLowestStep);
                    desiredArgDirection = stepArgDirections(singleLowestStep);

            
            
            % Goal oriented controller ************************
            if timestep>1
                difference = desiredArgDirection - obj.p_c(3,timestep-1);
            else
                difference = desiredArgDirection - obj.p_c(3,timestep);
            end
%             difference = desiredArgDirection - obj.p_c(3,1);
            
            if difference>1 % we need to spin left
                dec_omega_r = (difference*obj.l_s)/obj.r_w;
                dec_omega_l = 0;
            else
                dec_omega_l = (-1*difference*obj.l_s)/obj.r_w;
                dec_omega_r = 0;
            end
            % *********************************
            
            
            
            % Combining to calculate 
            omega_l = obj.randomMotionGain * rand_omega_l +  dec_omega_l + obj.straightMotionGain;
            omega_r = obj.randomMotionGain * rand_omega_r +  dec_omega_r + obj.straightMotionGain;      
            
                                
            % Update ant position (Braitenberg code)
            if timestep>1
                timestep;
                

                v_c = (omega_l*obj.r_w + omega_r*obj.r_w)/2; % Velocity
                dphi = (omega_r*obj.r_w - omega_l*obj.r_w)/2/(obj.l_s/2); %Orientation. Remove minus sign to switch polarity
                obj.p_c(:,timestep) = obj.p_c(:,timestep-1) + [v_c*cos(obj.p_c(3,timestep-1));v_c*sin(obj.p_c(3,timestep-1));dphi]*obj.dt;
%                 obj.p_c(:,1) = obj.p_c_prev(:,1) + ...
%                     [v_c*cos(obj.p_c_prev(3,1));...
%                     v_c*sin(obj.p_c_prev(3,1));...
%                     dphi]*obj.dt;
            end
        end
        
        function obj = ant(initPosition)
            % class constructor
            obj.p_c = initPosition;
            obj.p_c_old = initPosition;
            obj.p_c_prev = initPosition;
%             obj.testValue(2) = 1;
        end
    end
end