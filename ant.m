% This defines the properties of an ANT and its CONTROLLER
% MA Kurien (ma581)

classdef ant < handle
    properties
        %Braitenberg robot properties
        p_c;            %Array to store x,y,theta position
        p_c_old;        %save old state for trajectory
        p_c_prev;       %Previous position (UNUSED)
        p_c_round;      %Rounded position
        rho = 10;       %light intensity to rotational speed constant
        l_s = 0.001;      %shaft length vehicle
        r_w = 0.05;     %radius wheel
        %         dt = 1e-3;      %time increment
        dt = 3;
        d_s = 0.1;      %sensor distance
        
        
        % Additional properties
        randomMotionGain = 0;   % Gain for random motion
        straightMotionGain = 1; % Gain to keep driving straight
        directionsHeaded;       % For debugging
        omega;                  %For debugging
        noiseGain = 10;         %Noise to avoid getting stuck in a maze
        wheelGain = 0.1;
        diff;                   %difference between desired and current orientation
        lp;                     %Local pheremone matrix
        wallHeight = 500;       %deciding available routes
        stepsTravelled;
    end
    
    methods
        function obj = antController(obj,surface,timestep)
            % This will carry out OBSTACLE AVOIDANCE with GRADIENT DECENT for a
            % potential field. This is the ANT CONTROLLER and will update
            % ant.p_c and ant.p_c_old
            % MA Kurien (ma581)
            
            %                       flipped_surface = flipud(surface); %This is because when plotting, the flipped_surface is flipped (array indexing)
            flipped_surface = surface;
            
            % Random noise generation
            rand_omega_r = rand(1);
            rand_omega_l = rand(1);
            
            %% POSITION
            % Find discrete position (ie the index)
            if timestep>1
                index_y = floor(obj.p_c(2,timestep-1)); % Coordinates starting at (1,1)
                index_x = floor(obj.p_c(1,timestep-1));
            else
                index_y = floor(obj.p_c(2,timestep)); % Coordinates starting at (1,1)
                index_x = floor(obj.p_c(1,timestep));
            end
            
           
            
            %Reading in neighbours in 3x3 grid
            listOfNearbyPot = []; %init
            
            %Considering which of three cases we are in a 3x3 grid
            if ((index_x-1)>0 && (index_x+1)<=size(flipped_surface,2) && (index_y-1)>0 && (index_y+1)<=size(flipped_surface,1))
                s = 1;  %Case 1 (middle of 3x3 grid)
                %                 disp('s= 1');
            elseif((index_y-1)==0 && (index_x-1)>0 && (index_x+1)<=size(flipped_surface,2) ||...
                    (index_y+1)>=size(flipped_surface,1) && (index_x-1)>=0 && (index_x+1)<=size(flipped_surface,2) ||...
                    (index_x-1)==0 && (index_y-1)>=0 && (index_y+1)<=size(flipped_surface,1)||...
                    (index_x+1)>=size(flipped_surface,2) && (index_y-1)>0 && (index_y+1)<=size(flipped_surface,1))
                s = 2; %Case 2 (edge of 3x3 grid)
                disp('s= 2');
                %             elseif((index_y-1)==0 && (index_x-1)==0 && (index_x+1)<size(flipped_surface,2) && (index_y+1)<size(flipped_surface,1)||...
                %                     (index_y-1)==0 && (index_x-1)>0 && (index_x+1)>size(flipped_surface,2) && (index_y+1)<size(flipped_surface,1)||...
                %                     (index_y-1)>0 && (index_x-1)==0 && (index_x+1)<size(flipped_surface,2) && (index_y+1)>size(flipped_surface,1)||...
                %                     (index_y-1)>0 && (index_x-1)>0 && (index_x+1)>size(flipped_surface,2) && (index_y+1)>size(flipped_surface,1))
                %                 s = 3; %Case 3 (corner)
            else
                s=3;
                disp('s= 3');
            end
            
            
            %stepArgDirections = [3*pi/4, pi/2, pi/4, pi, 0, 5*pi/4,3*pi/2, 7*pi/4]; %Argand plot angles
            %             stepArgDirections = [5*pi/4, 3*pi/2, 7*pi/4,0, pi/4,3*pi/4, pi/2, pi/4]; %Flipped argand plot angles due to array numbering
            stepArgDirections = [5*pi/4, 3*pi/2, 7*pi/4, pi, 0, 3*pi/4, pi/2, pi/4]; %Flipped argand plot angles due to array numbering
            
            relArgDirections = []; % the relevant directions for each case
            
            switch s
                case 1 %Case 1 (middle of 3x3 grid)
                    %disp('Case 1');
                    for i = -1:1
                        for j = -1:1
                            listOfNearbyPot = [listOfNearbyPot;flipped_surface(index_y+i,index_x+j)];
                        end
                    end
                    
                    listOfNearbyPot(5) = []; %ignoring current position
                    listOfNearbyPot(1) = 999; %Moving only in a cross now
                    listOfNearbyPot(3) = 999;
                    listOfNearbyPot(6) = 999;
                    listOfNearbyPot(8) = 999;
                    relArgDirections = stepArgDirections; %relevant
                    
                case 2 %Case 2 (edge of 3x3 grid)
                    %disp('Case 2')
                    if (index_x-1)==0 %Middle left
                        for i = -1:1
                            for j = 0:1
                                listOfNearbyPot = [listOfNearbyPot;flipped_surface(index_y+i,index_x+j)];
                            end
                        end
                        disp('Middle left');
                        listOfNearbyPot(3) = []; %ignoring current position
                        relArgDirections = [stepArgDirections(2:3),...
                            stepArgDirections(5),...
                            stepArgDirections(7:8)];
                        
                    elseif (index_x+1)>size(flipped_surface,2) %Middle right
                        for i = -1:1
                            for j = -1:0
                                listOfNearbyPot = [listOfNearbyPot;flipped_surface(index_y+i,index_x+j)];
                            end
                        end
                        disp('Middle right');
                        listOfNearbyPot(4) = []; %ignoring current position
                        relArgDirections = [stepArgDirections(1:2),...
                            stepArgDirections(4),...
                            stepArgDirections(6:7)];
                        
                    elseif (index_y-1)==0 %Middle top
                        disp('Middle top');
                        for i = 0:1
                            for j = -1:1
                                listOfNearbyPot = [listOfNearbyPot;flipped_surface(index_y+i,index_x+j)];
                            end
                        end
                        listOfNearbyPot(2) = []; %ignoring current position
                        relArgDirections = [stepArgDirections(4:8)];
                        
                    elseif (index_y+1)>size(flipped_surface,1) %Middle bottom
                        disp('Middle bottom');
                        for i = -1:0
                            for j = -1:1
                                listOfNearbyPot = [listOfNearbyPot;flipped_surface(index_y+i,index_x+j)];
                            end
                        end
                        listOfNearbyPot(5) = []; %ignoring current position
                        relArgDirections = stepArgDirections(1:5);
                    end
                    
                case 3
                    %disp('Case 3')
                    if ((index_x-1)==0 && (index_y-1) ==0 )%Top left
                        disp('Top left');
                        for i = 0:1
                            for j = 0:1
                                listOfNearbyPot = [listOfNearbyPot;flipped_surface(index_y+i,index_x+j)];
                            end
                        end
                        listOfNearbyPot(1) = []; %ignoring current position
                        relArgDirections = [stepArgDirections(5),stepArgDirections(7),stepArgDirections(8)];
                        
                    elseif ((index_x+1)>size(flipped_surface,2)&& (index_y-1)==0) %Top right
                        disp('Top right');
                        for i = 0:1
                            for j = -1:0
                                listOfNearbyPot = [listOfNearbyPot;flipped_surface(index_y+i,index_x+j)];
                            end
                        end
                        listOfNearbyPot(2) = []; %ignoring current position
                        relArgDirections = [stepArgDirections(4),stepArgDirections(6),stepArgDirections(7)];
                        
                    elseif ((index_x-1)==0 && (index_y+1)>size(flipped_surface,1)) %Bottom left
                        disp('Bottom left');
                        for i = -1:0
                            for j = 0:1
                                listOfNearbyPot = [listOfNearbyPot;flipped_surface(index_y+i,index_x+j)];
                            end
                        end
                        listOfNearbyPot(3) = []; %ignoring current position
                        relArgDirections = [stepArgDirections(2),stepArgDirections(3),stepArgDirections(5)];
                        
                    elseif ((index_x+1)>size(flipped_surface,2) && (index_y+1)>size(flipped_surface,1)) %Bottom right
                        disp('Bottom right');
                        for i = -1:0
                            for j = -1:0
                                listOfNearbyPot = [listOfNearbyPot;flipped_surface(index_y+i,index_x+j)];
                            end
                        end
                        listOfNearbyPot(4) = []; %ignoring current position
                        relArgDirections = [stepArgDirections(1),stepArgDirections(2),stepArgDirections(4)];
                    end
                    
                otherwise
                    disp('other value')
            end
            
            % Adding noise to the ant neighbouring positions
            noisyListOfNearbyPot = listOfNearbyPot - obj.noiseGain*rand(size(listOfNearbyPot));
            
            
            
            % Need to take into account multiple available
            % lowestSteps
            %                         lowestStep = find(listOfNearbyPot == min(listOfNearbyPot)); %index
            optionalDirections = find(noisyListOfNearbyPot <= obj.wallHeight);
            
            if size(optionalDirections)== 1
                %Go back
                lowestStep = find(noisyListOfNearbyPot == min(noisyListOfNearbyPot)); %index
                randomNumbers = rand(size(lowestStep));
                randomIndex = find(randomNumbers==min(randomNumbers));
                
                singleLowestStep = lowestStep(randomIndex); %Picks a random direction
                
            else
                % Don't go back
                if timestep>1
                    if obj.directionsHeaded(timestep-1) == 2
                        backDirection = 7;
                    elseif obj.directionsHeaded(timestep-1) == 7
                        backDirection = 2;
                    elseif obj.directionsHeaded(timestep-1) == 5
                        backDirection = 4; 
                        elseif obj.directionsHeaded(timestep-1) == 4
                        backDirection = 5;
                    end
                else
                    backDirection = find(noisyListOfNearbyPot == max(noisyListOfNearbyPot));
                end
               
                
                noisyListOfNearbyPot (backDirection) = obj.wallHeight;
                lowestStep = find(noisyListOfNearbyPot == min(noisyListOfNearbyPot)); %index
                randomNumbers = rand(size(lowestStep));
                randomIndex = find(randomNumbers==min(randomNumbers));
                
                singleLowestStep = lowestStep(randomIndex); %Picks a random direction
            end
            
            
            
            if timestep>2
                
                if floor(obj.p_c(1:2,timestep-2))- floor(obj.p_c(1:2,timestep-1))== [0,0]'
                    positionChange = 0;
                else
                    positionChange = 1;
                    obj.stepsTravelled = obj.stepsTravelled + 1 ;
                end
                
                % Adding one step memory for the ant
                if noisyListOfNearbyPot(obj.directionsHeaded(timestep-1))<500 && positionChange == 0
                    lowestStep = obj.directionsHeaded(timestep-1); %keep going in the same direction
                else
                    lowestStep = find(noisyListOfNearbyPot == min(noisyListOfNearbyPot)); %index
                end
                singleLowestStep = lowestStep;
                
            end
            
            
            obj.directionsHeaded(1,timestep) = singleLowestStep;
            %                     desiredArgDirection = relArgDirections(singleLowestStep);
            %                     fprintf('Heading to direction = %s.\n',singleLowestStep);
            desiredArgDirection = stepArgDirections(singleLowestStep);
            
            
            
            TF = isempty(desiredArgDirection);
            if TF==1
                disp('desiredArgDirection is empty!');
                timestep
            end
            
            
            
            %% Goal oriented CONTROLLER
            if timestep>1
                difference = desiredArgDirection - obj.p_c(3,timestep-1);
                obj.diff(1,timestep) = difference;
            else
                difference = desiredArgDirection - obj.p_c(3,timestep);
                obj.diff(1,timestep) = difference;
                
            end
            
            if difference>0 % we need to spin left
                if difference <= pi %considereing shortest angle to turn towards
                    % we need to spin left
                    dec_omega_r = (difference*obj.l_s)/obj.r_w;
                    dec_omega_l = 0;
                    % disp('Spinning left');
                else
                    theta = 2*pi - difference;
                    % we need to spin right
                    dec_omega_l = (theta*obj.l_s)/obj.r_w;
                    dec_omega_r = 0;
                end
            else
                if norm(difference) <= pi%considereing shortest angle to turn towards
                    % we need to spin right
                    dec_omega_l = (norm(difference)*obj.l_s)/obj.r_w;
                    dec_omega_r = 0;
                    % disp('Spinning right');
                else
                    % we need to spin left
                    theta = 2*pi - norm(difference);
                    dec_omega_l = 0;
                    dec_omega_r = (theta*obj.l_s)/obj.r_w;
                end
                
            end
            
            % Driving straight vs turning
            if norm(difference)>pi/100
                drivingStraight = 0; %we are turning
            else
                drivingStraight = 1; %driving straight
            end
            
            
            
            % Combining to calculate
            omega_l = obj.randomMotionGain * rand_omega_l +  obj.wheelGain * dec_omega_l + obj.straightMotionGain * drivingStraight;
            omega_r = obj.randomMotionGain * rand_omega_r +  obj.wheelGain * dec_omega_r + obj.straightMotionGain * drivingStraight;
            obj.omega(1,timestep) = omega_l;
            obj.omega(2,timestep) = omega_r;
            
            %% UPDATE ant position (Braitenberg code)
            if timestep>1
                timestep;
                v_c = (omega_l*obj.r_w + omega_r*obj.r_w)/2; % Velocity
                %                 dphi = (omega_r*obj.r_w - omega_l*obj.r_w)/2/(obj.l_s/2); %Orientation. Remove minus sign to switch polarity
                dphi = (omega_r*obj.r_w - omega_l*obj.r_w)/(obj.l_s); %Orientation. Remove minus sign to switch polarity
                %                 phi = obj.p_c(3,timestep-1) + dphi;
                obj.p_c(:,timestep) = obj.p_c(:,timestep-1) + [v_c*cos(obj.p_c(3,timestep-1));v_c*sin(obj.p_c(3,timestep-1));dphi]*obj.dt;
                
                % To take into account that direction should always be
                % 0<theta<2*pi
                obj.p_c(3,timestep) = mod(obj.p_c(3,timestep),2*pi);
                
                obj.p_c_round(:,timestep) = floor(obj.p_c(:,timestep));
                
            end
        end
        
        function obj = ant(initPosition,wxy)
            % class constructor
            obj.p_c = initPosition + [0.5 0.5 0]';
            obj.p_c_old = initPosition;
            obj.p_c_prev = initPosition;
            obj.p_c_round = initPosition;
            obj.lp = zeros(wxy,wxy);
            obj.stepsTravelled = 0;
            %             obj.testValue(2) = 1;
        end
    end
end