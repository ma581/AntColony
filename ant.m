% This defines the properties of an ANT and its CONTROLLER
% MA Kurien (ma581)

classdef ant
    properties
        %Braitenberg robot properties
        p_c = [0.6;0.6;pi/2];  %initial robot position and orientation
        p_c_old = obj.p_c;         %save old state for trajectory
        rho = 10;       %light intensity to rotational speed constant
        l_s = 0.1;      %shaft length vehicle
        r_w = 0.02;     %radius wheel
        dt = 1e-3;      %time increment
        
        % Additional properties
        randomMotionGain = 0.1; % Gain for random motion
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
                y = round(obj.p_c(1,timestep-1))+1; % +1 to account for indexing starting at 1
                x = round(obj.p_c(2,timestep-1))+1;
            else
                y = round(obj.p_c(1,timestep))+1; % +1 to account for indexing starting at 1
                x = round(obj.p_c(2,timestep))+1;
            end
                
            %Reading in neighbours in 3x3 grid
            listOfNearbyPot = []; %init
            for i = -1:1
                for j = -1:1
                    listOfNearbyPot = [listOfNearbyPot;surface(y+i,x+j)];
                end
            end
                    
            listOfNearbyPot(5) = []; %ignoring current position
            lowestStep = find(listOfNearbyPot == min(listOfNearbyPot)); %index
            
            stepArgDirections = [3*pi/4, pi/2, pi/4, pi, 0, 5*pi/4,3*pi/2, 7*pi/4]; %Argand plot angles
            desiredArgDirection = stepArgDirections(lowestStep);
            
            if timestep>1
                difference = desiredArgDirection - obj.p_c(3,timestep-1);
            else
                difference = desiredArgDirection - obj.p_c(3,timestep);
            end
            
            if difference>1 % we need to spin left
                dec_omega_r = (difference*obj.l_s)/obj.r_w;
                dec_omega_l = 0;
            else
                dec_omega_l = (-1*difference*obj.l_s)/obj.r_w;
                dec_omega_r = 0;
            end
            
            
            
            % Combining to calculate 
            omega_l = obj.randomMotionGain * rand_omega_l +  dec_omega_l;
            omega_r = obj.randomMotionGain * rand_omega_r +  dec_omega_r;      
            
                                
            % Update ant position (Braitenberg code)
            if timestep>1
                v_c = (omega_l*obj.r_w + omega_r*obj.r_w)/2; % Velocity
                dphi = (omega_r*obj.r_w - omega_l*obj.r_w)/2/(obj.l_s/2); %Orientation. Remove minus sign to switch polarity
                obj.p_c(:,timestep) = obj.p_c(:,timestep-1) + [v_c*cos(obj.p_c(3,timestep-1));v_c*sin(obj.p_c(3,timestep-1));dphi]*obj.dt;
            end
        end
    end
end