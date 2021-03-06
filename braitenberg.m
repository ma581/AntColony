%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%          Braitenberg vehicle                                  %%%%%%
%%%%%%          4M20 Robotics, coursework template                   %%%%%%
%%%%%%          University of Cambridge                              %%%%%%
%%%%%%          Michaelmas 2015                                      %%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Main function                                                         %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Blank slate
clear all
close all
clc

%% Parameters
% p_ls1 = [1;1];  %position light source
% l_s = 0.1;      %shaft length vehicle
% r_w = 0.02;     %radius wheel
% d_s = 0.1;      %sensor distance
% rho = 10;       %light intensity to rotational speed constant
% dt = 1e-3;      %time increment
% N = 31400;
N = 8000;

%% Initialisation
% p_c = [0.6;0.6;pi/2];  %initial robot position and orientation
% p_c_old = p_c;      %save old state for trajectory
Z1 = load('Zsurface.mat');
Z1 = struct2cell(Z1);
Z1 = cell2mat(Z1);
sr = 9; sc = 5;

% [Z1,sc,sr,ec,er] = antmaze(4);
start_pos = [sc,sr]; 

% ant1 = ant([sc;sr;pi]);
ant1 = ant([5;4;pi],11);

figure(1)
% plot([p_c(1);p_c_old(1)],[p_c(2);p_c_old(2)])
    plot([ant1.p_c(1);ant1.p_c_old(1)],[ant1.p_c(2);ant1.p_c_old(2)])
hold on

%positions of sensors 1 and 2
%  p_s1 = p_c(1:2) + ant1.l_s/2*[cos(p_c(3));sin(p_c(3))] + ant1.d_s/2*[-sin(p_c(3));cos(p_c(3))];
%  p_s2 = p_c(1:2) + ant1.l_s/2*[cos(p_c(3));sin(p_c(3))] + ant1.d_s/2*[sin(p_c(3));-cos(p_c(3))];

%initialisation of animation
ll = line(0,0,'color','k','LineWidth',2);   %left side of vehicle
lr = line(0,0,'color','k','LineWidth',2);   %right side of vehicle
lf = line(0,0,'color','k','LineWidth',2);   %front of vehicle
lb = line(0,0,'color','k','LineWidth',2);   %back of vehile
lw1 = line(0,0,'color','k','LineWidth',5);  %wheel left
lw2 = line(0,0,'color','k','LineWidth',5);  %wheel right
ls1 = line(0,0,'color','r','Marker','.','MarkerSize',20);   %sensor left
ls2 = line(0,0,'color','r','Marker','.','MarkerSize',20);   %sensor right
lli1 = line(0,0,'color',[0.7 0.7 0],'Marker','.','MarkerSize',30);  %light source

%% Simulation
% set(lli1,'xdata',p_ls1(1),'ydata',p_ls1(2))
axis([0 12 0 12])
% 
% [X,Y] = meshgrid(-2:0.2:2,-2:0.2:2);
% Z = X.*exp(-X.^2 - Y.^2);
% % Z = Z.*0;
% %Making edge walls of the plane
% wallHeight = 15;
% Z(1,:) = wallHeight;
% Z(size(Z,1),:) = wallHeight;
% Z(:,1) = wallHeight;
% Z(:,size(Z,2)) = wallHeight;
% 
% 
% % Making a wall
% Z(1:10,10) = wallHeight;
% % Z(1:10,9) = wallHeight;
% % Z(1:10,13) = wallHeight;
% Z(1:10,14) = wallHeight;
% 
% % Z(15,1:10) = wallHeight;
% Z(14,1:10) = wallHeight;
% Z(12,10:15) = wallHeight;
% 
% 
% Z(14:15,15:20) = wallHeight;
% Z(5,1:7) = wallHeight;
% Z(18:21,6) = wallHeight;
% Z(18:21,17) = wallHeight;
% Z(18:21,13) = wallHeight;
% 
% 
% Z(5,20) = -1*wallHeight; %food
% Z = Z-1000;
% figure(2)
% surface(X,Y,Z)
% view(3)

t = 0:ant1.dt:N;
for i = 1:N
    
    
        ant1.antController(Z1,i);
%         if ant1.p_c(1,i) ==20 && ant1.p_c(2,i) == 5
%             disp('Reached food!')
%         end
%         ant2.antController(Z,i);
%         ant3.antController(Z,i);

%     r_ls1 = norm(p_s1-p_ls1);
%     r_rs1 = norm(p_s2-p_ls1);
% 
%     omega_l = 1/r_ls1^2*rho - 1/(1.8*r_ls1)^3*rho;
%     omega_r = 1/r_rs1^2*rho - 1/(1.8*r_rs1)^3*rho;
% 
%     v_c = (omega_l*r_w + omega_r*r_w)/2;
%     dphi = -(omega_r*r_w - omega_l*r_w)/2/(ant1.l_s/2); %Remove minus sign to switch polarity
%             
%     if i>1
%         p_c(:,i) = p_c(:,i-1) + [v_c*cos(p_c(3,i-1));v_c*sin(p_c(3,i-1));dphi]*dt;
%         p_s1 = p_c(1:2,i) + ant1.l_s/2*[cos(p_c(3,i));sin(p_c(3,i))] + ant1.d_s/2*[-sin(p_c(3,i));cos(p_c(3,i))];
%         p_s2 = p_c(1:2,i) + ant1.l_s/2*[cos(p_c(3,i));sin(p_c(3,i))] + ant1.d_s/2*[sin(p_c(3,i));-cos(p_c(3,i))];
%         p_ls1 = [0.5*sin(t(i)/5)+1;0.5*cos(t(i)/5)+1];
%     end

    

end


% p_c_old = p_c(:,1);
t_next = 0;   %variable for timing of frame capture
RepSpeed = 1; %replay speed
fps = 30;     %frames per second
tic
% surf(Z1);view(2);hold on;
%  while toc < t(end)
%         
%     % Animation
%     if mod(toc,1/fps) > mod(toc,1/fps+ant1.dt)
% 
%         idx = floor(toc/ant1.dt*RepSpeed);
%         if idx>N
%             break
%         end
% 
%         r_c1 = ant1.p_c(1:2,idx) + ant1.l_s/2*[-sin(ant1.p_c(3,idx));cos(ant1.p_c(3,idx))] - ant1.l_s/2*[cos(ant1.p_c(3,idx));sin(ant1.p_c(3,idx))];
%         r_c2 = r_c1 + ant1.l_s*[cos(ant1.p_c(3,idx));sin(ant1.p_c(3,idx))];
%         r_c3 = r_c2 + ant1.l_s*[sin(ant1.p_c(3,idx));-cos(ant1.p_c(3,idx))];
%         r_c4 = r_c3 + ant1.l_s*[-cos(ant1.p_c(3,idx));-sin(ant1.p_c(3,idx))];
%         r_w1 = [r_c1 + (r_c2-r_c1)/4,r_c1 + (r_c2-r_c1)*3/4];
%         r_w2 = [r_c3 + (r_c4-r_c3)/4,r_c3 + (r_c4-r_c3)*3/4];
% 
%         p_s1 = ant1.p_c(1:2,idx) + ant1.l_s/2*[cos(ant1.p_c(3,idx));sin(ant1.p_c(3,idx))] + ant1.d_s/2*[-sin(ant1.p_c(3,idx));cos(ant1.p_c(3,idx))];
%         p_s2 = ant1.p_c(1:2,idx) + ant1.l_s/2*[cos(ant1.p_c(3,idx));sin(ant1.p_c(3,idx))] + ant1.d_s/2*[sin(ant1.p_c(3,idx));-cos(ant1.p_c(3,idx))];
% 
%         p_ls1 = [0.5*sin(t(idx)/5)+1;0.5*cos(t(idx)/5)+1];
%         
%         plot([ant1.p_c(1,idx);ant1.p_c_old(1)],[ant1.p_c(2,idx);ant1.p_c_old(2)])
% 
%         set(ll,'xdata',[r_c1(1) r_c2(1)],'ydata',[r_c1(2) r_c2(2)])
%         set(lf,'xdata',[r_c2(1) r_c3(1)],'ydata',[r_c2(2) r_c3(2)])
%         set(lr,'xdata',[r_c3(1) r_c4(1)],'ydata',[r_c3(2) r_c4(2)])
%         set(lb,'xdata',[r_c4(1) r_c1(1)],'ydata',[r_c4(2) r_c1(2)])
%         set(lw1,'xdata',[r_w1(1,1) r_w1(1,2)],'ydata',[r_w1(2,1) r_w1(2,2)])
%         set(lw2,'xdata',[r_w2(1,1) r_w2(1,2)],'ydata',[r_w2(2,1) r_w2(2,2)])
%         set(ls1,'xdata',p_s1(1),'ydata',p_s1(2))
%         set(ls2,'xdata',p_s2(1),'ydata',p_s2(2))
%         set(lli1,'xdata',p_ls1(1),'ydata',p_ls1(2))
% 
%         drawnow
%         ant1.p_c_old = ant1.p_c(:,idx);
%     end
%  end
% contour(Z); %To plot the surface as well
% figure; subplot(1,2,1)
% colormap(gray);
% surf(Z);hold on;
% view(2);
% plot3(ant1.p_c(1,:),ant1.p_c(2,:),1*ones(size(ant1.p_c(1,:),2),1),'r')
% % plot3(ant2.p_c(1,:),ant2.p_c(2,:),1*ones(size(ant1.p_c(1,:),2),1),'b')
% % plot3(ant3.p_c(1,:),ant3.p_c(2,:),1*ones(size(ant1.p_c(1,:),2),1),'g')
% subplot(1,2,2)
% surf(Z);hold on;
% view(2);
% plot3(ant1.p_c_round(1,:),ant1.p_c_round(2,:),1*ones(size(ant1.p_c(1,:),2),1),'r')
% plot3(ant2.p_c_round(1,:),ant2.p_c_round(2,:),1*ones(size(ant1.p_c(1,:),2),1),'b')
% plot3(ant3.p_c_round(1,:),ant3.p_c_round(2,:),1*ones(size(ant1.p_c(1,:),2),1),'g')


figure;
image(1.5,1.5,Z1);
colormap(gray);
hold on;
plot(ant1.p_c(1,:),ant1.p_c(2,:),'r')
plot(ant1.p_c(1,:),ant1.p_c(2,:),'r')
plot(ant1.p_c(1,1),ant1.p_c(2,1),'bx')
plot(ant1.p_c(1,end),ant1.p_c(2,end),'g^')