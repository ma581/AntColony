%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Maze Generation  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% 4M20 Robotics %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Maze,startingcol,startingrow,endcol,endrow] = antmaze(n)
% set n for building a nxn maze

pathnum=10; % set number of paths

showProgress = false; % set to TRUE to view progress as maze generates

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
colormap([1,1,1;1,0,0;1,1,1;0,0,0;0,1,0]);
set(gcf,'color','w');

NoWALL      = 0.5;
WALL        = 1.5;
NotVISITED  = -1;
VISITED     = -2;
FIRST       = 3;
LAST        = -0.5;

m = 2*n+3;
M = NotVISITED(ones(m));
offsets = [-1, m, 1, -m];

% initialising borders of maze
M([1 2:2:end end],:) = WALL;
M(:,[1 2:2:end end]) = WALL;

% choose a random starting point along the borders of the maze
columnends = [3 2*n+1];
startingrow = randi(n)*2+1;
if startingrow == 3 || startingrow == 2*n+1
    startingcol = randi(n)*2+1;
else
    startingcol = columnends(randi(2));
end

currentCell = sub2ind(size(M),startingrow,startingcol);
M(currentCell) = FIRST;

S = currentCell;

% maze generation algorithm
while (~isempty(S))
    moves = currentCell + 2*offsets; % moving in north,east,south,west
    unvistedNeigbors = find(M(moves)==NotVISITED);
    
    if (~isempty(unvistedNeigbors)) % if there are unvisited neighbours
        next = unvistedNeigbors(randi(length(unvistedNeigbors),1));
        M(currentCell + offsets(next)) = NoWALL; % choose a random unvisited cell and remove wall
        
        newCell = currentCell + 2*offsets(next);
        if (any(M(newCell+2*offsets)==NotVISITED)) % if newcell is not deadend
            S = [S newCell]; % add it to the stack
        end
        
        currentCell = newCell;
        M(currentCell) = VISITED;
    else
        remove = randi(length(S));
        currentCell = S(remove); 
        S = S([1:remove-1,remove+1:end]);
    end
    
    if (showProgress)
        image(M-VISITED,'CDataMapping','scaled');
        axis equal off;
        drawnow;
        pause(0.01);     
    end
end

% Choose end point to be the corner furthest away from the starting square
Distance = zeros(4,1);
Distance(1) = abs((3-startingrow)^2+(3-startingcol)^2);
Distance(2) = abs((3-startingrow)^2+(2*n+1-startingcol)^2);
Distance(3) = abs((2*n+1-startingrow)^2+(3-startingcol)^2);
Distance(4) = abs((2*n+1-startingrow)^2+(2*n+1-startingcol)^2);
[a, b] = max(Distance);
switch b
    case 1
        endrow = 3;
        endcol = 3;
    case 2
        endrow = 3;
        endcol = 2*n+1;
    case 3
        endrow = 2*n+1;
        endcol = 3;
    case 4
        endrow = 2*n+1;
        endcol = 2*n+1;
end
M(endrow,endcol) = LAST;

% image(M-VISITED,'CDataMapping','scaled');
% axis equal off
% set(gca,'XTick',3:2:2*n+1);
% set(gca,'YTick',3:2:2*n+1);

% now remove walls to create multiple paths
removedwalls = zeros(pathnum,1);

i=1;
while i<=pathnum
    wall = sub2ind(size(M),randi(n)*2,randi(n)*2);
    if M(wall) == WALL && length(find(M(wall+offsets)==WALL))==2 && (length(find(M(wall+[-1,1])==WALL))==2 || length(find(M(wall+[-m,m])==WALL))==2)
        removedwalls(i) = wall;
        i=i+1;
    end
end

M(removedwalls) = NoWALL;

% disp(['Select ' num2str(pathnum) ' wall(s) to be removed'])
% [x,y] = ginput(pathnum);

% M(round(y),round(x)) = NoWALL;

% figure(1)
% image(M-VISITED,'CDataMapping','scaled');
% axis equal off

% output 
Maze = zeros(2*n+3);
for i=1:2*n+3
    for j=1:2*n+3
        switch M(i,j)
            case WALL
                Maze(i,j) = 1000;
            case VISITED
                Maze(i,j) = 0;
            case NoWALL
                Maze(i,j) = 0;
            case FIRST
                Maze(i,j) = 0;
            case LAST
                Maze(i,j) = 0;
        end
    end
end

% test number of paths to endpoint
% currentCell = sub2ind(size(M),startingrow,startingcol);
% path = currentCell;
% z=0;
% 
% % maze generation algorithm
% while (~isempty(path))
%     moves = currentCell + offsets;
%     possibleNeigbors = find(M(moves)==NoWALL);
%     
%     if (~isempty(possibleNeigbors)) 
%         next = possibleNeigbors(randi(length(possibleNeigbors),1));
%         currentCell = currentCell + offsets(next);
%         if currentCell == sub2ind(size(M),endrow,endcol);
%             z=z+1;
%         end
%         path = [path currentCell];
%     else
%         M(currentCell) = WALL;
%         currentCell = path(end-1); 
%         path = path(1:end-1);
%     end
%     
%     if (showProgress)
%         image(M-VISITED,'CDataMapping','scaled');
%         axis equal off;
%         drawnow;
%         pause(0.01);     
%     end
% end
% figure(2)
% contour(Maze,2);
% axis equal
% axis ij

% figure(3)
% waterfall(Maze);
end

