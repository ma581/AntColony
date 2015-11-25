clear
close all

% n=4;
% [Z1,sc,sr,ec,er] = antmaze(n); %maze Dimension: (2n+3)
Z1 = load('Zsurface.mat');
Z1 = struct2cell(Z1);
Z1 = cell2mat(Z1);
sr = 9; sc = 5;
start_pos = [sc,sr]; 

ant1 = ant([sc;sr;0]);
N = 30000*10;
for i = 1:N
    
    
        ant1.antController(Z1,i);
        
end

figure
image(Z1);colormap(summer)
hold on;
plot(ant1.p_c(1,:),ant1.p_c(2,:),'r')
plot(ant1.p_c(1,1),ant1.p_c(2,1),'bx')
plot(ant1.p_c(1,end),ant1.p_c(2,end),'g^')