clear
close all

n=4;
[Z1,sc,sr,ec,er] = antmaze(n); %maze Dimension: (2n+3)
start_pos = [sc,sr]; 

ant1 = ant([sc;sr;0]);
N = 30000;
for i = 1:N
    
    
        ant1.antController(Z1,i);
        
end

image(Z);
hold on;
plot(ant1.p_c(1,:),ant1.p_c(2,:),'r')