function [dynamics] = dynamics_lookahead(dynamics, constants,gait)
dynamics.A = cell(constants.N,1);
dynamics.B = [0,0;
	      0,0;
	      constants.T,0;
	      0,constants.T;
	      0,0];
for i=1:constants.N
w = dynamics.omega(i);
T = constants.T;
m = constants.mass;
z = dynamics.z(i);
alpha = 1; %Force decay term
dynamics.A{i} = [1,T,0,0,0;
		 w*T,1,-w*T,T/(m*z),T/m;
	 	 0,0,1,0,0;
		 0,0,0,1,0;
		 0,0,0,0,alpha];
		 
end
aRows = size(dynamics.A{1},1);
aCols = size(dynamics.A{1},2);
bRows = size(dynamics.B,1);
bCols = size(dynamics.B,2);
P = [dynamics.A{1},dynamics.B,zeros(bRows,bCols*(constants.N-1))];

for i = 1:constants.N-1
    P_i = [dynamics.A{i}*P(end-(aRows-1):end,1:aCols+bCols*i),dynamics.B,zeros(bRows,bCols*(constants.N-1-i))];
   
    P = [P;P_i];  
end

dynamics.P = P;

end
