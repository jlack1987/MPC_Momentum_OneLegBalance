
function [H,f] = computeCost(dynamics,gait,constants)

uSize = size(dynamics.PcopU,2);
eyeCmpDotU = zeros(uSize,uSize);
eyeLddotU = zeros(uSize,uSize);

for i = 1:uSize
    if(mod(i,2)==0)
       eyeCmpDotU(i,i) = 1; 
    else
        eyeLddotU(i,i) = 1;
    end
end

Hfcn = @(LdotU,PcopU,a,b,c,d) ...	
	eye(size(LdotU,2))*c*LdotU'*LdotU + ...
    eye(size(PcopU,2))*b*PcopU'*PcopU + a*eyeLddotU + d * eyeCmpDotU;

Ffcn = @(LdotX,LdotU,PcopRef,PcopX,PcopU,b,c,ic) ...
	(LdotX*ic)'*eye(size(LdotU,1))*c*LdotU - ...
    (PcopRef - PcopX*ic)'*eye(size(PcopX,1))*b*PcopU;	
%%%%%


Hx = Hfcn(dynamics.LdotU,dynamics.PcopU,...
    constants.a,constants.b,constants.c,constants.d);
Hy = Hfcn(dynamics.LdotU,dynamics.PcopU,...
    constants.a,constants.b,constants.c,constants.d);

fx = Ffcn(dynamics.LdotX,dynamics.LdotU,gait.PcopRef(:,1),...
    dynamics.PcopX,dynamics.PcopU,constants.b,constants.c,...
    dynamics.initialConditionX);
fy = Ffcn(dynamics.LdotX,dynamics.LdotU,gait.PcopRef(:,2),...
    dynamics.PcopX,dynamics.PcopU,constants.b,constants.c,...
    dynamics.initialConditionY);

f = [fx,fy];
H = blkdiag(Hx,Hy);

end
