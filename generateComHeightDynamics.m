function dynamics = generateComHeightDynamics(dynamics,constants)

doConstantComHeight = true;
if(doConstantComHeight)
% Constant COM Height Trajectory
    z = ones(constants.N+1,1)*constants.initialCenterOfMassHeight;
    zddot = zeros(constants.N+1,1);
else
    z = zeros(constants.N+1,1);
    zddot = zeros(constants.N+1,1);
    
    z0 = constants.initialCenterOfMassHeight;
    zf = constants.finalDesiredCenterOfMassHeight;
    
    stepTime = constants.initialDoubleSupportDuration + constants.singleSupportDuration + ...
        constants.finalDoubleSupportDuration;
    
    t = 0:constants.T:stepTime;
    
    for i = 1:length(t)
       [z(i),zddot(i)] = minJerk(z0,zf,t(i),stepTime);
    end

    for i = length(t)+1:length(z)
	[z(i),zddot(i)] = minJerk(z0,zf,stepTime,stepTime);
    end
end

dynamics.z = z;
dynamics.zddot = zddot;
dynamics.omega = (zddot + constants.gravity)./z;

end

function [z,zddot] = minJerk(z0,zf,t,stepTime)
	z = z0 + (zf - z0)*(10*(t/stepTime)^3 - 15*(t/stepTime)^4 + ...
           6*(t/stepTime)^5);
       
       zddot = (zf-z0)*(60*t/(stepTime^3) - 180*(t^2)/(stepTime^4) + ...
           120*(t^3)/(stepTime^5));
end
