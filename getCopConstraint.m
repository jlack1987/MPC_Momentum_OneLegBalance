function [Acop,bcop] = getCopConstraint(gait,dynamics,robot,constants)
copConstraintX = zeros(constants.N,2);
copConstraintY = zeros(constants.N,2);

N = constants.N;
fw = (robot.footWidth/2)*0.6;
fl = (robot.footLength/2)*0.5;

PcopX = dynamics.PcopX*dynamics.initialConditionX;
PcopU = dynamics.PcopU;

x1 = gait.footSteps{1}(1);
y1 = gait.footSteps{1}(2);


copConstraintX(1:N,1) = ones(N,1)*(x1+fl);
copConstraintX(1:N,2) = ones(N,1)*(-x1+fl);

copConstraintY(1:N,1) = ones(N,1)*(y1+fw);
copConstraintY(1:N,2) = ones(N,1)*(-y1+fw);

Acopx = [PcopU;-PcopU];
bcopx = [-PcopX + copConstraintX(:,1);
          PcopX + copConstraintX(:,2)];
   
Acopy = [PcopU;-PcopU];
bcopy = [-PcopX + copConstraintY(:,1);
          PcopX + copConstraintY(:,2)];

Acop = blkdiag(Acopx,Acopy);
bcop = [bcopx;bcopy];
