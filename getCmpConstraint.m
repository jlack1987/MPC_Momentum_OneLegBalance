function [Acmp,bcmp] = getCmpConstraint(gait,dynamics,robot,constants)
cmpConstraintX = zeros(constants.N,2);
cmpConstraintY = zeros(constants.N,2);

N = constants.N;
fw = (robot.footWidth/2)*0.6;
fl = (robot.footLength/2)*0.5;

PcopX = dynamics.PcopX*dynamics.initialConditionX;
PcopU = dynamics.PcopU;

LdotX = dynamics.LdotX*dynamics.initialConditionX;
LdotU = dynamics.LdotU;

LdotY = dynamics.LdotX*dynamics.initialConditionY;

x1 = gait.footSteps{1}(1);
y1 = gait.footSteps{1}(2);


cmpConstraintX(1:N,1) = ones(N,1)*(x1+fl);
cmpConstraintX(1:N,2) = ones(N,1)*(-x1+fl);

cmpConstraintY(1:N,1) = ones(N,1)*(y1+fw);
cmpConstraintY(1:N,2) = ones(N,1)*(-y1+fw);

zddot = dynamics.zddot(2:end);

for i = 1:size(LdotU,1)
   for j = 1:size(LdotU,2)
      LdotU(i,j) = LdotU(i,j)/(constants.mass*(zddot(i) + constants.gravity));
   end
   
   for j = 1:size(LdotX,2)
       LdotX(i,j) = LdotX(i,j)/(constants.mass*(zddot(i) + constants.gravity));
       LdotY(i,j) = LdotY(i,j)/(constants.mass*(zddot(i) + constants.gravity));
   end
end

PcmpU = PcopU + LdotU;
PcmpX = PcopX + LdotY;

PcmpY = PcopX + LdotX;

Acmpx = [PcmpU;-PcmpU];
bcmpx = [-PcmpX + cmpConstraintX(:,1);
          PcmpX + cmpConstraintX(:,2)];
   
Acmpy = [PcmpU;-PcmpU];
bcmpy = [-PcmpY + cmpConstraintY(:,1);
          PcmpY + cmpConstraintY(:,2)];

Acmp = blkdiag(Acmpx,Acmpy);
bcmp = [bcmpx;bcmpy];
