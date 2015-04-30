function gait = doMPC(constants, dynamics, robot, gait)
aRows = size(dynamics.A{1},1);
aCols = size(dynamics.A{1},2);
bRows = size(dynamics.B,1);
bCols = size(dynamics.B,2);

dynamics.PcomX = dynamics.P(1:aRows:end,1:aRows);
dynamics.PcomU = dynamics.P(1:aRows:end,aRows+1:end);

dynamics.VcomX = dynamics.P(2:aRows:end,1:aRows);
dynamics.VcomU = dynamics.P(2:aRows:end,aRows+1:end);

dynamics.PcopX = dynamics.P(3:aRows:end,1:aRows);
dynamics.PcopU = dynamics.P(3:aRows:end,aRows+1:end);

dynamics.LdotX = dynamics.P(4:aRows:end,1:aRows);
dynamics.LdotU = dynamics.P(4:aRows:end,aRows+1:end);

dynamics.Fx = dynamics.P(5:aRows:end,1:aRows);
dynamics.Fu = dynamics.P(5:aRows:end,aRows+1:end);

[H,f] = computeCost(dynamics,gait, constants);

u0 = -H\f';

Aeq = [];
beq = [];
Aiq = [];
biq = [];

%%
[Acmp,bcmp] = getCmpConstraint(gait,dynamics,robot,constants);
%%

[Aiq,biq] = add_constraint(Aiq,biq,Acmp,bcmp);

%%
[Acop,bcop] = getCopConstraint(gait,dynamics,robot,constants);
%%

[Aiq,biq] = add_constraint(Aiq,biq,Acop,bcop);

%%% Final Ldot Constraint %%%
ALdotEndx = dynamics.LdotU(end,:);
bLdotEndx = -dynamics.LdotX(end,:)*dynamics.initialConditionX;
ALdotEndy = dynamics.LdotU(end,:);
bLdotEndy = -dynamics.LdotX(end,:)*dynamics.initialConditionY;
ALdotEnd = blkdiag(ALdotEndx,ALdotEndy);
bLdotEnd = [bLdotEndx;bLdotEndy];

[Aeq,beq] = add_constraint(Aeq,beq,ALdotEnd,bLdotEnd);

%%% Final CMP Position Constraint %%%
AcmpEndx = dynamics.PcopU(end,:);
bcmpEndx = gait.COPs{end}(1) - dynamics.PcopX(end,:)*dynamics.initialConditionX;
AcmpEndy = dynamics.PcopU(end,:);
bcmpEndy = gait.COPs{end}(2) - dynamics.PcopX(end,:)*dynamics.initialConditionY;
AcmpEnd = blkdiag(AcmpEndx,AcmpEndy);
bcmpEnd = [bcmpEndx;bcmpEndy];

[Aeq,beq] = add_constraint(Aeq,beq,AcmpEnd,bcmpEnd);

%%% Final COM Position Constraint %%%
AcomEndx = dynamics.PcomU(end,:);
bcomEndx = gait.COPs{end}(1) - dynamics.PcomX(end,:)*dynamics.initialConditionX;
AcomEndy = dynamics.PcomU(end,:);
bcomEndy = gait.COPs{end}(2) - dynamics.PcomX(end,:)*dynamics.initialConditionY;
AcomEnd = blkdiag(AcomEndx,AcomEndy);
bcomEnd = [bcomEndx;bcomEndy];

[Aeq,beq] = add_constraint(Aeq,beq,AcomEnd,bcomEnd);

%%% Final COM Velocity Constraint %%%
AcomVelEndx = dynamics.VcomU(end,:);
bcomVelEndx = -dynamics.VcomX(end,:) * dynamics.initialConditionX;
AcomVelEndy = dynamics.VcomU(end,:);
bcomVelEndy = -dynamics.VcomX(end,:) * dynamics.initialConditionY;
AcomVelEnd = blkdiag(AcomVelEndx,AcomVelEndy);
bcomVelEnd = [bcomVelEndx;bcomVelEndy];

[Aeq,beq] = add_constraint(Aeq,beq,AcomVelEnd,bcomVelEnd);

if(constants.runningInMatlab)
options = optimset('Algorithm','interior-point-convex','Display','final','TolFun',1e-6,'TolX',1e-6,'MaxIter',1000);

x = quadprog(H,f,Aiq,biq,Aeq,beq,[],[],u0,options);
else
options = optimset('Display','final','TolFun',1e-6,'TolX',1e-6,'MaxIter',1000);
[out, OBJ, INFO, LAMBDA] = qp(u0,H,f',Aeq,beq);
end

ux = x(1:bCols*constants.N);
uy = x(bCols*constants.N+1:end);

gait.comX = [dynamics.initialConditionX(1);dynamics.PcomX * dynamics.initialConditionX + dynamics.PcomU * ux];
gait.comY = [dynamics.initialConditionY(1);dynamics.PcomX * dynamics.initialConditionY + dynamics.PcomU * uy];
gait.comZ = dynamics.z;
gait.com = [gait.comX,gait.comY,gait.comZ];

gait.copX = [dynamics.initialConditionX(3);dynamics.PcopX * dynamics.initialConditionX + dynamics.PcopU * ux];
gait.copY = [dynamics.initialConditionY(3);dynamics.PcopX * dynamics.initialConditionY + dynamics.PcopU * uy];
gait.cop = [gait.copX,gait.copY,zeros(length(gait.copX))];

gait.LdotY = [dynamics.initialConditionX(4);dynamics.LdotX*dynamics.initialConditionX + dynamics.LdotU * ux];
gait.LdotX = [dynamics.initialConditionY(4);dynamics.LdotX*dynamics.initialConditionY + dynamics.LdotU * uy];

gait.comddotX = gait.LdotY./(constants.mass.*dynamics.z) + (constants.gravity + dynamics.zddot).*(gait.comX - gait.copX)./dynamics.z;
gait.comddotY = gait.LdotX./(constants.mass.*dynamics.z) + (constants.gravity + dynamics.zddot).*(gait.comY - gait.copY)./dynamics.z;

gait.cmpX = gait.copX + gait.LdotY./(constants.mass.*(dynamics.zddot + constants.gravity));
gait.cmpY = gait.copY - gait.LdotX./(constants.mass.*(dynamics.zddot + constants.gravity));

end
