function main()

close all

constants = struct();
robot = struct();
dynamics = struct();
gait = struct();

constants.runningInMatlab = true;

robot.posLeftFootX = 0;
robot.posLeftFootY = 0.15;
robot.posRightFootX = 0;
robot.posRightFootY = -0.15;
robot.footWidth = 0.1;
robot.footLength = 0.15;
constants.T = 0.01;
constants.mass = 145;
constants.recoveryDuration = 2;
constants.initialCenterOfMassHeight = 0.75;
constants.gravity = 9.81;
constants.finalDesiredCenterOfMassHeight = constants.initialCenterOfMassHeight;

constants.a = 1; % uCmp
constants.b = 1; % COP-COPRef
constants.c = 1e-4; % Ldot
constants.d = 1e-7; %uLdot

gait.stanceSide = 1;
gait.desiredStepLength = 0.25;
gait.desiredStepWidth = abs(robot.posLeftFootY - robot.posRightFootY);
gait.footSteps = cell(1,1);
gait.footSteps{1} = [0,0,0];
constants.N = constants.recoveryDuration/constants.T;
gait.t = 0:constants.T:constants.N*constants.T;

gait.COPs = cell(1,1);
gait.COPs{1} = [gait.footSteps{1}(1),gait.footSteps{1}(2)];

dynamics.initialConditionY = [0;0;0;0;50];
dynamics.initialConditionX = [0;0;0;0;50];

dynamics = generateComHeightDynamics(dynamics,constants);

dynamics = dynamics_lookahead(dynamics,constants,gait);

gait.U = ones(constants.N,1);
gait.PcopRef = [gait.U*gait.COPs{1}(1),gait.U*gait.COPs{1}(2)];
gait = doMPC(constants,dynamics,robot,gait);

make_plots(robot,gait,dynamics,constants);
end
