function make_plots(robot,gait,dynamics,constants)

f1=figure(1);clf
set(f1,'Position',[100,100,800,500]);
plotFeet(robot,gait);
plotCOPs(gait);
plot3(gait.comX,gait.comY,dynamics.z,'-r','LineWidth',3);
hold on
plot3(gait.copX,gait.copY,zeros(size(gait.copX)),'-b','LineWidth',3);
hold on
plot3(gait.cmpX,gait.cmpY,zeros(size(gait.copX)),'-g','LineWidth',3);
axis([-0.08 0.08 -0.07 0.07]);
view([0,0,90]);
l1 = legend({'$COM$',...
    '$CMP_{Ref}$',...
    '$CMP$',...
    '$COP$'},...
    'Interpreter','LaTeX','FontSize',20,'Position',[0.4,-0.015,0.25,0.1],...
    'Orientation','Horizontal');
legend('boxoff');
c1 = get(l1,'Children');
set(c1(2),'Color','b');
set(c1(7),'Color',[0.545,0.27,0.074]);
set(c1(5),'Color','g');
alpha = 1;
set(c1(8),'Color',[alpha,alpha,alpha]);
set(c1(11),'Color','r');

f2=figure(2);
set(f2,'Position',[100,100,800,500]);
subplot(2,1,1);
plot(gait.t,gait.LdotX,'-r','LineWidth',3);
hold on
plot(gait.t,gait.LdotY,'-b','LineWidth',3);
axis([0 2 -7.5 0.1]);
l1 = legend({'$\dot{L}_{x}$','$\dot{L}_{y}$'},...
    'Interpreter','Latex','FontSize',20,'Location','South',...
    'Orientation','Horizontal');
title('\fontsize{14}Desired Rate of Change of Angular Momentum');
xlabel('\fontsize{14}time(s)');
ylabel({'\fontsize{14}angular momentum','rate(Nm)'});
subplot(2,1,2);
plot(gait.t,gait.comX,'k','LineWidth',3);
hold on
plot(gait.t,gait.comY,'b','LineWidth',3);
axis([0 2 -1e-3 1e-2]);
l2 = legend({'$COM_{x}$','$COM_{y}$'},...
    'Interpreter','Latex','FontSize',20,'Location','South',...
    'Orientation','Horizontal');
title('\fontsize{14}Center of Mass Trajectory');
xlabel('\fontsize{14}time(s)');
ylabel({'\fontsize{14}position(m)'});

end

function plotFeet(robot,gait)
    for i=1:length(gait.footSteps)
        plotFoot(gait.footSteps{i},robot); 
    end
end

function plotFoot(footStep,robot)
    heelLeftSide = [footStep(1)-robot.footLength/2,footStep(2)+robot.footWidth/2,0];
    heelRightSide = [footStep(1)-robot.footLength/2,footStep(2)-robot.footWidth/2,0];
    toeLeftSide = [footStep(1)+robot.footLength/2,footStep(2)+robot.footWidth/2,0];
    toeRightSide = [footStep(1)+robot.footLength/2,footStep(2)-robot.footWidth/2,0];
    
    plot3([heelLeftSide(1),toeLeftSide(1),toeRightSide(1),heelRightSide(1),heelLeftSide(1)], ...
    [heelLeftSide(2),toeLeftSide(2),toeRightSide(2),heelRightSide(2),heelLeftSide(2)], ...
    [heelLeftSide(3),toeLeftSide(3),toeRightSide(3),heelRightSide(3),heelLeftSide(3)],...
    'k','LineWidth',3);
    hold on
end

function plotCOPs(gait)
    for i=1:length(gait.COPs)
       COPs = plot3(gait.COPs{i}(1),gait.COPs{i}(2),0);
       set(COPs,'Color',[0.545,0.27,0.074],'Marker','o','MarkerSize',8,...
           'LineWidth',3);
       hold on
    end
end

function animate(gait,robot)

    figure(3125441);clf;
    plotFeet(robot,gait);
    hold on
    for i = 1:length(gait.com(:,1))
        plot3(gait.com(i,1),gait.com(i,2),gait.com(i,3),'or','MarkerSize',3);
        hold on
        plot3(gait.cop(i,1),gait.cop(i,2),gait.cop(i,3),'ok','MarkerSize',3);
        pause(0.05);
    end

end