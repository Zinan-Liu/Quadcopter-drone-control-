%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%  Code modified by Francisco Vasconcelos from
%%%%
%%%%  Visualisation code for quadcopter 
%%%%  Author: Daniel Butters
%%%%  Date: 16/11/17
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;
close all;

%Define total width, length and height of flight arena (metres)
spaceDim = 10;
spaceLimits = [-spaceDim/2 spaceDim/2 -spaceDim/2 spaceDim/2 0 spaceDim/2];

%do you want to draw a ground image on the figure?
draw_ground = true;
% if(draw_ground)
%     ground_img = imread('ground.png');
% end

%figure to display drone simulation
f1 = figure;
ax1 = gca;
view(ax1, 3);
axis equal;
axis(spaceLimits)
grid ON
grid MINOR
caxis(ax1, [0 spaceDim]);
hold(ax1,'on')
axis vis3d

num_drones = 1;

%instantiate a drone object, input the axis and arena limits
drones = [];

% this part only useful for 5e, uncomment if execute 5e
drones2 = [];
%5e part code end

for i = 1:num_drones
    drones = [drones, Drone(ax1, spaceDim, num_drones)];
end

% this part only useful for 5e, uncomment if execute 5e
for i = 1:num_drones
    drones2 = [drones2, Drone2(ax1, spaceDim, num_drones)];
end
%5e part code end

command = input('Enter move:','s');

while 1
    if command == "move"
        break;
    end
end
pos1 = [];
pos_ref_circle1 = [];

% this part only useful for 5e, uncomment if execute 5e
pos2 = [];
pos_ref_circle2 = [];
% gap between drones
gap = [];
%this part only useful for 5e, uncomment if execute 5e
global drone1pos
global rot_angle
global drone1tar
%5e part code end

while(drones(1).time <210)
    %clear axis
    cla(ax1);
    
    %update and draw drones
    for i = 1:num_drones
        update(drones(i));
    end
    
    %this part only useful for 5e, uncomment if execute 5e
    % set drone 1 target as global parameter for drone 2 follow
    drone1tar = drones.pos_ref_circle;
    rot_angle = drones.rot_angle;
    drone1pos = drones.pos;
    for i = 1:num_drones
        update2(drones2(i));
    end
    %5e part code end
    

    pos1 = [pos1,drones.pos]; % I add
    pos_ref_circle1 = [pos_ref_circle1, drones.pos_ref_circle]; % I add
    %this part only useful for 5e, uncomment if execute 5e
    pos2 = [pos2,drones2.pos]; % I add
    pos_ref_circle2 = [pos_ref_circle2, drones2.pos_ref_circle]; % I add
    gap = [gap,drones2.d];
    %5e part code end
      
%     %optionally draw the ground image
%     if(draw_ground)
%         imagesc([-spaceDim,spaceDim],[-spaceDim,spaceDim],ground_img);
%     end
    
%     %apply fancy lighting (optional)
%     camlight
%     
%     %update figure
%     drawnow
%     pause(0.01)
end

% % Figure plotting uncomment if need
% % ref = 2.5*ones(1,length(t));
% % Drone lead
x = pos1(1,:);
y = pos1(2,:);
z = pos1(3,:);
xr = pos_ref_circle1(1,:);
yr = pos_ref_circle1(2,:);
zr = pos_ref_circle1(3,:);
% % Drone Follow
xf = pos2(1,:);
yf = pos2(2,:);
zf = pos2(3,:);
xrf = pos_ref_circle2(1,:);
yrf = pos_ref_circle2(2,:);
zrf = pos_ref_circle2(3,:);
t = 0:0.02:0.02*(length(x)-1);
% % 
f2 = figure;
plot(t,x);
title('Leader Drone x vs time')
xlabel('t');
ylabel('x');
hold on
plot(t,xr);
legend('real','ref');
% % 
f3 = figure;
plot(t,y);
title('Leader Drone y vs time')
xlabel('t');
ylabel('y');
hold on
plot(t,yr);
legend('real','ref');
% % 
f4 = figure;
plot(t,z);
title('Leader Drone altitude vs time')
xlabel('t/s');
ylabel('z/m');
hold on
plot(t,zr);
legend('real','ref');
% % 
f5 = figure;
plot(x,y);
title('Leader Drone follow the circle trajectory')
xlabel('x/m');
ylabel('y/m');
hold on
plot(xr,yr);
legend('real','ref');
% 
f6 = figure;
plot(t,xf);
title('Follower Drone x vs time')
xlabel('t');
ylabel('x');
hold on
plot(t,xrf);
legend('real','ref');
% 
f7 = figure;
plot(t,yf);
title('Follower Drone y vs time')
xlabel('t');
ylabel('y');
hold on
plot(t,yrf);
legend('real','ref');
% 
f8 = figure;
plot(t,zf);
title('Follower Drone altitude vs time')
xlabel('t/s');
ylabel('z/m');
hold on
plot(t,z,'--');
legend('Follower','Leader');
% % 
f9 = figure;
plot(xf,yf);
title('Drones circle trajectory')
xlabel('x/m');
ylabel('y/m');
hold on
plot(x,y,'--');
legend('Follower','Leader');
% % 
f10 = figure;
plot(t,gap);
title('Drones gap vs time')
xlabel('t/s');
ylabel('gap/m');




