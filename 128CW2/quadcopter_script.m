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

for i = 1:num_drones
    drones = [drones, Drone(ax1, spaceDim, num_drones)];
end

command = input('Enter move:','s');

while 1
    if command == "move"
        break;
    end
end
pos1 = [];
pos_ref_circle1 = [];

while(drones(1).time <300)
    %clear axis
    cla(ax1);
    
    %update and draw drones
    for i = 1:num_drones
        update(drones(i));
    end

    

    pos1 = [pos1,drones.pos]; % I add
    pos_ref_circle1 = [pos_ref_circle1, drones.pos_ref_circle]; % I add
      
%     %optionally draw the ground image
%     if(draw_ground)
%         imagesc([-spaceDim,spaceDim],[-spaceDim,spaceDim],ground_img);
%     end
%     
%     %apply fancy lighting (optional)
%     camlight
    
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

t = 0:0.02:0.02*(length(x)-1);
f2 = figure;
plot(t,x);
title('Drone x vs time')
xlabel('t');
ylabel('x');
hold on
plot(t,xr);
legend('real','ref');
% % 
f3 = figure;
plot(t,y);
title('Drone y vs time')
xlabel('t');
ylabel('y');
hold on
plot(t,yr);
legend('real','ref');
% % 
f4 = figure;
plot(t,z);
title('Drone altitude vs time')
xlabel('t/s');
ylabel('z/m');
hold on
plot(t,zr);
legend('real','ref');
% % 
f5 = figure;
plot(x,y);
title('Drone follow the circle trajectory')
xlabel('x/m');
ylabel('y/m');
hold on
plot(xr,yr);
legend('real','ref');


