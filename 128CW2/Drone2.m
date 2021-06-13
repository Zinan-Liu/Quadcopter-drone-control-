%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%  Code modified by Francisco Vasconcelos from
%%%%
%%%%  Drone class, feel free to add functionality as you see fit
%%%%  Author: Daniel Butters
%%%%  Date: 16/11/17
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
classdef Drone2 < handle
    properties (Constant)
        %width, length, height offset between centre and rotors
        body = [0.6 0.6 0.0];
        
        %time interval for simulation (seconds)
        time_interval = 0.02;
        
        % size of floating window that follows drone
        axis_size = 2.;
        
        %colours of each component of drone model
        colours = [[.8 .3 .1];[.2 .2 .5];[.8 .1 .3];[.9 .6 .8];[.9 .2 .4]];
        
        %Follows the drone within the figure
        %Don't use if you are simulating more than one drone!
        %Switch to false to see the overall world view
        drone_follow = true;
    end
    properties
        %axis to draw on
        axis
        
        %length of one side of the flight arena
        spaceDim
        
        %limits of flight arena
        spaceLimits
        
        %drone position
        pos
        
        %drone position dot
        posdot
        
        %drone position dotdot
        posdotdot
        
        %drone orientation
        orient
        
        %drone orientation dot
        orientdot
        
        %drone omega
        omega
        
        %drone omega dot
        omegadot
        
        %drone rotation matrix
        R
        
        %Simulation time
        time
        
        %parameter to start drone in random position
        pos_offset
        
        %parameter to start drone in random position
        orient_offset
        
        % position ref
        pos_ref
        
        % position ref circle
        pos_ref_circle
        
        % orient ref
        orient_ref
        
        %number of drones
        num_drones
        
        %parameter to define
        m %mass
        g %gravity accleration
        k %thrust coefficient
        kd %drag force coefficient
        b %thrust torque coefficient
        Ixx %inertia x axis
        Iyy %inertia y axis
        Izz %inertia z axis
        I % inertia matrix
        L %arm length
        
        %orientation PID control
        Kp_o;
        Ki_o;
        Kd_o;
        eprev_o;
        ei_o;
        
        %position hori PID control
        Kp_h;
        Ki_h;
        Kd_h;
        eprev_h;
        ei_h;
        
        Kp_hori;
        Ki_hori;
        Kd_hori;
        eprev_hori
        ei_hori;
        
        %position landing PID control
        Kp_land;
        Ki_land;
        Kd_land;
        
        %control inputs
        inputs;
        
        %distance between drones
        d
      
    end
    methods
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %INSTANTIATION OF CLASS
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function obj = Drone2(axis, spaceDim, num_drones)
            if nargin > 1
                obj.axis = axis;
                
                obj.spaceDim = spaceDim;
                
                obj.spaceLimits = [(-spaceDim/2)+10 (spaceDim/2)-10 (-spaceDim/2)+10 (spaceDim/2)-10 10 spaceDim-10];
                
                obj.pos = [0.3;0;0];
                
                obj.posdot = [0;0;0];
                
                obj.posdotdot = [0;0;0];
                
                obj.orient = [0;0;0];
                
                obj.orientdot = [0;0;0];
                
                obj.orient_offset = obj.orientdot * obj.time_interval;
                
                obj.omega = [0;0;0];
                
                obj.omegadot = [0;0;0];
                
                obj.pos_offset = [0.3,0,0]; 
                
                obj.orient_offset = [0,0,0];
                
                obj.pos_ref = [0.3;0;2.5];

%                 obj.pos_ref_circle = [0.3;0;2.5];
                
                obj.pos_ref_circle = [0.613333083910761;0.019713246713805;2.5];
                
                obj.orient_ref = [0;0;0];
                
                obj.R = [1,0,0;0,1,0;0,0,1];
                
                obj.time = 0;
                
                obj.num_drones = num_drones;
                
                % predetermined parameter
                obj.m = 1; %kg
                obj.g = 9.81; %ms^-2
                obj.k = 1; 
                obj.kd = 0.1;
                obj.Ixx = 0.04; %kgm^2
                obj.Iyy = 0.04; %kgm^2
                obj.Izz = 0.008; %kgm^2
                obj.L = 0.25; %m
                obj.b = 0.007;
                obj.I = [obj.Ixx,0,0;0,obj.Iyy,0;0,0,obj.Izz];
                
                %orientation PID control
                obj.Kp_o =0.7;
                obj.Ki_o =0;
                obj.Kd_o =0.5;
                obj.eprev_o = [0;0;0];
                obj.ei_o = [0;0;0];

                %position PID control
                obj.Kp_h =0.3; %15
                obj.Ki_h =0.001;
                obj.Kd_h =4.5; %300
                obj.eprev_h = 0;
                obj.ei_h = 0;

                % PID control for hori [x,y]
                obj.Kp_hori =[0.01025,0.01025];
                obj.Ki_hori =[0,0];
                obj.Kd_hori =[0.025,0.025];
                
                obj.eprev_hori = [0;0;0];
                obj.ei_hori = [0;0;0];
                
                % PID control for hori [x,y]
                obj.Kp_land =0.002;
                obj.Ki_land =0;
                obj.Kd_land =0.009;
                
                %control inputs
                obj.inputs = [0;0;0;0];
                
                %distance between drones
                obj.d = 0;
            else
                error('Drone not initialised correctly')
            end
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %DRAWING OF DRONE TO FIGURE
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function draw2(obj)
            %how big should the moving window be
            cL = obj.axis_size;
            
            %set to false if you want to see world view
            %if(obj.drone_follow)
            %    axis([obj.pos(1)-cL obj.pos(1)+cL obj.pos(2)-cL obj.pos(2)+cL obj.pos(3)-cL obj.pos(3)+cL]);
            %end
            
            %create middle sphere
            [X Y Z] = sphere(8);
            %[X Y Z] = (obj.body(1)/5.).*[X Y Z];
            X = (obj.body(1)/5.).*X + obj.pos(1);
            Y = (obj.body(1)/5.).*Y + obj.pos(2);
            Z = (obj.body(1)/5.).*Z + obj.pos(3);
            s = surf(obj.axis,X,Y,Z);
            set(s,'edgecolor','none','facecolor',obj.colours(1,:));
            
            %create side spheres
            %front, right, back, left
            hOff = obj.body(3)/2;
            Lx = obj.body(1)/2;
            Ly = obj.body(2)/2;
            rotorsPosBody = [...
                0    Ly    0    -Ly;
                Lx    0    -Lx   0;
                hOff hOff hOff hOff];
            rotorsPosInertial = zeros(3,4);
            for i = 1:4
                rotorPosBody = rotorsPosBody(:,i);
                rotorsPosInertial(:,i) = bodyToInertial(obj,rotorPosBody);
                [X Y Z] = sphere(8);
                X = (obj.body(1)/8.).*X + obj.pos(1) + rotorsPosInertial(1,i);
                Y = (obj.body(1)/8.).*Y + obj.pos(2) + rotorsPosInertial(2,i);
                Z = (obj.body(1)/8.).*Z + obj.pos(3) + rotorsPosInertial(3,i);
                s = surf(obj.axis,X,Y,Z);
                set(s,'edgecolor','none','facecolor',obj.colours(i+1,:));
            end
            obj.axis.Title.String = ['Sim Time = ',num2str(obj.time,'%f'),' seconds'];
        end
        
        function vectorInertial = bodyToInertial(obj, vectorBody)
            vectorInertial = obj.R*vectorBody;
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %SIMULATION FUNCTIONS
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        function obj = change_pos_and_orientation_h_follow(obj) %function for follow drone1 aloft
            % altitude error computation
            h_e = obj.pos_ref(3)-obj.pos(3);
            h_ed = (h_e - obj.eprev_h)/obj.time_interval;
            obj.ei_h = obj.ei_h + h_e*obj.time_interval;
            % PID control input
            PID_h = obj.Kp_h*h_e + obj.Ki_h*obj.ei_h + obj.Kd_h*h_ed;
            % update the previous error
            obj.eprev_h = h_e;
            % orient angle
            phi = obj.orient(1);
            theta = obj.orient(2);
            psi = obj.orient(3);
            % Input 
            e_o = (obj.Kd_o*(obj.orient-obj.eprev_o)/obj.time_interval)+obj.Kp_o*obj.orient+obj.Ki_o*(obj.orient+obj.orient*obj.time_interval);
            obj.eprev_o = obj.orient;
            ephi = e_o(1);
            etheta = e_o(2);
            epsi = e_o(3);
            gamma1 = (obj.m*obj.g/(4*obj.k*cos(theta)*cos(phi)))-(2*obj.b*ephi*obj.Ixx+epsi*obj.Izz*obj.k*obj.L)/(4*obj.b*obj.k*obj.L)+PID_h;
            gamma2 = (obj.m*obj.g/(4*obj.k*cos(theta)*cos(phi)))+(epsi*obj.Izz)/(4*obj.b)-(etheta*obj.Iyy)/(2*obj.k*obj.L)+PID_h;
            gamma3 = (obj.m*obj.g/(4*obj.k*cos(theta)*cos(phi)))-(-2*obj.b*ephi*obj.Ixx+epsi*obj.Izz*obj.k*obj.L)/(4*obj.b*obj.k*obj.L)+PID_h;
            gamma4 = (obj.m*obj.g/(4*obj.k*cos(theta)*cos(phi)))+(epsi*obj.Izz)/(4*obj.b)+(etheta*obj.Iyy)/(2*obj.k*obj.L)+PID_h;
            obj.inputs = [gamma1;gamma2;gamma3;gamma4];
            
            obj.omega = orientdot2omega(obj);
       
            % normal simulation update 
            obj.posdotdot = acceleration(obj);
            
            obj.omegadot = angular_acceleration(obj);
            
            %omega update
            obj.omega = obj.omega + obj.time_interval*obj.omegadot;
            
            %orientdot update
            obj.orientdot = omega2orientdot(obj);
            
            %orient update
            obj.orient = obj.orient + obj.time_interval*obj.orientdot;
            
            %posdot update
            obj.posdot = obj.posdot + obj.time_interval*obj.posdotdot;
            
            %pos update
            obj.pos = obj.pos + obj.time_interval*obj.posdot;
        end
        
        function obj = change_pos_and_orientation_circle_follow(obj) %%function for follow drone1 circle
            global rot_angle
            global drone1pos
            if rot_angle <= -9e-3 || rot_angle >= -1e-4
                action = 'circle';
            else
                action = 'aloft_at_origin';
                if drone1pos(1) >= -0.001 && drone1pos(1) <= 0.001 && drone1pos(2) >= -0.001 && drone1pos(2) <= 0.001
                    action = 'landing';
                end
            end
       
            switch action
                case 'circle'
                    global drone1tar

                    % update the target point on the circle every iteration
                    % x coordinate
                    obj.pos_ref_circle(1) = drone1pos(1)+0.3;
                    % y coordinate
                    obj.pos_ref_circle(2) = drone1tar(2);
                    % distance between drone1 and drone2
                    droneleadpos = drone1tar';
                    dronefollowpos = obj.pos';
                    obj.d = pdist([droneleadpos;dronefollowpos],'euclidean');

                    %error of position PID controller
                    posx_e = obj.pos(1) - droneleadpos(1); % x position difference
                    posy_e = obj.pos(2) - droneleadpos(2); % y position difference
                    posz_e = droneleadpos(3) - obj.pos(3); % z position difference
                    % integral position error
                    obj.ei_hori(1) = obj.ei_hori(1) + posx_e * obj.time_interval; % x position error integral
                    obj.ei_hori(2) = obj.ei_hori(2) + posy_e * obj.time_interval; % y position error integral
                    obj.ei_hori(3) = obj.ei_hori(3) + posz_e * obj.time_interval; % z position error integral
                    % differentiate position error
                    posx_ed = (obj.pos(1) - obj.eprev_hori(1))/obj.time_interval; % x position error differentiate
                    posy_ed = (obj.pos(2) - obj.eprev_hori(2))/obj.time_interval; % y position error differentiate
                    posz_ed = (posz_e - obj.eprev_hori(3))/obj.time_interval; % z position error differentiate

                    % position PID control, use this as the offset of the orientation angle
                    posx_PID = obj.Kp_hori(1) * posx_e + obj.Kd_hori(1) * posx_ed + obj.Ki_hori(1) * obj.ei_hori(1); % x position PID
                    posy_PID = obj.Kp_hori(2) * posy_e + obj.Kd_hori(2) * posy_ed + obj.Ki_hori(2) * obj.ei_hori(2); % y position PID
                    posz_PID = obj.Kp_h * posz_e + obj.Kd_h * posz_ed + obj.Ki_h * obj.ei_hori(3); % z position PID

                    % position previous error update
                    obj.eprev_hori(1) = obj.pos(1);
                    obj.eprev_hori(2) = obj.pos(2);
                    obj.eprev_hori(3) = posz_e;

                    % orient angle
                    phi = obj.orient(1);
                    theta = obj.orient(2);
                    psi = obj.orient(3);

%                     if posy_PID > 0.6
%                         posy_PID = 0.6;
%                     elseif posy_PID < -0.6
%                         posy_PID = -0.6;
%                     end
% 
%                     if posx_PID > 0.6
%                         posx_PID = 0.6;
%                     elseif posx_PID < -0.6
%                         posx_PID = -0.6;
%                     end 

                    %error of orientation PID controller
                    phi_e = phi - posy_PID;      %phi orientation difference, (phi - variable) if variable (+,negative y;-,positive y)
                    theta_e = theta + posx_PID;  %theta orientation difference, (theta + variable) if variable (+,negative x;-, positive x)
                    psi_e = psi;                 %psi orientation difference

                    %integral orientation error
                    obj.ei_o(1) = obj.ei_o(1) + phi_e * obj.time_interval;     %phi orientation error integral
                    obj.ei_o(2) = obj.ei_o(2) + theta_e * obj.time_interval;   %theta orientation error integral
                    obj.ei_o(3) = obj.ei_o(3) + psi_e * obj.time_interval;     %psi orientation error integral
                    % differentiate orientation error
                    phi_ed = (phi_e - obj.eprev_o(1))/obj.time_interval;       %phi orientation error differentiation
                    theta_ed = (theta_e - obj.eprev_o(2))/obj.time_interval;   %theta orientation error differentiation
                    psi_ed = (psi_e - obj.eprev_o(3))/obj.time_interval;       %psi orientation error differentiation

                    % orientation PID control
                    ephi = obj.Kp_o * phi_e + obj.Kd_o * phi_ed + obj.Ki_o * obj.ei_o(1);        % phi orientation PID
                    etheta = obj.Kp_o * theta_e + obj.Kd_o * theta_ed + obj.Ki_o * obj.ei_o(2);  % theta orientation PID
                    epsi = obj.Kp_o * psi_e + obj.Kd_o * psi_ed + obj.Ki_o * obj.ei_o(3);        % psi orientation PID

                    % orientation previous error update
                    obj.eprev_o(1) = phi_e;
                    obj.eprev_o(2) = theta_e;
                    obj.eprev_o(3) = psi_e;

                    % Input computation
                    gamma1 = (obj.m*obj.g/(4*obj.k*cos(theta)*cos(phi)))-(2*obj.b*ephi*obj.Ixx+epsi*obj.Izz*obj.k*obj.L)/(4*obj.b*obj.k*obj.L)+posz_PID;
                    gamma2 = (obj.m*obj.g/(4*obj.k*cos(theta)*cos(phi)))+(epsi*obj.Izz)/(4*obj.b)-(etheta*obj.Iyy)/(2*obj.k*obj.L)+posz_PID;
                    gamma3 = (obj.m*obj.g/(4*obj.k*cos(theta)*cos(phi)))-(-2*obj.b*ephi*obj.Ixx+epsi*obj.Izz*obj.k*obj.L)/(4*obj.b*obj.k*obj.L)+posz_PID;
                    gamma4 = (obj.m*obj.g/(4*obj.k*cos(theta)*cos(phi)))+(epsi*obj.Izz)/(4*obj.b)+(etheta*obj.Iyy)/(2*obj.k*obj.L)+posz_PID;
                    obj.inputs = [gamma1;gamma2;gamma3;gamma4];

                    obj.omega = orientdot2omega(obj);
                    
                    % normal acceleration update
                    obj.posdotdot = acceleration_xyz(obj);
                    obj.omegadot = angular_acceleration(obj);

                    %omega update
                    obj.omega = obj.omega + obj.time_interval*obj.omegadot;

                    %orientdot update
                    obj.orientdot = omega2orientdot(obj);

                    %orient update
                    obj.orient = obj.orient + obj.time_interval*obj.orientdot;

                    %posdot update
                    obj.posdot = obj.posdot + obj.time_interval*obj.posdotdot;

                    %pos update
                    obj.pos = obj.pos + obj.time_interval*obj.posdot;
                    
                case 'aloft_at_origin'
                    
                    %error of position PID controller
                    posx_e = obj.pos(1) - 0; % x position difference
                    posy_e = obj.pos(2) - 0; % y position difference
                    posz_e = obj.pos_ref_circle(3) - obj.pos(3); % z position difference
                    % integral position error
                    obj.ei_hori(1) = obj.ei_hori(1) + posx_e * obj.time_interval; % x position error integral
                    obj.ei_hori(2) = obj.ei_hori(2) + posy_e * obj.time_interval; % y position error integral
                    obj.ei_hori(3) = obj.ei_hori(3) + posz_e * obj.time_interval; % z position error integral
                    % differentiate position error
                    posx_ed = (obj.pos(1) - obj.eprev_hori(1))/obj.time_interval; % x position error differentiate
                    posy_ed = (obj.pos(2) - obj.eprev_hori(2))/obj.time_interval; % y position error differentiate
                    posz_ed = (posz_e - obj.eprev_hori(3))/obj.time_interval; % z position error differentiate

                    % position PID control, use this as the offset of the orientation angle
                    posx_PID = obj.Kp_hori(1) * posx_e + obj.Kd_hori(1) * posx_ed + obj.Ki_hori(1) * obj.ei_hori(1); % x position PID
                    posy_PID = obj.Kp_hori(2) * posy_e + obj.Kd_hori(2) * posy_ed + obj.Ki_hori(2) * obj.ei_hori(2); % y position PID
                    posz_PID = obj.Kp_h * posz_e + obj.Kd_h * posz_ed + obj.Ki_h * obj.ei_hori(3); % z position PID

                    % position previous error update
                    obj.eprev_hori(1) = obj.pos(1);
                    obj.eprev_hori(2) = obj.pos(2);
                    obj.eprev_hori(3) = posz_e;

                    % orient angle
                    phi = obj.orient(1);
                    theta = obj.orient(2);
                    psi = obj.orient(3);

                    if posy_PID > 0.6
                        posy_PID = 0.6;
                    elseif posy_PID < -0.6
                        posy_PID = -0.6;
                    end

                    if posx_PID > 0.6
                        posx_PID = 0.6;
                    elseif posx_PID < -0.6
                        posx_PID = -0.6;
                    end 

                    %error of orientation PID controller
                    phi_e = phi - posy_PID;      %phi orientation difference, (phi - variable) if variable (+,negative y;-,positive y)
                    theta_e = theta + posx_PID;  %theta orientation difference, (theta + variable) if variable (+,negative x;-, positive x)
                    psi_e = psi;                 %psi orientation difference

                    %integral orientation error
                    obj.ei_o(1) = obj.ei_o(1) + phi_e * obj.time_interval;     %phi orientation error integral
                    obj.ei_o(2) = obj.ei_o(2) + theta_e * obj.time_interval;   %theta orientation error integral
                    obj.ei_o(3) = obj.ei_o(3) + psi_e * obj.time_interval;     %psi orientation error integral
                    % differentiate orientation error
                    phi_ed = (phi_e - obj.eprev_o(1))/obj.time_interval;       %phi orientation error differentiation
                    theta_ed = (theta_e - obj.eprev_o(2))/obj.time_interval;   %theta orientation error differentiation
                    psi_ed = (psi_e - obj.eprev_o(3))/obj.time_interval;       %psi orientation error differentiation

                    % orientation PID control
                    ephi = obj.Kp_o * phi_e + obj.Kd_o * phi_ed + obj.Ki_o * obj.ei_o(1);        % phi orientation PID
                    etheta = obj.Kp_o * theta_e + obj.Kd_o * theta_ed + obj.Ki_o * obj.ei_o(2);  % theta orientation PID
                    epsi = obj.Kp_o * psi_e + obj.Kd_o * psi_ed + obj.Ki_o * obj.ei_o(3);        % psi orientation PID

                    % orientation previous error update
                    obj.eprev_o(1) = phi_e;
                    obj.eprev_o(2) = theta_e;
                    obj.eprev_o(3) = psi_e;

                    % Input computation
                    gamma1 = (obj.m*obj.g/(4*obj.k*cos(theta)*cos(phi)))-(2*obj.b*ephi*obj.Ixx+epsi*obj.Izz*obj.k*obj.L)/(4*obj.b*obj.k*obj.L)+posz_PID;
                    gamma2 = (obj.m*obj.g/(4*obj.k*cos(theta)*cos(phi)))+(epsi*obj.Izz)/(4*obj.b)-(etheta*obj.Iyy)/(2*obj.k*obj.L)+posz_PID;
                    gamma3 = (obj.m*obj.g/(4*obj.k*cos(theta)*cos(phi)))-(-2*obj.b*ephi*obj.Ixx+epsi*obj.Izz*obj.k*obj.L)/(4*obj.b*obj.k*obj.L)+posz_PID;
                    gamma4 = (obj.m*obj.g/(4*obj.k*cos(theta)*cos(phi)))+(epsi*obj.Izz)/(4*obj.b)+(etheta*obj.Iyy)/(2*obj.k*obj.L)+posz_PID;
                    obj.inputs = [gamma1;gamma2;gamma3;gamma4];

                    obj.omega = orientdot2omega(obj);

                    %compute linear and angular accelerations
                    obj.posdotdot = acceleration_xyz(obj);
                    obj.omegadot = angular_acceleration(obj);

                    %omega update
                    obj.omega = obj.omega + obj.time_interval*obj.omegadot;

                    %orientdot update
                    obj.orientdot = omega2orientdot(obj);

                    %orient update
                    obj.orient = obj.orient + obj.time_interval*obj.orientdot;

                    %posdot update
                    obj.posdot = obj.posdot + obj.time_interval*obj.posdotdot;

                    %pos update
                    obj.pos = obj.pos + obj.time_interval*obj.posdot;
                    
                case 'landing'
                    
                    %error of position PID controller
                    posx_e = obj.pos(1) - 0; % x position difference
                    posy_e = obj.pos(2) - 0; % y position difference
                    posz_e = 0 - obj.pos(3); % z position difference
                    % integral position error
                    obj.ei_hori(1) = obj.ei_hori(1) + posx_e * obj.time_interval; % x position error integral
                    obj.ei_hori(2) = obj.ei_hori(2) + posy_e * obj.time_interval; % y position error integral
                    obj.ei_hori(3) = obj.ei_hori(3) + posz_e * obj.time_interval; % z position error integral
                    % differentiate position error
                    posx_ed = (obj.pos(1) - obj.eprev_hori(1))/obj.time_interval; % x position error differentiate
                    posy_ed = (obj.pos(2) - obj.eprev_hori(2))/obj.time_interval; % y position error differentiate
                    posz_ed = (posz_e - obj.eprev_hori(3))/obj.time_interval; % z position error differentiate

                    % position PID control, use this as the offset of the orientation angle
                    posx_PID = obj.Kp_hori(1) * posx_e + obj.Kd_hori(1) * posx_ed + obj.Ki_hori(1) * obj.ei_hori(1); % x position PID
                    posy_PID = obj.Kp_hori(2) * posy_e + obj.Kd_hori(2) * posy_ed + obj.Ki_hori(2) * obj.ei_hori(2); % y position PID
                    posz_PID = obj.Kp_land * posz_e + obj.Kd_land * posz_ed + obj.Ki_land * obj.ei_hori(3); % z position PID

                    % position previous error update
                    obj.eprev_hori(1) = obj.pos(1);
                    obj.eprev_hori(2) = obj.pos(2);
                    obj.eprev_hori(3) = posz_e;

                    % orient angle
                    phi = obj.orient(1);
                    theta = obj.orient(2);
                    psi = obj.orient(3);

                    if posy_PID > 0.6
                        posy_PID = 0.6;
                    elseif posy_PID < -0.6
                        posy_PID = -0.6;
                    end

                    if posx_PID > 0.6
                        posx_PID = 0.6;
                    elseif posx_PID < -0.6
                        posx_PID = -0.6;
                    end 

                    %error of orientation PID controller
                    phi_e = phi - posy_PID;      %phi orientation difference, (phi - variable) if variable (+,negative y;-,positive y)
                    theta_e = theta + posx_PID;  %theta orientation difference, (theta + variable) if variable (+,negative x;-, positive x)
                    psi_e = psi;                 %psi orientation difference

                    %integral orientation error
                    obj.ei_o(1) = obj.ei_o(1) + phi_e * obj.time_interval;     %phi orientation error integral
                    obj.ei_o(2) = obj.ei_o(2) + theta_e * obj.time_interval;   %theta orientation error integral
                    obj.ei_o(3) = obj.ei_o(3) + psi_e * obj.time_interval;     %psi orientation error integral
                    % differentiate orientation error
                    phi_ed = (phi_e - obj.eprev_o(1))/obj.time_interval;       %phi orientation error differentiation
                    theta_ed = (theta_e - obj.eprev_o(2))/obj.time_interval;   %theta orientation error differentiation
                    psi_ed = (psi_e - obj.eprev_o(3))/obj.time_interval;       %psi orientation error differentiation

                    % orientation PID control
                    ephi = obj.Kp_o * phi_e + obj.Kd_o * phi_ed + obj.Ki_o * obj.ei_o(1);        % phi orientation PID
                    etheta = obj.Kp_o * theta_e + obj.Kd_o * theta_ed + obj.Ki_o * obj.ei_o(2);  % theta orientation PID
                    epsi = obj.Kp_o * psi_e + obj.Kd_o * psi_ed + obj.Ki_o * obj.ei_o(3);        % psi orientation PID

                    % orientation previous error update
                    obj.eprev_o(1) = phi_e;
                    obj.eprev_o(2) = theta_e;
                    obj.eprev_o(3) = psi_e;

                    % Input computation
                    gamma1 = (obj.m*obj.g/(4*obj.k*cos(theta)*cos(phi)))-(2*obj.b*ephi*obj.Ixx+epsi*obj.Izz*obj.k*obj.L)/(4*obj.b*obj.k*obj.L)+posz_PID;
                    gamma2 = (obj.m*obj.g/(4*obj.k*cos(theta)*cos(phi)))+(epsi*obj.Izz)/(4*obj.b)-(etheta*obj.Iyy)/(2*obj.k*obj.L)+posz_PID;
                    gamma3 = (obj.m*obj.g/(4*obj.k*cos(theta)*cos(phi)))-(-2*obj.b*ephi*obj.Ixx+epsi*obj.Izz*obj.k*obj.L)/(4*obj.b*obj.k*obj.L)+posz_PID;
                    gamma4 = (obj.m*obj.g/(4*obj.k*cos(theta)*cos(phi)))+(epsi*obj.Izz)/(4*obj.b)+(etheta*obj.Iyy)/(2*obj.k*obj.L)+posz_PID;
                    obj.inputs = [gamma1;gamma2;gamma3;gamma4];

                    obj.omega = orientdot2omega(obj);

                    %compute linear and angular accelerations
                    obj.posdotdot = acceleration_xyz(obj);
                    obj.omegadot = angular_acceleration(obj);

                    %omega update
                    obj.omega = obj.omega + obj.time_interval*obj.omegadot;

                    %orientdot update
                    obj.orientdot = omega2orientdot(obj);

                    %orient update
                    obj.orient = obj.orient + obj.time_interval*obj.orientdot;

                    %posdot update
                    obj.posdot = obj.posdot + obj.time_interval*obj.posdotdot;
                    disp(obj.posdot(3));

                    %pos update
                    obj.pos = obj.pos + obj.time_interval*obj.posdot;
                    
            end

        end
        
        % Compute thrust given current inputs and thrust coefficient.
        function T = thrust(obj)
            % Inputs are values for wi^2
            T = [0; 0; obj.k * sum(obj.inputs)];
        end
        
        % Compute torques, given current inputs, length, drag coefficient, and thrust coefficient
        function tau = torques(obj)
            % Inputs are values for wi^2
            tau = [obj.L * obj.k * (obj.inputs(1) - obj.inputs(3));...
                   obj.L * obj.k * (obj.inputs(2) - obj.inputs(4));...
                   obj.b * (obj.inputs(1) - obj.inputs(2) + obj.inputs(3) - obj.inputs(4))
                   ];
        end
        
        function a = acceleration(obj)
            gravity = [0; 0; -obj.g];
            phi = obj.orient(1);
            theta = obj.orient(2);
            psi = obj.orient(3);
            Rotation = [cos(phi)*cos(theta)*cos(psi)-sin(phi)*sin(psi),-cos(psi)*sin(phi)-cos(phi)*cos(theta)*sin(psi),cos(phi)*sin(theta);...
                        cos(phi)*sin(psi)+cos(theta)*cos(psi)*sin(phi),cos(phi)*cos(psi)-cos(theta)*sin(phi)*sin(psi),sin(phi)*sin(theta);...
                        -cos(psi)*sin(theta),sin(theta)*sin(psi),cos(theta)];
            T = Rotation * thrust(obj);
            Fd = -obj.kd * obj.posdot;
            a = gravity + 1 / obj.m * T + Fd;
        end
        
        function a = acceleration_xyz(obj)
            gravity = [0; 0; -obj.g];
            phi = obj.orient(1);
            theta = obj.orient(2);
            psi = obj.orient(3);
            Rotation = [cos(theta)*cos(psi),                            -cos(theta)*sin(psi),                           sin(theta);...
                        cos(phi)*sin(psi)+cos(psi)*sin(phi)*sin(theta), cos(phi)*cos(psi)-sin(phi)*sin(theta)*sin(psi), -cos(theta)*sin(phi);...
                        sin(phi)*sin(psi)-cos(phi)*cos(psi)*sin(theta), cos(psi)*sin(phi)+cos(phi)*sin(theta)*sin(psi), cos(phi)*cos(theta)];
            T = Rotation * thrust(obj);
            Fd = -obj.kd * obj.posdot;
            a = gravity + 1 / obj.m * T + Fd;
        end
        
        function omegadot = angular_acceleration(obj)
            tau = torques(obj);
            omegadot = obj.I \ (tau - cross(obj.omega, obj.I * obj.omega));
        end
        
        function omega = orientdot2omega(obj)
            phi = obj.orient(1);
            theta = obj.orient(2);
            Rt = [1,0,-sin(theta);...
                  0,cos(phi),cos(theta)*sin(phi);...
                  0,-sin(phi),cos(theta)*cos(phi)];
            omega = Rt*obj.orientdot;
        end
        
        function orientdot = omega2orientdot(obj)
            phi = obj.orient(1);
            theta = obj.orient(2);
            Rt = [1,0,-sin(theta);...
                  0,cos(phi),cos(theta)*sin(phi);...
                  0,-sin(phi),cos(theta)*cos(phi)];
            orientdot = Rt\obj.omega;
        end
        
        function update2(obj)
            %update simulation time
            obj.time = obj.time + obj.time_interval;
            %change position and orientation of drone
            if obj.time >= 0 && obj.time <= 3
                obj = change_pos_and_orientation_h_follow(obj);
            end
            if obj.time > 3
                obj = change_pos_and_orientation_circle_follow(obj);
            end
            
%             %draw drone on figure
%             draw2(obj);
        end
        
    end
end
