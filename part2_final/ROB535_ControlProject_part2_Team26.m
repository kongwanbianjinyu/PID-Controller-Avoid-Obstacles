function [sol_2, FLAG_terminate] = ROB535_ControlProject_part2_Team26(TestTrack, Xobs_seen, curr_state)
% ROB535_ControlProject_part2_Team26

%%% Inputs
%   TestTrack - the structure loaded from TestTrack.mat
%   Xobs_seen - the output of the senseObstacles function.
%   curr_state - taken from the last state in Y, where Y is the actual trajectory 
%                updated after forward integrating the vehicle dynamics using control
%                inputs at every planning iteration
%%% Outputs
%   sol_2 - a vector of control inputs that will be passed to forwardIntegrateControlInput. 
%           sol 2 must have enough control inputs with time step 0.01 seconds to forward integrate 
%           vehicle dynamics for the next 0.5 second.
%   FLAG_terminate - a binary flag set by the function to indicate when to stop the simulation.

%% Avoid Obs 
target_path = AvoidObstacles_jjc(TestTrack,Xobs_seen);

%% Get Reference
[ref_u, ref_phi, ref_path] = GetReference(target_path);

%% PI Controller
[Utemp,Progress_bar] = PI_Controller(curr_state,ref_u,ref_phi,ref_path);

%% return and set flag
sol_2 = Utemp;
if Progress_bar > 0.997
    FLAG_terminate = 1;
else
    FLAG_terminate = 0;
end

end

%% get target aviod obs path(x,y,theta) according to obs and test track
function target_path = AvoidObstacles_jjc(TestTrack,Xobs)

    bl = TestTrack.bl;
    br = TestTrack.br;
    cline = TestTrack.cline;
    theta = TestTrack.theta;

    target_path = [cline;theta];

    % if no obs, set our target_path as cline
    if size(Xobs) == 0
        target_path = [cline;theta]; 
    else
    %if see obs
        linear_interp_points = 20; % linear interp number
        obs_idx = 1; % obs passed number

        Obs_exist = false; % whether detected obs
        Obs_detected = zeros(size(Xobs{1})); % save obs detected


        for idx = 2: size(cline,2)
            % linear interp between current points and previous points
            x = linspace(cline(1,idx-1),cline(1,idx),linear_interp_points);
            y = linspace(cline(2,idx-1),cline(2,idx),linear_interp_points);


            % detect whether there is any obs between two points

            if obs_idx <= size(Xobs,2)

                % get circle around obs
                Obs = Xobs{obs_idx};
                Obs_center = mean(Obs);
                Obs_radius = norm(mean(Obs)-Obs(1,:))+0.15;

                % detect whether the line intersect with the obs circle
                for i = 1:length(x)
                    if norm([x(i),y(i)] - Obs_center)< Obs_radius
                        Obs_exist = true;
                        Obs_detected = Obs;
                        continue
                    end
                end

            end

            % if Obs exist in this idx
            if Obs_exist
                % find detected obs center
                Obs_center_detected = mean(Obs_detected);

                % find the min distance from obs center to right/left 
                [~, left_distance] = knnsearch(bl', Obs_center_detected);
                [~, right_distance] = knnsearch(br', Obs_center_detected);

                % avoid obs by changing lane
                if right_distance < left_distance % if object is on right
                    target_path(1:2,idx-1:idx+1) = (2*bl(:,idx-1:idx+1)+cline(:,idx-1:idx+1)) ./ 3;

                else % if object is on left
                    target_path(1:2,idx-1:idx+1) = (2*br(:,idx-1:idx+1)+cline(:,idx-1:idx+1)) ./ 3;
                end

                obs_idx  = obs_idx + 1; % passed the obs
            end

            % recover values

            Obs_exist = false;
            Obs_detected = zeros(size(Xobs{1}));


        end

        % Set Heading
        theta_new = atan2(diff(target_path(2,:)),diff(target_path(1,:)));
        theta_new(end+1) = theta_new(end);

        target_path(3,:) = theta_new;

    end


end

%% Get referencce u(velosity) and phi(angle) according to target path
function [ref_u,ref_phi,ref_path] = GetReference(target_path)

    % Interpolate target path

    interpolate_num = 20;
    target_path_interp = interp1(1:size(target_path,2),target_path',1:1/interpolate_num:size(target_path,2),'spline');
    target_path_interp = target_path_interp';
    % size(target_path_interp) % (3,4901)


    % get referencce u(velosity) and phi(angle)

    total_time = 300; % total time to finish the path
    time_per_point_interp = total_time/(size(target_path_interp,2)-1); % time used passing per point in target_path_interp
    t = 0:0.01:total_time-0.01; % simulate time step

    % For each simulate time step, find the closest point index in target_path_interp
    start_point_idx = floor(t./time_per_point_interp)+1;
    % Add additional points (keeping the final u) to make sure finish the line
    start_point_idx = [start_point_idx, (size(target_path_interp,2)-1)*ones(1,50)];

    end_point_idx = start_point_idx + 1;

    % get the ref_u by distance/time in target_path_interp
    ref_u = vecnorm(target_path_interp(1:2,end_point_idx) - target_path_interp(1:2,start_point_idx))./time_per_point_interp;
    % get the ref_phi by theta in target_path_interp
    ref_phi = target_path_interp(3,start_point_idx);
    % get the ref_path 
    ref_path = target_path_interp(:,start_point_idx); %(3,31000)
    %size(ref_u)  %(1,31000)
end

%% PI controller outputs control [Fx, delta] to meet reference.
function [Utemp,Progress_Bar] = PI_Controller(initial_state,ref_u,ref_phi,ref_path)
    
    % PI controller parameters
    Kp_u = 3000; % velocity gain of P part
    Kp_phi = 8; % angle gain of P part
    
    Ki_phi = 1; % angle gain of I part
    I_part_saturation = 2; % saturation of I part
    I_part = 0; % initial I part

    % set simulate time
    simulate_time = 0.5;
    simulate_points = simulate_time/0.01;

    % choose a segment of 50 points in reference path based on current
    % state's position
    desired_center_line = ref_path(1:2,:);
    segment_start_idx = knnsearch(desired_center_line',[initial_state(1),initial_state(3)]) + 1;

    Progress_Bar = segment_start_idx/size(ref_path,2)
    
    segment_idx = segment_start_idx:segment_start_idx + simulate_points;
    
    next_state = initial_state;

    Utemp = zeros(simulate_points + 1, 2);
    prev_control = [0,0];
    % for each point in the segment
    for i =  1:simulate_points
        idx = segment_idx(i);
        
        % get the ref_u,ref_phi and current_state in this point
        curr_ref_u = ref_u(idx);
        curr_ref_phi = ref_phi(idx);
        
        current_state = next_state;
        
        % Calculate the error of velocity and angle (P_part)
        P_part_u = curr_ref_u - current_state(2);
        P_part_phi = curr_ref_phi - current_state(5);
        
        
        % Add integral gain to angle (I_part)
        
        % determine whether the ref point on the left of curr_state or on the
        % right of the curr_state by rotating the current point to x axis.
        curr_phi = current_state(5);
        
        % rotate matrix
        R = [cos(curr_phi), sin(curr_phi);
             -sin(curr_phi), cos(curr_phi)];

        rotated_current_point = R * [current_state(1);current_state(3)];
        rotated_reference_point = R * [ref_path(1,idx);ref_path(2,idx)];

        % the sign of the angle indicate the left/right.
        sign_angle = sign(rotated_reference_point(2) - rotated_current_point(2));

        % get the distance from current point to reference point
        distance_from_ref = ((current_state(1)-ref_path(1,idx))^2 + (current_state(3)- ref_path(2,idx))^2)^0.5;

        I_part = I_part + 0.01*sign_angle*distance_from_ref;
        
        % no excess I_part_saturation
        if I_part > I_part_saturation
            I_part = I_part_saturation;
        end
        if I_part < -I_part_saturation
            I_part = -I_part_saturation;
        end


        % Calculate control input using PI.
        Fx_control = Kp_u * P_part_u;
        delta_control = Kp_phi * P_part_phi + Ki_phi*I_part;
        
        % Saturate the control constrain
        [Fx_control,delta_control] = SaturationConstrain(current_state,Fx_control,delta_control);
        
        % Set the control input
        control_input = [prev_control;...
                         delta_control,Fx_control]; % 2*2
        prev_control = [delta_control,Fx_control];
        Utemp(i+1,:) = [delta_control,Fx_control]; % save the result 
     
        % forward system to get the new state
        [Y,~] = forwardIntegrateControlInput(control_input, current_state);
        % save the result state to next state
        next_state = Y(end,:);

    end
end

%% constrain inputs
function [Fx_control,delta_control] = SaturationConstrain(current_state,Fx_control,delta_control)

    %Vehicle parameters
    m = 1400;
    g = 9.806;
    delta_max = [-0.5,0.5];
    Fx_max = [-5000,5000];
    Nw = 2;
    f = 0.01;
    Iz = 2667;
    a = 1.35;
    b = 1.45;
    By = 0.27;
    Cy = 1.2;
    Dy = 0.7;
    Ey = -1.6;
    Shy = 0;
    Svy = 0;
    F_max = 0.7*m*g;
    
    
    if Fx_control > Fx_max(2)
       Fx_control = Fx_max(2);
    end

    if Fx_control < Fx_max(1)
       Fx_control = Fx_max(1);
    end
    
    %Set constraints based total traction
    u = current_state(2);
    v = current_state(4);
    r = current_state(6);
    Fzr = a/(a+b)*m*g;
    alpha_r = rad2deg(-atan((v-b*r)/u));
    phi_yr = (1-Ey)*(alpha_r + Shy) + Ey/By*atan(By*(alpha_r+Shy));
    Fyr = Fzr*Dy*sin(Cy*atan(By*phi_yr)+Svy);
    F_total = sqrt(Nw*Fx_control^2 + Fyr^2);
    
    
    if F_total >= F_max         
        F_x_max = sqrt(F_max^2 - Fyr^2)/Nw;
        
        Fx_control = F_x_max;
                
    end
   
    %Total saturation
    
   if Fx_control > Fx_max(2)
       Fx_control = Fx_max(2);
   end
   
   if Fx_control < Fx_max(1)
       Fx_control = Fx_max(1);
   end
   
   
   if delta_control > delta_max(2)
       delta_control = delta_max(2);
   end
   
   if delta_control < delta_max(1)
       delta_control = delta_max(1);
   end
   
end

