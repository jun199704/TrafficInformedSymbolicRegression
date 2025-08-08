clc;
clear;

import java.net.ServerSocket
import java.net.Socket
import java.io.*


% define safety metrics related functions
function distance = cal_distance_two_points(posX0, posY0, posX1, posY1)
    distance = sqrt((posX0 - posX1)^2 + (posY0 - posY1)^2);
end



function [distanceList,near_crash_list, near_crash_flag] = cal_traj_distance(single_vehicle_distance, single_vehicle_latdev, single_follow_distance)
    LW=3;
    distanceList=[];
    for i = 1:length(single_vehicle_distance)
        distance = sqrt((single_vehicle_distance(i)-single_follow_distance(i))^2+(LW-abs(single_vehicle_latdev(i)))^2);
        distanceList = [distanceList,distance];
    end
    if min(distanceList)>10
        near_crash_flag = 0;
    else
        near_crash_flag = 1;
    end

    near_crash_list = [];
    for i = 1:length(distanceList)
        if distanceList(i)>10
            near_crash_list = [near_crash_list, 0];
        else
            near_crash_list = [near_crash_list, 1];
        end
    end

end

function PETNearCrashList = generate_PET_near_crash_list(PETList,near_crash_list)

    PETNearCrashList = [];

    for idx = 1:length(PETList)
        if near_crash_list(idx) ~= 0
            PETNearCrashList = [PETNearCrashList, PETList(idx)];
        end
    end

end

function crashFlag = cal_crash_time(X0List, Y0List, X1List, Y1List)
%disp(X0List);
%disp(Y0List);
%disp(X1List);
%disp(Y1List);
    R = 1.8 / 2;
    crashFlag = 0;
    for i = 1:length(X0List)
        for j = 1:length(X1List)
            distance = cal_distance_two_points(X0List(i), Y0List(i), X1List(j), Y1List(j));
            if distance <= 2 * R
                crashFlag = 1;
                return;
            end
        end
    end
end


function [X0List, Y0List, X1List, Y1List] = cal_circle_center(posX0, posY0, posX1, posY1, dx, dy)
    d = sqrt(dx^2 + dy^2);
    a = dx * 2.5 / d;
    b = dy * 2.5 / d;
    X0List = [posX0, posX0 - a, posX0 - 2 * a];
    Y0List = [posY0, posY0 - b, posY0 - 2 * b];
    X1List = [posX1, posX1 - 2.5, posX1 - 5];
    Y1List = [posY1, posY1, posY1];
end

function  [crashFlagList, crashIdx] = cal_traj_crash_time(distanceAtk, latdev, followPosition)
    crashFlagList= [];
    LW = 3;
    for i = 2:length(distanceAtk)
        dx = abs(distanceAtk(i) - distanceAtk(i-1));
        dy = abs(latdev(i) - latdev(i-1));
        [X0List, Y0List, X1List, Y1List] = cal_circle_center(distanceAtk(i), abs(latdev(i)), followPosition(i), LW, dx, dy);
        
        %disp([X0List, Y0List, X1List, Y1List]);
        crashFlag = cal_crash_time(X0List, Y0List, X1List, Y1List);
        crashFlagList = [crashFlagList, crashFlag];
    end

    crashIdx = 10000;
    for i = 1:length(crashFlagList)
        if crashFlagList(i) == 1
            crashIdx = i;
            break;
        end
    end
    %disp(crashIdx);
end

function PET = cal_TTC_LC(LCSpeed, LKSpeed, dx, dy, LW, dev, dis_diff)
    
    L = dx * (LW - dev) / dy;  % Calculate the longitudinal distance
    dis_LC = sqrt(L^2 + (LW - dev)^2);  % Distance of the lane-changing vehicle
    T_LC = dis_LC / LCSpeed;  % Time for the lane-changing vehicle to cover the distance
    
    dis_LK = L + dis_diff;  % Distance of the following vehicle
    T_LK = dis_LK / LKSpeed;  % Time for the following vehicle to cover the distance
    
    % Time to collision (TTC)
    TTC = abs(dis_LC - dis_LK) / abs(LCSpeed - LKSpeed);
    
    % Post encroachment time (PET)
    PET = abs(dis_LC / LCSpeed - dis_LK / LKSpeed);
end

function PETList = cal_traj_TTC_LC(distanceAtk, followPosition, speed, followSpeed, latdev, crashIdx)
    
    PETList=[];
    LW = 3;  

    for i = 1:min((length(distanceAtk) - 1), crashIdx)
        LCSpeed = speed(i);   % change lane vehicle speed
        LKSpeed = followSpeed(i); %follow vehicle speed
        dx = abs(distanceAtk(i+1) - distanceAtk(i));
        dy = abs(latdev(i+1) - latdev(i));
        dev = abs(latdev(i));
        dis_diff = distanceAtk(i) - followPosition(i);

        if dy > 0
            PET_LC = cal_TTC_LC(LCSpeed, LKSpeed, dx, dy, LW, dev, dis_diff);
        else
            PET_LC = 10000;
        end
        PETList = [PETList, PET_LC];
        
    end
    
 
end

function counts = cal_traj_PET_distribution (PETList)
    
    % Define the bin edges
    bin_edges = 0:0.5:5;

    % Calculate the frequency distribution
    [counts, bin_edges] = histcounts(PETList, bin_edges);

end

function counts = cal_traj_DELTAV_distribution (DELTAVList)
    
    %Define the bin edges
    bin_edges = 0:1:30;
    
    [counts, bin_edges] = histcounts(DELTAVList, bin_edges);

end

function counts = cal_traj_injury_level_distribution(DELTAVList)
    
    % Define the bin edges for no injury, minor injury and fatal injury
    bin_edges = [0,8,14,24,10000];

    [counts, bin_edges] = histcounts(DELTAVList, bin_edges);
    
end

function counts = cal_lane_depart_time_distribution (LaneDepartList)

    % Define the bin edges for lane depart time
    bin_edges = [0:0.1:8];

    [counts, bin_edges] = histcounts(LaneDepartList, bin_edges); 
    
end


function H = hellinger_distance(P, Q)

    % Calculate Hellinger distance
    H = (1/sqrt(2)) * sqrt(sum((sqrt(P) - sqrt(Q)).^2));
end

function KL = KL_divergence (P, Q)
    % Ensure Q has no zero where P is non-zero by adding eps
    % Find indices where both P and Q are non-zero
    nonZeroIdx = (P > 0) & (Q > 0);

    % Calculate KL Divergence using only non-zero entries of both P and Q
    KL = sum(P(nonZeroIdx) .* log(P(nonZeroIdx) ./ Q(nonZeroIdx)));

end


function deltaV = perfect_inelastic_crash_change_of_velocity(v1_velocity, v2_velocity, v1_heading, v2_heading, v1_mass, v2_mass)
    % perfect_inelastic_crash_change_of_velocity calculates the change of velocity (deltaV)
    % for a perfect inelastic collision between two objects (vehicles) based on conservation of momentum.
    
    % Nested function to calculate change of velocity
    function deltaV = cal_change_of_velocity(v_vec_before, v_vec_after)
        deltaV = norm(v_vec_before - v_vec_after);
    end

    % Extracting properties from the objects
    %v1_velocity = v1_obj.speed;
    %v1_heading = v1_obj.speed_heading;
    %v1_mass = v1_obj.mass;
    
    %v2_velocity = v2_obj.speed;
    %v2_heading = v2_obj.speed_heading;
    %v2_mass = v2_obj.mass;

    % Calculate velocity vectors based on speed and heading
    v1_velocity_vec = [v1_velocity * cos(v1_heading), v1_velocity * sin(v1_heading)];
    v2_velocity_vec = [v2_velocity * cos(v2_heading), v2_velocity * sin(v2_heading)];
    %disp(v1_velocity_vec);
    %disp(v2_velocity_vec);
    % Conservation of momentum for a perfect inelastic collision
    original_momentum = v1_mass * v1_velocity_vec + v2_mass * v2_velocity_vec;
    %disp(original_momentum);
    velocity_vec_after_crash = original_momentum / (v1_mass + v2_mass);
    velocity_heading_after_crash = atan2(velocity_vec_after_crash(2), velocity_vec_after_crash(1));
    %disp(velocity_vec_after_crash);
    % Calculate delta V for each vehicle
    v1_deltaV = cal_change_of_velocity(v1_velocity_vec, velocity_vec_after_crash);
    v2_deltaV = cal_change_of_velocity(v2_velocity_vec, velocity_vec_after_crash);

    % Maximum delta V between the two vehicles
    deltaV = max(v1_deltaV*2.237, v2_deltaV*2.237);
end


function [lane_depart_time_list, lane_depart_time_MSF, lane_depart_time_PySR] = calculate_lane_depart_time(complexity)
    %filePath = sprintf('/home/perception/Jun/MODELINGATTACKTRAJECTORY/PySRData/%dlatdevMSFPySR.xlsx', complexity);
    %filePath = sprintf('/home/drivesim/Documents/SR/PySRData/%dlatdevMSFPySR.xlsx', complexity);
    filePath = '/home/drivesim/Documents/SR/DSRData/latdevMSFDSR.xlsx';
    data_latdev_MSF = readmatrix(filePath);

    %filePath = sprintf('/home/perception/Jun/MODELINGATTACKTRAJECTORY/PySRData/%dlatdevPySR.xlsx', complexity);
    %filePath = sprintf('/home/drivesim/Documents/SR/PySRData/%dlatdevPySR.xlsx', complexity);
    filePath = '/home/drivesim/Documents/SR/DSRData/latdevDSR.xlsx';
    data_latdev_PySR = readmatrix(filePath);
    %data_latdev_PySR = readmatrix('/home/perception/Jun/MODELINGATTACKTRAJECTORY/PySRData/latdevPySR.xlsx');

    [vehicle_len,vehicle_num] = size(data_latdev_MSF);
    disp(vehicle_len);
    disp(vehicle_num);

    latdev_threshold = 1.2; %determine the lane departure threshold
    lane_depart_time_list = []; %initialize the lane_departure_time_list

    lane_depart_time_MSF = [];
    lane_depart_time_PySR = [];

    for i = 1:vehicle_num
        % Example check

        % Initialize variables for the current vehicle
        single_vehicle_latdev_MSF = [];
        single_vehicle_latdev_PySR = [];

        % Gather data for the single vehicle
        for j = 1:vehicle_len
            if data_latdev_MSF(j, i) ~= 10000
                single_vehicle_latdev_MSF = [single_vehicle_latdev_MSF, data_latdev_MSF(j, i)];
                single_vehicle_latdev_PySR = [single_vehicle_latdev_PySR, data_latdev_PySR(j, i)];
            end
        end

        vehicle_lane_depart_time_MSF = 10000;
        vehicle_lane_depart_time_PySR = 10000;

        for j = 1:length(single_vehicle_latdev_MSF)
            if abs(single_vehicle_latdev_MSF(j))>latdev_threshold
                vehicle_lane_depart_time_MSF = j/10;
                break
            end
        end

        for j = 1:length(single_vehicle_latdev_PySR)
            if abs(single_vehicle_latdev_PySR(j))>latdev_threshold
                vehicle_lane_depart_time_PySR = j/10;
                break
            end
        end
        
        lane_depart_time_MSF = [lane_depart_time_MSF, vehicle_lane_depart_time_MSF];
        lane_depart_time_PySR = [lane_depart_time_PySR, vehicle_lane_depart_time_PySR];

        lane_depart_time_list = [lane_depart_time_list, abs(vehicle_lane_depart_time_PySR-vehicle_lane_depart_time_MSF)];


    end
end



% Load Excel file as a matrix
function [CRMatrix_sum, PETDistribution, DELTAVMatrix, DELTAVDistribution, InjuryLevelDistribution, PETNearCrashDistribution, ...
AllSceneDeltaVList, AllScenePETList, AllScenePETNearCrashList, PETNearCrashMatrixList] = ...
    calculate_CR_matrix_PySR(TrainingFlag, complexity)

    if TrainingFlag == 0  %MSF
        data_speed = readmatrix('/home/drivesim/Documents/SR/IDMData/speed.xlsx');
        data_accel = readmatrix('/home/drivesim/Documents/SR/IDMData/accel.xlsx');
        data_distance = readmatrix('/home/drivesim/Documents/SR/IDMData/distanceAtk.xlsx');
        data_latdev = readmatrix('/home/drivesim/Documents/SR/IDMData/latdev.xlsx');
    else  %PySR

        %filePath = sprintf('/home/drivesim/Documents/SR/PySRData/%dspeedPySR.xlsx', complexity);
        filePath = '/home/drivesim/Documents/SR/DSRData/speedDSR.xlsx';
        data_speed = readmatrix(filePath);
        %filePath = sprintf('/home/drivesim/Documents/SR/PySRData/%daccelPySR.xlsx', complexity);
        filePath = '/home/drivesim/Documents/SR/DSRData/accelDSR.xlsx';
        data_accel = readmatrix(filePath);
        %filePath = sprintf('/home/drivesim/Documents/SR/PySRData/%ddistanceAtkPySR.xlsx', complexity);
        filePath = '/home/drivesim/Documents/SR/DSRData/distanceAtkDSR.xlsx';
        data_distance = readmatrix(filePath);
        %filePath = sprintf('/home/drivesim/Documents/SR/PySRData/%dlatdevPySR.xlsx', complexity);
        filePath = '/home/drivesim/Documents/SR/DSRData/latdevDSR.xlsx';
        data_latdev = readmatrix(filePath);
        %data_speed = readmatrix('/home/perception/Jun/MODELINGATTACKTRAJECTORY/PySRData/speedPySR.xlsx');
        %data_accel = readmatrix('/home/perception/Jun/MODELINGATTACKTRAJECTORY/PySRData/accelPySR.xlsx');
        %data_distance = readmatrix('/home/perception/Jun/MODELINGATTACKTRAJECTORY/PySRData/distanceAtkPySR.xlsx');
        %data_latdev = readmatrix('/home/perception/Jun/MODELINGATTACKTRAJECTORY/PySRData/latdevPySR.xlsx');
    end

    [vehicle_len,vehicle_num] = size(data_speed);
    [vehicle_len_speed, vehicle_num_speed] = size(data_speed);
    [vehicle_len_accel, vehicle_num_accel] = size(data_accel);
    [vehicle_len_distance, vehicle_num_distance] = size(data_distance);
    [vehicle_len_latdev, vehicle_num_latdev] = size(data_latdev);
    vehicle_len = min([vehicle_len_speed,vehicle_len_accel,vehicle_len_distance,vehicle_len_latdev]);
    

    vehicle_num = vehicle_num_speed;  % Set vehicle_num based on speed data
    disp(vehicle_len);
    disp(vehicle_num);

    %generate follow_speed_diff_list and follow_distance_list
    follow_distance_list = 0:2:61;

    startRangeRate = -5;
    endRangeRate = 5;
    gap = 0.2;

    follow_speed_diff_list = [];
    current_value = startRangeRate;

    while current_value <= endRangeRate
        follow_speed_diff_list = [follow_speed_diff_list, round(current_value, 1)];
        current_value = current_value + gap;
    end

    follow_speed_diff_list = [follow_speed_diff_list, endRangeRate];

    % define the IDM follow vehicle reaction threshold
    latdev_threshold = 1.2;

    % define time gap
    delta_t = 0.1;

    % Initialize total crash num matrix
    
    CRMatrix = cell(length(follow_distance_list), length(follow_speed_diff_list));
    % added for PET
    PETMatrix = cell(vehicle_num,length(follow_distance_list),length(follow_speed_diff_list));  
    % add for delta_V
    DELTAVMatrix = cell(vehicle_num,length(follow_distance_list),length(follow_speed_diff_list)); 
    % add for injury level
    InjuryLevelMatrix = cell(vehicle_num, length(follow_distance_list), length(follow_speed_diff_list)); 
    % add the minimum distance to describe the near crash scenario
    MinDisMatrix = cell(vehicle_num, length(follow_distance_list), length(follow_speed_diff_list)); 
    % add PET near crash
    PETNearCrashMatrix = cell(vehicle_num,length(follow_distance_list), length(follow_speed_diff_list));
    PETNearCrashMatrixList = cell(vehicle_num, length(follow_distance_list), length(follow_speed_diff_list));
    
    % Start the timer
    tic;

    % Create a cell array to collect results from parfor
    temp_CRMatrix = cell(vehicle_num, length(follow_distance_list), length(follow_speed_diff_list));
    
    % Run the parfor loop
    parfor i = 1:vehicle_num
        % Example check

        % Initialize variables for the current vehicle
        single_vehicle_speed = [];
        single_vehicle_accel = [];
        single_vehicle_distance = [];
        single_vehicle_latdev = [];

        % Gather data for the single vehicle
        for j = 1:vehicle_len
            if data_speed(j, i) ~= 10000
                single_vehicle_speed = [single_vehicle_speed, data_speed(j, i)];
                single_vehicle_accel = [single_vehicle_accel, data_accel(j, i)];
                single_vehicle_distance = [single_vehicle_distance, data_distance(j, i)];
                single_vehicle_latdev = [single_vehicle_latdev, data_latdev(j, i)];
            end
        end
          
        % Temporary matrix for the current vehicle
        vehicle_CRMatrix = cell(length(follow_distance_list), length(follow_speed_diff_list));
        vehicle_PETMatrix = cell(length(follow_distance_list), length(follow_speed_diff_list));  % added for PET
        vehicle_DELTAVMatrix = cell(length(follow_distance_list),length(follow_speed_diff_list)); % added for delta v
        vehicle_PETMatrix_near_crash = cell(length(follow_distance_list), length(follow_speed_diff_list)); % PET near crash
        vehicle_PETList_near_crash = cell(length(follow_distance_list), length(follow_speed_diff_list));

        for m = 1:length(follow_distance_list)
            follow_dis = follow_distance_list(m);
            for n = 1:length(follow_speed_diff_list)
                follow_speed_diff = follow_speed_diff_list(n);

                % Initialize follow vehicle parameters
                single_follow_speed = [single_vehicle_speed(1) + follow_speed_diff];
                single_follow_distance = [single_vehicle_distance(1) - follow_dis];
                single_follow_accel = [0];

                % IDMModel parameters
                v0 = single_vehicle_speed(1) + follow_speed_diff;
                idm = IDMModel(v0, 1.5, 9, 4, 4, 7);

                % Simulate vehicle dynamics
                for j = 2:length(single_vehicle_speed)
                    v = single_follow_speed(j-1);
                    v_lead = 10000;
                    if abs(single_vehicle_latdev(j-1)) < latdev_threshold   % vehicle have not cross the lane boundary
                        s = 10000;
                        v_lead = 10000;
                    else
                        if single_vehicle_distance(j-1) > single_follow_distance(j-1)
                            s = single_vehicle_distance(j-1) - single_follow_distance(j-1);
                            v_lead = single_vehicle_speed(j-1);
                        else
                            s = 10000;
                            v_lead = 10000;
                        end
                    end

                    acceleration = max(idm.calculateAcceleration(v, v_lead, s),-9);
                    acceleration = min(acceleration,9);
                    new_follow_speed = single_follow_speed(j-1) + acceleration * delta_t;
                    new_follow_distance = single_follow_distance(j-1) + single_follow_speed(j-1) * delta_t + 0.5 * acceleration * delta_t^2;

                    % Update follow vehicle parameters
                    single_follow_accel = [single_follow_accel, acceleration];
                    single_follow_speed = [single_follow_speed, new_follow_speed];
                    single_follow_distance = [single_follow_distance, new_follow_distance];
                end

                % Determine the near crash scenario
                [distanceList ,near_crash_list, near_crash_flag] = cal_traj_distance(single_vehicle_distance, single_vehicle_latdev, single_follow_distance);
                 
                

                % Determine if crash happens
                [crashFlagList, crashIdx] = cal_traj_crash_time(single_vehicle_distance, single_vehicle_latdev, single_follow_distance);
                if crashIdx ~=10000
                    v1_velocity = single_vehicle_speed(crashIdx-1);
                    v2_velocity = single_follow_speed(crashIdx-1);
                    v1_heading = atan2(abs(single_vehicle_latdev(crashIdx)-single_vehicle_latdev(crashIdx-1)),abs(single_vehicle_distance(crashIdx)-single_vehicle_distance(crashIdx-1)));
                    v2_heading = atan2(0,abs(single_follow_distance(crashIdx-1)-single_follow_distance(crashIdx)));
                    v1_mass = 1;
                    v2_mass = 1;

                    deltaV = perfect_inelastic_crash_change_of_velocity(v1_velocity, v2_velocity, v1_heading, v2_heading, v1_mass, v2_mass); %added for delta V
                else
                    deltaV = 0;
                end
                
                PETList = cal_traj_TTC_LC(single_vehicle_distance, single_follow_distance, single_vehicle_speed, single_follow_speed, single_vehicle_latdev, crashIdx);


                PETNearCrashList = generate_PET_near_crash_list(PETList, near_crash_list);

                minPET = min(PETList);
                if near_crash_flag == 1
                    minPET_near_crash = minPET;
                   
                else
                    minPET_near_crash = 10000;
                end

                %minPET_near_crash = minPET*near_crash_flag;

                

                if sum(crashFlagList)~=0
                    singleCrashFlag = 1
                else
                    singleCrashFlag = 0
                end
                % Store result in temporary matrix for the vehicle
                vehicle_CRMatrix{m, n} = singleCrashFlag;
                vehicle_PETMatrix{m,n} = minPET;  % added for PET 
                vehicle_DELTAVMatrix{m,n} = deltaV; %added for delta V
                vehicle_PETMatrix_near_crash{m,n} = minPET_near_crash;
                vehicle_PETList_near_crash{m,n} = PETNearCrashList;

            end
        end

        % Store the temporary matrix in the cell array
        temp_CRMatrix(i, :, :) = vehicle_CRMatrix;
        PETMatrix(i, :, :) = vehicle_PETMatrix;    % added for PET
        DELTAVMatrix(i, :, :) = vehicle_DELTAVMatrix;
        PETNearCrashMatrix(i,:,:) = vehicle_PETMatrix_near_crash;
        PETNearCrashMatrixList(i,:,:) = vehicle_PETList_near_crash;
    end

    

    % Combine results from temp_CRMatrix into CRMatrix
    for m = 1:length(follow_distance_list)
        for n = 1:length(follow_speed_diff_list)
            CRMatrix{m, n} = [];
            for i = 1:vehicle_num
                CRMatrix{m, n} = [CRMatrix{m, n}, temp_CRMatrix{i, m, n}];
            end
        end
    end

    CRMatrix_sum = zeros(length(follow_distance_list), length(follow_speed_diff_list));
    for m = 1:length(follow_distance_list)
        for n = 1:length(follow_speed_diff_list)
            CRMatrix_sum(m,n) = sum(CRMatrix{m,n})/vehicle_num;
        end
    end
    
    PETDistribution = zeros(10,length(follow_distance_list),length(follow_speed_diff_list));
    PETNearCrashDistribution = zeros(10, length(follow_distance_list), length(follow_speed_diff_list));

    % added for PET
    AllScenePETList = [];
    AllScenePETNearCrashList = [];
    AllSceneDeltaVList = [];
    AllSceneInjuryLevelList = [];

    for m = 1:length(follow_distance_list)
        for n = 1:length(follow_speed_diff_list)
            PETList = cell2mat(PETMatrix(:,m,n));
            single_PET_Distribution = cal_traj_PET_distribution (PETList)/sum(cal_traj_PET_distribution (PETList));     
            PETDistribution(:,m,n) = single_PET_Distribution;
            

            PETNearCrashList = cell2mat(PETNearCrashMatrix(:,m,n));
            single_PET_Near_Crash_Distribution = cal_traj_PET_distribution (PETNearCrashList)/sum(cal_traj_PET_distribution(PETNearCrashList));
            PETNearCrashDistribution(:,m,n) = single_PET_Near_Crash_Distribution;

            for k = 1:length(PETList)
                AllScenePETList = [AllScenePETList, PETList(k)];
                if PETNearCrashList(k) ~=10000
                    AllScenePETNearCrashList = [AllScenePETNearCrashList, PETNearCrashList(k)];
                end
            end
        end
    end

    DELTAVDistribution = zeros(30, length(follow_distance_list),length(follow_speed_diff_list));
    InjuryLevelDistribution = zeros (4, length(follow_distance_list), length(follow_speed_diff_list));

    for m = 1:length(follow_distance_list)
        for n = 1:length(follow_speed_diff_list)
            delta_v_list = cell2mat(DELTAVMatrix(:,m,n));
            single_delta_v_distribution = cal_traj_DELTAV_distribution (delta_v_list)/sum(cal_traj_DELTAV_distribution(delta_v_list));   
            DELTAVDistribution(:,m,n) = single_delta_v_distribution;  

            single_injury_level_distribution = cal_traj_injury_level_distribution(delta_v_list)/sum(cal_traj_injury_level_distribution(delta_v_list));
            InjuryLevelDistribution(:,m,n) = single_injury_level_distribution;

            for k = 1:length(delta_v_list)
                AllSceneDeltaVList = [AllSceneDeltaVList, delta_v_list(k)]; 
            end
        end
    end


    % Create a heatmap
    %h = heatmap(CRMatrix_sum,'XLabel', 'Speed Difference', 'YLabel', 'Distance', 'Title', 'Crash Flag Heatmap');

    % Customize the heatmap
    %colormap(parula); % Options include 'parula'
    %h.ColorLimits = [0 1]; % Set the color limits (equivalent to vmin and vmax)

    % End the timer
    %elapsedTime = toc;

    % Display the elapsed time
    %fprintf('Elapsed time: %.2f seconds\n', elapsedTime);

end

[CRMatrix_sum_MSF, PETDistribution_MSF, DELTAVMatrix_MSF, DELTAVDistribution_MSF, InjuryLevelDistribution_MSF, PETNearCrashDistribution_MSF, ...
    AllSceneDeltaVList_MSF, AllScenePETList_MSF, AllScenePETNearCrashList_MSF, PETNearCrashMatrixList_MSF] = ...
    calculate_CR_matrix_PySR(0, 0);  %generate MSF matrix


%figure;
% Create a heatmap
%subplot(1, 3, 1);
%h = heatmap(CRMatrix_sum_MSF,'XLabel', 'Speed Difference', 'YLabel', 'Distance', 'Title', 'Crash Flag Heatmap MSF');

% Customize the heatmap
%colormap(parula); % Options include 'parula'
%h.ColorLimits = [0 1]; % Set the color limits (equivalent to vmin and vmax)


CRList=[];
LDTList=[];
HDList=[];
HDNearCrashList = [];
CrashSeverityList = [];
DeltaVList = [];
HDPETNearCrashList = [];
HDLaneDepartTimeList= [];

KLCrashSeverityList = [];
KLDeltaVList = [];
KLPETNearCrashList = [];
KLLaneDepartTimeList = [];
complexity_list =  readmatrix('/home/drivesim/Documents/SR/PySRData/ComplexityPySR.xlsx');

for i = 1:length(complexity_list)
    complexity = complexity_list(i)

    [lane_departure_time_list, lane_depart_time_list_MSF, lane_depart_time_list_PySR] = calculate_lane_depart_time(complexity);
    mean_lane_departure_time = mean(lane_departure_time_list);
    lane_depart_time_MSF_distribution = cal_lane_depart_time_distribution(lane_depart_time_list_MSF)/sum(cal_lane_depart_time_distribution(lane_depart_time_list_MSF));
    lane_depart_time_PySR_distribution = cal_lane_depart_time_distribution(lane_depart_time_list_PySR)/sum(cal_lane_depart_time_distribution(lane_depart_time_list_PySR));
    
    HD_lane_depart_time = hellinger_distance(lane_depart_time_MSF_distribution, lane_depart_time_PySR_distribution);
    HDLaneDepartTimeList = [HDLaneDepartTimeList, HD_lane_depart_time];
    %KL_lane_depart_time = KL_divergence(lane_depart_time_MSF_distribution, lane_depart_time_PySR_distribution);
    %KLLaneDepartTimeList = [KLLaneDepartTimeList, KL_lane_depart_time];
    

    LDTList = [LDTList, mean_lane_departure_time];

    [CRMatrix_sum_PySR,PETDistribution_PySR, DELTAVMatrix_PySR, DELTAVDistribution_PySR, InjuryLevelDistribution_PySR, PETNearCrashDistribution_PySR,...
        AllSceneDeltaVList_PySR, AllScenePETList_PySR, AllScenePETNearCrashList_PySR, PETNearCrashMatrixList_PySR]  = ...
        calculate_CR_matrix_PySR(1, complexity);
    % Create a heatmap
    %subplot(1, 3, 2);
    %h = heatmap(CRMatrix_sum_PySR,'XLabel', 'Speed Difference', 'YLabel', 'Distance', 'Title', 'Crash Flag Heatmap PySR');

    % Customize the heatmap
    %colormap(parula); % Options include 'parula'
    %h.ColorLimits = [0 1]; % Set the color limits (equivalent to vmin and vmax)

    %added for non-zero items
    weightMatrix = CRMatrix_sum_MSF ~= 0;  % Logical indexing: elements not equal to 0 are true (1)

    % Optionally, convert the logical matrix to double
    weightMatrix = double(weightMatrix);

    CRMatrix_sum_Diff = CRMatrix_sum_MSF-CRMatrix_sum_PySR;
    CRMatrix_sum_Diff = abs(CRMatrix_sum_Diff);

    mean_CR=sum(CRMatrix_sum_Diff(:))/sum(weightMatrix(:));
    CRList=[CRList,mean_CR];
   

    dimension = size(CRMatrix_sum_Diff);

    HDMatrix = zeros(dimension(1),dimension(2));
    HDMatrix_PET_Near_Crash = zeros(dimension(1), dimension(2));
    for m = 1:dimension(1)
        for n = 1:dimension(2)
            HD  = hellinger_distance(PETDistribution_MSF(:,m,n), PETDistribution_PySR(:,m,n));
            HDMatrix(m,n) = HD;

            HD_Near_Crash = hellinger_distance(PETNearCrashDistribution_MSF(:,m,n), PETNearCrashDistribution_PySR(:,m,n));
            if sum(PETNearCrashDistribution_MSF(:,m,n)) == 0
                HDMatrix_PET_Near_Crash(m,n) = 0;
            else
                HDMatrix_PET_Near_Crash(m,n) = HD_Near_Crash;
            end
        end
    end

    %using the weighted HD matrix
    HDMatrix_w=HDMatrix.*weightMatrix;
    mean_HD = sum(HDMatrix_w(:))/sum(weightMatrix(:));
    fprintf('LDT%d,CR%d,\n', mean_lane_departure_time, mean_CR);

    HDMatrix_Delta_V = zeros(dimension(1),dimension(2));
    HDMatrix_Injury_Level = zeros(dimension(1),dimension(2));
    for m = 1:dimension(1)
        for n = 1:dimension(2)
            HD_delta_v = hellinger_distance(DELTAVDistribution_MSF(:,m,n), DELTAVDistribution_PySR(:,m,n));
            HDMatrix_Delta_V(m,n)= HD_delta_v;

            HD_injury_level = hellinger_distance(InjuryLevelDistribution_MSF(:,m,n), InjuryLevelDistribution_PySR(:,m,n));
            HDMatrix_Injury_Level(m,n) = HD_injury_level;
        end
    end

    PETNearCrashMSF = [];
    for m = 1:dimension(1)
        for n= 1:dimension(2)
            for k = 1:length(PETNearCrashMatrixList_MSF)
                PETNearCrashMSF = [PETNearCrashMSF, PETNearCrashMatrixList_MSF(k)];
            end

        end
    end

    PETNearCrashPySR = [];
    for m = 1:dimension(1)
        for n= 1:dimension(2)
            for k = 1:length(PETNearCrashMatrixList_PySR)
                PETNearCrashMSF = [PETNearCrashMSF, PETNearCrashMatrixList_MSF(k)];
            end

        end
    end

    %Calculate PET distribution for all scenario and all near crash 
    %PETDistributionAllNearCrash_MSF = cal_traj_PET_distribution (PETNearCrashMSF)/sum(cal_traj_PET_distribution (PETNearCrashMSF));
    %PETDistributionAllNearCrash_PySR = cal_traj_PET_distribution (PETNearCrashPySR)/sum(cal_traj_PET_distribution (PETNearCrashPySR));
    %HDMatrix_PETAll = hellinger_distance(PETDistributionAllNearCrash_MSF, PETDistributionAllNearCrash_PySR);
    % calculate PET Distribution for all scenario with min PET
    AllScenePETDistribution_MSF = cal_traj_PET_distribution (AllScenePETList_MSF);
    AllScenePETDistribution_PySR = cal_traj_PET_distribution (AllScenePETList_PySR);
    HDMatrix_AllScenePET = hellinger_distance(AllScenePETDistribution_MSF, AllScenePETDistribution_PySR);
    AllScenePETNearCrashDistribution_MSF = cal_traj_PET_distribution (AllScenePETNearCrashList_MSF)/sum(cal_traj_PET_distribution (AllScenePETNearCrashList_MSF));
    AllScenePETNearCrashDistribution_PySR = cal_traj_PET_distribution (AllScenePETNearCrashList_PySR)/sum(cal_traj_PET_distribution (AllScenePETNearCrashList_PySR));
    HDMatrix_AllScenePETNearCrash = hellinger_distance(AllScenePETNearCrashDistribution_MSF, AllScenePETNearCrashDistribution_PySR);
    KL_AllScenePETNearCrash = KL_divergence(AllScenePETNearCrashDistribution_MSF, AllScenePETNearCrashDistribution_PySR);

    % calculate Delta V Distribution for all scenario
    AllSceneDeltaVDistribution_MSF = cal_traj_DELTAV_distribution (AllSceneDeltaVList_MSF)/sum(cal_traj_DELTAV_distribution (AllSceneDeltaVList_MSF));
    AllSceneDeltaVDistribution_PySR = cal_traj_DELTAV_distribution (AllSceneDeltaVList_PySR)/sum(cal_traj_DELTAV_distribution (AllSceneDeltaVList_PySR));
    HDMatrix_AllSceneDeltaV = hellinger_distance(AllSceneDeltaVDistribution_MSF, AllSceneDeltaVDistribution_PySR);
    KL_AllSceneDeltaV = KL_divergence(AllSceneDeltaVDistribution_MSF, AllSceneDeltaVDistribution_PySR);
    
    %calculate Injury level Distribution for all scenario
    AllSceneInjuryLevelDistribution_MSF = cal_traj_injury_level_distribution(AllSceneDeltaVList_MSF)/sum(cal_traj_injury_level_distribution(AllSceneDeltaVList_MSF));
    AllSceneInjuryLevelDistribution_PySR = cal_traj_injury_level_distribution(AllSceneDeltaVList_PySR)/sum(cal_traj_injury_level_distribution(AllSceneDeltaVList_PySR));
    HDMatrix_AllSceneInjuryLevel = hellinger_distance(AllSceneInjuryLevelDistribution_MSF, AllSceneInjuryLevelDistribution_PySR);
    KL_AllSceneInjuryLevel = KL_divergence(AllSceneInjuryLevelDistribution_MSF, AllSceneInjuryLevelDistribution_PySR);

    DeltaVList = [DeltaVList, HDMatrix_AllSceneDeltaV];
    CrashSeverityList =[CrashSeverityList, HDMatrix_AllSceneInjuryLevel];
    HDPETNearCrashList = [HDPETNearCrashList, HDMatrix_AllScenePETNearCrash];

    KLCrashSeverityList = [KLCrashSeverityList, KL_AllSceneInjuryLevel];
    KLDeltaVList = [KLDeltaVList, KL_AllSceneDeltaV];
    KLPETNearCrashList = [KLPETNearCrashList, KL_AllScenePETNearCrash];
    




    % Create a heatmap
    %subplot(1, 3, 3);
    %h = heatmap(HDMatrix_w,'XLabel', 'Speed Difference', 'YLabel', 'Distance', 'Title', 'HD Heatmap');

    % Customize the heatmap
    %colormap(parula); % Options include 'parula'
    %h.ColorLimits = [0 1]; % Set the color limits (equivalent to vmin and vmax)

    %mean_error = mean_CR + mean_HD;
end



