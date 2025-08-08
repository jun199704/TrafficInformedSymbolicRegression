clc;
clear;

import java.net.ServerSocket
import java.net.Socket
import java.io.*


% define safety metrics related functions
function distance = cal_distance_two_points(posX0, posY0, posX1, posY1)
    distance = sqrt((posX0 - posX1)^2 + (posY0 - posY1)^2);
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

function  crashFlagList= cal_traj_crash_time(distanceAtk, latdev, followPosition)
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

function PETList = cal_traj_TTC_LC(distanceAtk, followPosition, speed, followSpeed, latdev)
    
    PETList=[];
    LW = 3;    
    for i = 1:(length(distanceAtk) - 1)
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

        
    end
    PETList = [PETList, PET_LC];
end

function counts = cal_traj_PET_distribution (PETList)
    
    % Define the bin edges
    bin_edges = 0:0.25:5;

    % Calculate the frequency distribution
    [counts, bin_edges] = histcounts(PETList, bin_edges);

end

function H = hellinger_distance(P, Q)

    % Calculate Hellinger distance
    H = (1/sqrt(2)) * sqrt(sum((sqrt(P) - sqrt(Q)).^2));
end

% Load Excel file as a matrix
function [CRMatrix_sum, PETDistribution, feasibleFlag] = calculate_CR_matrix(TrainingFlag)

    if TrainingFlag == 0  %MSF
        data_speed = readmatrix('/home/drivesim/Documents/SR/IDMData/speed.xlsx');
        data_accel = readmatrix('/home/drivesim/Documents/SR/IDMData/accel.xlsx');
        data_distance = readmatrix('/home/drivesim/Documents/SR/IDMData/distanceAtk.xlsx');
        data_latdev = readmatrix('/home/drivesim/Documents/SR/IDMData/latdev.xlsx');
    else  %PySR
        data_speed = readmatrix('/home/drivesim/Documents/SR/MSFTrainData/speed37PySR.csv');
        data_accel = readmatrix('/home/drivesim/Documents/SR/MSFTrainData/accel37PySR.csv');
        data_distance = readmatrix('/home/drivesim/Documents/SR/MSFTrainData/distanceAtk37PySR.csv');
        data_latdev = readmatrix('/home/drivesim/Documents/SR/MSFTrainData/latdev37PySR.csv');
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

    if vehicle_len > 10
        feasibleFlag = 1;

        %generate follow_speed_diff_list and follow_distance_list
        follow_distance_list = 0:4:41;

        startRangeRate = -1;
        endRangeRate = 5;
        gap = 0.4;

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
        PETMatrix = cell(vehicle_num,length(follow_distance_list),length(follow_speed_diff_list));  % added for PET

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

                    % Determine if crash happens
                    crashFlagList = cal_traj_crash_time(single_vehicle_distance, single_vehicle_latdev, single_follow_distance);
                    % comment PET
                    %PETList = cal_traj_TTC_LC(single_vehicle_distance, single_follow_distance, single_vehicle_speed, single_follow_speed, single_vehicle_latdev);
                    %minPET = min(PETList);

                    if sum(crashFlagList)~=0
                        singleCrashFlag = 1
                    else
                        singleCrashFlag = 0
                    end
                    % Store result in temporary matrix for the vehicle
                    vehicle_CRMatrix{m, n} = singleCrashFlag;
                    % comment PET
                    %vehicle_PETMatrix{m,n} = minPET;  % added for PET
                end
            end

            % Store the temporary matrix in the cell array
            temp_CRMatrix(i, :, :) = vehicle_CRMatrix;
            % comment PET
            %PETMatrix(i, :, :) = vehicle_PETMatrix;    % added for PET
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
        
        PETDistribution = 0;
        %PETDistribution = zeros(20,length(follow_distance_list),length(follow_speed_diff_list));
        % added for PET
        %for m = 1:length(follow_distance_list)
        %    for n = 1:length(follow_speed_diff_list)
        %        PETList = cell2mat(PETMatrix(:,m,n));
        %        single_PET_Distribution = cal_traj_PET_distribution (PETList)/sum(cal_traj_PET_distribution (PETList));
        %        PETDistribution(:,m,n) = single_PET_Distribution;
        %    end
        %end



        % Create a heatmap
        %h = heatmap(CRMatrix_sum,'XLabel', 'Speed Difference', 'YLabel', 'Distance', 'Title', 'Crash Flag Heatmap');

        % Customize the heatmap
        %colormap(parula); % Options include 'parula'
        %h.ColorLimits = [0 1]; % Set the color limits (equivalent to vmin and vmax)

        % End the timer
        %elapsedTime = toc;

        % Display the elapsed time
        %fprintf('Elapsed time: %.2f seconds\n', elapsedTime);
    else
        CRMatrix_sum = 0;
        PETDistribution = 0;
        feasibleFlag = 0;
    end

end

[CRMatrix_sum_MSF, PETDistribution_MSF, feasibleFlag_MSF] = calculate_CR_matrix(0);  %generate MSF matrix


udp_socket = udpport("LocalPort", 50036);


while true
    % Receive the data
    while udp_socket.BytesAvailable == 0
        pause(0.001);
    end

    data_received = read(udp_socket,udp_socket.BytesAvailable);

    % Check if data was received
    if ~isempty(data_received)
        tic;
        % Process the received data (assuming it's JSON)
        json_str = char(data_received');

        % Convert individual characters to a cell array of character vectors
        json_cell = cellstr(json_str);

        % Concatenate the characters into a single string
        json_chars = strjoin(json_cell, '');
        % Parse the JSON string
        parsed_data = jsondecode(json_chars);

        
        startLoadingFlag = parsed_data(1);
        
        if startLoadingFlag ==1
            [CRMatrix_sum_PySR,PETDistribution_PySR, feasibleFlag_PySR]  = calculate_CR_matrix(1);
        end

        if feasibleFlag_PySR ~= 0
            %added for non-zero items
            weightMatrix = CRMatrix_sum_MSF ~= 0;  % Logical indexing: elements not equal to 0 are true (1)

            % Optionally, convert the logical matrix to double
            weightMatrix = double(weightMatrix);

            CRMatrix_sum_Diff = CRMatrix_sum_MSF-CRMatrix_sum_PySR;
            CRMatrix_sum_Diff = abs(CRMatrix_sum_Diff);

            mean_CR=sum(CRMatrix_sum_Diff(:))/sum(weightMatrix(:));

            dimension = size(CRMatrix_sum_Diff);

            %HDMatrix = zeros(dimension(1),dimension(2));
            %for m = 1:dimension(1)
            %    for n = 1:dimension(2)
            %        HD  = hellinger_distance(PETDistribution_MSF(:,m,n), PETDistribution_PySR(:,m,n));
            %        HDMatrix(m,n) = HD;
            %    end
            %end

            %using the weighted HD matrix
            %HDMatrix_w=HDMatrix.*weightMatrix;
            %mean_HD = sum(HDMatrix_w(:))/sum(weightMatrix(:));
            %fprintf('HD%d,CR%d,\n', mean_HD, mean_CR);
            fprintf('CR%d,\n', mean_CR);
            mean_error = mean_CR ;
        else
            mean_error = 10000;
        end
    


        % Convert the variable to JSON string
        mean_error_json_str = jsonencode(mean_error);

        % Convert the JSON string to a byte array
        data_bytes = uint8(mean_error_json_str);

        % Send the data to the specified IP address and port
        write(udp_socket, data_bytes, "LocalHost", 50035);

        disp('Data sent successfully.');
        elapsedTime = toc;
        % Display the elapsed time
        fprintf('Elapsed time: %.2f seconds\n', elapsedTime);
        
        
    else
        % Wait for a short time before checking again
        pause(0.001); % Adjust the pause time as needed
    end
end
%catch ME
%    disp(ME.message);