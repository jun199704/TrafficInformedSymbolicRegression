classdef IDMModel
    properties
        v0     % Desired speed
        T      % Safe time headway
        a      % Maximum acceleration
        b      % Comfortable deceleration
        delta  % Acceleration exponent
        s0     % Minimum bumper-to-bumper distance
    end
    
    methods
        function obj = IDMModel(v0, T, a, b, delta, s0)
            % Constructor for the IDMModel class
            obj.v0 = v0;
            obj.T = T;
            obj.a = a;
            obj.b = b;
            obj.delta = delta;
            obj.s0 = s0;
        end
        
        function acceleration = calculateAcceleration(obj, v, v_lead, s)
            % Calculate desired acceleration
            % s: distance between ego vehicle and front vehicle
            % v_lead: speed of leading vehicle
            % v: speed of ego vehicle
            
            % Compute s_star
            s_star = obj.s0 + max(0, v * obj.T + (v * (v - v_lead)) / (2 * sqrt(obj.a * obj.b)));
            
            % Compute desired acceleration
            a_desired = obj.a * (1 - (v / obj.v0) ^ obj.delta - (s_star / s) ^ 2);
            
            % Adjust acceleration based on constraints (optional)
            % acceleration = max(-obj.b, min(a_desired, obj.a));
            acceleration = a_desired; % Adjust if needed
        end
    end
end