%% Kinematic Model Class - GRAAL Lab
classdef cartesianControl < handle
    % KinematicModel contains an object of class GeometricModel
    % gm is a geometric model (see class geometricModel.m)
    properties
        gm % An instance of GeometricModel
        k_a
        k_l
    end

    methods
        % Constructor to initialize the geomModel property
        function self = cartesianControl(gm,angular_gain,linear_gain)
            if nargin > 2
                self.gm = gm;
                self.k_a = angular_gain;
                self.k_l = linear_gain;
            else
                error('Not enough input arguments (cartesianControl)')
            end
        end
        function [x_dot]=getCartesianReference(self,bTg)
            %% getCartesianReference function
            % Inputs :
            % bTg : goal frame
            % Outputs :
            % x_dot : cartesian reference for inverse kinematic control
            
            %calcolarsi w_g_0 e v_g_0

            %compute error
            T_b_t = self.gm.getToolTransformWrtBase();
            O_b_t =  T_b_t(1:3,4);
            O_b_g = bTg(1:3,4);

            tTg = inv(T_b_t) * bTg;

            [h, theta] = RotToAngleAxis(tTg(1:3,1:3));

            rho_tg = h*theta;
            r_tg = tTg(1:3,4);

            b_rho_tg = T_b_t(1:3,1:3) * rho_tg;
            b_r_tg = T_b_t(1:3,1:3) * r_tg;

            b_e_tg = [b_rho_tg; b_r_tg];
            

            %la trasformata tra t e g
            %prendo l'asse di rotazione h e theta con Rottoangleaxis

            % r = O_b_g - O_b_t; % Compute position error
            % 
            % %% errore sull'angolo test
            % bRt = T_b_t(1:3,1:3);
            % bRg = bTg(1:3,1:3);
            % Rerr = bRg * bRt'; % cambia l'ordine
            % [h, theta] = RotToAngleAxis(Rerr);
            % e = [h*theta; r];
            % disp("e")
            % disp(e);
            
            Ka = self.k_a * eye(3);
            Kl = self.k_l * eye(3);
            K = [Ka, zeros(3,3);
                 zeros(3,3), Kl];
            
            x_dot = K * b_e_tg; % Compute the control output

            
        end
    end
end

