% %% Kinematic Model Class - GRAAL Lab
% classdef kinematicModel < handle
%     % KinematicModel contains an object of class GeometricModel
%     % gm is a geometric model (see class geometricModel.m)
%     properties
%         gm % An instance of GeometricModel
%         J % Jacobian
%     end
% 
%     methods
%         % Constructor to initialize the geomModel property
%         function self = kinematicModel(gm)
%             if nargin > 0
%                 self.gm = gm;
%                 self.J = zeros(6, self.gm.jointNumber);
%             else
%                 error('Not enough input arguments (geometricModel)')
%             end
%         end
% 
%         function bJi = getJacobianOfJointWrtBase(self, i)
%             %% getJacobianOfJointWrtBase function
%             % This method computes the Jacobian matrix bJi of joint i wrt base.
%             % Inputs:
%             % i : joint indnex ;
% 
%             % The function returns:
%             % bJi
% 
%             %TO DO
%         end
% 
%         function updateJacobian(self)
%         %% updateJacobian function
%         % The function update:
%         % - J: end-effector jacobian matrix
%             % TO DO
% 
% 
%         end
%     end
% end
% 


%% Kinematic Model Class - GRAAL Lab
classdef kinematicModel < handle
    % KinematicModel contains an object of class GeometricModel
    % gm is a geometric model (see class geometricModel.m)
    properties
        gm % An instance of GeometricModel
        J % Jacobian
    end

    methods
        % Constructor to initialize the geomModel property
        function self = kinematicModel(gm)
            if nargin > 0
                self.gm = gm;
                self.J = zeros(6, self.gm.jointNumber);
            else
                error('Not enough input arguments (geometricModel)')
            end
        end

        function bJi = getJacobianOfLinkWrtBase(self, i)
            %%% getJacobianOfJointWrtBase
            % This method computes the Jacobian matrix bJi of joint i wrt base.
            % Inputs:
            % i : joint indnex ;

            % The function returns:
            % bJi

            bJi=zeros(6,length(self.gm.jointType));

            T_0_i = self.gm.getTransformWrtBase(i);
            for j=1:i
                T_0_l = self.gm.getTransformWrtBase(j);
               
                bJl = zeros(6,1); %final
                % Compute the Jacobian for joint i
                kl = T_0_l(1:3,3);
                r_il = T_0_i(1:3,4) - T_0_l(1:3,4);

                if(self.gm.jointType(j) == 0) %for rotatioinal joint
                    w = kl;
                    v = cross(kl, r_il);
                else                          % for prismatic joint
                    v = kl;
                    w = [0,0,0]';
                end
        
                bJl(1:3) = w;
                bJl(4:6) = v;
                bJi(:,j) = bJl;
            end



        end

        function updateJacobian(self)
        %% Update Jacobian function
        % The function update:
        % - J: end-effector jacobian matrix
            
            bJe = self.getJacobianOfLinkWrtBase(length(self.gm.jointType));

            bTe = self.gm.getTransformWrtBase(length(self.gm.jointType));

            bRe = bTe(1:3, 1:3);

            r_et = self.gm.eTt(1:3,4);

            b_r_et = bRe * r_et;

            b_r_et_skew = skew(b_r_et);


            B = [eye(3) , zeros(3,3)
                 b_r_et_skew' ,eye(3)];

            self.J = B * bJe;
            
            
        end
    end
end



