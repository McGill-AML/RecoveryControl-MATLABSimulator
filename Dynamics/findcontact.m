function [ Contact ] = findcontact(rotMat,state)
    %findcontact.m Returns contact information based on current state and
    % pole radius 
    
    %   Author: Fiona Chui (gareth.dicker@mail.mcgill.ca
    %   Last Updated: April 17th, 2017
    %   Description: Pole contact geometry, revised from Fiona Chui's
    %   thesis

    global BUMP_RADII BUMP_POSNS BUMP_NORMS BUMP_TANGS poleRadius
    Contact = initcontact(0);

    for iBumper = 1:4

        % Transform bumper definitions to world frame
        bumperNormalWorld = rotMat'*BUMP_NORMS(:,iBumper); % normal to bumper circle
        bumperTangentWorld = rotMat'*BUMP_TANGS(:,iBumper); % in plane with bumper circle
        bumperCrossProduct = cross(bumperNormalWorld,bumperTangentWorld); % orthogonal vector in bumper plane
        
        % Center of bumper
        bumperCenterWorld = rotMat'*BUMP_POSNS(:,iBumper) + state(7:9); % center of bumper

        % Pole radial frame
        poleNormal = [bumperCenterWorld(1:2)/norm(bumperCenterWorld(1:2));0]; % assumes pole centered at origin
        poleVertical = [0; 0; 1]; % fixed to world frame
        poleTangent = cross(poleVertical,poleNormal)/norm(cross(poleVertical,poleNormal)); % RH rule
        
        % find point on bumper closest to origin along pole normal by
        % projecting pole normal onto bumper plane and scaling by bumper radius
        centerToContact = BUMP_RADII(iBumper)*(dot(poleNormal,bumperNormalWorld)*bumperNormalWorld - poleNormal);
        % project this point onto pole normal
        centerToProjection = dot(poleNormal,centerToContact)*poleNormal;

        Contact.point.contactWorld(:,iBumper) = bumperCenterWorld + centerToContact;
        horizProjection = bumperCenterWorld + centerToProjection;

        distanceToOrigin = dot(horizProjection,poleNormal); % horizontal distance from origin
        
        % if contact has occurred
        if(distanceToOrigin < poleRadius)
            Contact.defl(iBumper) = poleRadius - distanceToOrigin;
            
            % Transform contact point into quadrotor body frame
            Contact.point.contactBody(:,iBumper) = real(rotMat*(Contact.point.contactWorld(:,iBumper) - state(7:9)));
            % Compute velocity of contact point in world frame
            contactPointVelocityWorld = rotMat'*([state(1);state(2);state(3)] ...
                                        + cross([state(4);state(5);state(6)],Contact.point.contactBody(:,iBumper)));
            % log this value
            Contact.pointVelocityWorld(:,iBumper) = contactPointVelocityWorld;

            % compute deflection derivative value along pole normal
            Contact.deflDeriv(iBumper) = dot(contactPointVelocityWorld,poleNormal);
            
            % Project velocity onto pole tangent plane
            Contact.slidingVelocityWorld(:,iBumper) = contactPointVelocityWorld - Contact.deflDeriv(iBumper)*poleNormal;
            
            % Normalize to find vector of sliding direction
            tol = 0.1;
            
            if(norm(Contact.slidingVelocityWorld(:,iBumper))) > tol
                Contact.slidingDirectionWorld(:,iBumper) = Contact.slidingVelocityWorld(:,iBumper)...
                                                            /norm(Contact.slidingVelocityWorld(:,iBumper));
            else
                Contact.slidingDirectionWorld(:,iBumper) = zeros(size(Contact.slidingVelocityWorld(:,iBumper)));
            end
            
        end
    end
end

