function [ Contact ] = findcontact(rotMat,state)
    %findcontact.m Returns contact information based on current state and
    % pole radius 

    global BUMP_RADII BUMP_POSNS BUMP_NORMS BUMP_TANGS poleRadius
    Contact = initcontact(0);

    for iBumper = 1:4
        % Transform bumper definitions to world frame
        bumperNormalWorld = rotMat'*BUMP_NORMS(:,iBumper); % normal to bumper circle
        bumperTangentWorld = rotMat'*BUMP_TANGS(:,iBumper); % in plane with bumper circle
        bumperCrossProduct = cross(bumperNormalWorld,bumperTangentWorld); % orthogonal vector in bumper plane
        bumperCenterWorld = rotMat'*BUMP_POSNS(:,iBumper) + state(7:9); % center of bumper

        % Pole radial frame
%         poleAxisToBumperCenterNormal = [bumperCenterWorld(1:2)/norm(bumperCenterWorld(1:2));0]; % assumes pole centered at origin
%         poleVertical = [0; 0; 1]; % fixed to world frame
%         poleTangent = cross(poleVertical,poleAxisToBumperCenterNormal)/norm(cross(poleVertical,poleAxisToBumperCenterNormal)); % RH rule

        coeffs = [(BUMP_RADII(iBumper)^2*bumperCrossProduct(2)*bumperTangentWorld(2)*2i + BUMP_RADII(iBumper)^2*bumperCrossProduct(1)*bumperTangentWorld(1)*2i - BUMP_RADII(iBumper)^2*bumperTangentWorld(2)^2 - BUMP_RADII(iBumper)^2*bumperTangentWorld(1)^2 + BUMP_RADII(iBumper)^2*bumperCrossProduct(2)^2 + BUMP_RADII(iBumper)^2*bumperCrossProduct(1)^2 )
                  (-4*bumperCenterWorld(2)*BUMP_RADII(iBumper)*bumperTangentWorld(2) - 4*bumperCenterWorld(1)*BUMP_RADII(iBumper)*bumperTangentWorld(1) + bumperCenterWorld(2)*BUMP_RADII(iBumper)*bumperCrossProduct(2)*4i + bumperCenterWorld(1)*BUMP_RADII(iBumper)*bumperCrossProduct(1)*4i)
                  (-2*BUMP_RADII(iBumper)^2*bumperTangentWorld(2)^2 - 2*BUMP_RADII(iBumper)^2*bumperTangentWorld(1)^2 - 2*BUMP_RADII(iBumper)^2*bumperCrossProduct(2)^2 - 2*BUMP_RADII(iBumper)^2*bumperCrossProduct(1)^2 + 4*poleRadius^2 - 4*bumperCenterWorld(2)^2 - 4*bumperCenterWorld(1)^2)
                  (-4*bumperCenterWorld(2)*BUMP_RADII(iBumper)*bumperTangentWorld(2) - 4*bumperCenterWorld(1)*BUMP_RADII(iBumper)*bumperTangentWorld(1) - bumperCenterWorld(2)*BUMP_RADII(iBumper)*bumperCrossProduct(2)*4i - bumperCenterWorld(1)*BUMP_RADII(iBumper)*bumperCrossProduct(1)*4i)
                  (-BUMP_RADII(iBumper)^2*bumperCrossProduct(2)*bumperTangentWorld(2)*2i - BUMP_RADII(iBumper)^2*bumperCrossProduct(1)*bumperTangentWorld(1)*2i - BUMP_RADII(iBumper)^2*bumperTangentWorld(2)^2 - BUMP_RADII(iBumper)^2*bumperTangentWorld(1)^2 + BUMP_RADII(iBumper)^2*bumperCrossProduct(2)^2 + BUMP_RADII(iBumper)^2*bumperCrossProduct(1)^2)];

        % Returns roots of quartic polynomial with up to 4 solutions
        soln = (roots(coeffs));

        beta = [];
        for iter=1:size(soln)
            if abs(imag(-log(soln(iter))*1i)) <= 1e-5
                beta = [beta; real(-log(soln(iter))*1i)];
            end
        end      

        if isempty(beta)
            if norm([bumperCenterWorld(1) bumperCenterWorld(2)]) < poleRadius
                error('The bumper center is inside the pole!');
            end
            %disp('no contact')
        elseif length(beta) == 2
            disp('2 points');
            ptsIntersection(1:3,iBumper) = BUMP_RADII(iBumper)*cos(beta(1))*bumperTangentWorld + BUMP_RADII(iBumper)*sin(beta(1))*bumperCrossProduct + bumperCenterWorld;
            ptsIntersection(4:6,iBumper) = BUMP_RADII(iBumper)*cos(beta(2))*bumperTangentWorld + BUMP_RADII(iBumper)*sin(beta(2))*bumperCrossProduct + bumperCenterWorld;
            
            contactAxisWorld = (ptsIntersection(1:3,iBumper)+ptsIntersection(4:6,iBumper))/2 - bumperCenterWorld; 
            contactAxisBody = rotMat*contactAxisWorld;
            contactAxisBody = contactAxisBody/norm(contactAxisBody);

            Contact.point.contactBody(:,iBumper) = BUMP_POSNS(:,iBumper) + BUMP_RADII(iBumper)*contactAxisBody;
            Contact.point.contactWorld(:,iBumper) = real(rotMat'*Contact.point.contactBody(:,iBumper) + state(7:9));
            
            % Find deflection
            Contact.defl(iBumper) = poleRadius - norm(Contact.point.contactWorld(1:2,iBumper));
            
            % horizontal normal from center of pole to contact point
            poleContactNormal = [Contact.point.contactWorld(1:2,iBumper); 0]/norm(Contact.point.contactWorld(1:2,iBumper));
           
            % Compute velocity of contact point in world frame
            contactPointVelocityWorld = rotMat'*([state(1);state(2);state(3)] ...
                                        + cross([state(4);state(5);state(6)],Contact.point.contactBody(:,iBumper)));
            % log this value
            Contact.pointVelocityWorld(:,iBumper) = contactPointVelocityWorld;

            % compute deflection derivative value along pole normal
            Contact.deflDeriv(iBumper) = dot(contactPointVelocityWorld,poleContactNormal);

            % Project velocity onto pole tangent plane
            Contact.slidingVelocityWorld(:,iBumper) = contactPointVelocityWorld - Contact.deflDeriv(iBumper)*poleContactNormal;

            % Normalize to find vector of sliding direction
            tol = 0.1;

            if(norm(Contact.slidingVelocityWorld(:,iBumper))) > tol
                Contact.slidingDirectionWorld(:,iBumper) = Contact.slidingVelocityWorld(:,iBumper)...
                                                            /norm(Contact.slidingVelocityWorld(:,iBumper));
            else
                Contact.slidingDirectionWorld(:,iBumper) = zeros(size(Contact.slidingVelocityWorld(:,iBumper)));
            end
        
        elseif length(beta) == 4 % if bumper is vertical into the pole
            disp('4 points');
            ptsIntersection(1:3,iBumper) = BUMP_RADII(iBumper)*cos(beta(1))*bumperTangentWorld + BUMP_RADII(iBumper)*sin(beta(1))*bumperCrossProduct + bumperCenterWorld;
            ptsIntersection(4:6,iBumper) = BUMP_RADII(iBumper)*cos(beta(2))*bumperTangentWorld + BUMP_RADII(iBumper)*sin(beta(2))*bumperCrossProduct + bumperCenterWorld;
            point3 = BUMP_RADII(iBumper)*cos(beta(3))*bumperTangentWorld + BUMP_RADII(iBumper)*sin(beta(3))*bumperCrossProduct + bumperCenterWorld;
            point4 = BUMP_RADII(iBumper)*cos(beta(4))*bumperTangentWorld + BUMP_RADII(iBumper)*sin(beta(4))*bumperCrossProduct + bumperCenterWorld;
            
            contactAxisWorld1 = (ptsIntersection(1:3,iBumper)+ptsIntersection(4:6,iBumper))/2 - bumperCenterWorld; 
            contactAxisWorld2 = (point3 + point4)/2 - bumperCenterWorld; 

            contactAxisBody1 = rotMat*contactAxisWorld1;
            contactAxisBody2 = rotMat*contactAxisWorld2;
            contactAxisBody1 = contactAxisBody1/norm(contactAxisBody1);
            contactAxisBody2 = contactAxisBody2/norm(contactAxisBody2);

            % this averages the two contact points to generate an imaginary point
            % within the radius of the bumper but not on the perimeter
            Contact.point.contactBody(:,iBumper) = BUMP_POSNS(:,iBumper) + (BUMP_RADII(iBumper)*contactAxisBody1 + BUMP_RADII(iBumper)*contactAxisBody2)/2;
            Contact.point.contactWorld(:,iBumper) = real(rotMat'*Contact.point.contactBody(:,iBumper) + state(7:9));
            
                        % Find deflection
            Contact.defl(iBumper) = poleRadius - norm(Contact.point.contactWorld(1:2,iBumper));
            
            % horizontal normal from center of pole to contact point
            poleContactNormal = [Contact.point.contactWorld(1:2,iBumper); 0]/norm(Contact.point.contactWorld(1:2,iBumper));
           
            % Compute velocity of contact point in world frame
            contactPointVelocityWorld = rotMat'*([state(1);state(2);state(3)] ...
                                        + cross([state(4);state(5);state(6)],Contact.point.contactBody(:,iBumper)));
            % log this value
            Contact.pointVelocityWorld(:,iBumper) = contactPointVelocityWorld;

            % compute deflection derivative value along pole normal
            Contact.deflDeriv(iBumper) = dot(contactPointVelocityWorld,poleContactNormal);

            % Project velocity onto pole tangent plane
            Contact.slidingVelocityWorld(:,iBumper) = contactPointVelocityWorld - Contact.deflDeriv(iBumper)*poleContactNormal;

            % Normalize to find vector of sliding direction
            tol = 0.1;

            if(norm(Contact.slidingVelocityWorld(:,iBumper))) > tol
                Contact.slidingDirectionWorld(:,iBumper) = Contact.slidingVelocityWorld(:,iBumper)...
                                                            /norm(Contact.slidingVelocityWorld(:,iBumper));
            else
                Contact.slidingDirectionWorld(:,iBumper) = zeros(size(Contact.slidingVelocityWorld(:,iBumper)));
            end
        else
            error('1 or 3 contact points have occurred - should not happen');
        end
    end
end

