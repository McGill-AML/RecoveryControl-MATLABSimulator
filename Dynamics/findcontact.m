function [ Contact ] = findcontact(rotMat,state)
    %findcontact.m Returns contact information based on current state and
    % pole radius 

    global BUMP_RADII BUMP_POSNS BUMP_NORMS BUMP_TANGS poleRadius
    Contact = initcontact(0);

    for iBumper = 1:4
        % Transform bumper definitions to world frame
        bumperCenterWorld = rotMat'*BUMP_POSNS(:,iBumper) + state(7:9); % center of bumper
            if (sqrt(bumperCenterWorld(1)^2+bumperCenterWorld(2)^2) <= (poleRadius + BUMP_RADII(iBumper)))
            bumperNormalWorld = rotMat'*BUMP_NORMS(:,iBumper); % normal to bumper center
            bumperTangentWorld = rotMat'*BUMP_TANGS(:,iBumper); % in plane with bumper circle
            bumperCrossProduct = [bumperNormalWorld(2)*bumperTangentWorld(3) - bumperNormalWorld(3)*bumperTangentWorld(2); ...
                                  -(bumperNormalWorld(1)*bumperTangentWorld(3) - bumperNormalWorld(3)*bumperTangentWorld(1)); ...
                                  bumperNormalWorld(1)*bumperTangentWorld(2) - bumperNormalWorld(2)*bumperTangentWorld(1)]; 

            poleVertical = [0; 0; 1]; % fixed to world frame

            H = [(bumperNormalWorld(2)*poleVertical(3)-poleVertical(2)*bumperNormalWorld(3)) -(bumperNormalWorld(1)*poleVertical(3)-bumperNormalWorld(3)*poleVertical(1)) (bumperNormalWorld(1)*poleVertical(2)-poleVertical(1)*bumperNormalWorld(2))]; %cross(bumperNormalWorld,poleVertical)
            I = -[(bumperNormalWorld(2)*bumperCenterWorld(3)-bumperCenterWorld(2)*bumperNormalWorld(3)) -(bumperNormalWorld(1)*bumperCenterWorld(3)-bumperNormalWorld(3)*bumperCenterWorld(1)) (bumperNormalWorld(1)*bumperCenterWorld(2)-bumperCenterWorld(1)*bumperNormalWorld(2))]; %cross(bumperNormalWorld,-bumperCenterWorld)
            A = sqrt(H(1)^2+H(2)^2+H(3)^2);%norm(H);
            B = sqrt(I(1)^2+I(2)^2+I(3)^2);%norm(I);
            D = H(1)*I(1) +H(2)*I(2)+H(3)*I(3);%dot(H,I)
            E = -(poleVertical(1)*bumperCenterWorld(1) + poleVertical(2)*bumperCenterWorld(2) + poleVertical(3)*bumperCenterWorld(3)); %dot(poleVertical,-bumperCenterWorld);

            coeffs = [A^2, ...
                      2*E*A^2 + 2*D, ...
                      E^2*A^2 + 4*D*E + B^2 - (BUMP_RADII(iBumper))^2*A^4, ...
                      2*D*E^2 + 2*E*B^2 - 2*(BUMP_RADII(iBumper))^2*A^2*D, ...
                      B^2*E^2 - D^2*(BUMP_RADII(iBumper))^2];

            allSolutions = roots(coeffs);
            realSolutions = real(allSolutions(abs(imag(allSolutions)) < 1e-3));

            if ~isempty(realSolutions)
                distances = [];
                for iter = 1:length(realSolutions)
                    G = poleVertical*realSolutions(iter) - bumperCenterWorld;
                    J = bumperNormalWorld(1)*G(1) + bumperNormalWorld(2)*G(2) + bumperNormalWorld(3)*G(3);
                    K = [(bumperNormalWorld(2)*G(3)-G(2)*bumperNormalWorld(3)) -(bumperNormalWorld(1)*G(3)-bumperNormalWorld(3)*G(1)) (bumperNormalWorld(1)*G(2)-G(1)*bumperNormalWorld(2))];
                    distances(iter) = (J^2 + (sqrt(K(1)^2 + K(2)^2 + K(3)^2) - (BUMP_RADII(iBumper)))^2)/2;
                end  

                [~, minIndex] = min(distances);
                t = realSolutions(minIndex);

                G = poleVertical*t - bumperCenterWorld;

                J = bumperNormalWorld(1)*G(1) + bumperNormalWorld(2)*G(2) + bumperNormalWorld(3)*G(3);
                L = G - J*bumperNormalWorld;
                Contact.point.contactWorld(:,iBumper) = bumperCenterWorld + (BUMP_RADII(iBumper))*L/sqrt(L(1)^2+L(2)^2+L(3)^2);
                Contact.point.contactBody(:,iBumper) = rotMat*(Contact.point.contactWorld(:,iBumper) - state(7:9));

                % need to find minimum distance and the bumper frame point

                if norm([Contact.point.contactWorld(1,iBumper) Contact.point.contactWorld(2,iBumper)]) < poleRadius

                    Contact.defl(iBumper) = poleRadius - sqrt(Contact.point.contactWorld(1,iBumper)^2 + ...
                                                              Contact.point.contactWorld(2,iBumper)^2);
                    % horizontal normal from center of pole to contact point
                    poleContactNormal = [Contact.point.contactWorld(1:2,iBumper); 0] ...
                                        /sqrt(Contact.point.contactWorld(1,iBumper)^2 + ...
                                              Contact.point.contactWorld(2,iBumper)^2);
                    % Compute velocity of contact point in world frame
                    contactPointVelocityWorld = rotMat'*([state(1);state(2);state(3)] ...
                                                + [(state(5)*Contact.point.contactBody(3,iBumper) - state(6)*Contact.point.contactBody(2,iBumper)); ...
                                                   -(state(4)*Contact.point.contactBody(3,iBumper) -state(6)*Contact.point.contactBody(1,iBumper)); ...
                                                   state(4)*Contact.point.contactBody(2,iBumper) - state(5)*Contact.point.contactBody(1,iBumper)]);
                                                  %cross([state(4);state(5);state(6)],Contact.point.contactBody(:,iBumper)));
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
                end 
            else
                disp(iBumper);
            end
        end
    end
end

