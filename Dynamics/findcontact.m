function [ Contact ] = findcontact( rotMat,state,wallLoc)

%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
global BUMP_RADIUS PROP_POSNS BUMP_NORMS BUMP_TANGS

Contact = initcontact(0);
ptsIntersection = Contact.point.intersection;
anglesIntersection = Contact.point.intersectionAngle;

for iBumper = 1:4

    % Transform bumper definitions to world frame
    bumperNormalWorld = rotMat'*BUMP_NORMS(:,iBumper); %normal to bumper circle
    bumperTangentWorld = rotMat'*BUMP_TANGS(:,iBumper); %in plane with bumper circle
    bumperCenterWorld = rotMat'*PROP_POSNS(:,iBumper) + state(7:9);

    % Solve for intersection angle
    bumperCrossProduct = cross(bumperNormalWorld,bumperTangentWorld);

    anglesIntersection(1,iBumper) = -log((wallLoc - bumperCenterWorld(1) + (bumperCenterWorld(1)^2 - 2*bumperCenterWorld(1)*wallLoc - BUMP_RADIUS^2*bumperTangentWorld(1)^2 - BUMP_RADIUS^2*bumperCrossProduct(1)^2 + wallLoc^2)^(1/2))/(BUMP_RADIUS*(bumperTangentWorld(1) - bumperCrossProduct(1)*1i)))*1i;
    anglesIntersection(2,iBumper) = -log(-(bumperCenterWorld(1) - wallLoc + (bumperCenterWorld(1)^2 - 2*bumperCenterWorld(1)*wallLoc - BUMP_RADIUS^2*bumperTangentWorld(1)^2 - BUMP_RADIUS^2*bumperCrossProduct(1)^2 + wallLoc^2)^(1/2))/(BUMP_RADIUS*(bumperTangentWorld(1) - bumperCrossProduct(1)*1i)))*1i;

    if abs(imag(anglesIntersection(1,iBumper))) <= 1e-5
        anglesIntersection(1,iBumper) = real(anglesIntersection(1,iBumper));
        anglesIntersection(2,iBumper) = real(anglesIntersection(2,iBumper));

        % Calculate point of contact, deflection
        if anglesIntersection(1,iBumper) == anglesIntersection(2,iBumper) %1 pt of intersection
            Contact.defl(iBumper) = 0;

        else %2 pts of intersection

            % Find points of intersection
            ptsIntersection(1:3,iBumper) = BUMP_RADIUS*cos(anglesIntersection(1,iBumper))*bumperTangentWorld + BUMP_RADIUS*sin(anglesIntersection(1,iBumper))*bumperCrossProduct + bumperCenterWorld;
            ptsIntersection(4:6,iBumper) = BUMP_RADIUS*cos(anglesIntersection(2,iBumper))*bumperTangentWorld + BUMP_RADIUS*sin(anglesIntersection(2,iBumper))*bumperCrossProduct + bumperCenterWorld;

            % Find point of contact
            contactAxisWorld = (ptsIntersection(1:3,iBumper)+ptsIntersection(4:6,iBumper))/2 - bumperCenterWorld; %axis pc lies on
            contactAxisBody = rotMat*contactAxisWorld;
            contactAxisBody = contactAxisBody/norm(contactAxisBody);

            if ptsIntersection(1,iBumper) >= bumperCenterWorld(1)
                Contact.point.contactBody(:,iBumper) = PROP_POSNS(:,iBumper) + BUMP_RADIUS*contactAxisBody;
            else
                Contact.point.contactBody(:,iBumper) = PROP_POSNS(:,iBumper) - BUMP_RADIUS*contactAxisBody;
            end

            Contact.point.contactWorld(:,iBumper) = real(rotMat'*Contact.point.contactBody(:,iBumper) + state(7:9));

            % Find deflection
            Contact.defl(iBumper) = Contact.point.contactWorld(1,iBumper) - wallLoc;

            % Find deflection rate
            Contact.deflDeriv(iBumper) = rotMat(:,1)'*([state(1);state(2);state(3)] + cross([state(4);state(5);state(6)],Contact.point.contactBody(:,iBumper)));

            if Contact.defl(iBumper) <= 0     
                disp('Warning: Deflection calc error');
                disp(Contact.defl(iBumper));
                Contact.defl(iBumper) = 0;
                Contact.point.contactBody(:,iBumper) = [0;0;0];
                Contact.point.contactWorld(:,iBumper) = [100;100;0];
                Contact.deflDeriv(iBumper) = 0;

            end

        end %else no contact

    end
end
    
    Contact.point.intersection = ptsIntersection;
    Contact.point.intersectionAngle = anglesIntersection;

end

