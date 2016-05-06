function Contact = initcontact(impactOccured)
    
    Contact.hasOccured = impactOccured;
    Contact.firstImpactOccured = 0;
    
    Contact.point.intersection = repmat([100;100;0],2,4);
    Contact.point.contactWorld = repmat([100;100;0],1,4);
    Contact.point.contactBody = repmat([0;0;0],1,4);
    Contact.point.intersectionAngle = zeros(2,4);
    Contact.defl = zeros(1,4);
    Contact.deflDeriv = zeros(1,4);
    Contact.normalForceMag = zeros(1,4);
    Contact.slidingVelocityWorld = zeros(3,4);
    Contact.slidingDirectionWorld = zeros(3,4);
    Contact.tangentialForceWorld = zeros(3,4);
    Contact.pointVelocityWorld = zeros(3,4);
    Contact.muSliding = 0;
end