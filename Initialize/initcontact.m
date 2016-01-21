function Contact = initcontact(impactOccured)
    
    Contact.hasOccured = impactOccured;
    
    Contact.point.intersection = repmat([100;100;0],2,4);
    Contact.point.contactWorld = repmat([100;100;0],1,4);
    Contact.point.contactBody = repmat([0;0;0],1,4);
    Contact.point.intersectionAngle = zeros(2,4);
    Contact.defl = zeros(1,4);
    Contact.deflDeriv = zeros(1,4);
    Contact.normalForceMag = zeros(1,4);
    
end