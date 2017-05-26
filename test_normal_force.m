defl = [0.00187103236938478,0,0,0.00545900684490347];
deflDeriv = [0.0436171577502326,0,0,-0.445085864200488];
initialNormalVel = [-0.417514733154669;0;0;0.016230742384709];

% defl = [0.00195986091131513,0,0,0.00312793069540129];
% deflDeriv = [-0.0536074549924407,0,0,-0.142344322408057];
eContact = 0.9;
kContact = 372;
nContact = 0.66;
for i = 4
    lambdaContact = 6*(1-eContact)*kContact/(((2*eContact-1)^2+3)*initialNormalVel(i))  
    normalForceMag = kContact*defl(i)^nContact ...
                     + lambdaContact*defl(i)^nContact*deflDeriv(i)
    firstHalf =  kContact*defl(i)^nContact
    secondHalf = lambdaContact*defl(i)^nContact*deflDeriv(i)
end 
expected = [5.78859104559948,0,0,-42.0341471407039];