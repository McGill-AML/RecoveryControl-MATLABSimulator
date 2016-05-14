function options = getOdeOptions()

    options = odeset('RelTol',1e-3,'AbsTol',[1e-5 1e-5 1e-5 1e-3 1e-3 1e-3 1e-3 1e-3 1e-3 1e-3 1e-3 1e-3 1e-3]); %Default: RelTol 1e-3, AbsTol 1e-6

end