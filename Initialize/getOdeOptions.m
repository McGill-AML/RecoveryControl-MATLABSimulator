function options = getOdeOptions()
    tight = 1e-5;
    loose = 1e-3;
    options = odeset('RelTol',loose,'AbsTol',...
        [tight tight tight loose loose loose loose loose loose loose loose loose loose],...
        'Stats','off'); %Default: RelTol 1e-3, AbsTol 1e-6

end