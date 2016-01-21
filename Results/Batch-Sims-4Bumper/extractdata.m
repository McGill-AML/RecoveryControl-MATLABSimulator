

numSims = size(all_data_norecover,1);

for k = 1:numSims

    sim_recover(k) = sim;
    sim_recover(k).e = all_data_norecover(k,1);    
    sim_recover(k).Vc = all_data_norecover(k,2);    
    sim_recover(k).tilt = all_data_norecover(k,3);    
    sim_recover(k).defl_init = all_data_norecover(k,4);    
    sim_recover(k).defl_max = all_data_norecover(k,5);    
    sim_recover(k).stable = all_data_norecover(k,6);
    
    
end