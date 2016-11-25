function [Est_sensPars] = initEst_sensPars(sensParams)

Est_sensPars = sensParams;

 Est_sensPars.var_acc = sensParams.var_acc + [0.1; -0.01 ; 0.1];

 Est_sensPars.var_gyr =  sensParams.var_gyr + [0.0003; -0.0003 ; 0.0004];

 Est_sensPars.var_mag =  sensParams.var_mag + [0.00004; -0.00004 ; 0.00003];

 Est_sensPars.var_gps =  sensParams.var_gps + [0.00000001; -0.00000001 ; 0.00003; 0.00001; 0.00001]; %x, y, height, x-dot y-dot

 Est_sensPars.var_baro =  sensParams.var_baro;

    %% sensor bias variance 
 Est_sensPars.var_bias_acc =   sensParams.var_bias_acc;

  Est_sensPars.var_bias_gyr =  sensParams.var_bias_gyr;

  Est_sensPars.var_bias_mag =  sensParams.var_bias_mag;