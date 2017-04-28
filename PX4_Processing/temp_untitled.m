for ii = 1:5
    mean_5(:,ii) = mean(rmseEUL.total.SPKF_norm_eul(:,[(ii*5-4):ii*5]),2);
end


mean(rmseEUL.total.SPKF_norm_eul(:,[1:13, 15:40, 42:58, 60:65, 67, 71:77]),2)


mean(rmseEUL.total.SPKF_eul(:,[2, 6, 9, 15, 16,  22,  30,  34, 45, 49, 54, 60, 73]),2)



diag(cov(rmseEUL.total.EKF_att_eul'))


mean(rmseEUL.crash.COMP_eul(:,[1:13, 15:40, 42:58, 61:65, 67, 71:77]),2)


mean(rmseEUL.total.EKF_att_eul,2)


[(ii*18-17):(ii*18-13),(ii*18-10),(ii*18-8),(ii*18-6),(ii*18-5),(ii*18-3):ii*18-2,ii*18]