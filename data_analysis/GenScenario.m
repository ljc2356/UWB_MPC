function [d_los,d_mpc,diff_d_mpc,diff_index_mpc,index_mpc,phi_mpc,phi_los] = GenScenario(h,x,y,baseAngle)

    d_los = norm([x,y]);
    d_mpc = norm([x, h + h - y]);
    diff_d_mpc = d_mpc - d_los;
    diff_index_mpc = diff_d_mpc/0.3;
    index_mpc = diff_index_mpc +8;
    
    phi_mpc = wrapToPi(atan2(h+h-y, x) + baseAngle);
    phi_index_mpc = (phi_mpc + pi)/0.01;
    phi_los = wrapToPi(atan2(y,x) + baseAngle);
end