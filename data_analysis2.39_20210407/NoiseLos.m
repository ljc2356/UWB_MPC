global result;
real_index = 200;
antenna_num = 8;
index = antenna_num - 2;
mean_d = 0.4;
Sigma_d = 0.01;
mean_phi = 0.3;
Sigma_phi = 0.001;

num = useful_num - real_index +1 ;
result(index,1).los_d.data(real_index:useful_num,1) = result(index,1).los_d.data(real_index:useful_num,1)  + mvnrnd(mean_d,Sigma_d,num);
result(index,1).los_phi.data(real_index:useful_num,1) = result(index,1).los_phi.data(real_index:useful_num,1) + mvnrnd(mean_phi,Sigma_phi, num);

















