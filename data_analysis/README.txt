一、主要可执行脚本：
（1）联邦自适应扩展卡尔曼滤波定位
AEKF_new.m           使用LOS+MPC+IMU
AEKF_new_noneIMU.m   使用LOS+MPC
（2）画图
DrawOne.m            绘制动图
DrawOneFast.m        绘制定位结果
DrawTwoFast.m        绘制定位结果对比图

二、文件夹：
data                   存储测距测角输出文件，以及加速度计文件

三、操作过程
 (1) 联邦定位
    更改AEKF_new.m 或者 AEKF_new_noneIMU.m中文件路径 并 执行脚本 
    例:AEKF_new.m 中的
    load('data/20210413_indoor/Square.mat') ;
    load('data/Acc_data/shape/Square_Acc.mat'); 
    将Square.mat Square_Acc.mat 更改成文件夹中对应形状对应的文件名即可

    同时在 init_state = [3 -1 0 0 0 0 4 0]  更改一下初值，以期在运算初期就获得优秀的融合结果
    注：仅需要更改 第一个值 对应 x坐标
        正方形 八字形 为 3
        三角形 菱形   为 4
（2）画图
    可选择附加执行 Draw*.m系列 查看定位结果
 

    
    
   
