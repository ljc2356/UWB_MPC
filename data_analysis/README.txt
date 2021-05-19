一、主要可执行脚本：
（1）联邦自适应扩展卡尔曼滤波定位
AEKF_new.m           使用LOS+MPC+IMU
AEKF_new_noneIMU.m   使用LOS+MPC
（2） EKF 两段更新滤波器
EKF_move.m           使用LOS+MPC
（3）画图
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

    同时在 init_state = [3        -1    0 0 0 0 4 0]  更改一下初值的坐标，以期在运算初期就获得优秀的融合结果 
                       x坐标     y坐标

（2）EKF 两段融合定位
    更改EKF_move.m 中的文件路径 并执行脚本
    同时在 m_result(index,1).m(1,:) = [3     -1 0 0.5 4 0]; 更改一下初值坐标，以期在运算初期就获得优秀的融合结果 
                                      x坐标 y坐标
    
    注：EKF算法对噪声协方差矩阵的设置非常敏感。本代码中对于噪声协方差大小的改变 会改变定位结果
 （3）各个数据初值坐标
    a. ./data/20210413_indoor/
                        x坐标 y坐标
        正方形 八字形 为 3     -1
        三角形 菱形   为 4     -1
    b. ./data/20210407_indoor/                               
                        x坐标  y坐标
                 L字型   3      -2
                 方 型   3      -2
（4）画图
    可选择附加执行 Draw*.m系列 查看定位结果


    
    
   
