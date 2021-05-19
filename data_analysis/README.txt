一、主要可执行脚本：
（1）测距测角
main.m
（2）联邦自适应扩展卡尔曼滤波定位
AEKF_new.m           使用LOS+MPC+IMU
AEKF_new_noneIMU.m   使用LOS+MPC
（3）画图
DrawOne.m            绘制动图
DrawOneFast.m        绘制定位结果
DrawTwoFast.m        绘制定位结果对比图

二、文件夹：
data                   存储测距测角输出文件，以及加速度计文件
ml_data                存储原始.json数据文件
CaliProcess            校准脚本文件夹
8天线普通功率_操作系统  8天线超宽带系统操作系统

三、操作过程
（1）测量数据
    在配置好计算机环境后 打开dos 执行record_input.bat 中的命令，可修改存储文件名，以及每隔N条数目输出一次 此时N = 100
    操作系统的校准配置文件  ./8天线普通功率_操作系统/config/device/DBB31430FF48C24.json 这个文件对非常重要 随后会提到。
（2）校准数据
    a. 以基站中心为原点 一号天线与x轴重合 在基站（1，0) (0,1) (-1,0) (0,-1)四个方向 距离为一米处采集校准数据
    b. 运行./8天线普通功率_操作系统/FastReplay.bat 生成当前校准文件所校准的测量信息  结果保存在 ./8天线普通功率_操作系统/After  文件夹中 （注意修改FastReplay.bat 中文件夹路径）
    c. 将以上校准数据.json 存入 ./ml_data文件夹中
    d. 将上文提到的校准配置文件DBB31430FF48C24.json 复制入./CaliProcess中
    e. 根据自己文件路径调整./CaliProcess/static_calibrate.m 中文件夹以及文件名配置
    f. 在Cali_Process 文件夹中运行static_calibrate.m 输出校准文件配置
    g. 复制输出的结果写入DBB31430FF48C24.json 并将其拷贝进操作系统文件夹中

    重复运行b到g步 直到校准配置收敛（一般两到三次即可）
（3）采集数据 并 校准数据
    根据上述调整好的校准配置文件，运行FastReplay.bat 
    注意：此时运行FastReplay.bat中注释部分代码  注释掉上一行 ，这是为了将测量的结果取平均，获得更可靠的值。
    此时./8天线普通功率_操作系统/After 中即为校准好的数据.json
（4）测距测角
    将上述校准好的数据存储到./ml_data中
    按照调整好的校准配置文件的 phase_diff_cali 修改 Properties.m中 cali 数据中 wrapToPi()函数中的内容 （复制即可）
    更改期望测距测角的文件名称
    运行main.m文件 在main.m 末尾 将测距测角输出result存储到./data文件夹中
 (5) 联邦定位
    更改AEKF_new.m 或者 AEKF_new_noneIMU.m中文件路径 并 执行脚本 
    可选择附加执行 Draw*.m系列 查看定位结果

    
    
   
