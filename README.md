## 实验流程

此处部分步骤应该有配图，如果由需要的话可以日后补上。

1. 找到实验室的位置：去科技实验楼西楼二楼*附楼*，找到home组所在的实验室。具体而言，进入科技实验楼西楼之后向西侧走廊前进，上右侧第一个楼梯到二楼附楼即可。如果门没开，可以按门铃。
2. 打开MCS：在MCS控制电脑的左侧有一个连接到交换机的插线板，按下插线板的按钮就可以开启MCS。
3. 将实验平台上的遮挡物（如果有）取下，并将实验平台移动到中央区域（否则MCS不能识别标定点）。*注意*：需要将实验平台水平摆放，手臂的末端朝向MCS控制室的入口处。错误的摆放可能会导致控制算法出现问题。
4. 取走手臂下的支撑物。
5. 检查手臂状态：看一看手臂上的标定点有没有缺失，有的话需要重新将标定点粘上。
6. 打开手臂的控制电脑。如果没有电，请检查实验平台下方插线板的开关是否开启。控制电脑的密码是`softsoft`。
7. 打开气泵：将气泵的阀门（与导气管相连接）从与导气管垂直的位置旋转至与导气管平行的位置，并且将气泵开关（在气压计左侧）从与地面垂直的位置扳至与地面平行的位置。务必注意需要打开气泵开关，否则气压不够的话会导致手臂不能正常抬起。
8. 打开比例阀：与手臂连接的控制箱上有一个开关，打开它使得控制箱内的风扇持续旋转即可。如果没有电，请检查实验平台下方的插线板。*注意*:比例阀的开关开启的时候具有一定的*玄学因素*，如果按下之后开关弹回，请多按几次。
9. 如果一切顺利，做到这一步的时候，MCS系统已经完成了初始化工作。双击桌面上的.cal文件来打开Motive（MCS控制面板）。成功打开后MCS的灯光将会亮起。
10. 开启Motive之后，请在`Streaming Panel`内打开`broadcast`选项，来开启MCS的数据传送功能。如果摄像头的标定文件没有正确加载（这个时候摄像头会排列成一条直线），需要手工打开桌面上的`.cal`（`sway.cal`或者其它能用的标定文件，左上角-文件-打开-桌面-`sway.cal`）来完成摄像头标定文件的加载。
11. 调整MCS：左侧的panel内，将LED的亮度调到8左右，来保证标定点能够正确识别。效果仍然不理想的话，可以试着调调右侧panel内的其他选项。接下来需要手工标定刚体，方法是选择三个点（ctrl+鼠标左键，或者框选均可），然后右键-`create rigid body`。刚体的标定方向为从手臂尖端到手臂根部，最后一个刚体是控制箱上三个标定点所构成的刚体。如果使用MotionControl，需要将手臂的每一段末端都标记，否则只需要标记尖端和控制箱上的标定点。
12. 完成。在控制电脑上打开Matlab，运行主函数即可。

实验完成之后，需要按相反的步骤将实验平台放回原位。即：

1. 关比例阀；
2. 关气泵；
3. 关控制电脑；
4. 将支撑物放回手臂下方；
5. 将实验平台移回原位；
6. 关闭MCS。直接按开关即可，MCS的控制电脑不需要关闭。

## FeedBackControl控制接口

FeedBackControl控制接口已经写在主函数内，主函数如下：（在实验平台上，这个函数可能不叫这个名字。函数在`fuzhi`-`goto goal 3d 5seg`文件夹内，可以找找）

```matlab
function Main
    %打开比例阀控制器端口，否则pressure函数将不能正确运行
    delete(instrfind);
    global s;
    s = serial('COM3', 'baudrate', 9600);
    fopen( s );
	%将手臂放气归位，设置迭代次数，设置参数k
    pressure_for_16_05s( zeros(1, 16) );
    time = 30;
    %k_dpddis k_dpdph k_dpdtheta k_dpdx k_dpdy
    k_p=[0.02,0.0004,0.0004,0.02,0.02];
    %设置目标，每一行为一个目标点
    %x_dest y_dest z_dest ph_dest theta_dest  
    %ph为xz平面内的角度，theta为yz平面内的角度
    goal = [0,-160,350,0,0;
            0,-120,380,0,0;
            0,-60,390,0,0];
    %得到目标点的数量
    [point_num, ~] = size(goal);
	%初始气压为0
    p_now=zeros(1,16);
	%开启MCS数据传输接口
    mcs_initialize();
    for i = 1:point_num
    	%依次到达各个目标点
        [p_now,k_p] = goto_goal_3d_5seg( goal(i,:), time, p_now,k_p);
    end
	%关闭MCS数据传输接口
    mcs_clear();
    %放气使手臂归位
    pressure_for_16_05s( zeros(1, 16) );
```

一般而言，只需要改动`goal`内目标点的的参数，就可以完成不同的功能。`time`表示每一个标定点对应的迭代次数，这个值可以自行设置；另外，`goto_goal_3d_5seg`内有一个`MAXPVAR`参数，用于控制每一次气压的最大变化量，这个值也可以调节，但最好不要超过500。也可以把`goto_goal_3d_5seg`作为接口直接调用。

## FeedBackMotionControl控制接口

姿态控制的接口大同小异，如下：

```matlab
function Main

    %Open Controller
    delete( instrfind );
    global s;
    s = serial( 'COM3', 'baudrate', 9600 );
    fopen( s );

    %Definition of values
    global SEGNUM; %The number of segments
    global MAXP; %The MAXIMUM of Pressure
    global DEBUGFLAG; %1 for DEBUG 
    SEGNUM = 5;
    MAXP = 3000;
    DEBUGFLAG = 1;
    ERRORRANGE = 0; %If Error < ERRORRANGE then break, its scale is mm
    TIMES = 20; %Iteration time for one motion
    pNow = zeros( 1, 4*SEGNUM ); %Current pressure
    K = [0.009, 0.06, 0.002, 0.06]; %k_dpdx, k_dpddis, k_dpdy, k_dpddis

    load('data2.mat','relative_coordinate');
    motion = relative_coordinate;
    motionNum = size( motion, 1 ) / SEGNUM;
    
    Pressure( pNow );
  
    mcs_initialize( );
    for i = 1 : motionNum
        [pNow, K] = ReachMotion( motion(i*5-4:i*5,:), TIMES, pNow, K, ERRORRANGE );
    end
    mcs_clear( );
    
    pNow = zeros( 1, 5*SEGNUM );
    Pressure( pNow );

end
```

参数的定义已经在注释中说明，和上文的接口大同小异。这里唯一的不同是状态（即`motion`）的定义。这里，`motion`是一个`（n*SEGNUM）*3`的矩阵，每个`SEGNUM*3`的矩阵都描述了一个手臂当前的姿态，举例如下：

```matlab
motion = [   -2.1059   -3.2569   53.6323;
		    -9.8920  -17.0373  141.3948;
   			-17.8105  -50.3920  238.8652;
   			-34.8795  -78.1039  319.7628;
   			-52.5440  -94.9787  404.5254	];
```

每一行对应每一段手臂末端的坐标（从手臂根部到尖端），三列分别为`x`，`y`，`z`在手臂坐标系内的相对坐标。坐标系的定义和上文中相同，均遵照论文里的格式；长度单位为毫米，角度的单位为弧度。

## MCS控制接口

与MCS相关的接口除了`mcs_initialize()`和`mcs_clear()`之外主要有以下两个函数：

```matlab
 [body1, body2] = mcs_date_2_body();
 [stateone] = deal_data_from_mcs_2body( body1,body2)
```

在`MotionControl`中为以下的形式：

```matlab
[body1, body2, body3, body4, body5, body6] = GetBodyFromMCS( );%From tip to root
[currentState] = ComputeState( body1, body2, body3, body4, body5, body6 );
```

形式差不多，含义也都一样。第一个函数负责从MCS处拿取各个刚体的坐标并返回，第二个函数负责将拿取的坐标转换为在手臂根部坐标系中的相对坐标。注意，当坐标点的形式改变时，可能需要修改第二个函数内部的子函数，具体可以参见下文的教程。

## 比例阀控制接口

建议使用这个接口来控制比例阀：

```
[pNow] = Pressure( pNow );
```

pNow是一个`1*20`的矩阵，定义服从论文里的格式，可以参见图片`five_part_arm.pdf`：从手臂尖端到根部分别为1-20，4个气囊为一组。如果下层硬件改动，需要修改这个函数。

## 教程：实现更加复杂的功能

这一部分简单介绍如何利用上述的接口来完成一个更加复杂的功能：利用`FeedBackControl`接口，来实现手臂跟随物体的效果。

1. 首先，需要构建一个能够被MCS识别的物体。利用三个点在一个支架上粘成`等腰钝角三角形`即可。三角形是钝角还是锐角，取决于你调用了哪个`deal_body`函数。

2. 在MCS中标记这个物体。一般地，为了不影响程序结构，建议最后标记这个物体。比如在使用`FeedBackControl`的时候，这个刚体应该标记为3号，因为前面两个标号都已经被使用过了（手臂尖端和根部坐标系）。

3. 改动MCS控制接口，获取这个刚体的坐标。具体如下：

   1. 改动`mcs_date()`函数（或者是`GetBodyFromMCS( )`)。修改循环的范围（从2到3），并且在最后加上一行：

      ```matlab
      body3 = reshape(bodyR(3,:,:), 3, 3);
      ```

      调用这个函数的时候写成这样的形式：

      ```matlab
      [body1, body2, body3] = GetBodyFromMCS();
      ```

   2. 接下来，需要计算出这个刚体在手臂根部坐标系中的坐标，这个时候需要修改`ComputeState( )`函数。这个函数大概形式如下：

      ```matlab
      function [currentState] = ComputeState( body1, body2 )
        [mCenter, mX, mY, mZ] = deal_body_min( body2 ); %SetMainCoordinateSystem
        [center1, vx1, vy1, vz1] = deal_body( body1 ); %vz is ignored

        [coordinate1, ph1, theta1] = deal_tran( mCenter, mX, mY, mZ, center1, vz1 );   

        currentState = [1000 * coordinate1, ph1, theta1];
      ```

      需要在返回值里加上一个刚体，改成这样的形式：

      ```matlab
      function [currentState, goal] = ComputeState( body1, body2, body3 )
        [mCenter, mX, mY, mZ] = deal_body_min( body2 ); %SetMainCoordinateSystem
        [center1, vx1, vy1, vz1] = deal_body( body1 ); %vz is ignored
        [center2, vx2, vy2, vz2] = deal_body( body3 ); %vz is ignored
        center2 = center2 - vz*0.2;%注意，要将采集到目标的位置后移，因为手臂前段还有手爪，这一部分的长度要去除，否则手抓会撞上物体

        [coordinate1, ph1, theta1] = deal_tran( mCenter, mX, mY, mZ, center1, vz1 );
        [coordinate2, ph2, theta2] = deal_tran( mCenter, mX, mY, mZ, center2, vz2 ); 

        currentState = [1000 * coordinate1, ph1, theta1];
        goal = [1000 * coordinate2, ph2, theta2];
      ```

      这一部分比较繁琐，如果有兴趣的话可以尝试把它改成更优雅的形式。你调用了哪一个`deal_body`函数决定了你要将所识别的物体上的标定点粘成什么形式。

4. 修改主函数。具体来讲，就是把循环的部分改成

   ```matlab
   while(1)
     [body1, body2, body3] = GetBodyFromMCS();
     [~, goal] = ComputeState( body1, body2, body3 );
     %依次到达各个目标点
     [p_now,k_p] = goto_goal_3d_5seg( goal, time, p_now, k_p);
   end
   ```

   这样的形式，并令`time = 1`。至此修改完成，运行就可以获得结果。

## FAQ

1. 主函数里的参数k都是什么意思？

   答：具体参见论文。简单地将，某一个值对应的参数k越小，那么这个方向的变化效果就会越明显。比如说，把`k_dpddis`改小一些，那么手臂在到达目标点的过程中会优先伸长。

2. 怎么获得合适的姿态/目标点？

   答：手臂坐标系的原点在控制箱标定点三角形的中心（中线交点）上。坐标系的摆放参见图片，长度单位为毫米，角度为弧度。如果你有自信的话，可以自己尝试估计一下可行的位置并输入。或者，你也可以调用`Pressure`函数随便给手臂充气，然后调用

   ```matlab
   [body1, body2, body3] = GetBodyFromMCS();
   [state] = ComputeState( body1, body2, body3 );
   ```

   来获取手臂当前的状态，并且将这个状态作为手臂的目标状态输入，这样可以保证手臂的目标姿态总在手臂的可行域内。当然，也可以用其他方法来提供目标，上文的教程就是一个例子。


1. 手臂乱动的话怎么办？为什么会出现这种情况？

   答：一般来说，这种情况都是因为MCS追踪刚体的时候，出现了掉点的情况。MCS必须要至少识别到三个点才能确定一个刚体，因此若某一个标定点被遮挡，那么这个刚体的位置就不能够确定，`GetBodyFromMCS()`会返回没有意义的值。出现这种情况，请尝试远离手臂，撤去遮挡物体，从MCS控制面板里看一下摄像头的情况，推断一下什么原因导致了掉点；当然，这也可能只是偶然情况，你也可以什么都不做，把程序关了再开一次。手臂出现没有意义的运动时，按`ctrl+c`关闭程序，并且运行`Pressure(zeros(1,20))`来将手臂归位。

2. 为什么`mcs_initialize()`会初始化失败？

   答：请检查是否打开boardcast选项，控制机器是否连接到`AtWork`网络，以及本机IP和服务器IP是否和`mcs_initialize()`函数内的IP一致。可以在命令行运行`ipconfig`来查看局域网IP。

3. 为什么`GetBodyFromMCS()`会报错？

   答：运行这个函数之前，必须保证已经调用过`mcs_initialize()`函数。

4. 为什么调用`Pressure`函数没有反应？

   答：运行这个函数之前，必须打开比例阀控制端口。尝试在matlab命令行内运行以下代码：

   ```matlab
   delete(instrfind);
   global s;
   s = serial('COM3', 'baudrate', 9600);
   fopen( s );
   ```

5. 为什么matlab会崩溃？

   答：可能是在没有运行`mcs_clear()`函数的时候，多次调用了`mcs_initialize()`函数。请尽量避免这种情况；如果还是崩溃的话，我也很绝望，直接关了matlab重开一次吧。如果你有兴趣可以调查一下为什么会这样。

6. MCS控制电脑没有打开怎么办？

   答：直接开机即可。电脑密码为`zhimakaimen`。

## 还有什么工作？

1. 有兴趣的话，可以尝试重写一下MCS控制接口，把形式写得更优雅一些；
2. FeedBackControl（也就是末端控制的程序）只适用于目前的手臂，基本上不具有可移植性。如果有兴趣的话可以尝试参考`FeedBackMotionControl`内的接口重写这个部分，方法也可以加以改变，例如将中间段的标记点信息利用上。
3. `Pressure()`和`GetBodyFromMCS()`内还有一些pause没有去掉，可以尝试去掉之后看看结果。如果结果不理想的话，可以思考一下为什么会这样。
4. 以上只是目前程序可能还有的问题。如果你想到了什么其他问题的话，可以进行尝试。