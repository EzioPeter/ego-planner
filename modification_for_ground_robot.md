# ego-planner_ackerman 版本修改说明
本文档详细说明 ego-planner_ackerman 版本相较于原版 ego-planner 的主要修改内容及其目的，主要是为了将原无人机路径规划算法适配到阿克曼底盘地面机器人上。

## 1. 状态机与规划逻辑修改: ego_replan_fsm.cpp/h
### 1.1 相关的订阅话题与定时回调修改
除了上述核心函数修改外，阿克曼版本还进行了以下相关调整：

```
// 原版ego-planner
odom_sub_ = nh.subscribe("/odom_world", 1, &
EGOReplanFSM::odometryCallback, this);
exec_timer_ = nh.createTimer(ros::Duration(0.01), &
EGOReplanFSM::execFSMCallback, this);
safety_timer_ = nh.createTimer(ros::Duration(0.05), &
EGOReplanFSM::checkCollisionCallback, this);

// ego-planner_ackerman版本
odom_sub_ = nh.subscribe("/legOdom", 1, &
EGOReplanFSM::odometryCallback, this);
exec_timer_ = nh.createTimer(ros::Duration(0.5), &
EGOReplanFSM::execFSMCallback, this);
safety_timer_ = nh.createTimer(ros::Duration(2), &
EGOReplanFSM::checkCollisionCallback, this);
go_timer_ = nh.createTimer(ros::Duration(0.01), &
EGOReplanFSM::goFlagCallback, this);
goFlagPub_ = nh.advertise<std_msgs::Int16>("go_flag",
10);
```
修改说明 ：

- 里程计话题从 /odom_world 改为 /legOdom ，适配实际机器人系统
- 降低了状态机执行频率（从100Hz降至2Hz）和碰撞检测频率（从20Hz降至0.5Hz），减少计算负担
- 新增goFlag相关的定时器回调和发布器，用于控制机器人运动使能状态
### 1.2 新增控制标志发布
- 添加了 goFlag 发布功能，用于向底层控制器传递导航状态
```
void EGOReplanFSM::goFlagCallback(const ros::TimerEvent &e)
{
    goFlag_.data = have_target_;
    goFlagPub_.publish(goFlag_);
}
```
### 1.3 waypointCallback函数修改
在阿克曼版本中，waypointCallback函数有一处关键修改，用于适配地面机器人特性：

```
// 原版ego-planner (z轴设为1.0)
end_pt_ << msg->poses[0].pose.position.x, msg->poses
[0].pose.position.y, 1.0;

// ego-planner_ackerman版本 (z轴固定为0.0)
end_pt_ << msg->poses[0].pose.position.x, msg->poses
[0].pose.position.y, 0.0;
```
修改原因 ：将目标点的Z坐标固定为0.0，使规划系统适应阿克曼底盘只能在平面上运动的特性，移除了原本为无人机设计的高度维度规划。
### 1.4 odometryCallback函数修改
在阿克曼版本中，odometryCallback函数进行了两处重要修改：

```
// 原版ego-planner (使用真实z轴数据)
odom_pos_(2) = msg->pose.pose.position.z;
odom_vel_(2) = msg->twist.twist.linear.z;

// ego-planner_ackerman版本 (z轴固定为0.0)
// odom_pos_(2) = msg->pose.pose.position.z;
odom_pos_(2) = 0.0;
// odom_vel_(2) = msg->twist.twist.linear.z;
odom_vel_(2) = 0.0;
```
修改原因 ：

- 将机器人当前位置和速度的Z轴分量固定为0.0，确保规划系统始终在二维平面上进行计算
- 注释掉了原本读取真实Z轴数据的代码，避免高度变化对地面机器人规划造成干扰
### 1.5 EXEC_TRAJ状态切换逻辑修改
只保留欧式距离判断
```
    case EXEC_TRAJ:
    {
      cout << "Executing... "<< endl;
      Eigen::Vector3d pos = odom_pos_;

      if ((end_pt_ - pos).norm() < 0.5)
      {    
        cout << " near end position" << endl;
        cout << "### change state to WAIT_TARGET in '!!!EXEC_TRAJ'" << endl;
        have_target_ = false;
        changeFSMExecState(WAIT_TARGET, "FSM");
        return;
      }
      else
      {
        cout << "### change state to REPLAN_TRAJ in '!!!EXEC_TRAJ'" << endl;
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }
```
### 1.6 添加轨迹起始位置显式设置
- 在planFromCurrentTraj()和checkCollisionCallback()函数中添加 info->start_pos_ = odom_pos_ 确保轨迹从机器人实际位置开始
- 这提高了规划的准确性，尤其在机器人位置与规划轨迹有偏差时
### 1.7 里程计数据使用方式的改变
- 将规划起始状态计算方式从 B 样条轨迹插值改为直接使用里程计数据
- 在 planFromCurrentTraj() 函数中：
```
// 原版：使用 B 样条插值获取当前状态
// start_pt_ = info->position_traj_.
evaluateDeBoorT(t_cur);
// start_vel_ = info->velocity_traj_.
evaluateDeBoorT(t_cur);
// start_acc_ = info->acceleration_traj_.
evaluateDeBoorT(t_cur);

// 阿克曼版本：直接使用里程计数据
start_pt_ = odom_pos_;
start_vel_ = odom_vel_;
start_acc_.setZero();
```
- 在 callReboundReplan() 函数中也进行了类似修改

### 1.8 紧急停止机制调整
- 在 checkCollisionCallback() 中，当检测到障碍物时，注释掉了直接进入紧急停止状态的代码
- 改为优先尝试重新规划轨迹，只有在重新规划失败时才考虑紧急措施
```
  void EGOReplanFSM::checkCollisionCallback(const ros::TimerEvent &e)
  {
    LocalTrajData *info = &planner_manager_->local_data_;
    info->start_pos_ = odom_pos_;
    auto map = planner_manager_->grid_map_;

    if (exec_state_ == WAIT_TARGET || info->start_time_.toSec() < 1e-5)
      return;

    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    double t_cur = (ros::Time::now() - info->start_time_).toSec();
    double t_2_3 = info->duration_ * 2 / 3;
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
      if (t_cur < t_2_3 && t >= t_2_3) // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
        break;

      if (map->getInflateOccupancy(info->position_traj_.evaluateDeBoorT(t)))
      {
        if (planFromCurrentTraj()) // Make a chance
        {
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          return;
        }
        // else
        // {
        //   if (t - t_cur < emergency_time_) // 0.8s of emergency time
        //   {
        //     ROS_WARN("Suddenly discovered obstacles. emergency stop! time=%f", t - t_cur);
        //     changeFSMExecState(EMERGENCY_STOP, "SAFETY");
        //   }
        //   else
        //   {
        //     //ROS_WARN("current traj in collision, replan.");
        //     changeFSMExecState(REPLAN_TRAJ, "SAFETY");
        //   }
        //   return;
        // }
        break;
      }
    }
  }
```
   <!-- - 这可能是为了减少地面机器人不必要的急停，提高运行稳定性 -->
### 1.9 话题发布调整
- 里程计发布话题从原版改为 /legOdom ，适配阿克曼底盘的里程计发布
### 1.10 给ego_replan_fsm.h的class EGOReplanFSM增添属性与头文件
```
#include <std_msgs/Int16.h>
...
  class EGOReplanFSM
  {
  private:
    ...
    std_msgs::Int16 goFlag_;
    ros::Publisher goFlagPub_;
    ...
    ros::Timer exec_timer_, safety_timer_, go_timer_;
    ...
    void goFlagCallback(const ros::TimerEvent &e);
    ......
  }
```


## 2. advanced_param.xml参数调整
### 2.1 重规划触发阈值 (thresh_replan)
- 原版：1.5
- 阿克曼版：0.3
- 原因 ：降低触发阈值以适应阿克曼底盘较低的运动速度，提高轨迹跟踪精度，避免机器人偏离轨迹过远
### 2.2 不重规划阈值 (thresh_no_replan)
- 原版：2.0
- 阿克曼版：0.1
- 原因 ：大幅减小不重规划的范围，使系统对轨迹偏差更敏感，确保地面机器人运动稳定性
### 2.3 规划时间范围 (planning_horizen_time)
- 原版：3秒
- 阿克曼版：2秒
- 原因 ：缩短前瞻规划时间，适应地面机器人较慢的运动速度，提高规划实时性和响应速度

## 3. 环境地图与感知修改: grid_map.cpp/h
### 3.1 雷达位姿变换矩阵调整   
- 雷达到机体的变换矩阵从原版的旋转矩阵改为单位矩阵
```
// grid_map.cpp

// 原版的旋转矩阵（无人机视角）
// md_.cam2body_ << 0.0, 0.0, 1.0, 0.0,
//     -1.0, 0.0, 0.0, 0.0,
//     0.0, -1.0, 0.0, -0.02,
//     0.0, 0.0, 0.0, 1.0;

// 阿克曼版本的单位矩阵（地面机器人视角）
md_.cam2body_ << 1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0;

// grid_map.h
  Eigen::Matrix3d camera_r_m_, last_camera_r_m_;
```
### 3.2 话题订阅调整
- 修改里程计订阅话题
- 修改点云订阅话题
```
odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(node_, "/legOdom", 100));

indep_cloud_sub_ = node_.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 10, &GridMap::cloudCallback, this);
indep_odom_sub_ = node_.subscribe<nav_msgs::Odometry>("/legOdom", 10, &GridMap::odomCallback, this);
```
### 3.3 调整深度图像投影处理逻辑
- 取消深度图像投影，减小计算量
```
// 原版
void GridMap::updateOccupancyCallback(const ros::TimerEvent & /*event*/)
{
  if (!md_.occ_need_update_)
    return;

  /* update occupancy */
  // ros::Time t1, t2, t3, t4;
  // t1 = ros::Time::now();

  projectDepthImage();
  // t2 = ros::Time::now();
  raycastProcess();
  
  ......
}

// 阿克曼版本
void GridMap::updateOccupancyCallback(const ros::TimerEvent & /*event*/)
{
  if (!md_.occ_need_update_)
    return;

  /* update occupancy */
  // ros::Time t1, t2, t3, t4;
  // t1 = ros::Time::now();

  // projectDepthImage();
  // t2 = ros::Time::now();
  raycastProcess();
    
  ......
}
```
### 3.4 添加雷达到base的变换矩阵相关计算
```
//odomCallback函数
void GridMap::odomCallback(const nav_msgs::OdometryConstPtr &odom)
{
  if (md_.has_first_depth_)
    return;

  // md_.camera_pos_(0) = odom->pose.pose.position.x;
  // md_.camera_pos_(1) = odom->pose.pose.position.y;
  // md_.camera_pos_(2) = odom->pose.pose.position.z;

  /* get pose */
  Eigen::Quaterniond body_q = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                 odom->pose.pose.orientation.x,
                                                 odom->pose.pose.orientation.y,
                                                 odom->pose.pose.orientation.z);

  Eigen::Matrix3d body_r_m = body_q.toRotationMatrix();
  Eigen::Matrix4d body2world;
  body2world.block<3, 3>(0, 0) = body_r_m;
  body2world(0, 3) = odom->pose.pose.position.x;
  body2world(1, 3) = odom->pose.pose.position.y;
  body2world(2, 3) = odom->pose.pose.position.z;
  body2world(3, 3) = 1.0;

  Eigen::Matrix4d cam_T = body2world * md_.cam2body_;
  md_.camera_pos_(0) = cam_T(0, 3);
  md_.camera_pos_(1) = cam_T(1, 3);
  md_.camera_pos_(2) = cam_T(2, 3);
  md_.camera_r_m_ = cam_T.block<3, 3>(0, 0);


  md_.has_odom_ = true;
  md_.local_updated_ = true;
}

  //cloudCallback函数
  for (size_t i = 0; i < latest_cloud.points.size(); ++i)
  {
    // pt = latest_cloud.points[i];
    // p3d(0) = pt.x, p3d(1) = pt.y, p3d(2) = pt.z;
    Eigen::Vector3d pt_raw;
    pt_raw(0) = latest_cloud.points[i].x;
    pt_raw(1) = latest_cloud.points[i].y;
    pt_raw(2) = latest_cloud.points[i].z;

    p3d = md_.camera_r_m_ * pt_raw + md_.camera_pos_;
    pt.x = p3d(0);
    pt.y = p3d(1);
    pt.z = p3d(2);

    ......
  }
```
### 3.5 点云回调函数处理
- 清空点云防止保证每次都是最新
```
void GridMap::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &img)
{
  md_.occupancy_buffer_ .clear();
  md_.occupancy_buffer_inflate_.clear();

  md_.count_hit_and_miss_.clear();
  md_.count_hit_.clear();
  md_.flag_rayend_.clear();
  md_.flag_traverse_.clear();

  pcl::PointCloud<pcl::PointXYZ> latest_cloud;
  pcl::fromROSMsg(*img, latest_cloud);

  ......
}
```

## 4. 路径搜索算法调整
### 4.1 dyn_a_star.cpp
- 保留了 A* 搜索的核心逻辑，但令dz=0
```
for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++)
                // for (int dz = -1; dz <= 1; dz++)
                {
                    int dz = 0;
                    if (dx == 0 && dy == 0 && dz == 0)
                        continue;

                    Vector3i neighborIdx;
                    
                    ......
                }
```
## 5. 路径生成方法: planner_manager.cpp
### 5.1 对reboundReplan 函数 if-else 修改进行分析：

- 原版代码 : 包含完整的 if-else 结构，根据条件选择不同的轨迹初始化策略：

```
if (flag_first_call || flag_polyInit || 
flag_force_polynomial) {
  // 使用多项式轨迹生成新路径
  // ...
}
else {
  // 基于上一条轨迹生成新路径
  // ...
}
```
阿克曼版本 : 将整个 if-else 结构注释掉，但保留并无条件执行原 if 分支内的多项式轨迹生成逻辑，无条件使用多项式插值方法可以确保每次生成的轨迹更加平滑

### 5.2 增加可视化调试
- 取消可视化调试注释，第一行：优化失败时可视化当前轨迹控制点，便于调试问题。
第二行：优化成功后可视化优化轨迹控制点，便于观察轨迹质量。
```
      visualization_->displayOptimalList( ctrl_pts, vis_id );
    ......
    visualization_->displayOptimalList( ctrl_pts, vis_id );
```

## 6. launch文件参数修改: run_in_ackermann.launch
- map_size_x从40.0改为100.0 - 扩大地图X轴范围，为阿克曼底盘提供更大的平面移动空间。
- map_size_y从40.0改为100.0 - 扩大地图Y轴范围，适应地面移动机器人需要更大平面活动区域的特点。
- map_size_z从3.0改为1.0 - 减小地图Z轴范围，因为阿克曼底盘主要在平面上运动，无需较大的高度空间。
- odom_topic从/visual_slam/odom改为/legOdom - 适配腿部机器人的里程计数据来源。
- cloud_topic从/pcl_render_node/cloud改为/velodyne_points - 从渲染点云切换到激光雷达原始点云数据，提高感知精度。
- max_vel从2.0改为2.5 - 略微提高最大速度设置，适应特定阿克曼底盘的性能特性。
- max_acc从3.0大幅降至0.3 - 显著降低最大加速度限制，符合地面移动机器人较低的加速度物理特性。
- planning_horizon从7.5改为5 - 减小规划视野距离，更适合低速地面机器人的规划需求。
- 新增taskset命令行 - 设置RViz可视化进程运行在CPU核心6和7上，优化多线程性能和计算资源分配。
```
taskset -c 6,7 roslaunch ego_planner  rviz.launch 2>/dev/null
```
- 新增从launch文件启动rviz:
```
  <node name="rviz" pkg="rviz" type="rviz" args="-d $(find ego_planner)/launch/default.rviz" required="true" />
```

## 7. traj_server.cpp的修改
### 7.1 添加头文件
- 添加了很多用到的头文件
### 7.1 新增变量与数据结构
- 增加 pos_vel_pub 发布器用于输出速度命令到阿克曼底盘
- 添加 robot_yaw_world_ 、 odom_Pose 、 odom_vel 等变量存储机器人状态信息
- 新增 goFlag_ 标志位用于控制机器人启停状态
- 引入 control_point_state_pub_ 发布器和 control_point_state 用于可视化控制点状态
作用 ：扩展了原版仅用于发布位置命令的功能，增加了对阿克曼底盘所需的速度命令发布和状态监控能力。

### 7.2 简化偏航角计算逻辑
- 重写 calculate_yaw 函数，不再基于轨迹预测计算偏航角
- 改为直接根据控制点和机器人当前姿态计算偏航角差异
- 移除了原版中的复杂约束和低通滤波逻辑
作用 ：简化了偏航角计算，更适合阿克曼底盘的2D平面运动控制需求。

### 7.3 增加运动控制逻辑
- 在 cmdCallback 中添加 goFlag_ 判断，当标志为0时发送停止命令
- 硬编码设置时间为1.3秒
作用 ：实现了机器人的启停控制功能，增强了系统安全性。

### 7.4 新增坐标转换和控制点位姿发布
- 添加了从世界坐标系到机器人本体坐标系的速度转换
- 实现了基于轨迹速度的偏航角估计和四元数计算
- 发布控制点状态用于可视化和监控
- 添加了 LimitSpeed 函数用于速度限制，但是暂未使用
作用 ：适配阿克曼底盘的速度控制需求，实现了坐标系转换和速度限制保护。

### 7.5 完善节点初始化
- 订阅 /legOdom 话题替代原版未指定的里程计来源
- 添加 goFlagSub 订阅器接收启停控制信号
- 创建 /cmd_vel 发布器输出速度命令
- 降低定时器频率从0.01秒到0.1秒
作用 ：适配阿克曼底盘的传感器输入和控制输出需求，降低计算频率提高系统稳定性。

## 8. 其他修改
### 8.1 polynomial_traj.h
- int类可能会导致精度计算问题
```
\\ 原版
      for (int i = 3; i < order; i += 1)
        for (int j = 3; j < order; j += 1)
        {
          mat_jerk(i, j) =
              i * (i - 1) * (i - 2) * j * (j - 1) * (j - 2) * pow(ts, i + j - 5) / (i + j - 5);
        }
\\ 阿克曼版
      for (double i = 3; i < order; i += 1)
        for (double j = 3; j < order; j += 1)
        {
          mat_jerk(i, j) =
              i * (i - 1) * (i - 2) * j * (j - 1) * (j - 2) * pow(ts, i + j - 5) / (i + j - 5);
        }
```
### 8.2 default.rviz
- 修改了很多话题订阅，可视化改了重新save就行