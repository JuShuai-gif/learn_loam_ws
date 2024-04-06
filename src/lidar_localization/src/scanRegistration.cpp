#include <cmath>
#include <vector>
#include <string>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include "lidar_localization/tic_toc.h"
#include "lidar_localization/common.h"

using std::atan2;
using std::cos;
using std::sin;

const double scanPeriod = 0.1;// 激光雷达频率是10hz，0.1s

const int systemDelay = 0;
int systemInitCount = 0;
bool systemInited = false;
int N_SCANS = 16;                   // 雷达线数
float cloudCurvature[400000];       // 每个点曲率大小，数据大小应该能满足一帧点云
int cloudSortInd[400000];           // 存储按照点的曲率排好序的特征点ID
int cloudNeighborPicked[400000];    // 避免特征点密集
int cloudLabel[400000];             // 记录特征点属于那种类型：极大边线点、次极大边线点、极小平面点、次极小平面点

double MINIMUM_RANGE = 0.1;//雷达点距离原点的最近距离阈值


bool comp(int i,int j){
    return (cloudCurvature[i] < cloudCurvature[j]);
}

ros::Publisher pubLaserCloud;// 发布原始点云（经过无序-->有序的处理）
ros::Publisher pubCornerPointsSharp; // 发布极大边线点
ros::Publisher pubCornerPointsLessSharp;// 发布次极大边线点
ros::Publisher pubSurfPointsFlat;// 发布极小平面点
ros::Publisher pubSurfPointsLessFlat;// 发布次极小平面点
ros::Publisher pubRemovePoints;// 没用上
std::vector<ros::Publisher> pubEachScan;// 发布雷达的每条线扫

bool PUB_EACH_LINE = false;// 是否发布每条线扫

template <typename PointT>
void removeClosedPointCloud(const pcl::PointCloud<PointT> &cloud_in,
                                pcl::PointCloud<PointT>& cloud_out,float thres)
{
    // thres 是查找的范围
    // 输入输出点云不存在同一个变量中的情况
    if (&cloud_in != &cloud_out)
    {
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize(cloud_in.size());
    }

    size_t j = 0;
    // 将距离雷达原点比较近的点去除
    for (size_t i = 0; i < cloud_in.points.size(); ++i)
    {
        if (cloud_in.points[i].x * cloud_in.points[i].x + cloud_in.points[i].y * cloud_in.points[i].y + cloud_in.points[i].z * cloud_in.points[i].z < thres*thres)
            continue;
        cloud_out.points[j] = cloud_in.points[i];
        j++;
    }
    if (j != cloud_in.points.size())
    {
        cloud_out.points.resize(j);
    }

    // Kitti是无序的点云
    cloud_out.height = 1;
    cloud_out.width = static_cast<uint32_t>(j);
    cloud_out.is_dense = true;
}


/*
1. 无序点云-》有序点云
2. 提取特征点
*/
void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){
    /*
    ROS_INFO("data %d",laserCloudMsg->data);
    ROS_INFO("fields %d",laserCloudMsg->fields);
    ROS_INFO("header %d",laserCloudMsg->header);
    ROS_INFO("height %d",laserCloudMsg->height);
    ROS_INFO("is_bigendian %d",laserCloudMsg->is_bigendian);
    ROS_INFO("is_dense %d",laserCloudMsg->is_dense);
    ROS_INFO("point_step %d",laserCloudMsg->point_step);
    ROS_INFO("row_step %d",laserCloudMsg->row_step);
    ROS_INFO("width %d",laserCloudMsg->width);
    */

    if (!systemInited)
    {
        systemInitCount++;
        if (systemInitCount >= systemDelay)
        {
            systemInited = true;
        }
        else
            return;
    }

    TicToc t_whole;
    TicToc t_prepare;

    // 创建数组长度为 N_SCANS 的vector,并且里面的元素都为0
    // std::vector<int> array(4);//得到长度为4，默认值为0
    // std::vector<int> array{4} ;//数组一个元素，为4
    // std::vector<int> array = {4}; //和上面一样
    std::vector<int> scanStartInd(N_SCANS,0);// 数组N_SCANS个元素，为0
    std::vector<int> scanEndInd(N_SCANS,0);

    pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
    // 从ROS消息转到pcl
    pcl::fromROSMsg(*laserCloudMsg,laserCloudIn);
    std::vector<int> indices;

    // 每一帧的indices的大小是不同的，这说明每帧激光雷达获取的点云数据也是不同的
    // indices 对应保留的点云索引
    // 此时点云已经移除非法数据了，is_dense应该为true
    pcl::removeNaNFromPointCloud(laserCloudIn,laserCloudIn,indices);
     std::cout << "indices.size(): " << indices.size() << std::endl;
    // 移除最近的点
    removeClosedPointCloud(laserCloudIn,laserCloudIn,MINIMUM_RANGE);

    int cloudSize = laserCloudIn.points.size();

    // velodyne是顺时针旋转，角度取负值是将逆时针转换成顺时针运动
    float startOri = -atan2(laserCloudIn.points[0].y,laserCloudIn.points[0].x);
    float endOri = -atan2(laserCloudIn.points[cloudSize - 1].y,laserCloudIn.points[cloudSize - 1].x) + 2 * M_PI;
    
    std::cout << "startOri: " << startOri << "endOri: " << endOri << std::endl; 

    if (endOri - startOri > 3 * M_PI)
    {
        endOri -= 2 * M_PI;
    }else if(endOri - startOri < M_PI){
        endOri += 2 * M_PI;
    }

    // std::cout << "end Ori: " << endOri << std::endl;
    
    bool halfPassed = false;
    int count = cloudSize;
    PointType point;
    // 
    std::vector<pcl::PointCloud<PointType>> laserCloudScans(N_SCANS);

    for (int i = 0; i < cloudSize; ++i)
    {
        point.x = laserCloudIn.points[i].x;
        point.y = laserCloudIn.points[i].y;
        point.z = laserCloudIn.points[i].z;

        float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;

        int scanID = 0;

        if (N_SCANS == 16)
        {
            // -15-15 变成 0-30
            scanID = int((angle + 15) / 2 + 0.5f);

            if (scanID > (N_SCANS - 1) || scanID < 0)
            {
                count--;
                continue;
            }
        }else if(N_SCANS == 32){
            scanID = int((angle + 92.0/3.0) * 3.0 / 4.0f);

            if (scanID > (N_SCANS - 1) || scanID < 0)
            {
                count--;
                continue;
            }
        }else if(N_SCANS == 64){
            if (angle >= -8.83f)
                scanID = int((2 - angle) * 3.0f + 0.5f);
            else
                scanID = N_SCANS / 2 + int((-8.83 - angle) * 2.0 + 0.5);

            if (angle > 2 || angle < -24.33 ||  scanID > 50 || scanID < 0)
            {
                count--;
                continue;
            }
        }else{
            std::cout << "wrong scan number" << std::endl;
            ROS_BREAK();
        }

        float ori = -atan2(point.y,point.x);

        if (!halfPassed)
        {
            if (ori < startOri - M_PI / 2)
            {
                ori += 2 * M_PI;
            }else if (ori > startOri + M_PI * 3 / 2)
            {
                ori -= 2 * M_PI;
            }

            if (ori - startOri > M_PI)
            {
                halfPassed = true;
            }
        }else{
            ori += 2 * M_PI;
            if (ori < endOri - M_PI * 3 / 2)
            {
                ori += 2 * M_PI;
            }else if (ori > endOri + M_PI / 2)
            {
                ori -= 2 * M_PI;
            }
        }
        // 当前点占整个时间的比例
        float realTime = (ori - startOri) / (endOri - startOri);
        // 这个强度包含两个信息，整数部分是ID 小数部分是时间
        point.intensity = scanID + scanPeriod * realTime;
        // 一圈按照顺序存进去
        laserCloudScans[scanID].push_back(point);
    }

        cloudSize = count;
        std::cout << "point size " << cloudSize << std::endl;

        pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());
        
        for (int i = 0; i < N_SCANS; ++i)
        {
            scanStartInd[i] = laserCloud->size() + 5;
            // 将多行激光转换为一行激光
            *laserCloud += laserCloudScans[i];
            scanEndInd[i] = laserCloud->size() - 6;
        }    

        std::cout << "prepare time " << t_prepare.toc() << std::endl;

        for (int i = 5; i < cloudSize - 5; ++i)
        {
            float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x + laserCloud->points[i + 5].x;
            float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y + laserCloud->points[i + 5].y;
            float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z + laserCloud->points[i + 5].z;
            
            // 对应论文中的公式（1），但是没有进行除法
            cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
            cloudSortInd[i] = i;
            cloudNeighborPicked[i] = 0;
            cloudLabel[i] = 0;
        }

        TicToc t_pts;
        
        pcl::PointCloud<PointType> cornerPointsSharp;        // 极大边线点
        pcl::PointCloud<PointType> cornerPointsLessSharp;   // 次极大边线点
        pcl::PointCloud<PointType> surfPointsFlat;          // 极小平面点
        pcl::PointCloud<PointType> surfPointsLessFlat;      // 次极小平面点(经过降采样)

        float t_q_sort = 0;
        for (int i = 0; i < N_SCANS; ++i)
        {
            if (scanEndInd[i] - scanStartInd[i] < 6)
                continue;
            pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<PointType>);

            // 为了使特征点均匀分布，将一个scan分为6个扇区
            for (size_t j = 0; j < 6; ++j)
            {
                // 起始 index
                int sp = scanStartInd[i] + (scanEndInd[i] - scanEndInd[i]) * j / 6;
                // 终止 index
                int ep = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * (j + 1)/6 - 1;

                TicToc t_tmp;
                // 根据曲率大小排序
                std::sort(cloudSortInd + sp, cloudSortInd + ep + 1,comp);

                t_q_sort += t_tmp.toc();

                // 选取极大边线点和次极大边线点
                int largestPickedNum = 0;
                for (int k = sp; k >= sp; k--)
                {
                    int ind = cloudSortInd[k];
                    if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > 0.1)
                    {
                        largestPickedNum++;// 曲率大于0的点
                        if (largestPickedNum <= 2)//子扇区中曲率最大的前2个点认为是corner_sharp特征点
                        {
                            cloudLabel[ind] = 2;
                            cornerPointsSharp.push_back(laserCloud->points[ind]);
                            cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                        }
                        else if (largestPickedNum <= 20)// 子扇区曲率最大的前20个点认为是corner_less_sharp特征点
                        {
                            cloudLabel[ind] = 1;
                            cornerPointsLessSharp.push_back(laserCloud->points[ind]);
                        }else{
                            break;
                        }

                        cloudNeighborPicked[ind] = 1;//标记该点已经被选择过了
                        // 下面这两个for循环表示，这个点周围的左右各五个点都已经被选过了，就不要选了
                        for (int l = 1; l <= 5; l++)
                        {
                            float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                            float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                            float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                            {
                                break;
                            }

                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--)
                        {
                            float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                            float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                            float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                            {
                                break;
                            }

                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }
                // 提取surf平面特征，和上述类似，选取该subscan曲率最小的前4个点为surf_flat
                int smallestPickedNum = 0;
                for (int k = sp; k <= ep; k++)
                {
                    int ind = cloudSortInd[k];

                    if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < 0.1)
                    {
                        cloudLabel[ind] = -1;
                        surfPointsFlat.push_back(laserCloud->points[ind]);

                        smallestPickedNum++;
                        if (smallestPickedNum >= 4)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind] = 1;
                        for (int l = 1; l <= 5; l++)
                        { 
                            float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l - 1].x;
                            float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l - 1].y;
                            float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l - 1].z;
                            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                            {
                                break;
                            }

                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--)
                        {
                            float diffX = laserCloud->points[ind + l].x - laserCloud->points[ind + l + 1].x;
                            float diffY = laserCloud->points[ind + l].y - laserCloud->points[ind + l + 1].y;
                            float diffZ = laserCloud->points[ind + l].z - laserCloud->points[ind + l + 1].z;
                            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                            {
                                break;
                            }

                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }
                // 其它的非Corner特征点与surf_flat特征点一起组成surf_less_flat特征点
                for (int k = sp; k <= ep; k++)
                {
                    if (cloudLabel[k] <=0)
                    {
                        surfPointsLessFlatScan->push_back(laserCloud->points[k]);
                    }
                    
                }
            }
            // 最后对该scan点云中提取到的所有surf_less_flat特征点进行降采样，因为点有点多
            // 对每一条scan线上的次极小平面点进行一次降采样
            pcl::PointCloud<PointType> surfPointsLessFlatScanDS;// 降采样之后的点云
            pcl::VoxelGrid<PointType> downSizeFilter;
            downSizeFilter.setInputCloud(surfPointsLessFlatScan);
            downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
            downSizeFilter.filter(surfPointsLessFlatScanDS);

            surfPointsLessFlat += surfPointsLessFlatScanDS;  
        }
        std::cout << "sort q time " << t_q_sort << std::endl;
        std::cout << "seperatr points time " << t_pts.toc() << std::endl;

    // 将原始点云（经过无序-->有序的处理）发布出去
    sensor_msgs::PointCloud2 laserCloudOutMsg;
    pcl::toROSMsg(*laserCloud, laserCloudOutMsg);
    laserCloudOutMsg.header.stamp = laserCloudMsg->header.stamp;
    laserCloudOutMsg.header.frame_id = "map";
    pubLaserCloud.publish(laserCloudOutMsg);

    // 将极大边线点发布出去
    sensor_msgs::PointCloud2 cornerPointsSharpMsg;
    pcl::toROSMsg(cornerPointsSharp, cornerPointsSharpMsg);
    cornerPointsSharpMsg.header.stamp = laserCloudMsg->header.stamp;
    cornerPointsSharpMsg.header.frame_id = "map";
    pubCornerPointsSharp.publish(cornerPointsSharpMsg);

    // 将次极大边线点发布出去
    sensor_msgs::PointCloud2 cornerPointsLessSharpMsg;
    pcl::toROSMsg(cornerPointsLessSharp, cornerPointsLessSharpMsg);
    cornerPointsLessSharpMsg.header.stamp = laserCloudMsg->header.stamp;
    cornerPointsLessSharpMsg.header.frame_id = "map";
    pubCornerPointsLessSharp.publish(cornerPointsLessSharpMsg);

    // 将极小平面点发布出去
    sensor_msgs::PointCloud2 surfPointsFlat2;
    pcl::toROSMsg(surfPointsFlat, surfPointsFlat2);
    surfPointsFlat2.header.stamp = laserCloudMsg->header.stamp;
    surfPointsFlat2.header.frame_id = "map";
    pubSurfPointsFlat.publish(surfPointsFlat2);

    // 将次极小平面点发布出去
    sensor_msgs::PointCloud2 surfPointsLessFlat2;
    pcl::toROSMsg(surfPointsLessFlat, surfPointsLessFlat2);
    surfPointsLessFlat2.header.stamp = laserCloudMsg->header.stamp;
    surfPointsLessFlat2.header.frame_id = "map";
    pubSurfPointsLessFlat.publish(surfPointsLessFlat2);
        

    // 将每条scan的线扫发布出去
    if(PUB_EACH_LINE)
    {
        for(int i = 0; i< N_SCANS; i++)
        {
            sensor_msgs::PointCloud2 scanMsg;
            pcl::toROSMsg(laserCloudScans[i], scanMsg);
            scanMsg.header.stamp = laserCloudMsg->header.stamp;
            scanMsg.header.frame_id = "map";
            pubEachScan[i].publish(scanMsg);
        }
    }

    printf("scan registration time %f ms *************\n", t_whole.toc());
    if(t_whole.toc() > 100)
        ROS_WARN("scan registration process over 100ms");      
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"scanRegistration");
    ros::NodeHandle nh;

    // 16线
    nh.param<int>("scan_line", N_SCANS, 16);

    nh.param<double>("minimum_range", MINIMUM_RANGE, 0.1);

    printf("scan line number %d \n", N_SCANS);

    if(N_SCANS != 16 && N_SCANS != 32 && N_SCANS != 64)
    {
        printf("only support velodyne with 16, 32 or 64 scan line!");
        return 0;
    }

    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/kitti/velo/pointcloud",100,laserCloudHandler);
    
    pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 100);

    pubCornerPointsSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100);

    pubCornerPointsLessSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 100);

    pubSurfPointsFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 100);

    pubSurfPointsLessFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 100);

    pubRemovePoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_remove_points", 100);

    if(PUB_EACH_LINE)
    {
        for(int i = 0; i < N_SCANS; i++)
        {
            ros::Publisher tmp = nh.advertise<sensor_msgs::PointCloud2>("/laser_scanid_" + std::to_string(i), 100);
            pubEachScan.push_back(tmp);
        }
    }
    
    
    ros::spin();

    return 0;
}
