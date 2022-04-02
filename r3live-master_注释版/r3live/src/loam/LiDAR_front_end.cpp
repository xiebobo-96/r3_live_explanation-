#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver/CustomMsg.h>
#include "../tools/tools_logger.hpp"

using namespace std;

#define IS_VALID( a ) ( ( abs( a ) > 1e8 ) ? true : false )

typedef pcl::PointXYZINormal PointType;

ros::Publisher pub_full, pub_surf, pub_corn;

enum LID_TYPE
{
    MID,
    HORIZON,
    VELO16,
    OUST64
};

enum Feature
{
    Nor,
    Poss_Plane,
    Real_Plane,
    Edge_Jump,
    Edge_Plane,
    Wire,
    ZeroPoint
};
enum Surround
{
    Prev,
    Next
};
enum E_jump
{
    Nr_nor,
    Nr_zero,
    Nr_180,
    Nr_inf,
    Nr_blind
};

struct orgtype
{
    double  range;
    double  dista;
    double  angle[ 2 ];
    double  intersect;
    E_jump  edj[ 2 ];
    Feature ftype;
    orgtype()
    {
        range = 0;
        edj[ Prev ] = Nr_nor;
        edj[ Next ] = Nr_nor;
        ftype = Nor;
        intersect = 2;
    }
};

const double rad2deg = 180 * M_1_PI;

int    lidar_type;
double blind, inf_bound;
int    N_SCANS;
int    group_size;
double disA, disB;
double limit_maxmid, limit_midmin, limit_maxmin;
double p2l_ratio;
double jump_up_limit, jump_down_limit;
double cos160;
double edgea, edgeb;
double smallp_intersect, smallp_ratio;
int    point_filter_num;
int    g_if_using_raw_point = 1;
int    g_LiDAR_sampling_point_step = 3;
void   mid_handler( const sensor_msgs::PointCloud2::ConstPtr &msg );
void   horizon_handler( const livox_ros_driver::CustomMsg::ConstPtr &msg );
void   velo16_handler( const sensor_msgs::PointCloud2::ConstPtr &msg );
void   oust64_handler( const sensor_msgs::PointCloud2::ConstPtr &msg );
void   give_feature( pcl::PointCloud< PointType > &pl, vector< orgtype > &types, pcl::PointCloud< PointType > &pl_corn,
                     pcl::PointCloud< PointType > &pl_surf );
void   pub_func( pcl::PointCloud< PointType > &pl, ros::Publisher pub, const ros::Time &ct );
int    plane_judge( const pcl::PointCloud< PointType > &pl, vector< orgtype > &types, uint i, uint &i_nex, Eigen::Vector3d &curr_direct );
bool   small_plane( const pcl::PointCloud< PointType > &pl, vector< orgtype > &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct );
bool   edge_jump_judge( const pcl::PointCloud< PointType > &pl, vector< orgtype > &types, uint i, Surround nor_dir );

int main( int argc, char **argv )
{
    ros::init( argc, argv, "feature_extract" );
    ros::NodeHandle n;

    n.param< int >( "Lidar_front_end/lidar_type", lidar_type, 0 );
    n.param< double >( "Lidar_front_end/blind", blind, 0.1 );
    n.param< double >( "Lidar_front_end/inf_bound", inf_bound, 4 );
    n.param< int >( "Lidar_front_end/N_SCANS", N_SCANS, 1 );
    n.param< int >( "Lidar_front_end/group_size", group_size, 8 );
    n.param< double >( "Lidar_front_end/disA", disA, 0.01 );
    n.param< double >( "Lidar_front_end/disB", disB, 0.1 );
    n.param< double >( "Lidar_front_end/p2l_ratio", p2l_ratio, 225 );
    n.param< double >( "Lidar_front_end/limit_maxmid", limit_maxmid, 6.25 );
    n.param< double >( "Lidar_front_end/limit_midmin", limit_midmin, 6.25 );
    n.param< double >( "Lidar_front_end/limit_maxmin", limit_maxmin, 3.24 );
    n.param< double >( "Lidar_front_end/jump_up_limit", jump_up_limit, 170.0 );
    n.param< double >( "Lidar_front_end/jump_down_limit", jump_down_limit, 8.0 );
    n.param< double >( "Lidar_front_end/cos160", cos160, 160.0 );
    n.param< double >( "Lidar_front_end/edgea", edgea, 2 );
    n.param< double >( "Lidar_front_end/edgeb", edgeb, 0.1 );
    n.param< double >( "Lidar_front_end/smallp_intersect", smallp_intersect, 172.5 );
    n.param< double >( "Lidar_front_end/smallp_ratio", smallp_ratio, 1.2 );
    n.param< int >( "Lidar_front_end/point_filter_num", point_filter_num, 1 );
    n.param< int >( "Lidar_front_end/point_step", g_LiDAR_sampling_point_step, 3 );
    n.param< int >( "Lidar_front_end/using_raw_point", g_if_using_raw_point, 1 );

    jump_up_limit = cos( jump_up_limit / 180 * M_PI );
    jump_down_limit = cos( jump_down_limit / 180 * M_PI );
    cos160 = cos( cos160 / 180 * M_PI );
    smallp_intersect = cos( smallp_intersect / 180 * M_PI );

    ros::Subscriber sub_points;

    switch ( lidar_type )
    {
    case MID:
        printf( "MID40\n" );
        sub_points = n.subscribe( "/livox/lidar", 1000, mid_handler, ros::TransportHints().tcpNoDelay() );
        break;

    case HORIZON:
        printf( "HORIZON\n" );
        sub_points = n.subscribe( "/livox/lidar", 1000, horizon_handler, ros::TransportHints().tcpNoDelay() );
        break;

    case VELO16:
        printf( "VELO16\n" );
        sub_points = n.subscribe( "/velodyne_points", 1000, velo16_handler, ros::TransportHints().tcpNoDelay() );//回调函数是空的.
        break;

    case OUST64:
        printf( "OUST64\n" );
        sub_points = n.subscribe( "/os_cloud_node/points", 1000, oust64_handler, ros::TransportHints().tcpNoDelay() );
        break;

    default:
        printf( "Lidar type is wrong.\n" );
        exit( 0 );
        break;
    } 

    pub_full = n.advertise< sensor_msgs::PointCloud2 >( "/laser_cloud", 100 );//发布所有点
    pub_surf = n.advertise< sensor_msgs::PointCloud2 >( "/laser_cloud_flat", 100 );//发布面点
    pub_corn = n.advertise< sensor_msgs::PointCloud2 >( "/laser_cloud_sharp", 100 );//发布角点(线点)

    ros::spin();
    return 0;
}

double vx, vy, vz;
void   mid_handler( const sensor_msgs::PointCloud2::ConstPtr &msg )
{
    pcl::PointCloud< PointType > pl;
    pcl::fromROSMsg( *msg, pl );

    pcl::PointCloud< PointType > pl_corn, pl_surf;
    vector< orgtype >            types;
    uint                         plsize = pl.size() - 1;
    pl_corn.reserve( plsize );
    pl_surf.reserve( plsize );
    types.resize( plsize + 1 );

    for ( uint i = 0; i < plsize; i++ )
    {
        types[ i ].range = pl[ i ].x;
        vx = pl[ i ].x - pl[ i + 1 ].x;
        vy = pl[ i ].y - pl[ i + 1 ].y;
        vz = pl[ i ].z - pl[ i + 1 ].z;
        types[ i ].dista = vx * vx + vy * vy + vz * vz;//前后两点间距离的平方
    }
    // plsize++;
    types[ plsize ].range = sqrt( pl[ plsize ].x * pl[ plsize ].x + pl[ plsize ].y * pl[ plsize ].y );//只计算了最后一个点的range,应该每个点都要计算,这里似乎有bug!!!!!

    give_feature( pl, types, pl_corn, pl_surf );

    ros::Time ct( ros::Time::now() );
    pub_func( pl, pub_full, msg->header.stamp );
    pub_func( pl_surf, pub_surf, msg->header.stamp );
    pub_func( pl_corn, pub_corn, msg->header.stamp );
}

//msg为livox_ros_driver发过来的数据,livox_ros_driver是雷达的硬件驱动程序.
void horizon_handler( const livox_ros_driver::CustomMsg::ConstPtr &msg )
{
    double                                 t1 = omp_get_wtime();
    vector< pcl::PointCloud< PointType > > pl_buff( N_SCANS );// N_SCANS =6  scan的数量?
    vector< vector< orgtype > >            typess( N_SCANS );
    pcl::PointCloud< PointType >           pl_full, pl_corn, pl_surf; //所有点,角点,面点

    uint plsize = msg->point_num;

    pl_corn.reserve( plsize );
    pl_surf.reserve( plsize );
    pl_full.resize( plsize );

    for ( int i = 0; i < N_SCANS; i++ )
    {
        pl_buff[ i ].reserve( plsize );
    }
    // ANCHOR - remove nearing pts.
    for ( uint i = 1; i < plsize; i++ )
    {
        // clang-format off
        if ( ( msg->points[ i ].line < N_SCANS ) 
            && ( !IS_VALID( msg->points[ i ].x ) ) 
            && ( !IS_VALID( msg->points[ i ].y ) ) 
            && ( !IS_VALID( msg->points[ i ].z ) )
            && msg->points[ i ].x > 0.7 )
        {
            // https://github.com/Livox-SDK/Livox-SDK/wiki/Livox-SDK-Communication-Protocol
            // See [3.4 Tag Information] 
            if ( ( msg->points[ i ].x > 2.0 )
                && ( ( ( msg->points[ i ].tag & 0x03 ) != 0x00 )  ||  ( ( msg->points[ i ].tag & 0x0C ) != 0x00 ) )//msg->points[ i ].tag & 0x03 ) != 0x00 )  ||  ( ( msg->points[ i ].tag & 0x0C ) != 0x00 这个判断是为剔除噪点.
                )
            {
                // Remove the bad quality points,剔除噪点
                continue;
            }
         // clang-format on
            pl_full[ i ].x = msg->points[ i ].x;
            pl_full[ i ].y = msg->points[ i ].y;
            pl_full[ i ].z = msg->points[ i ].z;
            pl_full[ i ].intensity = msg->points[ i ].reflectivity;

            //curvature为每个激光点的时间戳
            pl_full[ i ].curvature = msg->points[ i ].offset_time / float( 1000000 ); // use curvature as time of each laser points

            if ( ( std::abs( pl_full[ i ].x - pl_full[ i - 1 ].x ) > 1e-7 ) || ( std::abs( pl_full[ i ].y - pl_full[ i - 1 ].y ) > 1e-7 ) ||
                 ( std::abs( pl_full[ i ].z - pl_full[ i - 1 ].z ) > 1e-7 ) )//要求前后两点间满足一定的距离
            {
                pl_buff[ msg->points[ i ].line ].push_back( pl_full[ i ] );//一帧数据有多个scans,因为是花瓣式扫描
            }
        }
    }
    if ( pl_buff.size() != N_SCANS )//不能缺少任意一个scan
    {
        return;
    }
    if ( pl_buff[ 0 ].size() <= 7 )//一个scan的点太少也return掉
    {
        return;
    }
    for ( int j = 0; j < N_SCANS; j++ )//一个scan循环一次
    {
        pcl::PointCloud< PointType > &pl = pl_buff[ j ];
        vector< orgtype > &           types = typess[ j ];
        plsize = pl.size();
        if ( plsize < 7 )
        {
            continue;
        }
        types.resize( plsize );
        plsize--;
        for ( uint pt_idx = 0; pt_idx < plsize; pt_idx++ )
        {
            types[ pt_idx ].range = pl[ pt_idx ].x * pl[ pt_idx ].x + pl[ pt_idx ].y * pl[ pt_idx ].y;

            //vx,vy,vz为全局变量
            vx = pl[ pt_idx ].x - pl[ pt_idx + 1 ].x;
            vy = pl[ pt_idx ].y - pl[ pt_idx + 1 ].y;
            vz = pl[ pt_idx ].z - pl[ pt_idx + 1 ].z;
            // std::cout<<vx<<" "<<vx<<" "<<vz<<" "<<std::endl;
        }
        // plsize++;
        types[ plsize ].range = pl[ plsize ].x * pl[ plsize ].x + pl[ plsize ].y * pl[ plsize ].y;

        //最主要的函数,计算线点与面点.
        give_feature( pl, types, pl_corn, pl_surf );
    }
    if ( pl_surf.points.size() < 100 )
    {
        return;
    }
    ros::Time ct;
    ct.fromNSec( msg->timebase );
    pub_func( pl_full, pub_full, msg->header.stamp );//msg->header.stamp为livox_ros_driver中传过来的数据的时间戳.
    pub _func( pl_surf, pub_surf, msg->header.stamp );
    pub_func( pl_corn, pub_corn, msg->header.stamp );
}

int orders[ 16 ] = { 0, 2, 4, 6, 8, 10, 12, 14, 1, 3, 5, 7, 9, 11, 13, 15 };

void velo16_handler( const sensor_msgs::PointCloud2::ConstPtr &msg )
{
    // TODO
}

void velo16_handler1( const sensor_msgs::PointCloud2::ConstPtr &msg )
{
    // TODO
}

namespace ouster_ros {

struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t ambient;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace ouster_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    // use std::uint32_t to avoid conflicting with pcl::uint32_t
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
)
// clang-format on

void oust64_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pcl::PointCloud< PointType > pl_processed;
    pcl::PointCloud< ouster_ros::Point > pl_orig;
    // pcl::PointCloud<pcl::PointXYZI> pl_orig;
    pcl::fromROSMsg( *msg, pl_orig );
    uint plsize = pl_orig.size();

    double time_stamp = msg->header.stamp.toSec();
    pl_processed.clear();
    pl_processed.reserve( pl_orig.points.size() );
    for ( int i = 0; i < pl_orig.points.size(); i++ )
    {
        double range = std::sqrt( pl_orig.points[ i ].x * pl_orig.points[ i ].x + pl_orig.points[ i ].y * pl_orig.points[ i ].y +
                                  pl_orig.points[ i ].z * pl_orig.points[ i ].z );
        if ( range < blind )
        {
            continue;
        }
        Eigen::Vector3d pt_vec;
        PointType       added_pt;
        added_pt.x = pl_orig.points[ i ].x;
        added_pt.y = pl_orig.points[ i ].y;
        added_pt.z = pl_orig.points[ i ].z;
        added_pt.intensity = pl_orig.points[ i ].intensity;
        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        double yaw_angle = std::atan2( added_pt.y, added_pt.x ) * 57.3;
        if ( yaw_angle >= 180.0 )
            yaw_angle -= 360.0;
        if ( yaw_angle <= -180.0 )
            yaw_angle += 360.0;

        added_pt.curvature = ( pl_orig.points[ i ].t / 1e9 ) * 1000.0;//  为什么这样做?

        pl_processed.points.push_back( added_pt );
        if ( 0 ) // For debug
        {
            if ( pl_processed.size() % 1000 == 0 )
            {
                printf( "[%d] (%.2f, %.2f, %.2f), ( %.2f, %.2f, %.2f ) | %.2f | %.3f,  \r\n", i, pl_orig.points[ i ].x, pl_orig.points[ i ].y,
                        pl_orig.points[ i ].z, pl_processed.points.back().normal_x, pl_processed.points.back().normal_y,
                        pl_processed.points.back().normal_z, yaw_angle, pl_processed.points.back().intensity );
                // printf("(%d, %.2f, %.2f)\r\n", pl_orig.points[i].ring, pl_orig.points[i].t, pl_orig.points[i].range);
                // printf("(%d, %d, %d)\r\n", pl_orig.points[i].ring, 1, 2);
                cout << ( int ) ( pl_orig.points[ i ].ring ) << ", " << ( pl_orig.points[ i ].t / 1e9 ) << ", " << pl_orig.points[ i ].range << endl;
            }
        }
    }
    pub_func( pl_processed, pub_full, msg->header.stamp );
    pub_func( pl_processed, pub_surf, msg->header.stamp );
    pub_func( pl_processed, pub_corn, msg->header.stamp );
}


void give_feature( pcl::PointCloud< PointType > &pl, vector< orgtype > &types, pcl::PointCloud< PointType > &pl_corn,
                   pcl::PointCloud< PointType > &pl_surf )
{
    uint plsize = pl.size();
    uint plsize2;
    if ( plsize == 0 )
    {
        printf( "something wrong\n" );
        return;
    }
    uint head = 0;
    while ( types[ head ].range < blind )//blind默认为0.1
    {
        head++;
    }

    // Surf
    plsize2 = ( plsize > group_size ) ? ( plsize - group_size ) : 0;//group_size默认为8

    Eigen::Vector3d curr_direct( Eigen::Vector3d::Zero() );
    Eigen::Vector3d last_direct( Eigen::Vector3d::Zero() );

    uint i_nex = 0, i2;
    uint last_i = 0;
    uint last_i_nex = 0;
    int  last_state = 0;
    int  plane_type;

    
    //第一个for循环
    PointType ap;
    for ( uint i = head; i < plsize2; i += g_LiDAR_sampling_point_step )//g_LiDAR_sampling_point_step=1
    {
        if ( types[ i ].range > blind )
        {
            ap.x = pl[ i ].x;
            ap.y = pl[ i ].y;
            ap.z = pl[ i ].z;
            ap.curvature = pl[ i ].curvature;
            pl_surf.push_back( ap );
        }
        if ( g_if_using_raw_point )
        {
            continue;
        }
        // i_nex = i;
        i2 = i;
        // std::cout<<" i: "<<i<<" i_nex "<<i_nex<<"group_size: "<<group_size<<" plsize "<<plsize<<" plsize2
        // "<<plsize2<<std::endl;
        plane_type = plane_judge( pl, types, i, i_nex, curr_direct );

        if ( plane_type == 1 )// plane_judge返回值为1,表示当前点为面点
        {
            for ( uint j = i; j <= i_nex; j++ )
            {
                if ( j != i && j != i_nex )
                {
                    types[ j ].ftype = Real_Plane;
                }
                else
                {
                    types[ j ].ftype = Poss_Plane;//i与 i_nex处的点被标记为Poss_Plane,这两个点会通过下面的if语句判断其是否为Real_Plane
                }
            }

            // if(last_state==1 && fabs(last_direct.sum())>0.5)
            if ( last_state == 1 && last_direct.norm() > 0.1 )
            {
                double mod = last_direct.transpose() * curr_direct;
                if ( mod > -0.707 && mod < 0.707 )//应该是if ( mod > -0.707 ||mod < 0.707 ),上次i与i_nex形成的向量(单位向量)与这次i与i_nex形成的向量的点积,判断他们形成的夹角是否大于45°.
                {
                    types[ i ].ftype = Edge_Plane;
                }
                else
                {
                    types[ i ].ftype = Real_Plane;
                }
            }

            i = i_nex - 1;//更新 i 下个循环从i_nex 开始
            last_state = 1;
        }
        else if ( plane_type == 2 )
        {
            i = i_nex;//下个循环从 i_nex +1开始
            last_state = 0;
        }
        else if ( plane_type == 0 )//时plane_type == 0,i与i_nex间存在线点
        {
            if ( last_state == 1 )//last_state=1才会运行
            {
                uint i_nex_tem;
                uint j;
                for ( j = last_i + 1; j <= last_i_nex; j++ )//当上面的plane_judge返回为1时,进行二次搜索计算
                {
                    uint            i_nex_tem2 = i_nex_tem;
                    Eigen::Vector3d curr_direct2;

                    uint ttem = plane_judge( pl, types, j, i_nex_tem, curr_direct2 );

                    if ( ttem != 1 )
                    {
                        i_nex_tem = i_nex_tem2;//i_nex_tem2是上次for循环的 i_nex_tem值,作用是当函数plane_judge返回值不为1时,i_nex_tem就等于上次的i_nex_tem
                        break;
                    }
                    curr_direct = curr_direct2;
                }

                if ( j == last_i + 1 )
                {
                    last_state = 0;
                }
                else
                {
                    for ( uint k = last_i_nex; k <= i_nex_tem; k++ )
                    {
                        if ( k != i_nex_tem )
                        {
                            types[ k ].ftype = Real_Plane;
                        }
                        else
                        {
                            types[ k ].ftype = Poss_Plane;
                        }
                    }
                    i = i_nex_tem - 1;
                    i_nex = i_nex_tem;
                    i2 = j - 1;
                    last_state = 1;
                }
            }
        }

        last_i = i2;
        last_i_nex = i_nex;
        last_direct = curr_direct;
    }
    if ( g_if_using_raw_point )
    {
        return;
    }

    
    //第二个for循环
    plsize2 = plsize > 3 ? plsize - 3 : 0;
    for ( uint i = head + 3; i < plsize2; i++ )
    {
        if ( types[ i ].range < blind || types[ i ].ftype >= Real_Plane )//已经判断为平面点(Real_Plane)的不需要再次进行判断.
        {
            continue;
        }

        if ( types[ i - 1 ].dista < 1e-16 || types[ i ].dista < 1e-16 )
        {
            continue;
        }

        Eigen::Vector3d vec_a( pl[ i ].x, pl[ i ].y, pl[ i ].z );
        Eigen::Vector3d vecs[ 2 ];

        for ( int j = 0; j < 2; j++ )
        {
            int m = -1;
            if ( j == 1 )
            {
                m = 1;     //m这样取值是为了取i前后的两点
            }

            if ( types[ i + m ].range < blind )
            {
                if ( types[ i ].range > inf_bound )
                {
                    types[ i ].edj[ j ] = Nr_inf;//3   i点前后存在blind时edj会设置成Nr_inf
                }
                else
                {
                    types[ i ].edj[ j ] = Nr_blind;//4
                }
                continue;
            }

            vecs[ j ] = Eigen::Vector3d( pl[ i + m ].x, pl[ i + m ].y, pl[ i + m ].z );
            vecs[ j ] = vecs[ j ] - vec_a;

            types[ i ].angle[ j ] = vec_a.dot( vecs[ j ] ) / vec_a.norm() / vecs[ j ].norm();//向量点乘公式,求cos@.

            //8~170edj都保持为Nr_nor
            if ( types[ i ].angle[ j ] < jump_up_limit )//jump_up_limit=170  jump_up_limit应该是cos170才对,表示小于这个阈值角度就为180
            {
                types[ i ].edj[ j ] = Nr_180;//2
            }
            else if ( types[ i ].angle[ j ] > jump_down_limit )// jump_down_limit=8  jump_up_limit应该是cos8才对,表示小于这个阈值角度就为0
            {
                types[ i ].edj[ j ] = Nr_zero;//1                           
            }
        }
               //  enum Surround
              // {
               //     Prev,
              //     Next
               // };
        types[ i ].intersect = vecs[ Prev ].dot( vecs[ Next ] ) / vecs[ Prev ].norm() / vecs[ Next ].norm();//前后两点与自己形成的夹角.
        if ( types[ i ].edj[ Prev ] == Nr_nor && types[ i ].edj[ Next ] == Nr_zero && types[ i ].dista > 0.0225 &&
             types[ i ].dista > 4 * types[ i - 1 ].dista )
        {
            if ( types[ i ].intersect > cos160 )//传入的参数为160,但是这里应该是cos(160),可能是参数传错了.表示夹角小于160°时可能是角点,进而进行角点判断.
            {
                if ( edge_jump_judge( pl, types, i, Prev ) )
                {
                    types[ i ].ftype = Edge_Jump;
                }
            }
        }
        else if ( types[ i ].edj[ Prev ] == Nr_zero && types[ i ].edj[ Next ] == Nr_nor && types[ i - 1 ].dista > 0.0225 &&
                  types[ i - 1 ].dista > 4 * types[ i ].dista )
        {
            if ( types[ i ].intersect > cos160 )
            {
                if ( edge_jump_judge( pl, types, i, Next ) )
                {
                    types[ i ].ftype = Edge_Jump;
                }
            }
        }
        else if ( types[ i ].edj[ Prev ] == Nr_nor && types[ i ].edj[ Next ] == Nr_inf )
        {
            if ( edge_jump_judge( pl, types, i, Prev ) )
            {
                types[ i ].ftype = Edge_Jump;
            }
        }
        else if ( types[ i ].edj[ Prev ] == Nr_inf && types[ i ].edj[ Next ] == Nr_nor )
        {
            if ( edge_jump_judge( pl, types, i, Next ) )
            {
                types[ i ].ftype = Edge_Jump;
            }
        }
        else if ( types[ i ].edj[ Prev ] > Nr_nor && types[ i ].edj[ Next ] > Nr_nor )
        {
            if ( types[ i ].ftype == Nor )
            {
                types[ i ].ftype = Wire;
            }
        }
    }


   //第三个for循环
    plsize2 = plsize - 1;
    double ratio;
    for ( uint i = head + 1; i < plsize2; i++ )
    {
        if ( types[ i ].range < blind || types[ i - 1 ].range < blind || types[ i + 1 ].range < blind )
        {
            continue;
        }

        if ( types[ i - 1 ].dista < 1e-8 || types[ i ].dista < 1e-8 )
        {
            continue;
        }

        if ( types[ i ].ftype == Nor )
        {
            if ( types[ i - 1 ].dista > types[ i ].dista )
            {
                ratio = types[ i - 1 ].dista / types[ i ].dista;
            }
            else
            {
                ratio = types[ i ].dista / types[ i - 1 ].dista;//ratio表示当前点与前一点(i-1)与后一点(i+1)间距离的比值.
            }
            // smallp_intersect应该是cos172.5,表示前后两点与自己形成的夹角大于172.5°
            if ( types[ i ].intersect < smallp_intersect && ratio < smallp_ratio )//types[ i ].intersect为前后两点与自己形成的夹角 ,smallp_ratio =1.2
            {
                if ( types[ i - 1 ].ftype == Nor )
                {
                    types[ i - 1 ].ftype = Real_Plane;
                }
                if ( types[ i + 1 ].ftype == Nor )
                {
                    types[ i + 1 ].ftype = Real_Plane;
                }
                types[ i ].ftype = Real_Plane;
            }
        }
    }


    //第四个for循环
    int last_surface = -1;
    for ( uint j = head; j < plsize; j++ )
    {
        if ( types[ j ].ftype == Poss_Plane || types[ j ].ftype == Real_Plane )
        {
            if ( last_surface == -1 )
            {
                last_surface = j;
            }

            if ( j == uint( last_surface + point_filter_num - 1 ) )//point_filter_num=1(point_filter_num=1就是每个平面点都push进pl_surf,point_filter_num=x就是x个平面点只有一个push进pl_surf)
            {
                PointType ap;
                ap.x = pl[ j ].x;
                ap.y = pl[ j ].y;
                ap.z = pl[ j ].z;
                ap.curvature = pl[ j ].curvature;
                pl_surf.push_back( ap );

                last_surface = -1;
            }
        }
        else
        {
            if ( types[ j ].ftype == Edge_Jump || types[ j ].ftype == Edge_Plane )
            {
                pl_corn.push_back( pl[ j ] );
            }
            if ( last_surface != -1 )
            {
                PointType ap;
                for ( uint k = last_surface; k < j; k++ )
                {
                    ap.x += pl[ k ].x;
                    ap.y += pl[ k ].y;
                    ap.z += pl[ k ].z;
                    ap.curvature += pl[ k ].curvature;
                }
                ap.x /= ( j - last_surface );
                ap.y /= ( j - last_surface );
                ap.z /= ( j - last_surface );
                ap.curvature /= ( j - last_surface );
                pl_surf.push_back( ap );
            }
            last_surface = -1;
        }
    }
}

void pub_func( pcl::PointCloud< PointType > &pl, ros::Publisher pub, const ros::Time &ct )
{
    pl.height = 1;
    pl.width = pl.size();
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg( pl, output );
    output.header.frame_id = "livox";
    output.header.stamp = ct;
    pub.publish( output );
}

int plane_judge( const pcl::PointCloud< PointType > &pl, vector< orgtype > &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct )
{
    double group_dis = disA * types[ i_cur ].range + disB;
    group_dis = group_dis * group_dis;
    // i_nex = i_cur;

    double           two_dis;
    vector< double > disarr;
    disarr.reserve( 20 );

    for ( i_nex = i_cur; i_nex < i_cur + group_size; i_nex++ )//group_size=8  查看包括cur再内的8个点   i_nex会更新
    {
        if ( types[ i_nex ].range < blind )
        {
            curr_direct.setZero();
            return 2;//return表示下此从 i_nex处开始寻找面点,而i_cur到 i_nex间的点都不要
        }
        //  vx = pl[ i ].x - pl[ i + 1 ].x;
        // vy = pl[ i ].y - pl[ i + 1 ].y;    mid_handler中存在
        // vz = pl[ i ].z - pl[ i + 1 ].z;
        // types[ i ].dista = vx * vx + vy * vy + vz * vz;
        disarr.push_back( types[ i_nex ].dista ); 
    }

    for ( ;; )//死循环,相当于while(1)
    {
        if ( ( i_cur >= pl.size() ) || ( i_nex >= pl.size() ) )
            break;

        if ( types[ i_nex ].range < blind )
        {
            curr_direct.setZero();
            return 2;
        }
        vx = pl[ i_nex ].x - pl[ i_cur ].x;
        vy = pl[ i_nex ].y - pl[ i_cur ].y;
        vz = pl[ i_nex ].z - pl[ i_cur ].z;
        two_dis = vx * vx + vy * vy + vz * vz;//其实就是两3D点间的距离的平方
        if ( two_dis >= group_dis )
        {
            break;
        }
        disarr.push_back( types[ i_nex ].dista );
        i_nex++; //i_nex会更新
    }

    double leng_wid = 0;
    double v1[ 3 ], v2[ 3 ];
    for ( uint j = i_cur + 1; j < i_nex; j++ )
    {
        if ( ( j >= pl.size() ) || ( i_cur >= pl.size() ) )
            break;
        v1[ 0 ] = pl[ j ].x - pl[ i_cur ].x;
        v1[ 1 ] = pl[ j ].y - pl[ i_cur ].y;
        v1[ 2 ] = pl[ j ].z - pl[ i_cur ].z;

        v2[ 0 ] = v1[ 1 ] * vz - vy * v1[ 2 ];
        v2[ 1 ] = v1[ 2 ] * vx - v1[ 0 ] * vz;
        v2[ 2 ] = v1[ 0 ] * vy - vx * v1[ 1 ];   //负([vx,vy,vz]向量叉乘v1向量)

        double lw = v2[ 0 ] * v2[ 0 ] + v2[ 1 ] * v2[ 1 ] + v2[ 2 ] * v2[ 2 ];//三角形面积的平方:长度与宽度间的乘积关系
        if ( lw > leng_wid )
        {
            leng_wid = lw;//leng_wid:长度与宽度   长度:i_nex与i_cur两点间的长度  宽度:i_nex与i_cu 间所有的点到i_nex与i_cur处两点组成的直线的距离的最大值
        }
    }

    if ( ( two_dis * two_dis / leng_wid ) < p2l_ratio )// 判断i_nex与i_cur间所有的点到i_nex与i_cur处两点组成的直线的距离和i_nex与i_cur两点间的长度的一个关系(距离与长度都带有平方)
    {
        curr_direct.setZero();
        return 0;//宽度太大时会return0.
    }

    uint disarrsize = disarr.size();
    for ( uint j = 0; j < disarrsize - 1; j++ )
    {
        for ( uint k = j + 1; k < disarrsize; k++ )//相当于对disarr进行排序,从大到小依次排序(序号越小,disarr越大)
        {
            if ( disarr[ j ] < disarr[ k ] )
            {
                leng_wid = disarr[ j ];
                disarr[ j ] = disarr[ k ];
                disarr[ k ] = leng_wid;
            }
        }
    }

    if ( disarr[ disarr.size() - 2 ] < 1e-16 )//  disarr不能太小,两点间距离的平方不能太小.
    {
        curr_direct.setZero();
        return 0;//return0后应该是判断当前i_nex与i_cur间所有的点都不存在面点
    }

    if ( lidar_type == MID || lidar_type == HORIZON )
    {
        double dismax_mid = disarr[ 0 ] / disarr[ disarrsize / 2 ];//最大值与中间值的比值
        double dismid_min = disarr[ disarrsize / 2 ] / disarr[ disarrsize - 2 ];//中间值与最小值的比值

        if ( dismax_mid >= limit_maxmid || dismid_min >= limit_midmin )
        {
            curr_direct.setZero();
            return 0;
        }
    }
    else
    {
        double dismax_min = disarr[ 0 ] / disarr[ disarrsize - 2 ];//最大值与最小值的比值
        if ( dismax_min >= limit_maxmin )
        {
            curr_direct.setZero();
            return 0;
        }
    }

    curr_direct << vx, vy, vz;//curr_direct表示i_nex与i_cur间的向量
    curr_direct.normalize();//转为单位向量
    return 1;//return表示i_cur为面点
}

bool edge_jump_judge( const pcl::PointCloud< PointType > &pl, vector< orgtype > &types, uint i, Surround nor_dir )
{
    if ( nor_dir == 0 )
    {
        if ( types[ i - 1 ].range < blind || types[ i - 2 ].range < blind )
        {
            return false;
        }
    }
    else if ( nor_dir == 1 )
    {
        if ( types[ i + 1 ].range < blind || types[ i + 2 ].range < blind )
        {
            return false;
        }
    }
    double d1 = types[ i + nor_dir - 1 ].dista;
    double d2 = types[ i + 3 * nor_dir - 2 ].dista;
    double d;

    if ( d1 < d2 )
    {
        d = d1;
        d1 = d2;
        d2 = d;
    }

    d1 = sqrt( d1 );
    d2 = sqrt( d2 );

    if ( d1 > edgea * d2 || ( d1 - d2 ) > edgeb )//若i是角点,则i两边的点的dista不能相差太大
    {
        return false;
    }

    return true;
}
