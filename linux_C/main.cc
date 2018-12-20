#include <iostream>
#include <vector>
#include <math.h>
#include <Eigen/Eigen>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h> //PCL的PCD格式文件的输入输出头文件
#include <pcl/point_types.h> //PCL对各种格式的点的支持头文件
#include <pcl/visualization/cloud_viewer.h>//点云查看窗口头文件

using namespace Eigen;
using namespace std;


const double Pi =  3.141592;

// test success !!!
void compute_hori_angle(MatrixXd &velo, vector<double> &angle_v, vector<double> &angle_h, vector<int> &dangle_h, vector<int> &index){
    int ind_j = 0;
    double lastveh = 0;
    double ve = 0 , ve_h = 0;
    for(int i = 0; i < velo.rows(); i++){
        if(ind_j == 0){
            //lastveh = np.arctan2(i[1],-i[0])*180/np.pi
            lastveh = atan2(velo(i,1), -velo(i,0)) * 180 / Pi;
        }
        ind_j++;
        ve = atan2(velo(i,2), sqrt(pow(velo(i,0), 2) + pow(velo(i,1), 2))) * 180 / Pi;
        ve_h = atan2(velo(i,1), -velo(i,0)) * 180 / Pi;
        //ve = ((int)ve*1000) / 1000.0;
        angle_v.push_back(ve);
        angle_h.push_back(ve_h);
        lastveh = ve_h;
    }
    int ind_i = 0, ind_k = 0, maxS = 0, lastS = 0, backS = 0, selectCount = 0;
    bool select = true;
    for(int i = 0; i < angle_h.size(); i++){
        if(ind_i == 0){
            lastS = angle_h[i];
            backS = angle_h[i];
        }
        if(select && selectCount < 10){
            selectCount++;
        }
        else{
            select = false;
            selectCount = 0;
        }
        if(!select){
            if((angle_h[i] - lastS)>90 && angle_h[i] - backS >90){
                index.push_back(ind_i);
                ind_k++;
                maxS = angle_h[i];
                select = true;
                dangle_h.push_back(1);
            }
            else{
                dangle_h.push_back(0);
            }
        }
        backS = lastS;
        lastS = angle_h[i];
        ind_i++;
    }
    std::cout << "compute_hori_angle success！！！" << std::endl;
}


int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader reader;
    // 读入点云PCD文件
    reader.read("/home/jinxiao/YU/tutorails/pclShow/um_000000.pcd",*cloud);
    std::cerr << "Point cloud data: " << cloud->points.size () << " points" << std::endl;

    // 激光雷达数据 用MatrixXd存储
    long long lidarLens = cloud->points.size();
    int width = 3;
    MatrixXd matrix_lidar = MatrixXd::Random(lidarLens, width);
    for(int i = 0; i < lidarLens; i++){
        matrix_lidar(i,0) = cloud->points[i].x;
        matrix_lidar(i,1) = cloud->points[i].y;
        matrix_lidar(i,2) = cloud->points[i].z;
    }
    cout << "lidar row: " << matrix_lidar.rows() << "     cols: " << matrix_lidar.cols()<<endl;
    for(int i = 0; i < 10; i++){
        cout << matrix_lidar(i,0) << "  " << matrix_lidar(i,1) << " " << matrix_lidar(i,2) <<endl;
    }

    int lidarNums = 64;    //激光雷达线束64条
    int everylidarCount = 2190;

    MatrixXd lidar_image   = MatrixXd::Random(lidarNums, everylidarCount);
    MatrixXd lidar_image_x = MatrixXd::Random(lidarNums, everylidarCount);
    MatrixXd lidar_image_y = MatrixXd::Random(lidarNums, everylidarCount);
    MatrixXd lidar_image_z = MatrixXd::Random(lidarNums, everylidarCount);
    MatrixXd lidar_image_cor = MatrixXd::Random(lidarNums, everylidarCount);

    std::vector<std::vector<std::vector<double> > > lidar_image_all(64,vector<vector<double> >(2190,vector<double>(4,0)));

    cout << " the size of lidar_image_all: "       <<  lidar_image_all.size()       << endl;
    cout << " the size of lidar_image_all[0]: "    <<  lidar_image_all[0].size()    << endl;
    cout << " the size of lidar_image_all[0][0]: " <<  lidar_image_all[0][0].size() << endl;
    cout << " lidar_image_all[0][0][0]: "          <<  lidar_image_all[0][0][0]     << endl;
    cout << " lidar_image_all[63][2189][3]: "      <<  lidar_image_all[63][2189][3] << endl;

    // angle_v, angle_h, dangle_h, index = compute_hori_angle(s)
    vector<double> angle_v,  angle_h;
    vector<int> dangle_h, index;

    compute_hori_angle(matrix_lidar, angle_v, angle_h, dangle_h, index);    //测试通过


    int indAnleHold = 0;
    int countNot0 = 0;
    int indAnglePoint = 0;
    int lastHold = 0;


    for(int i = 0; i < index.size() - 1; i++){
        int lastAngle = 0;
        for(int j = index[i]; j < index[i+1]; j++){
            double dis = sqrt(pow(matrix_lidar(j,0), 2) + pow(matrix_lidar(index[i],1), 2) + pow(matrix_lidar(index[i],2),2) );
            double dis_x = matrix_lidar(i,0);
            double dis_y = matrix_lidar(j,1);
            double dis_z = matrix_lidar(j,2);

            int ind_cur_point = (int)round((180 - angle_h[j]) /0.1728);
            if(lidar_image(i, ind_cur_point) != 0){
                countNot0++;
            }

            if(fabs(lastAngle - angle_h[j]) > 0.1728*1.5){
                lidar_image(i, ind_cur_point)   = dis;
                lidar_image_x(i, ind_cur_point) = dis_x;
                lidar_image_y(i, ind_cur_point) = dis_y;
                lidar_image_z(i, ind_cur_point) = dis_z;
                indAnglePoint = j;
                lastHold = ind_cur_point;

                lidar_image_cor(i, ind_cur_point) = j;
                /*
                lidar_image_all[i][ind_cur_point][0] = matrix_lidar(j,0);
                lidar_image_all[i][ind_cur_point][1] = matrix_lidar(j,1);
                lidar_image_all[i][ind_cur_point][2] = matrix_lidar(j,2);
                lidar_image_all[i][ind_cur_point][3] = matrix_lidar(j,3);
                 */
            }

            else{
                lidar_image(i, j - indAnglePoint + lastHold)   = dis;
                lidar_image_x(i, j - indAnglePoint + lastHold) = dis_x;
                lidar_image_y(i, j - indAnglePoint + lastHold) = dis_y;
                lidar_image_z(i, j - indAnglePoint + lastHold) = dis_z;

                lidar_image_cor(i, j - indAnglePoint + lastHold) = j;
                /*
                lidar_image_all[i][j - indAnglePoint + lastHold][0] = matrix_lidar(j,0);
                lidar_image_all[i][j - indAnglePoint + lastHold][1] = matrix_lidar(j,1);
                lidar_image_all[i][j - indAnglePoint + lastHold][2] = matrix_lidar(j,2);
                lidar_image_all[i][j - indAnglePoint + lastHold][3] = matrix_lidar(j,3);
                 */

            }
            lastAngle = angle_h[j];
        }
    }

    for(int i = 0; i < lidarNums; i++){
        for(int j = 0; j < everylidarCount; j++){
            if(lidar_image_x(i,j) > 0.001){
                lidar_image_x(i,j) = 0;
            }
            else{
                lidar_image_x(i,j) = 1 / lidar_image_x(i,j);
            }
        }
    }


    for(int i = 0; i < lidarNums; i++){
        for(int j = 0; j < everylidarCount; j++){
           lidar_image(i,j) = lidar_image_x(i,j);
        }
    }

    return 0;
}




