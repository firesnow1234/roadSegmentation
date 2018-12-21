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
#include <opencv2/opencv.hpp>
#include "opencv/highgui.h"


using namespace Eigen;
using namespace std;
using namespace cv;


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
    index.push_back(0);
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

//生成[0,1]之间符合均匀分布的数
double uniformRandom(void)
{
    return (double)rand() / (double)RAND_MAX;
}

//根据点集拟合直线ax+by+c=0，res为残差
void calcLinePara(vector<Point2d> pts, double &a, double &b, double &c, double &res)
{
    res = 0;
    Vec4f line;
    vector<Point2f> ptsF;
    for (unsigned int i = 0; i < pts.size(); i++)
        ptsF.push_back(pts[i]);

    fitLine(ptsF, line, CV_DIST_L2, 0, 1e-2, 1e-2);
    a = line[1];
    b = -line[0];
    c = line[0] * line[3] - line[1] * line[2];

    for (unsigned int i = 0; i < pts.size(); i++)
    {
        double resid_ = fabs(pts[i].x * a + pts[i].y * b + c);
        res += resid_;
    }
    res /= pts.size();
}

//得到直线拟合样本，即在直线采样点集上随机选2个点
bool getSample(vector<int> set, vector<int> &sset)
{
    int i[2];
    if (set.size() > 2)
    {
        do
        {
            for (int n = 0; n < 2; n++)
                i[n] = int(uniformRandom() * (set.size() - 1));
        } while (!(i[1] != i[0]));
        for (int n = 0; n < 2; n++)
        {
            sset.push_back(i[n]);
        }
    }
    else
    {
        return false;
    }
    return true;
}

//直线样本中两随机点位置不能太近
bool verifyComposition(const vector<Point2d> pts)
{
    cv::Point2d pt1 = pts[0];
    cv::Point2d pt2 = pts[1];
    if (abs(pt1.x - pt2.x) < 5 && abs(pt1.y - pt2.y) < 5)
        return false;

    return true;
}

//RANSAC直线拟合
void fitLineRANSAC(vector<Point2d> ptSet, double &a, double &b, double &c, vector<bool> &inlierFlag)
{
    double residual_error = 2.99; //内点阈值

    bool stop_loop = false;
    int maximum = 0;  //最大内点数

    //最终内点标识及其残差
    inlierFlag = vector<bool>(ptSet.size(), false);
    vector<double> resids_(ptSet.size(), 3);
    int sample_count = 0;
    int N = 500;

    double res = 0;

    // RANSAC
    srand((unsigned int)time(NULL)); //设置随机数种子
    vector<int> ptsID;
    for (unsigned int i = 0; i < ptSet.size(); i++)
        ptsID.push_back(i);
    while (N > sample_count && !stop_loop)
    {
        vector<bool> inlierstemp;
        vector<double> residualstemp;
        vector<int> ptss;
        int inlier_count = 0;
        if (!getSample(ptsID, ptss))
        {
            stop_loop = true;
            continue;
        }

        vector<Point2d> pt_sam;
        pt_sam.push_back(ptSet[ptss[0]]);
        pt_sam.push_back(ptSet[ptss[1]]);

        if (!verifyComposition(pt_sam))
        {
            ++sample_count;
            continue;
        }

        // 计算直线方程
        calcLinePara(pt_sam, a, b, c, res);
        //内点检验
        for (unsigned int i = 0; i < ptSet.size(); i++)
        {
            Point2d pt = ptSet[i];
            double resid_ = fabs(pt.x * a + pt.y * b + c);
            residualstemp.push_back(resid_);
            inlierstemp.push_back(false);
            if (resid_ < residual_error)
            {
                ++inlier_count;
                inlierstemp[i] = true;
            }
        }
        // 找到最佳拟合直线
        if (inlier_count >= maximum)
        {
            maximum = inlier_count;
            resids_ = residualstemp;
            inlierFlag = inlierstemp;
        }
        // 更新RANSAC迭代次数，以及内点概率
        if (inlier_count == 0)
        {
            N = 500;
        }
        else
        {
            double epsilon = 1.0 - double(inlier_count) / (double)ptSet.size(); //野值点比例
            double p = 0.99; //所有样本中存在1个好样本的概率
            double s = 2.0;
            N = int(log(1.0 - p) / log(1.0 - pow((1.0 - epsilon), s)));
        }
        ++sample_count;
    }

    //利用所有内点重新拟合直线
    vector<Point2d> pset;
    for (unsigned int i = 0; i < ptSet.size(); i++)
    {
        if (inlierFlag[i])
            pset.push_back(ptSet[i]);
    }

    calcLinePara(pset, a, b, c, res);
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
    cout << "matrix_lidar data:" <<endl;
    for(int i = 0; i < 10; i++){
        cout << matrix_lidar(i,0) << " " << matrix_lidar(i,1) << " " << matrix_lidar(i,2) << endl;
    }

    int lidarNums = 64;    //激光雷达线束64条
    int everylidarCount = 2190;  //激光一圈2190个点

    MatrixXd lidar_image   = MatrixXd::Zero(lidarNums, everylidarCount);
    MatrixXd lidar_image_x = MatrixXd::Zero(lidarNums, everylidarCount);
    MatrixXd lidar_image_y = MatrixXd::Zero(lidarNums, everylidarCount);
    MatrixXd lidar_image_z = MatrixXd::Zero(lidarNums, everylidarCount);
    MatrixXd lidar_image_cor = MatrixXd::Zero(lidarNums, everylidarCount);

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
    cout << "size of angel_v: " << angle_v.size() << endl;
    cout << "size of angel_h: " << angle_h.size() << endl;
    cout << "size of dangel_h: " << dangle_h.size() << endl;
    cout << "size of index: " << index.size() << endl;

    int countNot0 = 0;
    int indAnglePoint = 0;
    int lastHold = 0;


    for(int i = 0; i < index.size()-1; i++){
        int lastAngle = 0;
        for(int j = index[i]; j < index[i+1]; j++){
            double dis = sqrt(pow(matrix_lidar(j,0), 2) + pow(matrix_lidar(index[i],1), 2) + pow(matrix_lidar(index[i],2),2) );
            double dis_x = matrix_lidar(j,0);
            double dis_y = matrix_lidar(j,1);
            double dis_z = matrix_lidar(j,2);

            int ind_cur_point = (int)round((180 - angle_h[j]) / 0.1728);
            if(lidar_image(i, ind_cur_point)!=0){
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

                lidar_image_cor(i, j - indAnglePoint + lastHold) = j;   //test success !
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
    // test lidar_imag:[0][2] =  42.918 ? 缺失
    for(int j = 0; j < 10; j++){
        cout << "lidar_image_cor:[i][j] " << j << ":     "<< lidar_image_cor(j,j) << endl;
    }
    for(int j = 0; j < 64; j++){
        cout << "lidar_image_x:[0][j] " << j << ":     "<< lidar_image_x(0,j) << endl;
    }


    for(int i = 0; i < lidarNums; i++){
        for(int j = 0; j < everylidarCount; j++){
            if(lidar_image_x(i,j) < 0.001){
                lidar_image_x(i,j) = 0;
            }
            else{
                lidar_image_x(i,j) = 1.0 / lidar_image_x(i,j);
            }
        }
    }
    // test success lidar_image_x
    for(int j = 0; j < 64; j++){
        cout << "lidar_image_x:[i][j] " << j << ":     "<< lidar_image_x(j,j) << endl;
    }


    for(int i = 0; i < lidarNums; i++){
        for(int j = 0; j < everylidarCount; j++){
           lidar_image(i,j) = lidar_image_x(i,j);
        }
    }
    // test lidar_image success
    for(int j = 0; j < 50; j++){
        cout << "lidar_image: " << j << " : " << lidar_image(0,j) << endl;
    }


    MatrixXd ss = MatrixXd::Zero(64, 1040);
    MatrixXd front_cor = MatrixXd::Zero(64, 1040);
    for(int i = 0; i < 1040; i++){
        if(i < 520){
            ss.col(i) = lidar_image.col(1561 + i);
            front_cor.col(i) = lidar_image_cor.col(1561 + i);
        }
        else{
            ss.col(i) = lidar_image.col(i - 520);
            front_cor.col(i) = lidar_image_cor.col(i - 520);
        }
    }

    // test ss 前三列差一列
    for(int j = 0; j < 50; j++){
        cout << "ss: " << j << " : " << ss(0,j) << endl;
    }
    for(int j = 0; j < 50; j++){
        cout << "ss: " << j << " : " << ss(j,0) << endl;
    }

    // test front_cor  有点错乱
    for(int j = 0; j < 50; j++){
        cout << "front_cor: " << j << " : " << front_cor(0,j) << endl;
    }
    for(int j = 0; j < 50; j++){
        cout << "front_cor: " << j << " : " << front_cor(j,0) << endl;
    }

    MatrixXd lidar_image_cor2 = MatrixXd::Zero(64, 1040);
    MatrixXd sss = MatrixXd::Zero(64, 1040);
    int ss_shape1 = ss.cols();  //1040
    for(int i = 0; i < ss_shape1; i++){
        for(int j = 0; j < ss.rows(); j++){    //64
            sss(j, i) = ss(j, ss_shape1 - i - 1);
            lidar_image_cor2(j, i ) = front_cor(j, ss_shape1 - i - 1);
        }
    }

    MatrixXd frontImage = MatrixXd::Zero(64, 520);
    MatrixXd lidar_image_cor3 = MatrixXd::Zero(64, 520);
    for(int i = 0; i < 520; i++){
        frontImage.col(i) = sss.col(260 + i);
        lidar_image_cor3.col(i) = sss.col(260 + i);
    }
    // test frontImage 数值不同
    for(int j = 0; j < 50; j++){
        cout << "frontImage: " << j << " : " << frontImage(0,j) << endl;
    }
    for(int j = 0; j < 50; j++){
        cout << "frontImage: " << j << " : " << frontImage(j,0) << endl;
    }

    // 完全获取激光xyzr数据


    // 直方图
    MatrixXd histogram = MatrixXd::Zero(64, 1000);
    for(int i = 0; i < 64; i++){
        for(int j = 0; j < 520; j++){
            int indhis = (int) (frontImage(i,j) * 1000);
            histogram(i, indhis) = histogram(i, indhis) + 1;
        }
    }
    // test histogram

    for(int i = 0; i < 64; i++){
        cout << "histogram line " << i << " : ";
        for(int j = 0; j < 520; j++){
            cout << histogram(i,j)<< " ";
        }
        cout << endl;
    }


    // imshow histogram
    /*
    int width_hist = 520;
    int height_hist = 64;
    vector<int> data;
    for(int i = 0; i< 64; i++){
        for(int j = 0; j< 520; j++){
            data.push_back(histogram(i,j));
        }
    }


    IplImage* img = cvCreateImage(cvSize(width_hist, height_hist), 8, 1);
    for(int j = 0; j < width_hist*height_hist; ++j) {
        img->imageData[j] = data[j];
    }

    cvNamedWindow( "Image", 2 );//创建窗口
    cvShowImage( "Image", img );//显示图像

    cvWaitKey(0); //等待按键
    cvDestroyWindow( "Image" );//销毁窗口
    cvReleaseImage( &img ); //释放图像
     */


    // ransac 拟合
    vector<Point2d> ptSet;
    for(int i = 20; i < 64; i++){
        for(int j = 1; j < 520; j++) {
            if (histogram(i, j) > 25) {
                Point2d pt(j, i);
                ptSet.push_back(pt);
            }
        }
    }
    //绘制点集中所有点
    Mat img(64, 520, CV_8UC3, Scalar(255, 255, 255));
    for (unsigned int i = 0; i < ptSet.size(); i++){
        circle(img, ptSet[i], 0.5, Scalar(255, 0, 0), 3, 8);
    }
    double A, B, C;
    vector<bool> inliers;
    fitLineRANSAC(ptSet, A, B, C, inliers);     // 调用ransac

    B = B / A;
    C = C / A;
    A = A / A;

    //绘制直线
    Point2d ptStart, ptEnd;
    ptStart.x = 0;
    ptStart.y = -(A*ptStart.x + C) / B;
    ptEnd.x = 250;
    ptEnd.y = -(A*ptEnd.x + C) / B;
    line(img, ptStart, ptEnd, Scalar(0, 0, 255), 0.3, CV_AA);
    cout << "A:" << A << " " << "B:" << B << " " << "C:" << C << " " << endl;

    imshow("line fitting", img);
    imwrite("/home/jinxiao/Pictures/ransac2.png", img);
    waitKey();


    // 路面分割的超参数
    const double alpha = 0.5;
    const double beta = 1.2;
    MatrixXd roadSegment = MatrixXd::Zero(64, 520);
    double m = -A/B;
    double b = -C/B;
    // i 激光扫描线  j 激光雷达获取图中的前方视野点
    for(int i = 0; i < 64; i++){
        for(int j = 0; j < 520; j++){
            int light = (int) (frontImage(i,j) * 1000);
            // water
            if(frontImage(i,j) == 0 && i <= m*light + beta*b){
                roadSegment(i,j) = 0;
            }
            //positive obstacle
            else if( i < m*light + alpha*b){
                roadSegment(i,j) = 1;
            }
            //negative
            else if(i > m*light + beta*b){
                roadSegment(i,j) = 2;
            }
            // road line
            else if(i >= m*light + alpha*b && i <= m*light+ beta*b){
                roadSegment(i,j) = 3;
            }
        }
    }



    return 0;
}




