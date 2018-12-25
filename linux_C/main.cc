#include <iostream>
#include <vector>
#include <math.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
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


void readCalibration(string filename, vector<double> &P0, vector<double> &velo2cam){
    string num[8][13];
    ifstream file(filename);
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 13; j++) {
            if(i == 4 && j >= 10){
                continue;
            }
            file >> num[i][j];
        }
    }
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 13; j++) {
            cout << num[i][j] << " ";
        }
        cout << endl;
    }
    for(int i = 1; i < 13; i++){
        double p0 =  (double) atof(num[0][i].c_str());
        P0.push_back(p0);
        double v1 = (double) atof(num[5][i].c_str());
        velo2cam.push_back(v1);
    }
    cout << "P0: " ;
    for(int i = 0; i < 12; i++){
        cout << P0[i] << " ";
    }
    cout << endl << "velo2cam: ";
    for(int i = 0; i < 12; i++){
        cout << velo2cam[i] << " ";
    }
    file.close();
}



void readPositioinPrior(string posiName, MatrixXd &posiMat){
    ifstream file(posiName);
    for (int i = 0; i < 64; i++) {
        for (int j = 0; j < 520; j++) {
            file >> posiMat(i,j);
        }
    }
    for (int i = 0; i < 64; i++) {
        for (int j = 0; j < 520; j++) {

            cout << posiMat(i,j) << " ";
        }
        cout << endl;
    }
    file.close();
}



Mat gray2pseudocolor(const Mat& scaledGray)
{
    Mat outputPseudocolor(scaledGray.size(), CV_8UC3);
    unsigned char grayValue;
    for (int y = 0; y < scaledGray.rows; y++)
        for (int x = 0; x < scaledGray.cols; x++)
        {
            grayValue = scaledGray.at<uchar>(y, x);
            Vec3b& pixel = outputPseudocolor.at<Vec3b>(y, x);
            pixel[0] = abs(255 - grayValue);
            pixel[1] = abs(127 - grayValue);
            pixel[2] = abs(0 - grayValue);
        }

    return outputPseudocolor;
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

    // lidar_all
    std::vector<std::vector<std::vector<double> > > lidar_image_all(64,vector<vector<double> >(2190,vector<double>(4,0)));
    MatrixXd lidar_all_x = MatrixXd::Zero(64, 2190);
    MatrixXd lidar_all_y = MatrixXd::Zero(64, 2190);
    MatrixXd lidar_all_z = MatrixXd::Zero(64, 2190);

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
                lidar_all_x(i, ind_cur_point) = matrix_lidar(j,0);
                lidar_all_y(i, ind_cur_point) = matrix_lidar(j,1);
                lidar_all_z(i, ind_cur_point) = matrix_lidar(j,2);
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
                lidar_all_x(i, j - indAnglePoint + lastHold) = matrix_lidar(j,0);  //test ok
                lidar_all_y(i, j - indAnglePoint + lastHold) = matrix_lidar(j,1);  //
                lidar_all_z(i, j - indAnglePoint + lastHold) = matrix_lidar(j,2);  //test ok
            }
            lastAngle = angle_h[j];
        }
    }
    // test lidar_imag:[0][2] =  42.918 ? 缺失
    for(int j = 0; j < 10; j++){
        cout << "lidar_image_cor:[i][j] " << j << ":     "<< lidar_image_cor(j,j) << endl;
    }
    for(int j = 0; j < 64; j++){
        cout << "lidar_image_y:[0][j] " << j << ":     "<< lidar_image_y(0,j) << endl;
    }

    // test lidar all [0][2] =  42.918 ? 缺失
    for(int i = 0; i < 64; i++){
        cout << "lidar_all_y: " << i << " : ";
        for(int j = 0; j < 2190; j++){
            cout << lidar_all_y(i,j) << " ";
        }
        cout << endl;
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
        lidar_image_cor3.col(i) = lidar_image_cor2.col(260 + i);
    }
    // test frontImage 数值不同
    for(int j = 0; j < 50; j++){
        cout << "frontImage: " << j << " : " << frontImage(0,j) << endl;
    }
    for(int j = 0; j < 50; j++){
        cout << "frontImage: " << j << " : " << frontImage(j,0) << endl;
    }

    // 完全获取激光xyzr数据
    MatrixXd lidar_xyzr_x = MatrixXd::Zero(64, 1040);
    MatrixXd lidar_xyzr_y = MatrixXd::Zero(64, 1040);
    MatrixXd lidar_xyzr_z = MatrixXd::Zero(64, 1040);
    for(int i = 0; i < 64; i++){
        for(int j = 0; j < 1040; j++){
            if(j < 520){
                lidar_xyzr_x(i,j) = lidar_all_x(i, j+1561);
                lidar_xyzr_y(i,j) = lidar_all_y(i, j+1561);
                lidar_xyzr_z(i,j) = lidar_all_z(i, j+1561);
            }
            else{
                lidar_xyzr_x(i,j) = lidar_all_x(i, j-520);
                lidar_xyzr_y(i,j) = lidar_all_y(i, j-520);
                lidar_xyzr_z(i,j) = lidar_all_z(i, j-520);
            }
        }
    }

    cout << "lidar_xyz_y" << " " << endl << lidar_xyzr_y << endl; //test ok
    // lidar_temp
    MatrixXd lidar_temp_x = MatrixXd::Zero(64, 1040);
    MatrixXd lidar_temp_y = MatrixXd::Zero(64, 1040);
    MatrixXd lidar_temp_z = MatrixXd::Zero(64, 1040);
    for(int i = 0; i < 64; i++){
        for(int j = 0; j < 1040; j++){
            lidar_temp_x(i,j) = lidar_xyzr_x(i, 1040-j-1);
            lidar_temp_y(i,j) = lidar_xyzr_y(i, 1040-j-1);
            lidar_temp_z(i,j) = lidar_xyzr_z(i, 1040-j-1);
        }
    }
    //cout << "lidar_temp: " << endl;
   // cout << lidar_temp_y<< endl;    // test ok
    //frontLidar
    MatrixXd frontLidar_x = MatrixXd::Zero(64, 520);
    MatrixXd frontLidar_y = MatrixXd::Zero(64, 520);
    MatrixXd frontLidar_z = MatrixXd::Zero(64, 520);
    for(int i = 0; i < 64; i++){
        for(int j = 0; j < 520; j++){
            frontLidar_x(i,j) = lidar_temp_x(i,j+260);
            frontLidar_y(i,j) = lidar_temp_y(i,j+260);
            frontLidar_z(i,j) = lidar_temp_z(i,j+260);
        }
    }

    cout << "frontLidar: "  << endl;
    cout << frontLidar_y << endl;


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


    //显示分割结果
    Mat colorImg(64, 520, CV_8UC3, Scalar(255, 255, 255));

    for(int i = 0; i < colorImg.rows; i++){
        for(int j = 0; j < colorImg.cols; j++){
            Vec3b pixel;
            if(roadSegment(i,j) == 3){
                pixel[0] = 255;
                pixel[1] = 0;
                pixel[2] = 255;
            }
            else{
                pixel[0] = 255;
                pixel[1] = 0;
                pixel[2] = 0;
            }
            colorImg.at<Vec3b>(i,j) = pixel;
        }
    }


    imwrite("/home/jinxiao/Pictures/colorImg.png", colorImg);
    Mat meanFilImg;
    medianBlur(colorImg,meanFilImg,3);
    imshow("road segmentation", meanFilImg);
    imwrite("/home/jinxiao/Pictures/medianFilter.png", meanFilImg);
    waitKey(0);



    // 计算属于道路的概率 roadProp
    const int minDist = 0;
    const int maxDist = 16;
    MatrixXd roadProp = MatrixXd::Zero(64, 520);

    for(int i = 0; i < 64; i++){
        for(int j = 0; j < 520; j++){
            int light =  (int) (frontImage(i,j)*1000);
            if(light == 0) continue;
            double dist = abs(m*light - i + b)/pow(((-1)*(-1) + m*m),0.5);
            if(dist > maxDist){
                roadProp(i,j) = 0;
                continue;
            }
            roadProp(i,j) = 1 - (dist / (maxDist - minDist));
        }
    }

    for(int i = 0; i < 64; i++){
        cout << "line " << i << " : ";
        for(int j = 0; j< 520; j++){
            cout << roadProp(i,j) << " ";
        }
        cout << endl;
    }


    Mat imgRoad(64 ,520, CV_8UC1);
    uchar *ptmp = NULL;
    for (int i = 0; i < 64; ++i)
    {
        ptmp = imgRoad.ptr<uchar>(i);

        for (int j = 0; j < 520; ++j)
        {
            ptmp[j] = roadProp(i, j) * 255;
        }
    }
    imshow("gray.jpg", imgRoad);
    imwrite("gray.jpg", imgRoad);
    waitKey(0);

    Mat pseudoImg = gray2pseudocolor(imgRoad);
    imshow("jet.jpg", pseudoImg);
    imwrite("jet.jpg", pseudoImg);
    waitKey(0);



    // 提取法线方向
    Vector3d upVec(0,0,1);

    cout << "upVec: " << upVec << endl;

    Vector3d norVec;

    MatrixXd normalVectorProp = MatrixXd::Zero(64, 520);
    MatrixXd block_x(4,4);
    MatrixXd block_y(4,4);
    MatrixXd block_z(4,4);
    MatrixXd newBox = MatrixXd::Zero(16,3);


    for(int i = 0; i < 64; i += 4){
        for(int j = 0; j < 520; j += 4){
            block_x = frontLidar_x.block(i,j,4,4);
            block_y = frontLidar_y.block(i,j,4,4);
            block_z = frontLidar_z.block(i,j,4,4);
            //cout << "i j"<< i << j << " : " << block_y <<endl;

            block_x.resize(16,1);
            block_y.resize(16,1);
            block_z.resize(16,1);



            for(int k = 0; k < 16; k++){
                newBox(k,0) = block_x(k);
                newBox(k,1) = block_y(k);
                newBox(k,2) = block_z(k);
            }
            //cout << "newBox: " << endl <<  newBox << endl;

            // 求协方差
            Eigen::MatrixXd meanVec = newBox.colwise().mean();
            Eigen::RowVectorXd meanVecRow(Eigen::RowVectorXd::Map(meanVec.data(),newBox.cols()));

            Eigen::MatrixXd zeroMeanMat = newBox;
            zeroMeanMat.rowwise()-=meanVecRow;
            Eigen::MatrixXd covMat;

            covMat = (zeroMeanMat.adjoint()*zeroMeanMat)/double(newBox.rows()-1);
            //cout << "covMat :" << endl << covMat << endl;



            // 最小特征值及其对应的特征向量
            EigenSolver<Matrix3d> es(covMat);
            MatrixXd D = es.pseudoEigenvalueMatrix();
            MatrixXd V = es.pseudoEigenvectors();
            //cout << V <<endl;
            //cout << D <<endl;

            double min_eigen_value = 65535.0;
            int ind_minEig = 0;
            for(int k = 2; k > 0;k--){
                if(D(k,k) < min_eigen_value){
                    min_eigen_value = D(k,k);
                    ind_minEig = k;
                }

            }

            norVec = V.col(ind_minEig);

            //cout << "vec: " << endl << norVec << endl;

            //求向量之间夹角
            double Lx = norVec.dot(norVec);
            double Ly = upVec.dot(upVec);
            double cos_angle = norVec.dot(upVec) / (Lx*Ly);
            double angle = acos(cos_angle);
            double angle2 = angle * 180 / Pi;

            cout << "Lx: "<< Lx << endl;
            cout << "Ly: "<< Ly << endl;
            cout << "cos_angle: " << cos_angle << endl;
            cout << "angle: " << angle << endl;
            cout << "angle2: " << angle2 << endl;
        }
    }



    //求均值



    //位置先验知识
    MatrixXd posiMat = MatrixXd::Zero(64, 520);
    string posiName  = "positionPrior.txt";
    readPositioinPrior(posiName, posiMat);


    //概率融合



    //投影到图像中
    //read callibration
    string filename = "um_000000.txt";
    vector<double> P;
    vector<double> velo2cam;
    readCalibration(filename, P, velo2cam);
    MatrixXd P0 = MatrixXd::Zero(4,4);
    MatrixXd Tr = MatrixXd::Zero(4,4);
    for(int i = 0 ; i < P.size(); i++){
        if(i < 4){
            P0(0,i)= P[i];
            Tr(0,i) = velo2cam[i];
        }
        else if(i < 8){
            P0(1,i%4) = P[i];
            Tr(1,i%4) = velo2cam[i];
        }
        else{
            P0(2,i%4) = P[i];
            Tr(2,i%4) = velo2cam[i];
        }
    }
    P0(3,3) = 1;
    Tr(3,3) = 1;

    //cout<<endl<< "P0: " << P0 << endl;
    //cout << "Tr: " << Tr <<endl;

    //read Img
    string imageName;
    imageName = "um_000000.png";
    Mat imgCamera = imread(imageName,1);

    //cout << "imgCamera.rows" << imgCamera.rows << "imgCamera.cols" << imgCamera.cols << endl;

    // 投影
    MatrixXd cPointCloud = MatrixXd::Zero(matrix_lidar.rows(), matrix_lidar.cols());
    int countOutPoint = 0;
    MatrixXd point(4,1);
    MatrixXd  newPoint(4,1);
    MatrixXd tmp(4,1);
    vector <int> rowIndex, colIndex;
    vector <double> propIndex;
    //cout<< "lidar_image_cor3.rows(): "  << lidar_image_cor3.rows() <<endl;
    //cout <<"lidar_image_cor3.cols(): " << lidar_image_cor3.cols() <<endl;

   // cout << lidar_image_cor3 << endl;

    for(int idx = 0; idx < lidar_image_cor3.rows(); idx++){
        for(int jdx = 0; jdx < lidar_image_cor3.cols(); jdx++){
            for(int k = 0; k < 3; k++){
                point(k)= matrix_lidar((int) lidar_image_cor3(idx, jdx), k);
            }
            //cout << point << endl;
            point(3,0) = 1;
            //cout <<"point: " << point <<endl;
            tmp = Tr*point;
            newPoint =  P0*(Tr*point);
            if(tmp(2,0) < 0){
                continue;
            }
            newPoint = newPoint/newPoint(2,0);

            if(newPoint(1,0) > 0 && (newPoint(1,0) < imgCamera.rows-1) && newPoint(0,0)>0 && (newPoint(0,0)<imgCamera.cols-1) ){
                rowIndex.push_back((int)newPoint(0,0));
                colIndex.push_back((int)newPoint(1,0));
                propIndex.push_back((roadProp(idx, jdx) + posiMat(idx, jdx))/2);

            }
        }
    }
   // cout <<"newPoint: " << point << endl << point.transpose() <<endl << Tr*point << endl << P0*(Tr*point);
    //cout << "newPoint:  " <<  newPoint<<endl;
   // cout << " rowIndex.size()" << rowIndex.size() << endl;
   // for(int k = 0; k < rowIndex.size(); k++){
       // cout << rowIndex[k] << " ";
   // }

    cv::Point cvPoint;//特征点，用以画在图像中
    for(int i = 0; i < rowIndex.size(); i++){
        cvPoint.x = rowIndex[i];//特征点在图像中横坐标
        cvPoint.y = colIndex[i];//特征点在图像中纵坐标
        double color0 = abs(255 - 255*propIndex[i]);
        double color1 = abs(127 - 255*propIndex[i]);
        double color2 = abs(0 - 255*propIndex[i]);
        cv::circle(imgCamera, cvPoint, 1, cv::Scalar(color0,color1,color2), -1);  //在图像中画出特征点，2是圆的半径 cv::Scalar(255*propIndex[i],0,0)
    }


    imshow("image", imgCamera);
    waitKey(0);
    imwrite("calibration.png", imgCamera);

    return 0;
}




