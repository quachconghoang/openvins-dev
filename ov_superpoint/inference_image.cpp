#include <memory>
#include <chrono>
#include <numeric>

#include "utils.h"
#include "super_glue.h"
#include "super_point.h"
#include "Hungarian.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core.hpp>

using namespace cv;
using namespace std;

int test_sinkhorn();

Mat sinkhorn_knopp(const Mat & a, const Mat & b, const Mat & M, double reg = 1e-1, int maxIter= 20)
{
    int dim_a = a.rows;
    int dim_b = b.rows;
    Mat u = Mat::ones(dim_a,1,CV_64FC1)/dim_a;
    Mat v = Mat::ones(dim_b,1,CV_64FC1)/dim_b;

    Mat K, KtransposeU, uprev, vprev;
    cv::exp(M.clone()/(-reg),K);

    Mat Kp = Mat::zeros(dim_a,dim_b, CV_64FC1);

    for (int i = 0; i < dim_a; i++){
        Kp.row(i) = K.row(i).mul(1/ a.at<double>(i,0)); // ROW multiple
    }

    for (int iter=0; iter<20; iter++){
        uprev = u.clone();
        vprev = v.clone();
        KtransposeU = K.t() * u;
        v = b / KtransposeU;
        u = 1. /(Kp * v);
    }

    Mat Rs = Mat::zeros(dim_a,dim_b, CV_64FC1);
    for (int i = 0; i < dim_a; i++){
        for (int j = 0; j < dim_b; j++){
            double scale = u.at<double>(i,0)*v.at<double>(j,0);
            Rs.at<double>(i,j) = K.at<double>(i,j)* scale;
        }
    }

    return Rs;
}

vector <vector<double>> mat2d_to_vec(Mat & m)
{
    vector <vector<double>> rs = vector<vector<double> >(m.rows, vector<double>(m.cols));
    for(int i=0; i< m.rows;i ++){
        for(int j=0; j< m.cols; j++){
            rs[i][j] = m.at<double>(i,j);
        }
    }
    return rs;
}

int test_sinkhorn()
{
    double reg = 1e-1;
    int maxIter = 20;
    Mat a,b,M;

    cv::FileStorage fs("/home/hoangqc/Datasets/Weights/sinkhorn_debug.yaml", cv::FileStorage::READ);
    fs["a"] >> a;
    fs["b"] >> b;
    fs["M"] >> M;

    int dim_a = a.rows;
    int dim_b = b.rows;
    Mat u = Mat::ones(dim_a,1,CV_64FC1)/dim_a;
    Mat v = Mat::ones(dim_b,1,CV_64FC1)/dim_b;

    Mat K, KtransposeU, uprev, vprev;
    cv::exp(M/(-reg),K);

    Mat Kp = Mat::zeros(dim_a,dim_b, CV_64FC1);

    for (int i = 0; i < dim_a; i++){
        Kp.row(i) = K.row(i).mul(1/ a.at<double>(i,0)); // ROW multiple
    }

    for (int iter=0; iter<maxIter; iter++)
    {
        uprev = u.clone();
        vprev = v.clone();
        KtransposeU = K.t() * u;
        v = b / KtransposeU;
        u = 1. / (Kp * v);
    }

    Mat Rs = Mat::zeros(dim_a,dim_b, CV_64FC1);
    for (int i = 0; i < dim_a; i++){
        for (int j = 0; j < dim_b; j++){
            Rs.at<double>(i,j) = K.at<double>(i,j)* u.at<double>(i,0)*v.at<double>(j,0);
        }
    }
    return 0;
}

//int main()
//{
//    test_sinkhorn();
//}

int main()
{
    std::string folder = "/home/hoangqc/Datasets/TartanAir/office/Easy/P004/";
    std::string config_path = "/home/hoangqc/Datasets/Weights/config.yaml";
    std::string model_dir = "/home/hoangqc/Datasets/Weights/";

    Configs configs(config_path, model_dir);

    int step = 5;
    cv::Mat img_l, img_r, image0, image1;
    std::vector<std::string> image_left, image_right;
    GetFileNames(folder+"image_left", image_left);
    GetFileNames(folder+"image_right", image_right);
    image0 = cv::imread(image_left[160], IMREAD_GRAYSCALE);
    image1 = cv::imread(image_left[180], IMREAD_GRAYSCALE);

    std::cout << "Building inference engine......" << std::endl;
    auto superpoint = std::make_shared<SuperPoint>(configs.superpoint_config);
    std::cout << configs.superpoint_config.onnx_file << std::endl << configs.superglue_config.onnx_file << std::endl;
    if (!superpoint->build()){
        std::cerr << "Error in SuperPoint building engine. Please check your onnx model path." << std::endl;
        return 0;
    }

    auto superglue = std::make_shared<SuperGlue>(configs.superglue_config);
    if (!superglue->build()){
        std::cerr << "Error in SuperGlue building engine. Please check your onnx model path." << std::endl;
        return 0;
    }

    std::cout << "SuperPoint and SuperGlue inference engine build success." << std::endl;
    Eigen::Matrix<double, 259, Eigen::Dynamic> feature_points0, feature_points1;

    superpoint->infer(image0, feature_points0);
    superpoint->infer(image1, feature_points1);

    std::vector<cv::KeyPoint> keypoints0, keypoints1;
    for(size_t i = 0; i < feature_points0.cols(); ++i){
        double score = feature_points0(0, i);
        double x = feature_points0(1, i);
        double y = feature_points0(2, i);
        keypoints0.emplace_back(x, y, 8, -1, score);
    }
    for(size_t i = 0; i < feature_points1.cols(); ++i){
        double score = feature_points1(0, i);
        double x = feature_points1(1, i);
        double y = feature_points1(2, i);
        keypoints1.emplace_back(x, y, 8, -1, score);
    }

    Eigen::Matrix<double,Eigen::Dynamic, 256> fp0 = feature_points0.block(3, 0, 256, feature_points0.cols()).transpose();
    Eigen::Matrix<double,Eigen::Dynamic, 256> fp1 = feature_points1.block(3, 0, 256, feature_points1.cols()).transpose();
//    cout << fp0.rows() << fp0.cols();

    cv::Mat desc0, desc1;
    cv::eigen2cv(fp0, desc0);
    cv::eigen2cv(fp1, desc1);


    int num_src = feature_points0.cols();
    int num_tar = feature_points1.cols();
    Mat norm_self_0 = Mat::zeros(num_src, num_src, CV_64FC1);
    Mat norm_self_1 = Mat::zeros(num_tar, num_tar, CV_64FC1);
    Mat norm_cross = Mat::zeros(num_src, num_tar, CV_64FC1);

    for(int i=0; i<num_src; i++){
        for(int j=0; j<num_src; j++){
            norm_self_0.at<double>(i,j) = cv::norm(desc0.row(i),desc0.row(j),NORM_L2);
        }}

    for(int i=0; i<num_tar; i++){
        for(int j=0; j<num_tar; j++){
            norm_self_1.at<double>(i,j) =cv::norm(desc1.row(i),desc1.row(j),NORM_L2);
        }}

//    vector<vector<double>> cost_matrix = vector<vector<double> >(num_src, vector<double>(num_tar));
//    cost_matrix[i][j] = rs;

    for(int i=0; i<num_src; i++){
        for(int j=0; j<num_tar; j++){
            double rs = cv::norm(desc0.row(i),desc1.row(j),NORM_L2);
            norm_cross.at<double>(i,j) = rs;
        }
    }

//    std::vector<double> his_src(num_src), his_tar(num_tar);
    Mat his_src = Mat::zeros(num_src,1, CV_64FC1);
    Mat his_tar = Mat::zeros(num_tar,1, CV_64FC1);

    for(int i=0; i<num_src; i++){
        his_src.at<double>(i,0) = cv::mean(norm_self_0.row(i))[0] - 1;
    }
    for(int j=0; j<num_tar; j++){
        his_tar.at<double>(j,0) = cv::mean(norm_self_1.row(j))[0] - 1;
    }
/* --- Sinkhorn --- */
    Mat Gs = sinkhorn_knopp(his_src,his_tar,norm_cross);
    double min_val, max_val;
    cv::minMaxLoc(Gs, &min_val, &max_val);
    Mat costMat = max_val-Gs;
    vector<vector<double>> cost_matrix = mat2d_to_vec(costMat);
/* ---------------- */

    //    norm_cross = cv::Mat(num_src,num_tar,CV_64FC1, cost_matrix.data());
    //    superglue->matching_points(feature_points0, feature_points1, superglue_matches);

    HungarianAlgorithm HungAlgo;
    vector<int> assignment;
    double cost = HungAlgo.Solve(cost_matrix, assignment);
    cout << "Cost = " << cost << endl;
    std::vector<cv::DMatch> hungarian_matches;



    for (int i=0; i < assignment.size(); i++){
        int tar_id = assignment[i];
        double distance = abs(costMat.at<double>(i,tar_id));
//        cout << distance << endl;
        if((tar_id != -1) && (distance < (max_val*0.5))){
            hungarian_matches.push_back(DMatch(i,tar_id,distance));
            cout << i << " - " << tar_id << " : " << distance << endl;
        }
    }

    cv::Mat match_image;
    cv::drawMatches(image0, keypoints0, image1, keypoints1, hungarian_matches, match_image);
    cv::imshow("match", match_image);
    cv::waitKey();

/* Task list:
 * - Major: Implement superpoint - sinkhorn - hungarian
 * - Minor: Implenent PnP-solver ? - Stereo
 * */
//    cv::destroyAllWindows();
    return 0;
}