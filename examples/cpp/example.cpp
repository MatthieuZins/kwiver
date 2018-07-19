#include <iostream>
#include <fstream>
#include <iomanip>

#include "vital/plugin_loader/plugin_manager.h"
#include "vital/algo/image_io.h"
#include "vital/algo/blur_image.h"
#include "arrows/ocv/image_container.h"
#include "arrows/vxl/image_container.h"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include <vil/algo/vil_gauss_filter.h>

#include <vital/types/camera_perspective.h>
#include <vital/types/camera.h>
#include <vital/types/camera_intrinsics.h>
#include <vital/types/landmark.h>

#include <vital/algo/compute_depth.h>

#include "arrows/ocv/detect_features_AGAST.h"
#include "arrows/ocv/feature_detect_extract_SIFT.h"
#include "arrows/ocv/feature_detect_extract_ORB.h"
#include "arrows/ocv/feature_detect_extract_SURF.h"
#include "arrows/ocv/feature_detect_extract_BRISK.h"

#include <vital/algo/match_features.h>
#include "arrows/ocv/match_set.h"
#include <vital/algo/estimate_fundamental_matrix.h>

void test()
{
    kwiver::vital::plugin_manager::instance().load_all_plugins();

    std::cout << "losc" << std::endl;
    kwiver::vital::algo::image_io_sptr ocv_io = kwiver::vital::algo::image_io::create("ocv");
    kwiver::vital::algo::image_io_sptr vxl_io = kwiver::vital::algo::image_io::create("vxl");

    kwiver::vital::image_container_sptr ocv_img = ocv_io->load("/home/matthieu/Lib/kwiver/examples/images/cat.jpg");
    kwiver::vital::image_container_sptr vxl_img = vxl_io->load("/home/matthieu/Lib/kwiver/examples/images/cat.jpg");


    kwiver::vital::algo::blur_image_sptr ocv_blur = kwiver::vital::algo::blur_image::create("ocv");
    kwiver::vital::algo::blur_image_sptr vxl_blur = kwiver::vital::algo::blur_image::create("vxl");

    cv::Mat mat = kwiver::arrows::ocv::image_container_to_ocv_matrix(*ocv_img.get(), kwiver::arrows::ocv::image_container::RGB_COLOR);
    cv::namedWindow("window");
    cv::imshow("window", mat);
    cv::waitKey(0);


    kwiver::vital::image_container_sptr blurred = ocv_blur->blur(vxl_img);
    cv::Mat filtered;
    filtered = kwiver::arrows::ocv::image_container_to_ocv_matrix(*blurred.get(), kwiver::arrows::ocv::image_container::RGB_COLOR);
    cv::imshow("window", filtered);
    cv::waitKey(0);


    kwiver::vital::image_container_sptr blurred_vxl = vxl_blur->blur(vxl_img);
    filtered = kwiver::arrows::ocv::image_container::vital_to_ocv(blurred_vxl->get_image(), kwiver::arrows::ocv::image_container::RGB_COLOR);
    cv::imshow("window", filtered);
    cv::waitKey(0);

    cv::destroyAllWindows();


//    vil_image_view<vxl_byte> image = kwiver::arrows::vxl::image_container::vital_to_vxl(ocv_img->get_image());
//    vil_image_view<vxl_byte> blurred2;
//    vil_gauss_filter_2d(image, blurred2, 1.2, 3, 1.2, 3);
}

void test_depth()
{
    kwiver::vital::plugin_manager::instance().load_all_plugins();
    kwiver::vital::algo::image_io_sptr io = kwiver::vital::algo::image_io::create("vxl");




    kwiver::vital::camera_intrinsics_sptr intr1(new kwiver::vital::simple_camera_intrinsics(3.310400000000000091e+03, kwiver::vital::vector_2d(3.167300000000000182e+02,2.005500000000000114e+02)));
    std::shared_ptr<kwiver::vital::simple_camera_perspective> cam1(new kwiver::vital::simple_camera_perspective);
    cam1->set_center(kwiver::vital::vector_3d(0.24337825, 0.17014022, -0.60485852));
    cam1->set_intrinsics(intr1);
    Eigen::Matrix<double, 3, 3> orientation1;
    orientation1 << -0.14396458,  0.96965263,  0.19760617,
                    -0.90366581, -0.04743335, -0.42560419,
                    -0.40331536, -0.23984131,   0.88306936;
    cam1->set_rotation(kwiver::vital::rotation_d(orientation1));

    kwiver::vital::camera_intrinsics_sptr intr2(new kwiver::vital::simple_camera_intrinsics(3.310400000000000091e+03, kwiver::vital::vector_2d(3.167300000000000182e+02,2.005500000000000114e+02)));
    std::shared_ptr<kwiver::vital::simple_camera_perspective> cam2(new kwiver::vital::simple_camera_perspective);
    cam2->set_center(kwiver::vital::vector_3d(4.479885925906316735e-01,1.826313465537660885e-01,-4.492737438843421582e-01));
    cam2->set_intrinsics(intr2);
    Eigen::Matrix<double, 3, 3> orientation2;
    orientation2 << -2.314368726285117028e-01,-6.586081116574981076e-01,-7.160120818719727387e-01,
                    9.642233202703062167e-01,-5.749426020475298382e-02,-2.587813785637697594e-01,
                    1.292693716555882721e-01,-7.502864048648203443e-01,6.483504961960695478e-01;
    orientation2 = orientation2.transpose();
    cam2->set_rotation(kwiver::vital::rotation_d(orientation2));

    kwiver::vital::camera_intrinsics_sptr intr3(new kwiver::vital::simple_camera_intrinsics(3.310400000000000091e+03, kwiver::vital::vector_2d(3.167300000000000182e+02,2.005500000000000114e+02)));
    std::shared_ptr<kwiver::vital::simple_camera_perspective> cam3(new kwiver::vital::simple_camera_perspective);
    cam3->set_center(kwiver::vital::vector_3d(5.735226881994851533e-01,1.965996453999410143e-01,-2.250549665435480073e-01));
    cam3->set_intrinsics(intr3);
    Eigen::Matrix<double, 3, 3> orientation3;
    orientation3 << -2.843764855912674361e-01,-3.044798573637405026e-01,-9.090781061285515552e-01,
                    9.581871718055249287e-01,-5.887271324279850365e-02,-2.800202682373388319e-01,
                    3.174117685280887502e-02,-9.506971501387793122e-01,3.084899546191077202e-01;
    orientation3 = orientation3.transpose();
    cam3->set_rotation(kwiver::vital::rotation_d(orientation3));









    std::vector<kwiver::vital::camera_perspective_sptr> cameras;
    cameras.push_back(cam1);
    cameras.push_back(cam2);
    cameras.push_back(cam3);

    std::vector<kwiver::vital::image_container_sptr> images;
    images.push_back(io->load("/home/matthieu/Downloads/dinoSparseRing/dinoSR0001.png"));
    images.push_back(io->load("/home/matthieu/Downloads/dinoSparseRing/dinoSR0002.png"));
    images.push_back(io->load("/home/matthieu/Downloads/dinoSparseRing/dinoSR0003.png"));

    std::vector<kwiver::vital::landmark_sptr> landmarks;

    landmarks.push_back(kwiver::vital::landmark_sptr(new kwiver::vital::landmark_d(Eigen::Matrix<double, 3, 1>(-0.061897, -0.018874, -0.057845))));
    landmarks.push_back(kwiver::vital::landmark_sptr(new kwiver::vital::landmark_d(Eigen::Matrix<double, 3, 1>(-0.061897, -0.018874, 0.015495))));
    landmarks.push_back(kwiver::vital::landmark_sptr(new kwiver::vital::landmark_d(Eigen::Matrix<double, 3, 1>(-0.061897,  0.068227, -0.057845))));
    landmarks.push_back(kwiver::vital::landmark_sptr(new kwiver::vital::landmark_d(Eigen::Matrix<double, 3, 1>(-0.061897,  0.068227, 0.015495))));

    landmarks.push_back(kwiver::vital::landmark_sptr(new kwiver::vital::landmark_d(Eigen::Matrix<double, 3, 1>(0.010897, -0.018874, -0.057845))));
    landmarks.push_back(kwiver::vital::landmark_sptr(new kwiver::vital::landmark_d(Eigen::Matrix<double, 3, 1>(0.010897, -0.018874, 0.015495))));
    landmarks.push_back(kwiver::vital::landmark_sptr(new kwiver::vital::landmark_d(Eigen::Matrix<double, 3, 1>(0.010897,  0.068227, -0.057845))));
    landmarks.push_back(kwiver::vital::landmark_sptr(new kwiver::vital::landmark_d(Eigen::Matrix<double, 3, 1>(0.010897,  0.068227, 0.015495))));

//    for (double x=-0.061897; x<=0.010897; x+= 0.01)
//    {
//        for (double y=-0.018874; y<= 0.068227; y+= 0.01)
//        {
//            for (double z=-0.057845; z<=0.015495; z+= 0.01)
//            {
//                landmarks.push_back(kwiver::vital::landmark_sptr(new kwiver::vital::landmark_d(Eigen::Matrix<double, 3, 1>(x, y, z))));
//            }
//        }
//    }

    for (int k=0; k < 100; ++k)
    {
        double x = ((double) rand() / (RAND_MAX));
        x = x * (0.010897 - -0.061897) + -0.061897;
        double y = ((double) rand() / (RAND_MAX));
        y = y * (0.068227 - -0.018874) + -0.018874;
        double z = ((double) rand() / (RAND_MAX));
        z = z * (0.015495 - -0.057845) + -0.057845;
        kwiver::vital::landmark_sptr pt(new kwiver::vital::landmark_d(
                                             Eigen::Matrix<double, 3, 1>(x, y, z)));
//        std::cout << pt->loc() << std::endl;
        landmarks.push_back(pt);
    }
    std::cout << "nb landmarks " << landmarks.size() << std::endl;
//    for (int i=0; i < 100; ++i)
//    {
//        double x = 0; //(((double) rand() / (RAND_MAX)) /*- 0.5*/) / 1000;
//        double y = 0; //(((double) rand() / (RAND_MAX)) /*- 0.5*/) / 1000;
//        double z = 0; //(((double) rand() / (RAND_MAX)) /*- 0.5*/) / 1000;

//        kwiver::vital::landmark_sptr pt(new kwiver::vital::landmark_d(
//                                             Eigen::Matrix<double, 3, 1>(x, y, z)));
//        landmarks.push_back(pt);
//    }

//    std::cout << "cam center: " << cam1->get_center() << std::endl;
//    std::cout << "cam rotation: " << cam1->get_rotation().matrix() << std::endl;

//    std::cerr << cam1->project(kwiver::vital::vector_3d(0, 0, 0));

    kwiver::vital::algo::compute_depth_sptr depth = kwiver::vital::algo::compute_depth::create("super3d");
    auto d_img = depth->compute(images, cameras, landmarks, 0);
    io->save("result.tif", d_img);
}


void test_track()
{
    kwiver::vital::plugin_manager::instance().load_all_plugins();

    kwiver::vital::algo::image_io_sptr io = kwiver::vital::algo::image_io::create("ocv");
    std::vector<kwiver::vital::image_container_sptr> images;
    for (int k=1; k < 17; ++k)
    {
        std::stringstream ss;
        ss << std::setw(4) << std::setfill('0') << k;
        std::string s = ss.str();
        images.push_back(io->load("/home/matthieu/Downloads/dinoSparseRing/dinoSR" + s + ".png"));
    }

//    kwiver::vital::algo::detect_features_sptr detect = kwiver::vital::algo::detect_features::create("ocv_BRIEF");
    kwiver::vital::algo::detect_features_sptr detect = kwiver::vital::algo::detect_features::create("ocv_SURF");
    kwiver::vital::algo::extract_descriptors_sptr extract = kwiver::vital::algo::extract_descriptors::create("ocv_SURF");

    std::vector<kwiver::vital::feature_set_sptr> list_of_features;
    std::vector<kwiver::vital::descriptor_set_sptr> list_of_desriptors;
    for (int id=0; id < images.size(); ++id)
    {
        kwiver::vital::feature_set_sptr features = detect->detect(images[id]);
        list_of_features.push_back(features);
        kwiver::vital::descriptor_set_sptr descriptors = extract->extract(images[id], features);
        list_of_desriptors.push_back(descriptors);
        cv::Mat image = kwiver::arrows::ocv::image_container::vital_to_ocv(images[id]->get_image(),
                kwiver::arrows::ocv::image_container::RGB_COLOR);
        for(auto feature : features->features())
        {
            auto loc = feature->loc();
            cv::Point2d pt(loc[0], loc[1]);
            cv::drawMarker(image, pt, cv::Scalar(255, 255, 255));
        }
        cv::namedWindow("window");
        cv::imshow("window", image);
        cv::waitKey(0);
    }

    kwiver::vital::algo::match_features_sptr match = kwiver::vital::algo::match_features::create("ocv_brute_force");
    kwiver::vital::match_set_sptr matches = match->match(list_of_features[0], list_of_desriptors[0],
                                                         list_of_features[1], list_of_desriptors[1]);
    std::vector<cv::DMatch> ocv_matches = kwiver::arrows::ocv::matches_to_ocv_dmatch(*matches);

    cv::Mat img1 = kwiver::arrows::ocv::image_container::vital_to_ocv(images[0]->get_image(),
                    kwiver::arrows::ocv::image_container::RGB_COLOR);
    cv::Mat img2 = kwiver::arrows::ocv::image_container::vital_to_ocv(images[1]->get_image(),
                    kwiver::arrows::ocv::image_container::RGB_COLOR);
    cv::Mat output_image;

    std::vector<cv::KeyPoint> keypoints_1;
    for (auto f: list_of_features[0]->features())
    {
        keypoints_1.push_back(cv::KeyPoint(f->loc()[0], f->loc()[1], 1));
    }
    std::vector<cv::KeyPoint> keypoints_2;
    for (auto f: list_of_features[1]->features())
    {
        keypoints_2.push_back(cv::KeyPoint(f->loc()[0], f->loc()[1], 1));
    }

    cv::drawMatches(img1, keypoints_1, img2, keypoints_2, ocv_matches, output_image);

    cv::imshow("window", output_image);
    cv::waitKey(0);
    cv::destroyAllWindows();

    kwiver::vital::algo::estimate_fundamental_matrix_sptr estimate_F = kwiver::vital::algo::estimate_fundamental_matrix::create("ocv");
    std::vector<bool> inliers;
    kwiver::vital::fundamental_matrix_sptr F = estimate_F->estimate(list_of_features[0], list_of_features[1], matches, inliers);
    std::cout << "F = " << *F << std::endl;



    kwiver::vital::camera_intrinsics_sptr intr1(new kwiver::vital::simple_camera_intrinsics(3.310400000000000091e+03, kwiver::vital::vector_2d(3.167300000000000182e+02,2.005500000000000114e+02)));
    std::shared_ptr<kwiver::vital::simple_camera_perspective> cam1(new kwiver::vital::simple_camera_perspective);
    cam1->set_center(kwiver::vital::vector_3d(0.24337825, 0.17014022, -0.60485852));
    cam1->set_intrinsics(intr1);
    Eigen::Matrix<double, 3, 3> orientation1;
    orientation1 << -0.14396458,  0.96965263,  0.19760617,
                    -0.90366581, -0.04743335, -0.42560419,
                    -0.40331536, -0.23984131,   0.88306936;
    cam1->set_rotation(kwiver::vital::rotation_d(orientation1));

    kwiver::vital::camera_intrinsics_sptr intr2(new kwiver::vital::simple_camera_intrinsics(3.310400000000000091e+03, kwiver::vital::vector_2d(3.167300000000000182e+02,2.005500000000000114e+02)));
    std::shared_ptr<kwiver::vital::simple_camera_perspective> cam2(new kwiver::vital::simple_camera_perspective);
    cam2->set_center(kwiver::vital::vector_3d(4.479885925906316735e-01, 1.826313465537660885e-01,-4.492737438843421582e-01));
    cam2->set_intrinsics(intr2);
    Eigen::Matrix<double, 3, 3> orientation2;
    orientation2 << -0.23143687262851170000, 0.96422332027030622000, 0.12926937165558827000,
                    -0.65860811165749811000, -0.05749426020475298400, -0.75028640486482034000,
                    -0.71601208187197274000, -0.25878137856376976000, 0.64835049619606955000;
    cam2->set_rotation(kwiver::vital::rotation_d(orientation2));

    int N = matches->size();
    std::cout << "N = " << N << std::endl;

    cv::Mat pts1(1, N, CV_64FC2);
//    for (int j=0; j < N; ++j)
//    {
//        pts1.at<cv::Vec2d>(0, j) = cv::Vec2d(keypoints_1[j].pt.x,
//                                             keypoints_1[j].pt.y);
//    }

    cv::Mat pts2(1, N, CV_64FC2);
//    for (int j=0; j < N; ++j)
//    {
//        pts2.at<cv::Vec2d>(0, j) = cv::Vec2d(keypoints_2[j].pt.x,
//                                             keypoints_2[j].pt.y);
//    }

//    std::sort(ocv_matches.begin(), ocv_matches.end(), [](const cv::DMatch& a, const cv::DMatch& b){
//        return a.distance < b.distance;
//    });

    for (int j=0; j < N; j++)
    {
        auto m = ocv_matches[j];
        auto m2 = matches->matches()[j];
        std::cout << "ocv match: " << m.queryIdx << " " << m.trainIdx << std::endl;
        std::cout << "vital match: " << m2.first << " " << m2.second<< std::endl;
        std::cout << ocv_matches[j].distance << std::endl;
        pts1.at<cv::Vec2d>(0, j) = cv::Vec2d(keypoints_1[m.queryIdx].pt.x,
                                             keypoints_1[m.queryIdx].pt.y);
        pts2.at<cv::Vec2d>(0, j) = cv::Vec2d(keypoints_2[m.trainIdx].pt.x,
                                             keypoints_2[m.trainIdx].pt.y);
        j += 1;
    }
    std::cout << "pts1 : " << pts1.size << " " << pts1.depth() << std::endl;
    std::cout << "pts2 : " << pts2.size << " " << pts2.depth() << std::endl;

    cv::Mat pts3d(1, N, CV_64FC4);

    auto mat1 = cam1->as_matrix();
    cv::Mat cv_mat1(3, 4, CV_64FC1);
    auto mat2 = cam2->as_matrix();
    cv::Mat cv_mat2(3, 4, CV_64FC1);

    for (int r=0; r<3; ++r)
    {
        for (int c=0; c<4; ++c)
        {
            cv_mat1.at<double>(r, c) = mat1.coeff(r, c);
            cv_mat2.at<double>(r, c) = mat2.coeff(r, c);
        }
    }
    std::cerr << cv_mat2 << std::endl;
    cv::triangulatePoints(cv_mat1, cv_mat2, pts1, pts2, pts3d);

    std::ofstream file("triangulation.obj");
    for (int i=0; i < N; ++i)
    {
        std::stringstream ss;
        ss << "v ";
        cv::Vec4d pt3d = pts3d.at<cv::Vec4d>(0, i);
        pt3d[0] /= pt3d[3];
        pt3d[1] /= pt3d[3];
        pt3d[2] /= pt3d[3];
        ss << pt3d[0] << " ";
        ss << pt3d[1] << " ";
        ss << pt3d[2] << "\n";
        file << ss.str();
    }
    file.close();
}
