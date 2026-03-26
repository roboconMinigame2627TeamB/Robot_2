#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>
#include <opencv2/objdetect/aruco_board.hpp>
#include <opencv2/objdetect/charuco_detector.hpp>
#include <opencv2/calib3d.hpp>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/visual_features/vpFeatureThetaU.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/vs/vpServo.h>
#include <visp3/robot/vpSimulatorCamera.h>

#include <iostream>
#include <vector>
#include <map>
#include <cmath>
#include <windows.h>
#include <string>

HANDLE initSerialPort(const char* portName) {
    HANDLE hSerial = CreateFileA(portName, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
    if (hSerial == INVALID_HANDLE_VALUE) return hSerial;

    DCB dcbSerialParams = {0};
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    GetCommState(hSerial, &dcbSerialParams);
    
    dcbSerialParams.BaudRate = CBR_115200; 
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity   = NOPARITY;
    
    SetCommState(hSerial, &dcbSerialParams);
    return hSerial;
}


struct MarkerInfo {
    int id;
    cv::Point2f centroid;
    std::vector<cv::Point2f> corners;
};


struct RobotVelocities {
    double v_x;     
    double v_z;     
    double w;
};

class MarkerDetectionSystem {
private:
    float marker_size_cm;
    cv::aruco::Dictionary dictionary;
    cv::aruco::DetectorParameters parameters;
    cv::aruco::ArucoDetector detector;
    cv::aruco::CharucoBoard board;
    cv::aruco::CharucoParameters charucoParams;
    cv::Ptr<cv::aruco::CharucoDetector> charucoDetector;
    cv::Mat camera_matrix;
    cv::Mat distortion_coefficients;
    
    cv::Mat current_rvec, current_tvec;
    cv::Mat prev_rvec, prev_tvec;
    bool has_prev_pose = false;
    bool board_detected = false;

    std::map<int, MarkerInfo> detected_markers; //for single marker
    int target_marker_id = 40;             
    float height_offset_m = 0.00f;
    std::vector<cv::Point3f> single_marker_3d_points;
    std::string current_tracking_mode = "NONE";

    double start_time;

public:
    MarkerDetectionSystem(float marker_size_cm = 0.1415) : marker_size_cm(marker_size_cm) {
      
        dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);
        parameters = cv::aruco::DetectorParameters();
        // detector = cv::aruco::ArucoDetector(dictionary, parameters);
        parameters.adaptiveThreshWinSizeMin = 3;
        parameters.adaptiveThreshWinSizeMax = 23;
        parameters.adaptiveThreshWinSizeStep = 10;        
        parameters.polygonalApproxAccuracyRate = 0.13;
        parameters.errorCorrectionRate = 0.8;
        parameters.adaptiveThreshConstant = 3.0;
        parameters.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
        charucoParams = cv::aruco::CharucoParameters();
        board = cv::aruco::CharucoBoard(cv::Size(5, 7), 0.03644f, 0.02274f, dictionary);

        camera_matrix = (cv::Mat_<float>(3, 3) << 
            620.23803836,   0.0,             325.07334255,
            0.0,            619.58392471,    239.71842371,
            0.0,            0.0,             1.0);
       
        distortion_coefficients = (cv::Mat_<float>(1, 5) << 
            0.13473783, -0.5264003,   0.00707333, -0.00199903,  0.24540998);

        charucoParams.tryRefineMarkers = true; 
        charucoParams.cameraMatrix = camera_matrix; 
        charucoParams.distCoeffs = distortion_coefficients;

        float half_size = marker_size_cm / 2.0f;
        single_marker_3d_points = {
            {-half_size,  half_size, 0}, // Corner 0 
            { half_size,  half_size, 0}, // Corner 1
            { half_size, -half_size, 0}, // Corner 2 
            {-half_size, -half_size, 0}  // Corner 3 
        };
        charucoDetector = cv::makePtr<cv::aruco::CharucoDetector>(board,charucoParams,parameters);
        start_time = (double)cv::getTickCount() / cv::getTickFrequency();
    }

    std::string getTrackingMode() const {
        return current_tracking_mode;
    }
    
    std::map<int, MarkerInfo> detect_markers(const cv::Mat& frame) { //for individual markers
        detected_markers.clear();
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        detector.detectMarkers(gray, corners, ids);

        if (!ids.empty()) {
            for (size_t i = 0; i < ids.size(); i++) {
                cv::Moments M = cv::moments(corners[i]);
                if (M.m00 != 0) {
                    int cX = static_cast<int>(M.m10 / M.m00);
                    int cY = static_cast<int>(M.m01 / M.m00);

                    MarkerInfo info;
                    info.id = ids[i];
                    info.centroid = cv::Point2f(cX, cY);
                    info.corners = corners[i];

                    detected_markers[ids[i]] = info;
                }
            }
        }
        return detected_markers;
    }


   bool process_frame(const cv::Mat& frame) {
    detected_markers.clear();
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    std::vector<int> charucoIds, markerIds;
    std::vector<cv::Point2f> charucoCorners;
    std::vector<std::vector<cv::Point2f>> markerCorners;

    charucoDetector->detectBoard(gray, charucoCorners, charucoIds, markerCorners, markerIds);

    board_detected = false;
    current_tracking_mode = "NONE";

    static double last_dist_m = 3.0f;
    if (has_prev_pose) {
        last_dist_m = cv::norm(prev_tvec); 
    }

    if (charucoIds.size() >= 6 && last_dist_m < 1.2f) {
        cv::Mat objPoints, imgPoints;
 
        board.matchImagePoints(charucoCorners, charucoIds, objPoints, imgPoints);

        if (has_prev_pose) {
            current_rvec = prev_rvec.clone();
            current_tvec = prev_tvec.clone();
        }

        bool success = cv::solvePnP(
            objPoints, imgPoints, 
            camera_matrix, distortion_coefficients, 
            current_rvec, current_tvec, 
            has_prev_pose,           
            cv::SOLVEPNP_ITERATIVE   
        );

        if (success) {
            prev_rvec = current_rvec.clone();
            prev_tvec = current_tvec.clone();
            has_prev_pose = true;
            board_detected = true;
            current_tracking_mode = "CHARUCO_BOARD";
            return true;
        }
    }

    if (!markerIds.empty() && last_dist_m > 1.2f) {
        for (size_t i = 0; i < markerIds.size(); i++) {
            if (markerIds[i] == target_marker_id) {
                cv::Moments M = cv::moments(markerCorners[i]);
                if (M.m00 != 0) {
                    int cX = static_cast<int>(M.m10 / M.m00);
                    int cY = static_cast<int>(M.m01 / M.m00);

                    MarkerInfo info;
                    info.id = markerIds[i];
                    info.centroid = cv::Point2f(cX, cY);
                    info.corners = markerCorners[i];

                    detected_markers[markerIds[i]] = info;
                }

                bool success = cv::solvePnP(
                    single_marker_3d_points, markerCorners[i], 
                    camera_matrix, distortion_coefficients, 
                    current_rvec, current_tvec, false, cv::SOLVEPNP_IPPE_SQUARE
                );

                if (success) {
                    current_tvec.convertTo(current_tvec, CV_64F);
                    current_rvec.convertTo(current_rvec, CV_64F);
                    current_tvec.at<double>(1, 0) += height_offset_m; // Use + or - depending on if OpenCV Y is pointing up or down in your setup

                    prev_rvec = current_rvec.clone();
                    prev_tvec = current_tvec.clone();
                    has_prev_pose = true;
                    board_detected = true; 
                    current_tracking_mode = "SINGLE_MARKER";
                    return true;
                }
            }
        }
    }
        
    has_prev_pose = false;
    return false;
}
    
    bool calculate_25d_velocity(int target_id, double desired_distance_cm, RobotVelocities& out_vel) { //for individual marker
        MarkerInfo target = detected_markers[target_id];
        if (!board_detected) {
            return false;
        }

        cv::Mat rvec = current_rvec.clone();
        cv::Mat tvec = current_tvec.clone();
        if (tvec.empty() || rvec.empty()) {
            return false; 
        }

        double desired_Z_m = desired_distance_cm/100;
        
        tvec.convertTo(tvec, CV_64F);
        rvec.convertTo(rvec, CV_64F);

        vpCameraParameters cam(
            camera_matrix.at<float>(0,0), camera_matrix.at<float>(1,1),
            camera_matrix.at<float>(0,2), camera_matrix.at<float>(1,2)
        );

        
        vpServo task;
        task.setServo(vpServo::EYEINHAND_CAMERA);
        task.setInteractionMatrixType(vpServo::CURRENT);
        vpAdaptiveGain lambda (2.35, 0.4, 35.0);
        task.setLambda(lambda);

        
        vpHomogeneousMatrix cMo_cv(
            vpTranslationVector(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2)),
            vpThetaUVector(rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2))
        );

        //Transform to ViSP Frame (Flip 180 degrees around X so Z points in, Theta is ~0 deg)
        vpHomogeneousMatrix oMv(0, 0, 0, M_PI, 0, 0); 
        vpHomogeneousMatrix cMo_visp = cMo_cv * oMv;

        vpHomogeneousMatrix cdMo_visp(vpTranslationVector(0, 0, desired_Z_m), vpThetaUVector(0, 0, 0));
        vpHomogeneousMatrix cdMo_cv(vpTranslationVector(0, 0, desired_Z_m), vpThetaUVector(M_PI, 0, 0));

        vpFeatureThetaU s_tu, s_tu_desired;
        s_tu.buildFrom(cMo_visp);
        s_tu_desired.buildFrom(cdMo_visp);
        task.addFeature(s_tu, s_tu_desired);

        std::vector<vpFeaturePoint> s_p(4), s_p_desired(4);
        for (int i = 0; i < 4; i++) {
            vpFeatureBuilder::create(s_p[i], cam, vpImagePoint(target.corners[i].y, target.corners[i].x));
            s_p[i].set_Z(tvec.at<double>(2));

            vpPoint pt_3d;
            pt_3d.setWorldCoordinates(single_marker_3d_points[i].x, single_marker_3d_points[i].y, 0);
            pt_3d.project(cdMo_cv); // Project using the native OpenCV target pose
            
            s_p_desired[i].buildFrom(pt_3d.get_x(), pt_3d.get_y(), desired_Z_m);
            task.addFeature(s_p[i], s_p_desired[i], vpFeaturePoint::selectX());
        }

        vpMatrix eJe(6, 3);
        eJe = 0.0; 
        eJe[0][0] = 1.0; // Camera X 
        eJe[2][1] = 1.0; // Camera Z 
        eJe[4][2] = 1.0; // Camera Yaw
        task.set_eJe(eJe);

        double current_time = ((double)cv::getTickCount() / cv::getTickFrequency()) - start_time;
        vpColVector v = task.computeControlLaw(current_time);
        
        out_vel.v_x =  v[0]; 
        out_vel.v_z =  v[1]; 
        out_vel.w   =  -v[2]; 
            
        return true;
    }

    bool calculate_3d_velocity(double desired_distance_cm, RobotVelocities& out_vel) {
        if (!board_detected) {
            return false;
        }
        
        double desired_Z_m = desired_distance_cm / 100.0;
        cv::Mat rvec = current_rvec.clone();
        cv::Mat tvec = current_tvec.clone();

        if (tvec.empty() || rvec.empty()) {
            return false; 
        }
        tvec.convertTo(tvec, CV_64F);
        rvec.convertTo(rvec, CV_64F);
        // try{
            vpServo task;
            task.setServo(vpServo::EYEINHAND_CAMERA);
            task.setInteractionMatrixType(vpServo::CURRENT);
            vpAdaptiveGain lambda(2.35, 0.7, 30.0);
            task.setLambda(lambda);

            vpHomogeneousMatrix cMo_visp(
                vpTranslationVector(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2)),
                vpThetaUVector(rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2))
            );

            // vpHomogeneousMatrix oMv(0, 0, 0, M_PI, 0, 0); 
            // vpHomogeneousMatrix cMo_visp = cMo_cv * oMv;

            vpHomogeneousMatrix cdMo_visp(vpTranslationVector(-0.0911, -0.12754, desired_Z_m), vpThetaUVector(0.0, 0.0, 0.0));
            
            vpHomogeneousMatrix cdMc = cdMo_visp * cMo_visp.inverse();

            vpFeatureTranslation s_t, s_t_desired;
            s_t.buildFrom(cdMc);
            s_t_desired.buildFrom(vpHomogeneousMatrix());
            task.addFeature(s_t, s_t_desired);

            vpFeatureThetaU s_tu, s_tu_desired;
            s_tu.buildFrom(cdMc);
            s_tu_desired.buildFrom(vpHomogeneousMatrix());
            task.addFeature(s_tu, s_tu_desired);
            vpMatrix eJe(6, 3);
            eJe = 0.0; 
            eJe[0][0] = 1.0; 
            eJe[2][1] = 1.0; 
            eJe[4][2] = 1.0; 
            task.set_eJe(eJe);
            
            double current_time = ((double)cv::getTickCount() / cv::getTickFrequency()) - start_time;
            vpColVector v = task.computeControlLaw(current_time);
            
            out_vel.v_x =  -v[0]; 
            out_vel.v_z =   v[1]; 
            out_vel.w   =   -v[2]; 
                    
            return true;
        // } catch (const vpException& e) {
        //     std::cerr << "ViSP Math Error: " << e.what() << std::endl;
        //     return false; 
        // }
    }

    void visualize_3d_coordinates(cv::Mat& frame) {
        if (board_detected) {
           
            cv::drawFrameAxes(frame, camera_matrix, distortion_coefficients, current_rvec, current_tvec, 0.1f);
            cv::Mat temp_tvec;
            current_tvec.convertTo(temp_tvec, CV_64F);

            double x = temp_tvec.at<double>(0);
            double y = temp_tvec.at<double>(1);
            double z = temp_tvec.at<double>(2);
            double dist = std::sqrt(x*x + y*y + z*z) * 100.0; 
            
            std::string text = "Dist: " + std::to_string(dist).substr(0, 5) + " cm";
            cv::putText(frame, text, cv::Point(50, 50), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
            cv::putText(frame, "Mode: " + current_tracking_mode, cv::Point(50, 80), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
        }
    }
};