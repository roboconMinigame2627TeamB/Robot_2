#include "main.h"
#include "OneEuroFilter.h"

int main() {
    OneEuroFilter euro_x(30.0, 0.52, 0.007, 1.0);
    OneEuroFilter euro_z(30.0, 0.3, 0.015, 1.0);
    OneEuroFilter euro_w(30.0, 0.6, 0.001, 1.0);

    HANDLE hSerial = initSerialPort("\\\\.\\COM7"); 
    if (hSerial == INVALID_HANDLE_VALUE) {
        std::cerr << "Warning: Could not open COM port. Is the STM32 plugged in?" << std::endl;
        
    } else {
        std::cout << "Successfully connected to STM32!" << std::endl;
    }

    cv::VideoCapture cap(1, cv::CAP_DSHOW);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_SETTINGS, 1);
    

    if (!cap.isOpened()) {
        std::cerr << "ERROR: Could not open the webcam!" << std::endl;
        return -1;
    }

    MarkerDetectionSystem detector(0.1415f);
    double desired_distance = 50.0; // in cm

    cv::Mat frame;
    bool success;

    while (true) {
        
        cap >> frame;
        if (frame.empty()) continue;
        // auto detected = detector.detect_markers(frame);

        // if (detected.find(target_marker_id) != detected.end()) {
            
        

        detector.process_frame(frame);
        
        RobotVelocities velocities;
        if(detector.getTrackingMode() == "CHARUCO_BOARD") {
            success = detector.calculate_3d_velocity(desired_distance, velocities);
        } else if(detector.getTrackingMode() == "SINGLE_MARKER") {
            success = detector.calculate_25d_velocity(40,desired_distance, velocities);
        }

        if (success) {
            double ts = get_time_seconds();
            double vx_filtered = euro_x.filter(velocities.v_x, ts);
            double vz_filtered = euro_z.filter(velocities.v_z, ts);
            double vw_filtered = euro_w.filter(velocities.w, ts);
                
            char buffer[64];
            snprintf(buffer, sizeof(buffer), "%.3f,%.3f,%.3f\n", 
                         vx_filtered, vz_filtered, vw_filtered);
            std::cout<<buffer<<std::endl;

            if (hSerial != INVALID_HANDLE_VALUE) {
                DWORD bytesWritten;
                WriteFile(hSerial, buffer, strlen(buffer), &bytesWritten, NULL);
            }
        }  
        
        else {
            if (hSerial != INVALID_HANDLE_VALUE) {
                DWORD bytesWritten;
                const char* stopCmd = "0.000,0.000,0.000\n";
                WriteFile(hSerial, stopCmd, strlen(stopCmd), &bytesWritten, NULL);
            }
        }

        detector.visualize_3d_coordinates(frame);

        cv::imshow("Omniwheel Robot View", frame);

        if (cv::waitKey(1) == 27) {
            std::cout << "Shutting down..." << std::endl;
            break;
        }
    }
    if (hSerial != INVALID_HANDLE_VALUE) CloseHandle(hSerial);
    cap.release();
    cv::destroyAllWindows();

    return 0;
}

//cmake --build . --config Release
//.\Release\omni_robot.exe