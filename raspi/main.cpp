#include <iostream>
#include <map>

#include <opencv2/core.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>

enum class ObjectTracker {
    CSRT,
    KCF
};

enum class TrackingState {
    IDLE,
    TRACK
};

int main() {
    cv::Mat frame;
    cv::VideoCapture capture;
    bool opened = capture.open(0, cv::CAP_ANY, {
        cv::VideoCaptureProperties::CAP_PROP_FRAME_WIDTH, 1280,
        cv::VideoCaptureProperties::CAP_PROP_FRAME_HEIGHT, 720
    });

    if(!opened) {
        std::cerr << "Error: opening the VideoCapture from the camera failed!\n";
    }

    while(true) {
        capture >> frame;
        std::cout << "got frame!\n";
        break;
        if(frame.empty())
            break;
    }
    return 0;
}