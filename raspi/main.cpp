#include <iostream>
#include <map>
#include <optional>

#include <opencv2/core.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>

// Serial: https://github.com/wjwwood/serial

enum class ObjectTracker {
    CSRT,
    KCF
};

enum class TrackingState {
    IDLE,
    TRACK
};

ObjectTracker TRACKER = ObjectTracker::CSRT;
TrackingState STATE = TrackingState::IDLE;

cv::Mat ACTIVE_FRAME;
std::optional<std::pair<cv::InputArray, cv::Rect>> TO_TRACK = std::nullopt; // frame and corresponding bounding box
std::optional<bool> ARDUINO = std::nullopt; // replace by correct type when integrating serial

cv::Ptr<cv::Tracker> getTracker(ObjectTracker trackerChoice) {
    if(trackerChoice == ObjectTracker::CSRT) {
        return cv::TrackerCSRT::create();
    } else {
        return cv::TrackerKCF::create();
    }
}

void displayGeneralInfo(cv::Mat &frame, int64 timer) {
    // Tracker Type
    cv::putText(frame, (TRACKER == ObjectTracker::CSRT ? "CSRT Tracker" : "KCF Tracker"), {100, 20}, cv::FONT_HERSHEY_SIMPLEX, 0.75, {50, 170, 50}, 2);

    // Resolution
    cv::putText(frame, frame.cols + "x" + frame.rows, {100, 80}, cv::FONT_HERSHEY_SIMPLEX, 0.75, {50, 170, 50}, 2);

    // Frames per Second
    int fps = cv::getTickFrequency() / (cv::getTickCount() - timer);
    cv::putText(frame, "FPS: " + fps, {100, 50}, cv::FONT_HERSHEY_SIMPLEX, 0.75, {50, 170, 50}, 2);

    // Connection State
    cv::putText(frame, (ARDUINO ? "Arduino connected" : "Arduino not connected"), {100, 170}, cv::FONT_HERSHEY_SIMPLEX, 0.75, {255, 255, 0}, 2);
}

void displayTrackingFailure(cv::Mat &frame) {
    cv::putText(frame, "Tracking failure detected", {100, 80}, cv::FONT_HERSHEY_SIMPLEX, 0.75, {0, 0, 255}, 2);
}

void displayTrackingSuccess(cv::Mat &frame, cv::Rect bbox, std::pair<int, int> f_center, std::pair<int, int> offset_vector){
    // TODO
}

std::pair<int, int> calculateOffsetVector(){
    // TODO
}

void sendToArduino(std::pair<int, int> offset_vector){
    // TODO
}

int main() {
    cv::Mat frame;
    cv::VideoCapture capture;
    bool opened = capture.open(0, cv::CAP_ANY, {
        cv::VideoCaptureProperties::CAP_PROP_FRAME_WIDTH, 1280,
        cv::VideoCaptureProperties::CAP_PROP_FRAME_HEIGHT, 720
    });

    if(!opened) {
        std::cerr << "[ERROR] opening the VideoCapture from the camera failed!\n";
    }

    cv::Rect bbox;
    while(true) {
        capture >> frame;
        if(frame.empty()) break;
        auto f_center = std::make_pair(frame.cols/2, frame.rows/2);
        auto tracker = getTracker(TRACKER);
        int64 timer = cv::getTickCount();
        
        if(STATE == TrackingState::TRACK) {
            if(TO_TRACK) {
                auto [reference_frame, bbox] = *TO_TRACK;
                tracker->init(reference_frame, bbox);
                TO_TRACK = std::nullopt;
            }

            if(tracker->update(frame, bbox)) {
                std::pair<int, int> offset_vector = calculateOffsetVector();
                displayTrackingSuccess(frame, bbox, f_center, offset_vector);
                sendToArduino(offset_vector);
            } else {
                displayTrackingFailure(frame);
            }

            displayGeneralInfo(frame, timer);
            ACTIVE_FRAME = frame;

        }
    }

    capture.release();
    return 0;
}