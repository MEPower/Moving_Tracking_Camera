#include <iostream>
#include <map>
#include <optional>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

// Serial: https://github.com/wjwwood/serial
// Websockets: https://github.com/zaphoyd/websocketpp/ (v0.8.2, also needs boost asio)

typedef websocketpp::server<websocketpp::config::asio> server;
typedef server::message_ptr message_ptr;
typedef websocketpp::frame::opcode::value opcode;

using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;
using websocketpp::lib::bind;

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

cv::Mat ACTIVE_FRAME, LAST_SEND_FRAME;
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

void websocket_handler(server* s, websocketpp::connection_hdl hdl, message_ptr msg) {
    std::string message = msg->get_payload();
    std::cerr << "Received message: " << message << "\n";

    if(message == "c"){
        std::cerr << "Companion is connected\n";
        s->send(hdl, "ok", opcode::TEXT);
    } else if(message == "f") {
        std::cerr << "Frame requested\n";
        LAST_SEND_FRAME = ACTIVE_FRAME;
        std::vector<uchar> buf;
        cv::imencode(".jpg", LAST_SEND_FRAME, buf);
        s->send(hdl, buf, opcode::BINARY); // TODO find out how to send buf correctly
    } else if(message == "t"){
        std::cerr << "Track region received\n";
        cv::Rect roi; // TODO find out how to receive the ROI
        TO_TRACK.emplace(LAST_SEND_FRAME, roi);
        STATE = TrackingState::TRACK;
    }
}

void start_websocket_server(){
    server ws_server;

    try {
        // Set logging settings
        ws_server.set_access_channels(websocketpp::log::alevel::all);
        ws_server.clear_access_channels(websocketpp::log::alevel::frame_payload);

        ws_server.init_asio();
        ws_server.set_message_handler(bind(&websocket_handler, &ws_server,::_1,::_2));
        ws_server.listen(8764);
        ws_server.start_accept();
        ws_server.run();
        std::cerr << "Server started successfully!\n";
    } catch (websocketpp::exception const & e) {
        std::cerr << "[ERROR] " << e.what() << std::endl;
    }
}

int main() {
    start_websocket_server();

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