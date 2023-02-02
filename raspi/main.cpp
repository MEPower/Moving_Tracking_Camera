#include <iostream>
#include <map>
#include <mutex>
#include <optional>
#include <thread>
#include <chrono>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>
#include <iostream>
#include <fstream>

#include <serial/serial.h>

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

/*
    == Library Installation ==
    OpenCV: use your distro's package, if any (can also be built manually, see install-opencv.sh)
    Serial: https://github.com/wjwwood/serial (requires https://github.com/ros/catkin for build, wrong installation dir in Makefile, needs to be added to ld config)
    Websockets: https://github.com/zaphoyd/websocketpp/ (v0.8.2, also needs boost asio - just use your distro's boost package)
*/

std::ofstream myfile;
//myfile.open ("C:Users\\Ella\\Ella-Kopie\\Studium\\Master\\Semester1\\Eingebettete Betriebssysteme\\Moving_Tracking_Camera\\raspi\\performance_logs_cpp");
myfile.open ("performance_logs_cpp.txt");
cout << "created file";

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
    RECEIVE_BBOX,
    TRACK,
    TERMINATE
};

enum class ArduinoState {
    NOT_CONNECTED,
    CONNECTED
};

// Configuration
ObjectTracker TRACKER = ObjectTracker::CSRT;
std::string SERIAL_PORT = "/dev/pts/2";
int BAUDRATE = 9600;
int WEBSOCKET_PORT = 8764;
int CAMERA_ID = 0;

// States
TrackingState STATE = TrackingState::IDLE;
ArduinoState ARDUINO_STATE = ArduinoState::NOT_CONNECTED;

// Current camera frames
cv::Mat ACTIVE_FRAME(1080, 1920, CV_8UC3);
cv::Mat LAST_SEND_FRAME;

// Offset vector and mutexes for the Serial thread
cv::Point OFFSET_VECTOR(5, 5);
bool DO_SEND_VECTOR = false;
std::mutex VECTOR_MUTEX;

std::optional<std::pair<cv::Mat, cv::Rect>> TO_TRACK = std::nullopt; // tracking object: bounding box and corresponding reference frame
std::optional<serial::Serial> ARDUINO = std::nullopt;

int WS_CONN_COUNT = 0; // number of open websocket connections

cv::Ptr<cv::Tracker> getTracker(ObjectTracker trackerChoice) {
    if(trackerChoice == ObjectTracker::CSRT) {
        return cv::TrackerCSRT::create();
    } else {
        return cv::TrackerKCF::create();
    }
}

void displayGeneralInfo(cv::Mat &frame, int fps) {
    // Tracker Type
    cv::putText(frame, (TRACKER == ObjectTracker::CSRT ? "CSRT Tracker" : "KCF Tracker"), {100, 20}, cv::FONT_HERSHEY_SIMPLEX, 0.75, {50, 170, 50}, 2);

    // Resolution
    cv::putText(frame, std::to_string(frame.cols) + "x" + std::to_string(frame.rows), {100, 80}, cv::FONT_HERSHEY_SIMPLEX, 0.75, {50, 170, 50}, 2);

    // Frames per Second
    cv::putText(frame, "FPS: " + std::to_string(fps), {100, 50}, cv::FONT_HERSHEY_SIMPLEX, 0.75, {50, 170, 50}, 2);

    // Connection State
    cv::putText(frame, (ARDUINO_STATE == ArduinoState::CONNECTED ? "Arduino connected" : "Arduino not connected - Press A to connect"), {100, 170}, cv::FONT_HERSHEY_SIMPLEX, 0.75, {255, 255, 0}, 2);
}

void displayTrackingFailure(cv::Mat &frame) {
    cv::putText(frame, "Tracking failure detected", {100, 110}, cv::FONT_HERSHEY_SIMPLEX, 0.75, {0, 0, 255}, 2);
}

void displayTrackingSuccess(cv::Mat &frame, cv::Rect bbox, cv::Point f_center, cv::Point offset_vector) {
    cv::Point p_center = (bbox.tl() + bbox.br()) / 2;
    cv::rectangle(frame, bbox, {255, 0, 0}, 2, 1);
    cv::line(frame, f_center, p_center, {255, 0, 0}, 2, 1);
    cv::putText(frame, "Center offset (scaled): (" + std::to_string(offset_vector.x) + ", " + std::to_string(offset_vector.y) + ")", {100, 110}, cv::FONT_HERSHEY_SIMPLEX, 0.75, {255, 255, 0}, 2);
}

cv::Point calculateOffsetVector(cv::Rect bbox, cv::Point2d f_center) {
    cv::Point2d p_center = (bbox.tl() + bbox.br()) / 2.0;
    cv::Point2d offset = p_center - f_center;
    cv::Point2d norm_offset(offset.x / f_center.x, offset.y / f_center.y);
    cv::Point2d clamped_offset(
        std::min(std::max(-1.0, norm_offset.x), 0.99),
        std::min(std::max(-1.0, norm_offset.y), 0.99)   
    );
    return 5 * clamped_offset;
}

void connectToArduino() {
    if(ARDUINO_STATE == ArduinoState::CONNECTED) return;
    ARDUINO.emplace(SERIAL_PORT, BAUDRATE, serial::Timeout::simpleTimeout(1000));

    while(!ARDUINO->isOpen()) {
        std::cerr << "Waiting to open Arduino connection\n";
        ARDUINO->open();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::string ready = ARDUINO->read();
    if(ready != "r") {
        ARDUINO->close();
        ARDUINO = std::nullopt;
    } else {
        ARDUINO_STATE = ArduinoState::CONNECTED;
    }
}

void sendToArduinoThread() {
    while(STATE != TrackingState::TERMINATE) {
        if(ARDUINO_STATE != ArduinoState::CONNECTED) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        } else if(DO_SEND_VECTOR) {
            cv::Point vector_to_send(5, 5); 
            {
                std::lock_guard<std::mutex> lock(VECTOR_MUTEX);
                vector_to_send = OFFSET_VECTOR + cv::Point(5, 5);
                DO_SEND_VECTOR = false;
            }
            std::string message = std::to_string(vector_to_send.x) + std::to_string(vector_to_send.y);
            ARDUINO->write(message);
            std::cerr << ARDUINO->readline();
        }
    }
}

void websocket_handler(server* serv, websocketpp::connection_hdl hdl, message_ptr msg) {
    std::string message = msg->get_payload();
    std::cerr << "Received message: " << message << "\n";

    if(STATE == TrackingState::RECEIVE_BBOX) {
        std::cerr << "Track region received\n";
        std::vector<int> bbox(4);
        message = message.substr(1, message.length()-2);
        for(int i = 0; i < 4; i++){
            size_t pos = message.find(',');
            bbox[i] = std::stoi(message.substr(0, pos));
            message.erase(0, pos+1);
        }
        cv::Rect roi(bbox[0], bbox[1], bbox[2], bbox[3]);
        TO_TRACK.emplace(LAST_SEND_FRAME.clone(), roi);
        STATE = TrackingState::TRACK;
    } else if(message == "c") {
        std::cerr << "Companion is connected\n";
        serv->send(hdl, "ok", opcode::TEXT);
    } else if(message == "f") {
        std::cerr << "Frame requested\n";
        LAST_SEND_FRAME = ACTIVE_FRAME.clone();
        std::vector<uchar> buf;
        cv::imencode(".jpg", LAST_SEND_FRAME, buf);
        serv->send(hdl, buf.data(), buf.size(), opcode::BINARY);
    } else if(message == "t") {
        STATE = TrackingState::RECEIVE_BBOX;
    } else if(message == "a") {
        connectToArduino();
    }
}

// maintain number of open connections and exit the program once all were closed by the client(s)
void websocket_on_open(websocketpp::connection_hdl hdl) {
    WS_CONN_COUNT += 1;
}

void websocket_on_close(server* serv, websocketpp::connection_hdl hdl) {
    WS_CONN_COUNT -= 1;
    if(WS_CONN_COUNT == 0) {
        serv->stop_listening();
        STATE = TrackingState::TERMINATE;
    }
}

void start_websocket_server() {
    server ws_server;

    try {
        // Set logging settings
        ws_server.set_access_channels(websocketpp::log::alevel::all);
        ws_server.clear_access_channels(websocketpp::log::alevel::frame_payload);

        ws_server.init_asio();
        ws_server.set_message_handler(bind(&websocket_handler, &ws_server,::_1,::_2));
        ws_server.set_open_handler(bind(&websocket_on_open, ::_1));
        ws_server.set_close_handler(bind(&websocket_on_close, &ws_server, ::_1));
        ws_server.listen(WEBSOCKET_PORT);
        ws_server.start_accept();
        ws_server.run();
    } catch (websocketpp::exception const & e) {
        std::cerr << "[ERROR] " << e.what() << std::endl;
    }
}

int main() {
    std::thread ws_thread(start_websocket_server);
    std::thread serial_thread(sendToArduinoThread);

    cv::Mat frame;
    cv::VideoCapture capture;
    bool opened = capture.open(CAMERA_ID, cv::CAP_ANY);

    if(!opened) {
        std::cerr << "[ERROR] opening the VideoCapture from the camera failed!\n";
    }

    cv::Rect bbox;
    auto tracker = getTracker(TRACKER);

    while(STATE != TrackingState::TERMINATE) {
        using namespace std::chrono;
        milliseconds start = duration_cast< milliseconds >(
                system_clock::now().time_since_epoch()
        );

        capture >> frame;
        if(frame.empty()) break;
        cv::Point2d f_center(frame.cols/2.0, frame.rows/2.0);
        int64 timer = cv::getTickCount();
        
        if(STATE == TrackingState::TRACK) {
            if(TO_TRACK) {
                auto [reference_frame, bbox] = *TO_TRACK;
                tracker->init(reference_frame, bbox);
                TO_TRACK = std::nullopt;
            }

            if(tracker->update(frame, bbox)) {
                cv::Point vector = calculateOffsetVector(bbox, f_center);
                displayTrackingSuccess(frame, bbox, f_center, vector);
                std::lock_guard<std::mutex> lock(VECTOR_MUTEX);
                OFFSET_VECTOR = vector;
                DO_SEND_VECTOR = true;
            } else {
                DO_SEND_VECTOR = false;
                displayTrackingFailure(frame);
            }
        }

        int fps = cv::getTickFrequency() / (cv::getTickCount() - timer);
        displayGeneralInfo(frame, fps);
        ACTIVE_FRAME = frame.clone();

        milliseconds end = duration_cast< milliseconds >(
                system_clock::now().time_since_epoch()
        );
        milliseconds total = end-start;

        std::ofstream myfile;
        myfile.open ("C:Users\\Ella\\Ella-Kopie\\Studium\\Master\\Semester1\\Eingebettete Betriebssysteme\\Moving_Tracking_Camera\\raspi\\performance_logs");
        myfile << total + " \n";
        myfile.close();

    }

    capture.release();
    ws_thread.join();
    serial_thread.join();
    ARDUINO->close();
    return 0;
}