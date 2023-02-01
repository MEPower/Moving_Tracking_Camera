# Moving Tracking Camera

We are creating a system that can move a Canon camera based on images captured by that camera. This allows to track the position of an object and move the camera accordingly to keep the object inside the camera frame. The idea for this is based on this project: <https://www.youtube.com/watch?v=7TkybpSQULk>

## How our system works

The system consists of three main components:

1. A 3D-printed pan/tilt mount driven by stepper motors that can move and tilt the camera in different directions. This is controlled by an Arduino Uno which steers the stepper motors.
2. A Raspberry Pi with a camera connected via USB. An object recognition server program processes the frames obtained from the camera and uses OpenCV to track the position of an user-selected object. Based on this position, it generates steering commands for the Arduino. The server program usually runs headless, that is, without a GUI. We provide two different implementations, one in Python and another one in C++, which were used to study the effect of the used programming language on performance.
3. A companion app for a laptop or desktop PC, which connects to the Raspberry through a network connection and allows to view the captured frames along with the current position of the tracked object. Also, this app is used to select the Region of Interest (ROI), in which the tracked object is located at the beginning.

## Arduino program

This program is located in the `microproccesor` (sic!) directory.

### Installation / Dependencies

The Arduino IDE can be used to build the program and flash it onto the Arduino.

You may need to install some libraries for this:

* TODO Daniel

### Use

TODO Daniel

## Object Recognition Server (Python)

This program is located in the `raspi` directory and consists of the file `main.py`.

### Configuration

In lines 10-15, the source code contains some configuration options that can be set by the user. These include:

* HEADLESS - whether the program should start a GUI or run as a server for the companion app
* TRACKER - the OpenCV tracking algorithm that should be used. Possible choices are CSRT, KCF or MIL. CSRT is more accurate, while the other algorithms are faster, but less accurate.
* SERIAL_PORT - the serial port which the Arduino is connected to. On unixoid systems, this is something in the `/dev` filesystem.
* BAUDRATE - the baudrate used for communication with the Arduino (9600 by default)
* WEBSOCKET_PORT - the network port which the Websocket server for the companion should listen on (8764 by default)
* FRAME_DIMENSIONS - the desired resolution of the camera image. Note that cameras can natively only provide a limited number of resolutions, so this setting will not always be successful.
* CAMERA_ID - allows to choose the camera device that will be used. If your camera is the device `/dev/videoX`, enter X here.

### Dependencies

Install the needed dependencies using ```pip3 install -r requirements.txt```.

### Use

The program can be started using your preferred Python 3 interpreter. In the not-headless mode, the hotkey **T** serves to start the selection of a tracking region (use your mouse to select and then press Enter) and the hotkey **A** starts the connection with the Arduino. Using the key **Q**, the program can be stopped.

## Object Recognition Server (C++)

This program is also located in the `raspi` directory and consists of the file `main.cpp`.

### Configuration

In lines 50-54 of the source code, similar configuration options to the Python server are available. However, there are the following changes:

* There is no configuration option HEADLESS, as the C++ server is only implemented to run in headless mode.
* There are only two TRACKER options, namely CSRT and KCF.
* There is no configuration option FRAME_DIMENSIONS, as the OpenCV version in the official package for Raspberry Pi OS is too old to support this option.

### Dependencies and build process

The program has multiple dependencies. These are:

* **OpenCV 4**: Preferrably install this using the respective package for your Linux distribution (something like `libopencv-dev` or `opencv-devel` - consult [pkgs.org](https://pkgs.org/search/?q=opencv)). However, this can also be installed from source. The instructions in `install-opencv.sh` should help you in this case.
* **Boost ASIO**: Just install the boost package of your Linux distribution (something like `libboost-dev` - consult [pkgs.org](https://pkgs.org/search/?q=boost)). This is needed by the websocket library.
* **Websocket++**: This is the websocket library that we use. Download it from <https://github.com/zaphoyd/websocketpp/> and install it using the usual trio of `cmake`, `make` and `make install`.
* **Catkin**: This is a build dependency of the Serial library. Download it from <https://github.com/ros/catkin> and install it using the instructions in <http://wiki.ros.org/catkin>.
* **Serial**. This is the serial connection library that we use. It can be downloaded from <https://github.com/wjwwood/serial> and installed using the instructions in its readme. However, note that the Makefile specifies a wrong installation directory at the very beginning. You can just remove the `tmp` part from it. Also be sure to make the library known to the linker `ld` using the following commands.

```bash
echo /usr/local/lib > /etc/ld.so.conf.d/serial-lib.conf
sudo ldconfig
```

After installing these dependencies, the program can be built via `cmake` and `make`.

### Use

You can just directly start the executable. The server will wait for connections. Multiple companion apps can be connected at the same time. When the last companion app closes its connection, the server will automatically shut down.

## GUI Companion App

This program is located in the `companion` directory and consists of the file `companion.py`.

### Configuration

At the beginning of the source code, please specify the address of the server (IP and port) as well as the type (Python or C++).

### Dependencies

Install the needed dependencies using ```pip3 install -r requirements.txt```.

### Use

Use your favorite Python interpreter to start the program. Like for the Python server, the hotkeys **T** for tracking region selection, **A** for connecting the Arduino and **Q** for quitting are available.

## Context of the Project

This project was developed as a part of the *Embedded Operating Systems* lecture at HPI in the winter semester 2022/23. Four people took part in this project:

* Ella Rosner
* Lukas Rost
* Philipp Schimmelfennig
* Daniel Stachnik

### Slides

During the lecture, we held three presentations: A concept presentation, a midterm presentation and a final presentation. You can find the slides for them in the `slides` folder in this repository.

### Questions?

If you have any questions, do not hesitate to contact us at `firstname.lastname@student.hpi.de`.

### Thanks

We want to thank the team of the *Operating Systems & Middleware* chair at HPI for their support, especially Daniel Richter, we provided us with most of the hardware for this project.
