# OpenCV Object Tracker

A lightweight real-time object tracker written in **C++ and OpenCV** using **Lucas–Kanade Pyramid Optical Flow**.

The program allows you to select an object manually with the mouse and then tracks its movement frame-by-frame using visual features inside the selected region.

## Features

* 🎯 Manual object selection using mouse drag
* 🔍 Automatic feature detection using `goodFeaturesToTrack()`
* 🚀 Real-time tracking using pyramidal Lucas–Kanade optical flow
* 📦 Dynamic bounding box movement
* 🔄 Approximate object scale tracking
* 📊 Tracking-point visualization
* 🧮 Robust median-based motion estimation
* 🔁 Automatic feature re-detection when tracking points become insufficient
* ⚠️ Object-lost detection
* 🎥 Automatic camera backend selection on Windows

  * DirectShow
  * Media Foundation
  * OpenCV default backend
* 📈 Real-time FPS and camera-resolution display
* 💻 Lightweight — no deep-learning model required

## How It Works

The tracker follows this basic pipeline:

```text
Camera
  │
  ▼
Capture Frame
  │
  ▼
Convert to Grayscale
  │
  ▼
User Selects Object
  │
  ▼
Detect Features Inside ROI
  │
  ▼
Lucas-Kanade Optical Flow
  │
  ▼
Filter Bad / Invalid Features
  │
  ▼
Calculate Median Motion
  │
  ▼
Update Object Position
  │
  ▼
Estimate Scale Change
  │
  ▼
Update Bounding Box
  │
  └──────────────► Next Frame
```

## Requirements

### Software

* C++17 or newer
* OpenCV 4.x
* A webcam or USB camera

### Supported Platforms

The project is primarily intended for:

* Windows
* Linux

Windows builds automatically try multiple camera backends to improve webcam compatibility.

## Build

### CMake

Create a `CMakeLists.txt` file:

```cmake
cmake_minimum_required(VERSION 3.16)

project(OpenCVObjectTracker)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

add_executable(ObjectTracker main.cpp)

target_link_libraries(ObjectTracker PRIVATE ${OpenCV_LIBS})

target_include_directories(ObjectTracker PRIVATE ${OpenCV_INCLUDE_DIRS})
```

Then build:

```bash
mkdir build
cd build
cmake ..
cmake --build . --config Release
```

### Windows

With Visual Studio:

```bash
cmake -S . -B build
cmake --build build --config Release
```

Run:

```bash
build\Release\ObjectTracker.exe
```

### Linux

```bash
cmake -S . -B build
cmake --build build -j
```

Run:

```bash
./build/ObjectTracker
```

## Usage

Start the application:

```bash
ObjectTracker
```

A camera window will appear.

### 1. Select an Object

Click and drag a rectangle around the object you want to track.

```text
+---------------------------------------+
|                                       |
|          ┌──────────────┐             |
|          │    OBJECT    │             |
|          │              │             |
|          └──────────────┘             |
|                                       |
+---------------------------------------+
```

### 2. Start Tracking

Press:

```text
SPACE
```

or:

```text
ENTER
```

The tracker will detect visual features inside the selected region and begin tracking them.

### 3. Reselect Object

Press:

```text
R
```

This clears the current tracking state and allows you to select another object.

### 4. Quit

Press:

```text
Q
```

or:

```text
ESC
```

## Controls

| Key / Action | Function        |
| ------------ | --------------- |
| Mouse drag   | Select object   |
| `SPACE`      | Start tracking  |
| `ENTER`      | Start tracking  |
| `R`          | Reselect object |
| `Q`          | Quit            |
| `ESC`        | Quit            |

## Tracking Algorithm

The tracker uses **sparse pyramidal Lucas–Kanade optical flow**.

Features are first detected inside the selected ROI using:

```cpp
cv::goodFeaturesToTrack()
```

The features are then tracked between consecutive grayscale frames using:

```cpp
cv::calcOpticalFlowPyrLK()
```

Only reliable points are retained based on:

* Optical-flow status
* Tracking error
* Maximum feature displacement
* Image boundaries

### Motion Estimation

Instead of using the average displacement of all points, the tracker calculates the **median displacement**:

```text
dx = median(feature_dx)
dy = median(feature_dy)
```

This makes the motion estimate less sensitive to individual incorrect feature tracks.

### Scale Estimation

The tracker also compares the distance of each feature from the object center between frames.

The median of these distance ratios is used to estimate approximate object scaling.

The scale is limited to:

```text
0.7x → 1.3x
```

to prevent sudden bounding-box explosions caused by incorrect optical-flow measurements.

## Tracking Status

The application displays three primary states:

### Waiting for Object

```text
Drag a box around the object, press SPACE to track
```

### Tracking

```text
Tracking 25 pts
```

The yellow bounding box indicates that the object is currently being tracked.

### Object Lost

```text
Object lost
```

The red bounding box indicates that insufficient reliable feature points were found.

## Feature Recovery

The tracker continuously monitors the number of valid feature points.

When the number drops below the configured threshold, new features are detected inside the current bounding box.

The default maximum number of tracking features is:

```cpp
const int maxPoints = 200;
```

Tracking requires at least:

```cpp
8
```

valid points.

## Configuration

Some parameters can be adjusted in the source code.

### Maximum Features

```cpp
const int maxPoints = 200;
```

Increase this for objects with complex textures.

### Minimum Feature Distance

```cpp
10.0
```

This controls the minimum distance between detected features.

### Optical Flow Window

```cpp
cv::Size(21, 21)
```

Larger windows can help with larger motion but increase computation.

### Pyramid Levels

```cpp
3
```

The tracker uses a three-level image pyramid.

### Tracking Error Threshold

```cpp
50.0
```

Features with larger optical-flow errors are rejected.

### Minimum Tracking Points

```cpp
goodNext.size() >= 8
```

The object is considered successfully tracked when at least eight valid features remain.

## Performance

Performance depends on:

* Camera resolution
* CPU
* Number of tracking features
* Optical-flow parameters
* Camera driver/backend

The application displays the measured FPS in the top-left corner.

Example:

```text
60 fps  1280x720  backend 700
```

## Limitations

This is a **feature-based tracker**, not an object-recognition system.

It does not understand what the object is.

For example, it may lose the object when:

* The object becomes heavily occluded
* The object rotates significantly
* The object has very few visual features
* Lighting changes dramatically
* The object leaves the camera frame
* The selected ROI contains a lot of background
* The background contains similar visual features
* The object moves extremely quickly

Because optical flow tracks visual features, background features can sometimes cause tracking drift.

## Recommended Improvements

Possible future improvements include:

* Kalman filtering
* Homography estimation using RANSAC
* Forward/backward optical-flow validation
* Better outlier rejection
* Object re-detection
* CSRT/KCF tracker integration
* YOLO-based object detection
* ByteTrack/DeepSORT-style multi-object tracking
* Camera calibration
* Object velocity estimation
* Target prediction
* ROS 2 integration
* LiDAR-camera fusion

## Robotics Applications

The tracker can serve as a lightweight vision component for robotics projects.

For example:

```text
Camera
   │
   ▼
OpenCV Tracker
   │
   ├── Object X
   ├── Object Y
   ├── Bounding Box
   └── Tracking Status
          │
          ▼
     Robot Controller
          │
          ▼
     Motor / Servo Control
```

This makes it possible to use the tracker as a front-end vision module for applications such as:

* Mobile robots
* Pan/tilt cameras
* Target following
* Object-following robots
* Vision-based navigation
* Robotic arms
* Autonomous platforms

## Project Structure

A simple project structure can be:

```text
OpenCVObjectTracker/
│
├── CMakeLists.txt
├── README.md
├── LICENSE
│
└── src/
    └── main.cpp
```

## License

This project is released under the MIT License.

See [LICENSE](LICENSE) for details.

## Author

Developed as an experimental real-time computer-vision and robotics project using C++ and OpenCV.

If you find this project useful, consider giving it a ⭐ on GitHub.
