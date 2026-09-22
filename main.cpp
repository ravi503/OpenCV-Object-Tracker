#include <opencv2/opencv.hpp>
#include <iostream>
#include <algorithm>
#include <vector>

namespace
{
    struct SelectState
    {
        bool dragging = false;
        cv::Point origin;
        cv::Rect roi;
    };

    void onMouse(int event, int x, int y, int flags, void* userdata)
    {
        SelectState* state = reinterpret_cast<SelectState*>(userdata);
        if (event == cv::EVENT_LBUTTONDOWN)
        {
            state->dragging = true;
            state->origin = cv::Point(x, y);
            state->roi = cv::Rect(x, y, 0, 0);
        }
        else if (event == cv::EVENT_MOUSEMOVE && state->dragging)
        {
            state->roi = cv::Rect(state->origin, cv::Point(x, y));
        }
        else if (event == cv::EVENT_LBUTTONUP)
        {
            state->dragging = false;
            state->roi = cv::Rect(state->origin, cv::Point(x, y));
        }
    }

    double medianOf(std::vector<double>& values)
    {
        if (values.empty()) return 0.0;
        std::sort(values.begin(), values.end());
        size_t n = values.size();
        return (n % 2) ? values[n / 2] : (values[n / 2 - 1] + values[n / 2]) * 0.5;
    }
}

int main()
{
    std::vector<int> backends;
#ifdef _WIN32
    backends = { cv::CAP_DSHOW, cv::CAP_MSMF, cv::CAP_ANY };
#else
    backends = { cv::CAP_ANY };
#endif

    auto streamsFrames = [](cv::VideoCapture& cam) -> bool {
        int okCount = 0;
        for (int i = 0; i < 4; ++i)
            if (cam.grab()) ++okCount;
        return okCount >= 1;
    };

    cv::VideoCapture cap;
    int activeBackend = cv::CAP_ANY;
    for (int backend : backends)
    {
        cap.open(0, backend);
        if (cap.isOpened() && streamsFrames(cap))
        {
            activeBackend = backend;
            break;
        }
        cap.release();
    }
    if (!cap.isOpened())
    {
        std::cerr << "Could not open the laptop camera." << std::endl;
        return -1;
    }

    cap.set(cv::CAP_PROP_BUFFERSIZE, 2);

    cv::Mat frame, gray, prevGray;
    cv::TermCriteria criteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01);

    auto readFrame = [&]() -> bool {
        if (cap.read(frame)) return true;
        cap.release();
        if (!(cap.open(0, activeBackend) && streamsFrames(cap))) return false;
        return cap.read(frame);
    };

    const std::string windowName = "Object Tracker";
    cv::namedWindow(windowName, cv::WINDOW_NORMAL);

    SelectState sel;
    cv::setMouseCallback(windowName, onMouse, &sel);

    std::vector<cv::Point2f> prevPoints;
    const int maxPoints = 200;
    cv::Rect box;
    cv::Point2f prevCenter;
    bool tracking = false;
    int lostFrames = 0;

    auto detectFeatures = [&]()
    {
        cv::Rect imageRect(0, 0, gray.cols, gray.rows);
        box = box & imageRect;
        if (box.width < 8 || box.height < 8) return;
        cv::Mat mask = cv::Mat::zeros(gray.size(), CV_8UC1);
        mask(box).setTo(255);
        prevPoints.clear();
        cv::goodFeaturesToTrack(gray, prevPoints, maxPoints, 0.02, 10.0, mask, 3, false, 0.04);
    };

    auto startTracking = [&]()
    {
        cv::Rect imageRect(0, 0, gray.cols, gray.rows);
        cv::Rect roi = sel.roi & imageRect;
        if (roi.width < 8 || roi.height < 8) return;
        box = roi;
        prevCenter = cv::Point2f((float)(box.x + box.width / 2), (float)(box.y + box.height / 2));
        lostFrames = 0;
        tracking = false;
        detectFeatures();
        prevGray = gray.clone();
        sel.roi = cv::Rect();
    };

    double lastTick = (double)cv::getTickCount();
    double fpsSmooth = 0.0;

    while (true)
    {
        if (cv::getWindowProperty(windowName, cv::WND_PROP_VISIBLE) < 1.0)
            break;

        bool gotFrame = false;
        for (int retry = 0; retry < 60; ++retry)
        {
            if (readFrame()) { gotFrame = true; break; }
            cv::waitKey(5);
            if (cv::getWindowProperty(windowName, cv::WND_PROP_VISIBLE) < 1.0)
                return 0;
        }
        if (!gotFrame) break;

        double elapsed = (cv::getTickCount() - lastTick) / cv::getTickFrequency();
        lastTick = (double)cv::getTickCount();
        double fps = 1.0 / std::max(elapsed, 1e-6);
        fpsSmooth = (fpsSmooth == 0.0) ? fps : fpsSmooth * 0.9 + fps * 0.1;

        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        if (box.width >= 8 && box.height >= 8 && prevPoints.size() < 12)
        {
            detectFeatures();
            lostFrames = 0;
            prevGray = gray.clone();
        }

        cv::Mat status, err;
        std::vector<cv::Point2f> nextPoints;
        if (!prevPoints.empty())
            cv::calcOpticalFlowPyrLK(prevGray, gray, prevPoints, nextPoints, status, err,
                                     cv::Size(21, 21), 3, criteria);

        cv::Rect imageRect(0, 0, gray.cols, gray.rows);
        double maxMove = (box.width > 0 && box.height > 0)
                            ? (double)std::min(box.width, box.height) * 0.8 + 20.0
                            : 60.0;

        std::vector<cv::Point2f> goodPrev, goodNext;
        std::vector<double> dxs, dys;
        for (size_t i = 0; i < nextPoints.size(); ++i)
        {
            if (!status.at<uchar>((int)i)) continue;
            if (err.at<float>((int)i) > 50.0) continue;
            cv::Point2f p = nextPoints[i];
            cv::Point2f q = prevPoints[i];
            double d = cv::norm(p - q);
            if (d > maxMove) continue;
            if (!imageRect.contains(cv::Point((int)cvRound(p.x), (int)cvRound(p.y)))) continue;

            goodPrev.push_back(q);
            goodNext.push_back(p);
            dxs.push_back(p.x - q.x);
            dys.push_back(p.y - q.y);
        }

        if (box.width >= 8 && box.height >= 8 && !prevPoints.empty())
        {
            tracking = goodNext.size() >= 8;
            if (tracking)
            {
                double tx = medianOf(dxs);
                double ty = medianOf(dys);
                cv::Point2f newCenter(prevCenter.x + (float)tx, prevCenter.y + (float)ty);

                std::vector<double> ratios;
                for (size_t i = 0; i < goodNext.size(); ++i)
                {
                    double dPrev = cv::norm(goodPrev[i] - prevCenter);
                    double dNext = cv::norm(goodNext[i] - newCenter);
                    if (dPrev > 8.0) ratios.push_back(dNext / dPrev);
                }

                double scale = 1.0;
                if (ratios.size() >= 4)
                {
                    scale = medianOf(ratios);
                    scale = std::clamp(scale, 0.7, 1.3);
                }

                cv::Size newSize(cv::saturate_cast<int>(box.width * scale),
                                 cv::saturate_cast<int>(box.height * scale));
                box = cv::Rect((int)cvRound(newCenter.x - newSize.width / 2.0),
                               (int)cvRound(newCenter.y - newSize.height / 2.0),
                               newSize.width, newSize.height);
                prevCenter = newCenter;
                lostFrames = 0;
                prevPoints = goodNext;
            }
            else
            {
                ++lostFrames;
                prevPoints.clear();
            }

            box = box & imageRect;
        }

        if (sel.roi.width > 4 && sel.roi.height > 4)
            cv::rectangle(frame, sel.roi & imageRect, cv::Scalar(0, 255, 0), 2);

        if (box.width >= 8 && box.height >= 8)
        {
            cv::rectangle(frame, box, tracking ? cv::Scalar(0, 255, 255) : cv::Scalar(0, 0, 255), 2);
        }

        for (size_t i = 0; i < goodNext.size(); ++i)
            cv::circle(frame, goodNext[i], 2, cv::Scalar(0, 255, 0), -1);

        std::string statusText;
        if (box.width < 8 || box.height < 8)
            statusText = "Drag a box around the object, press SPACE to track";
        else if (tracking)
            statusText = "Tracking " + std::to_string(goodNext.size()) + " pts";
        else
            statusText = "Object lost";
        cv::putText(frame, statusText, cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    (box.width >= 8 && box.height >= 8) ? (tracking ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255))
                                                        : cv::Scalar(255, 255, 255),
                    2);
        cv::putText(frame, "space: track   r: reselect   q/ESC: quit", cv::Point(10, frame.rows - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
        std::string camText = std::to_string((int)cvRound(fpsSmooth)) + " fps  " +
                              std::to_string(frame.cols) + "x" + std::to_string(frame.rows) +
                              "  backend " + std::to_string(activeBackend);
        cv::putText(frame, camText, cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(255, 255, 0), 1);

        cv::imshow(windowName, frame);

        prevGray = gray.clone();

        int key = cv::waitKey(30);
        if (key == 27 || key == 'q') break;
        if (key == 32 || key == 13) startTracking();
        if (key == 'r')
        {
            box = cv::Rect();
            prevPoints.clear();
            sel.roi = cv::Rect();
        }
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
