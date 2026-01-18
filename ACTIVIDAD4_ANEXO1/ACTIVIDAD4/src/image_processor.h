#pragma once

#include <opencv2/opencv.hpp>
#include <tuple>
#include <string>
#include <vector>
#include <QString>

#include <QLabel>
#include <QImage>
#include <QPixmap>

#include <Camera360RGB.h>   // RoboCompCamera360RGB::TImage

namespace rc
{
class ImageProcessor
{
public:
    // Devuelve: (success, room_index, left_right)
    // room_index: 0 = rojo, 1 = verde, -1 = ninguno
    // left_right: -1 si el parche está a la izquierda, +1 si a la derecha (para decidir el giro)
    std::tuple<bool, int, int> check_colour_patch_in_image(
        const RoboCompCamera360RGB::TImage &img,
        QLabel *label_img)
    {
        const auto red   = detect_patch(img, label_img, Patch::RED);
        const auto green = detect_patch(img, label_img, Patch::GREEN);

        if (!red.valid && !green.valid)
            return {false, -1, 1};

        if (red.valid && !green.valid)
            return {true, 0, red.left_right};

        if (!red.valid && green.valid)
            return {true, 1, green.left_right};

        // ambos válidos -> elegir el de mayor score
        if (red.score >= green.score)
            return {true, 0, red.left_right};
        else
            return {true, 1, green.left_right};
    }

    QString room_name_from_index(int idx) const
    {
        if (idx == 0) return QStringLiteral("red");
        if (idx == 1) return QStringLiteral("green");
        return QStringLiteral("unknown");
    }

private:
    enum class Patch { RED, GREEN };

    struct PatchResult
    {
        bool valid = false;
        int left_right = 1;      // -1 izquierda, +1 derecha
        float score = 0.f;
        cv::Point2f center = {0.f, 0.f};
        int non_zero = 0;
    };

    static PatchResult detect_patch(
        const RoboCompCamera360RGB::TImage &img,
        QLabel *label_img,
        Patch patch,
        int min_nonzero = 800)
    {
        PatchResult out;

        // Entrada: normalmente RGB (Camera360RGB)
        cv::Mat cv_img(img.height, img.width, CV_8UC3, (void*)img.image.data());
        cv::Mat display_img = cv_img.clone();

        // RGB -> HSV  (CLAVE para no fallar el color)
        cv::Mat hsv_img;
        cv::cvtColor(cv_img, hsv_img, cv::COLOR_RGB2HSV);

        cv::Mat mask;
        if (patch == Patch::RED)
        {
            cv::Mat m1, m2;
            cv::inRange(hsv_img, cv::Scalar(0,   80, 80),  cv::Scalar(10,  255, 255), m1);
            cv::inRange(hsv_img, cv::Scalar(170, 80, 80),  cv::Scalar(180, 255, 255), m2);
            mask = m1 | m2;
        }
        else // GREEN
        {
            cv::inRange(hsv_img, cv::Scalar(35, 80, 80), cv::Scalar(85, 255, 255), mask);
        }

        // Limpieza
        cv::erode(mask, mask, cv::Mat(), cv::Point(-1,-1), 1);
        cv::dilate(mask, mask, cv::Mat(), cv::Point(-1,-1), 2);

        out.non_zero = cv::countNonZero(mask);
        if (out.non_zero < min_nonzero)
        {
            if (label_img != nullptr)
            {
                QImage qimg(display_img.data, display_img.cols, display_img.rows,
                           (int)display_img.step, QImage::Format_RGB888);
                label_img->setPixmap(QPixmap::fromImage(qimg));
            }
            out.valid = false;
            out.score = (float)out.non_zero;
            return out;
        }

        // Contorno mayor
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        if (contours.empty())
        {
            out.valid = false;
            out.score = (float)out.non_zero;
            return out;
        }

        auto max_it = std::max_element(contours.begin(), contours.end(),
                                       [](const auto &a, const auto &b)
                                       { return cv::contourArea(a) < cv::contourArea(b); });

        const double area = cv::contourArea(*max_it);
        auto mu = cv::moments(*max_it);
        if (std::abs(mu.m00) < 1e-6)
        {
            out.valid = false;
            out.score = (float)out.non_zero;
            return out;
        }

        out.center = cv::Point2f((float)(mu.m10/mu.m00), (float)(mu.m01/mu.m00));
        out.left_right = (out.center.x < display_img.cols/2.f) ? -1 : 1;

        out.score = (float)area + 0.1f * (float)out.non_zero;
        out.valid = true;

        // Overlay debug
        cv::circle(display_img, out.center, 10, cv::Scalar(255,255,255), -1);
        cv::line(display_img,
                 cv::Point(display_img.cols/2, 0),
                 cv::Point(display_img.cols/2, display_img.rows),
                 cv::Scalar(255,255,255), 2);

        if (label_img != nullptr)
        {
            QImage qimg(display_img.data, display_img.cols, display_img.rows,
                       (int)display_img.step, QImage::Format_RGB888);
            label_img->setPixmap(QPixmap::fromImage(qimg));
        }

        return out;
    }
};
} // namespace rc
