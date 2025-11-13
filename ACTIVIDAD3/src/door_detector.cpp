//
// Created by pbustos on 11/11/25.
//

#include "door_detector.h"
#include <cppitertools/sliding_window.hpp>
#include <cppitertools/combinations.hpp>
#include <QGraphicsItem>

Doors DoorDetector::detect(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene)
{
    Doors detected_doors;
    std::vector<RoboCompLidar3D::TPoint> peaks;

    // --- 1. Detect peaks (large jumps in distance)
    for (const auto &[p1, p2] : iter::sliding_window<2>(points))
    {
        float diff = std::abs(p1.distance2d - p2.distance2d);
        if (diff > 1000)   // threshold in mm
        {
            // add the closer one
            peaks.push_back((p1.distance2d < p2.distance2d) ? p1 : p2);
        }
    }

    // --- 2. Draw peaks on scene (for debugging)
    if (scene)
    {
        for (const auto &p : peaks)
        {
            float x = p.x;
            float y = -p.y; // adjust coordinate if needed
            scene->addEllipse(x - 50, y - 50, 100, 100, QPen(Qt::red), QBrush(Qt::red));
        }
    }

    // --- 3. Non-Maximum Suppression (remove close peaks)
    std::vector<RoboCompLidar3D::TPoint> filtered_peaks;
    const float MIN_PEAK_DIST = 200; // mm
    for (const auto &p : peaks)
    {
        bool too_close = false;
        for (const auto &fp : filtered_peaks)
        {
            if ((Eigen::Vector2f(p.x, p.y) - Eigen::Vector2f(fp.x, fp.y)).norm() < MIN_PEAK_DIST)
            {
                too_close = true;
                break;
            }
        }
        if (!too_close)
            filtered_peaks.push_back(p);
    }

    // --- 4. Form doors from pairs of peaks
    for (const auto &[p1, p2] : iter::combinations(filtered_peaks, 2))
    {
        float dist = (Eigen::Vector2f(p1.x, p1.y) - Eigen::Vector2f(p2.x, p2.y)).norm();
        if (dist > 800 && dist < 1200)
        {
            Door d;
            d.p1 = Eigen::Vector2f(p1.x, p1.y);
            d.p2 = Eigen::Vector2f(p2.x, p2.y);
            d.p1_angle = p1.phi;
            d.p2_angle = p2.phi;
            detected_doors.push_back(d);
        }
    }

    return detected_doors;
}

// Method to use the Doors vector to filter out the LiDAR points that como from a room outside the current one
RoboCompLidar3D::TPoints DoorDetector::filter_points(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene)
{
    const auto doors = detect(points, scene);
    if(doors.empty()) return points;

    // for each door, check if the distance from the robot to each lidar point is smaller than the distance from the robot to the door
    RoboCompLidar3D::TPoints filtered;
    for(const auto &d : doors)
    {
        const float dist_to_door = d.center().norm();
        // Check if the angular range wraps around the -π/+π boundary
        const bool angle_wraps = d.p2_angle < d.p1_angle;
        for(const auto &p : points)
        {
            // Determine if point is within the door's angular range
            bool point_in_angular_range;
            if (angle_wraps)
            {
                // If the range wraps around, point is in range if it's > p1_angle OR < p2_angle
                point_in_angular_range = (p.phi > d.p1_angle) or (p.phi < d.p2_angle);
            }
            else
            {
                // Normal case: point is in range if it's between p1_angle and p2_angle
                point_in_angular_range = (p.phi > d.p1_angle) and (p.phi < d.p2_angle);
            }

            // Filter out points that are through the door (in angular range and farther than door)
            if(point_in_angular_range and p.distance2d >= dist_to_door)
                continue;

            //qInfo() << __FUNCTION__ << "Point angle: " << p.phi << " Door angles: " << d.p1_angle << ", " << d.p2_angle << " Point distance: " << p.distance2d << " Door distance: " << dist_to_door;
            filtered.emplace_back(p);
        }
    }
    return filtered;
}
