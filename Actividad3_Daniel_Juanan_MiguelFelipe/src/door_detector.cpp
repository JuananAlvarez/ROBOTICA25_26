//
// Created by pbustos on 11/11/25.
//

#include "door_detector.h"

#include <expected>
#include <cppitertools/sliding_window.hpp>
#include <cppitertools/combinations.hpp>
#include <QGraphicsItem>

Doors DoorDetector::detect(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene)
{
    Doors detected_doors;
    doors_cache.clear();

    if (points.size() < 2)
        return detected_doors;

    // ========== 1) Localizar saltos bruscos (peaks) ==========
    Peaks raw_peaks;
    raw_peaks.reserve(points.size());

    for (const auto &pair : iter::sliding_window(points, 2))
    {
        const auto &a = pair[0];
        const auto &b = pair[1];
        const float gap = std::fabs(a.distance2d - b.distance2d);

        if (gap > 1000.f)   // umbral
        {
            const auto &closest = (a.distance2d < b.distance2d ? a : b);
            raw_peaks.emplace_back(Eigen::Vector2f{closest.x, closest.y}, closest.phi);
        }
    }

    // ========== 2) Dibujar peaks (solo depuración) ==========
    if (scene)
    {
        static std::vector<QGraphicsItem*> graphic_peaks;

        // limpiar dibujos previos
        for (auto *g : graphic_peaks)
        {
            scene->removeItem(g);
            delete g;
        }
        graphic_peaks.clear();

        QPen p(Qt::yellow);
        p.setWidth(10);

        for (const auto &[pos, ang] : raw_peaks)
        {
            Q_UNUSED(ang);
            auto rect = scene->addRect(-20, -20, 40, 40, p);
            rect->setPos(pos.x(), pos.y());
            graphic_peaks.push_back(rect);
        }
    }

    // ========== 3) Non-Maximum Suppression ==========
    Peaks filtered_peaks;
    filtered_peaks.reserve(raw_peaks.size());

    for (const auto &[pos, ang] : raw_peaks)
    {
        bool neighbour_found = false;

        for (const auto &[p2, ang2] : filtered_peaks)
        {
            Q_UNUSED(ang2);
            if ((pos - p2).norm() < 500.f)
            {
                neighbour_found = true;
                break;
            }
        }

        if (!neighbour_found)
            filtered_peaks.emplace_back(pos, ang);
    }

    // ========== 4) Emparejar peaks para formar puertas ==========
    for (auto &&pair : iter::combinations(filtered_peaks, 2))
    {
        const auto &[pa, angA] = pair[0];
        const auto &[pb, angB] = pair[1];

        float separation = (pa - pb).norm();

        if (separation >= 800.f && separation <= 1200.f)
        {
            detected_doors.emplace_back(Door{pa, angA, pb, angB});
        }
    }

    doors_cache = detected_doors;
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

std::expected<Door, std::string> DoorDetector::get_current_door() const
{
    if (doors_cache.empty())
        return std::unexpected<std::string>{"No doors detected"};
    return doors_cache[0];
}
