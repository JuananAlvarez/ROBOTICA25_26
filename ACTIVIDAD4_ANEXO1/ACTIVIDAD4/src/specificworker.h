/*
 *    Copyright (C) 2025 by G3 {Guadalupe Gonzalez Santos, Maximo Bueno Martinez & Jose Antonio Bravo Romero}
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

// If you want to reduce the period automatically due to lack of use, you must uncomment the following line
//#define HIBERNATION_ENABLED

#include <genericworker.h>

// Qt
#include <QGraphicsLineItem>
#include <QGraphicsPolygonItem>
#include <QGraphicsRectItem>
#include <QRectF>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

// STL
#include <chrono>
#include <expected>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <tuple>
#include <utility>
#include <vector>
#include <MNIST.h>
// RoboComp / project
#include "abstract_graphic_viewer/abstract_graphic_viewer.h"
#include "door_crossing_tracker.h"
#include "door_detector.h"
#include "hungarian.h"
#include "image_processor.h"
#include "nominal_room.h"
#include "pointcloud_center_estimator.h"
#include "room_detector.h"
#include "time_series_plotter.h"

/**
 * \brief Class SpecificWorker implements the core functionality of the component.
 */
class SpecificWorker final : public GenericWorker
{
    Q_OBJECT

public:
    SpecificWorker(const ConfigLoader &configLoader, TuplePrx tprx, bool startup_check);
    ~SpecificWorker() override;

public slots:
    void initialize() override;
    void compute() override;
    void emergency() override;
    void restore() override;
    int startup_check();

private:
    bool startup_check_flag = false;
    bool relocalised_once = false;
    int relocalised_room = -1;
    int current_room = -1;
    int last_door_used = -1;

    struct Params
    {
        float ROBOT_WIDTH = 460.f;   // mm
        float ROBOT_LENGTH = 480.f;  // mm
        float MAX_ADV_SPEED = 1000.f;
        float MAX_ROT_SPEED = 1.f;
        float MAX_SIDE_SPEED = 50.f;
        float MAX_TRANSLATION = 500.f;
        float MAX_ROTATION = 0.2f;
        float STOP_THRESHOLD = 700.f;
        float ADVANCE_THRESHOLD = ROBOT_WIDTH * 3.f;
        float LIDAR_FRONT_SECTION = 0.2f;
        float LIDAR_RIGHT_SIDE_SECTION = static_cast<float>(M_PI) / 3.f;
        float LIDAR_LEFT_SIDE_SECTION = -static_cast<float>(M_PI) / 3.f;
        float WALL_MIN_DISTANCE = ROBOT_WIDTH * 1.2f;
        float MATCH_ERROR_SIGMA = 150.f;
        float DOOR_REACHED_DIST = 500.f;
        std::string LIDAR_NAME_LOW = "bpearl";
        std::string LIDAR_NAME_HIGH = "helios";
        QRectF GRID_MAX_DIM{-5000, 2500, 10000, -5000};

        // relocalization
        float RELOCAL_CENTER_EPS = 300.f;
        float RELOCAL_KP = 0.002f;
        float RELOCAL_MAX_ADV = 300.f;
        float RELOCAL_MAX_SIDE = 300.f;
        float RELOCAL_ROT_SPEED = 0.3f;
        float RELOCAL_DELTA = 5.0f * static_cast<float>(M_PI) / 180.f;
        float RELOCAL_MATCH_MAX_DIST = 2000.f;
        float RELOCAL_DONE_COST = 500.f;
        float RELOCAL_DONE_MATCH_MAX_ERROR = 3000.f;
        float RELOCAL_MIN_DISTANCE_TO_DOOR = 700.f;
        float RELOCAL_MAX_ORIENTED_ERROR = 0.1f;
    };
    Params params;

    // viewer
    AbstractGraphicViewer *viewer = nullptr;
    AbstractGraphicViewer *viewer_room = nullptr;
    QGraphicsPolygonItem *robot_draw = nullptr;
    QGraphicsPolygonItem *robot_room_draw = nullptr;

    // robot
    Eigen::Affine2f robot_pose = Eigen::Affine2f::Identity();

    // rooms
    std::vector<NominalRoom> nominal_rooms{NominalRoom{5500.f, 4000.f}, NominalRoom{8000.f, 4000.f}};
    rc::Room_Detector room_detector;
    rc::Hungarian hungarian;
    QGraphicsRectItem *habitacion = nullptr;
    std::vector<QGraphicsLineItem *> puertas;
    int current_door = -1;
    int room_index = -1;

    // state machine
    enum class STATE
    {
        GOTO_DOOR,
        ORIENT_TO_DOOR,
        LOCALISE,
        GOTO_ROOM_CENTER,
        TURN,
        IDLE,
        CROSS_DOOR
    };

    inline const char *to_string(const STATE s) const
    {
        switch (s)
        {
        case STATE::IDLE:
            return "IDLE";
        case STATE::LOCALISE:
            return "LOCALISE";
        case STATE::GOTO_DOOR:
            return "GOTO_DOOR";
        case STATE::TURN:
            return "TURN";
        case STATE::ORIENT_TO_DOOR:
            return "ORIENT_TO_DOOR";
        case STATE::GOTO_ROOM_CENTER:
            return "GOTO_ROOM_CENTER";
        case STATE::CROSS_DOOR:
            return "CROSS_DOOR";
        default:
            return "UNKNOWN";
        }
    }

    using RetVal = std::tuple<STATE, float, float>;
    RetVal goto_door();
    RetVal orient_to_door();
    RetVal cross_door(const RoboCompLidar3D::TPoints &points);
    RetVal localise(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene);
    RetVal IDLE_method();
    RetVal goto_room_center(const RoboCompLidar3D::TPoints &points);
    RetVal TURN_method(const Corners &corners);
    std::tuple<STATE, float, float> state_machine(STATE state, const RoboCompLidar3D::TPoints &filter_data, const Corners &corners);

    // aux
    RoboCompLidar3D::TPoints read_data();
    void draw_lidar(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene);
    std::optional<RoboCompLidar3D::TPoints> data_filter(const RoboCompLidar3D::TPoints &puntos);
    std::expected<int, std::string> closest_lidar_index_to_given_angle(const RoboCompLidar3D::TPoints &points, float angle);

    // controller
    std::tuple<float, float> robot_controller(const Eigen::Vector2f &target);

    // plotter
    std::unique_ptr<TimeSeriesPlotter> time_series_plotter;
    int match_error_graph = -1;

    // doors
    DoorDetector door_detector;
    rc::PointcloudCenterEstimator center_estimator;
    rc::ImageProcessor image_processor;
    DoorCrossing door_crossing;

    // timing
    std::chrono::time_point<std::chrono::high_resolution_clock> last_time = std::chrono::high_resolution_clock::now();
    bool localised = false;

    // pose update
    std::optional<std::pair<Eigen::Affine2f, float>> update_robot_pose(int room_index,
                                                                       const Corners &corners,
                                                                       const Eigen::Affine2f &r_pose,
                                                                       bool transform_corners);
    void choose_next_door(int current_room);
};

#endif // SPECIFICWORKER_H