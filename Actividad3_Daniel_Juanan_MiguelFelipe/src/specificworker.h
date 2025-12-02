/*
*    Copyright (C) 2025 by YOUR NAME HERE
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

/**
    \brief
    @author authorname
*/


#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H


// If you want to reduce the period automatically due to lack of use, you must uncomment the following line
//#define HIBERNATION_ENABLED

#include <genericworker.h>
#include "abstract_graphic_viewer/abstract_graphic_viewer.h"
#include <expected>
#include <random>
#include <doublebuffer/DoubleBuffer.h>
#include "time_series_plotter.h"

// <--- AÑADIDOS
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <QGraphicsPolygonItem>
#include <QRectF>
#include <chrono>

#ifdef emit
#undef emit
#endif
#include <execution>
#include <tuple>
#include <utility>
#include <vector>
#include "common_types.h" // <--- AÑADIDO: Corners, Match, Lines, etc.
#include "room_detector.h"
#include "hungarian.h"
#include "nominal_room.h"
#include "door_detector.h"
#include "image_processor.h"

/**
 * \brief Class SpecificWorker implements the core functionality of the component.
 */
class SpecificWorker final : public GenericWorker
{
    Q_OBJECT
    public:
        /**
         * \brief Constructor for SpecificWorker.
         * \param configLoader Configuration loader for the component.
         * \param tprx Tuple of proxies required for the component.
         * \param startup_check Indicates whether to perform startup checks.
         */
        SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check);
        //void JoystickAdapter_sendData(RoboCompJoystickAdapter::TData data);
        ~SpecificWorker();

    public slots:
        void initialize() override; // <--- MODIFICADO: Añadido override
        void compute() override;    // <--- MODIFICADO: Añadido override
        void emergency() override;  // <--- MODIFICADO: Añadido override
        void restore() override;    // <--- MODIFICADO: Añadido override
        int startup_check();

    private:
        bool startup_check_flag;

    // ============================
    // PARÁMETROS DEL ROBOT / MUNDO
    // ============================
    struct Params
        {
            float ROBOT_WIDTH = 460.f;  // mm
            float ROBOT_LENGTH = 480.f;  // mm
            float MAX_ADV_SPEED = 1000.f; // mm/s
            float MAX_ROT_SPEED = 1.f; // rad/s
            float MAX_SIDE_SPEED = 50.f; // mm/s
            float MAX_TRANSLATION = 500.f; // mm/s
            float MAX_ROTATION = 0.2f;
            float STOP_THRESHOLD = 700.f; // mm
            float ADVANCE_THRESHOLD = ROBOT_WIDTH * 3.f; // mm
            float LIDAR_FRONT_SECTION = 0.2f; // rads, aprox 12 degrees
            // wall
            float LIDAR_RIGHT_SIDE_SECTION = M_PI/3.f; // rads, 90 degrees
            float LIDAR_LEFT_SIDE_SECTION = -M_PI/3.f; // rads, 90 degrees
            float WALL_MIN_DISTANCE = ROBOT_WIDTH*1.2f;
            // match error correction
            float MATCH_ERROR_SIGMA = 150.f; // mm
            float DOOR_REACHED_DIST = 300.f;
            std::string LIDAR_NAME_LOW = "bpearl";
            std::string LIDAR_NAME_HIGH = "helios";
            QRectF GRID_MAX_DIM{-5000, 2500, 10000, -5000};

            // relocalization
            float RELOCAL_CENTER_EPS = 300.f;    // mm: stop when |mean| < eps
            float RELOCAL_KP = 0.002f;           // gain to convert mean (mm) -> speed (magnitude)
            float RELOCAL_MAX_ADV = 300.f;       // mm/s cap while re-centering
            float RELOCAL_MAX_SIDE = 300.f;      // mm/s cap while re-centering
            float RELOCAL_ROT_SPEED = 0.6f;      // rad/s while aligning
            float RELOCAL_DELTA = 5.0f * M_PI/180.f; // small probe angle in radians
            float RELOCAL_MATCH_MAX_DIST = 2000.f;    // mm for Hungarian gating
            float RELOCAL_DONE_COST = 500.f;
            float RELOCAL_DONE_MATCH_MAX_ERROR = 1000.f;
            float CROSS_DOOR_SPEED  = 400.f; // <--- AÑADIDO
            float CROSS_DOOR_DURATION = 6.f; // <--- AÑADIDO
        };
        Params params;

        // viewer
        AbstractGraphicViewer *viewer = nullptr; // <--- MODIFICADO: Inicialización
        AbstractGraphicViewer *viewer_room = nullptr; // <--- MODIFICADO: Inicialización
        QGraphicsPolygonItem *robot_draw = nullptr; // <--- MODIFICADO: Inicialización
        QGraphicsPolygonItem *robot_room_draw = nullptr; // <--- MODIFICADO: Inicialización

        // robot
        Eigen::Affine2d robot_pose = Eigen::Affine2d::Identity(); // <--- MODIFICADO: Inicialización

        // rooms
        std::vector<NominalRoom> nominal_rooms{ NominalRoom{5500.f, 4000.f}, NominalRoom{8000.f, 4000.f}};
        rc::Room_Detector room_detector;
        rc::Hungarian hungarian;

        // state machine
        enum class STATE {GOTO_DOOR, ORIENT_TO_DOOR, LOCALISE, GOTO_ROOM_CENTER, TURN, IDLE, CROSS_DOOR, UPDATE_POSE}; // <--- MODIFICADO: Añadido UPDATE_POSE
        inline const char* to_string(const STATE s) const
        {
            switch(s) {
                case STATE::IDLE:                 return "IDLE";
                case STATE::LOCALISE:             return "LOCALISE";
                case STATE::GOTO_DOOR:            return "GOTO_DOOR";
                case STATE::TURN:                 return "TURN";
                case STATE::ORIENT_TO_DOOR:       return "ORIENT_TO_DOOR";
                case STATE::GOTO_ROOM_CENTER:     return "GOTO_ROOM_CENTER";
                case STATE::CROSS_DOOR:           return "CROSS_DOOR";
                case STATE::UPDATE_POSE:          return "UPDATE_POSE"; // <--- AÑADIDO
                default:                          return "UNKNOWN";
            }
        }
        STATE state = STATE::LOCALISE;
        using RetVal = std::tuple<STATE, float, float>;
        RetVal goto_door(const RoboCompLidar3D::TPoints &points);
        RetVal orient_to_door(const RoboCompLidar3D::TPoints &points);
        RetVal cross_door(const RoboCompLidar3D::TPoints &points);
        RetVal localise(const Match &match);
        RetVal goto_room_center(const RoboCompLidar3D::TPoints &points, const Lines &lines); // <--- MODIFICADO: Añadido const Lines &lines
        RetVal update_pose(const Corners &corners, const Match &match);
        RetVal turn(const Corners &corners);
        RetVal process_state(const RoboCompLidar3D::TPoints &data,
                             const Corners &corners,
                             const Lines &lines, // <--- MODIFICADO: Añadido const Lines &lines
                             const Match &match,
                             AbstractGraphicViewer *viewer);

        // draw
        void draw_lidar(const RoboCompLidar3D::TPoints &filtered_points,
                        std::optional<Eigen::Vector2d> center,
                        QGraphicsScene *scene);

        // aux
        RoboCompLidar3D::TPoints read_data();
        std::expected<int, std::string> closest_lidar_index_to_given_angle(const auto &points, float angle);
        RoboCompLidar3D::TPoints filter_same_phi(const RoboCompLidar3D::TPoints &points);
        RoboCompLidar3D::TPoints filter_isolated_points(const RoboCompLidar3D::TPoints &points, float d);
        void print_match(const Match &match, const float error =1.f) const;

        // random number generator
        std::random_device rd;

        // DoubleBuffer for velocity commands
        DoubleBuffer<std::tuple<float, float, float, long>, std::tuple<float, float, float, long>> commands_buffer;
        std::tuple<float, float, float, long> last_velocities{0.f, 0.f, 0.f, 0.f};

        // plotter
        std::unique_ptr<TimeSeriesPlotter> time_series_plotter;
        int match_error_graph = -1; // <--- MODIFICADO: Inicialización

        // doors
        DoorDetector door_detector;

        // image processor
        rc::ImageProcessor image_processor;

        // timing
        std::chrono::time_point<std::chrono::high_resolution_clock> last_time = std::chrono::high_resolution_clock::now();

        // relocalization
        bool relocal_centered = false;
        bool localised = false;

        // <--- AÑADIDOS
        bool red_patch_detected = false;
        bool crossing_door = false;
        bool crossed_door = false;
        std::chrono::time_point<std::chrono::high_resolution_clock> cross_door_start;
        float door_travel_target_mm = 0.f;
        // ---

        // Pose update & control
        bool update_robot_pose(const Corners &corners, const Match &match);
        void move_robot(float adv, float rot, float max_match_error);
        Eigen::Vector3d solve_pose(const Corners &corners, const Match &match);
        void predict_robot_pose();
        std::tuple<float, float> robot_controller(const Eigen::Vector2f &target);

    int current_room_id = 0;
    bool need_redraw_room = false;
    QString getStateName(STATE st ) const;
signals:
        //void customSignal();
};

#endif // SPECIFICWORKER_H