#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

//#define HIBERNATION_ENABLED

#include <genericworker.h>
#include <vector>
#include <tuple>
#include <optional>
#include <random>
#include <execution>
#include <doublebuffer/DoubleBuffer.h>
#include "abstract_graphic_viewer/abstract_graphic_viewer.h"
#include "common_types.h"
#include "hungarian.h"
#include "room_detector.h"
#include "door_detector.h"
#include "image_processor.h"
#include "time_series_plotter.h"
#include "nominal_room.h"
#include "qcustomplot.h"
#include "ransac_line_detector.h"

// ===================== Enums =====================
enum class State {IDLE, FORWARD, TURN, SFO, SPIRAL};

enum class ROOM_STATE {GOTO_DOOR, ORIENT_TO_DOOR, LOCALISE, GOTO_ROOM_CENTER, TURN, IDLE, CROSS_DOOR};

// ===================== Class =====================
class SpecificWorker final : public GenericWorker
{
    Q_OBJECT
public:
    // Constructor / Destructor
    SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check);
    ~SpecificWorker();

public slots:
    void initialize();
    void compute();
    void emergency();
    void restore();
    int startup_check();
    void new_target_slot(QPointF point);
    void JoystickAdapter_sendData(RoboCompJoystickAdapter::TData data);

private:
    // ===================== Robot Geometry & Viewer =====================
    QRectF dimensions, room_dimensions;
    AbstractGraphicViewer *viewer = nullptr;
    AbstractGraphicViewer *room_viewer = nullptr;
    QGraphicsPolygonItem *robot_polygon = nullptr;
    QGraphicsPolygonItem *robot_draw = nullptr;
    QGraphicsPolygonItem *robot_room_draw = nullptr;
    Eigen::Affine2d robot_pose;

    const int ROBOT_LENGTH = 400;

    // ===================== Lidar =====================
    std::optional<RoboCompLidar3D::TPoints> filter_min_distance_cppitertools(const RoboCompLidar3D::TPoints& points);
    void draw_lidar(const RoboCompLidar3D::TPoints &points, std::optional<Eigen::Vector2d> center, QGraphicsScene* scene);
    RoboCompLidar3D::TPoints read_data();
    RoboCompLidar3D::TPoints filter_same_phi(const RoboCompLidar3D::TPoints &points);
    RoboCompLidar3D::TPoints filter_isolated_points(const RoboCompLidar3D::TPoints &points, float d);
    std::expected<int, std::string> closest_lidar_index_to_given_angle(const auto &points, float angle);

    // ===================== Rooms & Doors =====================
    std::vector<NominalRoom> nominal_rooms{ NominalRoom{5500.f, 4000.f}, NominalRoom{8000.f, 4000.f} };
    rc::Room_Detector room_detector;
    rc::Hungarian hungarian;
    DoorDetector door_detector;
    rc::ImageProcessor image_processor;

    // ===================== State Machines =====================
    // Basic navigation
    State state = State::FORWARD;
    std::tuple<State, float, float> FORWARD_method(const std::optional<RoboCompLidar3D::TPoints> &filtered_data);
    std::tuple<State, float, float> TURN_method(const std::optional<RoboCompLidar3D::TPoints> &filtered_data);
    std::tuple<State, float, float> FOLLOW_WALL_method(const std::optional<RoboCompLidar3D::TPoints> &filtered_data);
    std::tuple<State, float, float> SPIRAL_method(const std::optional<RoboCompLidar3D::TPoints> &filtered_data);
    std::tuple<State, float, float> stateMachine(const std::optional<RoboCompLidar3D::TPoints> &filtered_data);

    // Room navigation
    ROOM_STATE sm_state = ROOM_STATE::LOCALISE;
    using RetVal = std::tuple<ROOM_STATE, float, float>;
    RetVal goto_door(const RoboCompLidar3D::TPoints &points);
    RetVal orient_to_door(const RoboCompLidar3D::TPoints &points);
    RetVal cross_door(const RoboCompLidar3D::TPoints &points);
    RetVal localise(const Match &match);
    RetVal goto_room_center(const RoboCompLidar3D::TPoints &points);
    RetVal turn(const Corners &corners);
    RetVal update_pose(const Corners &corners, const Match &match);
    RetVal process_state(const RoboCompLidar3D::TPoints &data, const Corners &corners, const Match &match, AbstractGraphicViewer *viewer);

    // ===================== Relocalization =====================
    bool relocal_centered = false;
    bool localised = false;
    bool update_robot_pose(const Corners &corners, const Match &match);
    Eigen::Vector3d solve_pose(const Corners &corners, const Match &match);
    void predict_robot_pose();
    void move_robot(float adv, float rot, float max_match_error);
    std::tuple<float, float> robot_controller(const Eigen::Vector2f &target);

    // ===================== Parameters =====================
    struct Params
    {
        float ROBOT_WIDTH = 460.f;
        float ROBOT_LENGTH = 480.f;
        float MAX_ADV_SPEED = 1000.f;
        float MAX_ROT_SPEED = 1.f;
        float MAX_SIDE_SPEED = 50.f;
        float MAX_TRANSLATION = 500.f;
        float MAX_ROTATION = 0.2f;
        float STOP_THRESHOLD = 700.f;
        float ADVANCE_THRESHOLD = ROBOT_WIDTH*3;
        float LIDAR_FRONT_SECTION = 0.2f;
        float LIDAR_RIGHT_SIDE_SECTION = M_PI/3;
        float LIDAR_LEFT_SIDE_SECTION = -M_PI/3;
        float WALL_MIN_DISTANCE = ROBOT_WIDTH*1.2;
        float MATCH_ERROR_SIGMA = 150.f;
        float DOOR_REACHED_DIST = 300.f;
        std::string LIDAR_NAME_LOW = "bpearl";
        std::string LIDAR_NAME_HIGH = "helios";
        QRectF GRID_MAX_DIM{-5000, 2500, 10000, -5000};

        // Relocalization
        float RELOCAL_CENTER_EPS = 300.f;
        float RELOCAL_KP = 0.002f;
        float RELOCAL_MAX_ADV = 300.f;
        float RELOCAL_MAX_SIDE = 300.f;
        float RELOCAL_ROT_SPEED = 0.3f;
        float RELOCAL_DELTA = 5.f*M_PI/180.f;
        float RELOCAL_MATCH_MAX_DIST = 2000.f;
        float RELOCAL_DONE_COST = 500.f;
        float RELOCAL_DONE_MATCH_MAX_ERROR = 1000.f;
    };
    Params params;

    // ===================== Startup & Helpers =====================
    bool startup_check_flag = false;
    bool follow_right = true;
    bool spiral_done = false;

    // ===================== Plotter =====================
    std::unique_ptr<TimeSeriesPlotter> time_series_plotter;
    int match_error_graph = -1;

    // ===================== Random =====================
    std::random_device rd;

    // ===================== Timing =====================
    std::chrono::time_point<std::chrono::high_resolution_clock> last_time = std::chrono::high_resolution_clock::now();

signals:
    //void customSignal();
};

#endif
