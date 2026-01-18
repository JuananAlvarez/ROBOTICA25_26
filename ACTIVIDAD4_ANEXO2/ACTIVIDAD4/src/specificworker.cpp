/*
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

#include "specificworker.h"

#include <algorithm>
#include <cmath>
#include <cppitertools/enumerate.hpp>
#include <cppitertools/groupby.hpp>
#include <iostream>
#include <ranges>
#include <algorithm>   // std::clamp

// Qt
#include <QCoreApplication>
#include <QPen>
#include <QTimer>
#include <limits>

// Ice
#include <IceUtil/StringUtil.h>

/**
 * @brief Constructor for the SpecificWorker class.
 */
SpecificWorker::SpecificWorker(const ConfigLoader &configLoader, TuplePrx tprx, bool startup_check)
    : GenericWorker(configLoader, tprx)
{
    this->startup_check_flag = startup_check;
    if (this->startup_check_flag)
    {
        this->startup_check();
    }
    else
    {
#ifdef HIBERNATION_ENABLED
        hibernationChecker.start(500);
#endif

        statemachine.setChildMode(QState::ExclusiveStates);
        statemachine.start();

        auto error = statemachine.errorString();
        if (error.length() > 0)
        {
            qWarning() << error;
            throw error;
        }
    }
}

SpecificWorker::~SpecificWorker()
{
    std::cout << "Destroying SpecificWorker" << std::endl;
}

void SpecificWorker::initialize()
{
    std::cout << "Initialize worker" << std::endl;
    if (this->startup_check_flag)
    {
        this->startup_check();
        return;
    }

    // Viewer (LiDAR)
    viewer = new AbstractGraphicViewer(this->frame, params.GRID_MAX_DIM);
    auto [r, e] = viewer->add_robot(params.ROBOT_WIDTH, params.ROBOT_LENGTH, 0, 100, QColor("Blue"));
    robot_draw = r;

    // Viewer room (map)
    viewer_room = new AbstractGraphicViewer(this->frame_room, params.GRID_MAX_DIM);
    auto [rr, re] = viewer_room->add_robot(params.ROBOT_WIDTH, params.ROBOT_LENGTH, 0, 100, QColor("Blue"));
    robot_room_draw = rr;

    // initialise robot pose
    robot_pose.setIdentity();
    robot_pose.translate(Eigen::Vector2f(0.0f, 0.0f));

    // time series plotter for match error
    TimeSeriesPlotter::Config plotConfig;
    plotConfig.title = "Maximum Match Error Over Time";
    plotConfig.yAxisLabel = "Error (mm)";
    plotConfig.timeWindowSeconds = 15.0;
    plotConfig.autoScaleY = false;
    plotConfig.yMin = 0;
    plotConfig.yMax = 1000;
    time_series_plotter = std::make_unique<TimeSeriesPlotter>(frame_plot_error, plotConfig);
    match_error_graph = time_series_plotter->addGraph("", Qt::blue);
    try
    {
        auto img = camera360rgb_proxy->getROI(-1, -1, -1, -1, -1, -1);
        camera_width  = img.width;
        camera_height = img.height;
        qInfo() << "[INIT] Camera360RGB size =" << camera_width << "x" << camera_height;
    }
    catch(const Ice::Exception &e)
    {
        qWarning() << "[INIT] Cannot read Camera360RGB size. Using fallback 1800x900. Error:" << e.what();
        camera_width  = 1800;
        camera_height = 900;
    }

}

void SpecificWorker::compute()
{
    qInfo() << "Computing SpecificWorker";

    // If stop button exists and is checked, stop the robot and return
    if (pushButton_stop && pushButton_stop->isChecked())
    {
        try { omnirobot_proxy->setSpeedBase(0.f, 0.f, 0.f); }
        catch (const Ice::Exception &e) { std::cout << e << " setSpeedBase error" << std::endl; }
        return;
    }

    using Ret = std::tuple<STATE, float, float>;
    static STATE state = STATE::GOTO_ROOM_CENTER;   // Estado inicial

    // 1) Read + filter LiDAR
    auto filter_data = read_data();
    filter_data = door_detector.filter_points(filter_data, &viewer->scene);

    // Draw lidar in robot viewer
    draw_lidar(filter_data, &viewer->scene);

    // 2) Compute corners
    const auto &[measured_corners, _] = room_detector.compute_corners(filter_data, &viewer->scene);

    float max_match_error = -1.f;

    // 3) Update pose if already localised
    if (localised)
    {
        if (const auto res = update_robot_pose(room_index, measured_corners, robot_pose, true); res.has_value())
        {
            robot_pose = res.value().first;
            max_match_error = res.value().second;
            if (time_series_plotter)
                time_series_plotter->addDataPoint(match_error_graph, max_match_error);
        }
    }

    // 4) State machine -> commands
    const auto &[st, adv, rot] = state_machine(state, filter_data, measured_corners);
    state = st;

    qInfo() << "Estado actual ---------------" << to_string(state);

    // 5) Send speeds
    try { omnirobot_proxy->setSpeedBase(0.f, adv, rot); }
    catch (const Ice::Exception &e)
    {
        std::cout << e << " " << "Conexión con OmniRobot" << std::endl;
        return;
    }

    // 6) Draw robot in room viewer
    if (robot_room_draw)
    {
        robot_room_draw->setPos(robot_pose.translation().x(), robot_pose.translation().y());
        const double angle_deg = qRadiansToDegrees(std::atan2(robot_pose.rotation()(1, 0), robot_pose.rotation()(0, 0)));
        robot_room_draw->setRotation(angle_deg);

        // 7) Update GUI (your .ui widgets)
        if (time_series_plotter)
            time_series_plotter->update();

        if (label_state)
            label_state->setText(to_string(state));

        // You DON'T have label_localized nor lcdNumber_room in your .ui, so we show that info in label_img
        if (label_img)
            label_img->setText(QString("localised: %1   room: %2   match_err: %3")
                                   .arg(localised ? "yes" : "no")
                                   .arg(room_index)
                                   .arg(max_match_error, 0, 'f', 1));

        if (lcdNumber_adv)   lcdNumber_adv->display(adv);
        if (lcdNumber_rot)   lcdNumber_rot->display(rot);
        if (lcdNumber_x)     lcdNumber_x->display(robot_pose.translation().x());
        if (lcdNumber_y)     lcdNumber_y->display(robot_pose.translation().y());
        if (lcdNumber_angle) lcdNumber_angle->display(angle_deg);
    }

    last_time = std::chrono::high_resolution_clock::now();
}


// ---------------------------
// --- STATE METHODS
// ---------------------------

SpecificWorker::RetVal SpecificWorker::goto_door()
{
    static int near_count = 0;         // histéresis cerca de puerta
    constexpr int NEAR_N = 3;

    Doors doors = door_detector.doors();
    if (doors.empty())
    {
        near_count = 0;
        qWarning() << "[GOTO_DOOR] No doors detected -> searching (turning slowly)";
        return {STATE::GOTO_DOOR, 0.f, 0.25f};
    }

    if (room_index < 0 || room_index >= static_cast<int>(nominal_rooms.size()))
    {
        qCritical() << "[GOTO_DOOR] room_index out of range:" << room_index;
        near_count = 0;
        return {STATE::LOCALISE, 0.f, 0.f};
    }

    if (current_door < 0 || current_door >= static_cast<int>(nominal_rooms[room_index].doors.size()))
        current_door = 0;

    const Door &nominal_door = nominal_rooms[room_index].doors[current_door];

    const Eigen::Vector2f nominal_center_robot =
        robot_pose.inverse().cast<float>() * nominal_door.global_center();

    const auto target_it = std::ranges::min_element(
        doors,
        [&nominal_center_robot](const auto &a, const auto &b)
        {
            return (a.center() - nominal_center_robot).norm() <
                   (b.center() - nominal_center_robot).norm();
        });

    Door target_door = *target_it;

    Eigen::Vector2f centro = target_door.center();
    const float angulo = std::atan2(centro.x(), centro.y());
    const float dist   = centro.norm();

    // histéresis: exige varios frames seguidos "cerca"
    if (dist < 800.f)
        near_count++;
    else
        near_count = 0;

    if (near_count >= NEAR_N)
    {
        near_count = 0;
        qInfo() << "[GOTO_DOOR] near door -> ORIENT_TO_DOOR";
        return {STATE::ORIENT_TO_DOOR, 0.f, 0.f};
    }

    const float k_rot = 1.0f;
    float vrot = std::clamp(k_rot * angulo, -0.6f, 0.6f);
    const float brake = std::exp(-angulo * angulo / (M_PI / 10.f));
    const float adv   = 500.f * brake;

    return {STATE::GOTO_DOOR, adv, vrot};
}


SpecificWorker::RetVal SpecificWorker::orient_to_door()
{
    const auto doorsy = door_detector.doors();

    // Si no hay puertas, vuelve a buscarlas (no devuelvas {})
    if (doorsy.empty())
    {
        qWarning() << "[ORIENT_TO_DOOR] No doors -> back to GOTO_DOOR";
        return {STATE::GOTO_DOOR, 0.f, 0.25f};
    }

    // puerta con menor ángulo absoluto
    const auto sd = std::ranges::min_element(
        doorsy,
        [](const auto &a, const auto &b)
        { return std::fabs(a.center_angle()) < std::fabs(b.center_angle()); });

    const auto centro = sd->center();

    // OJO: esta atan2 está al revés en tu código original.
    // Lo correcto (ángulo del target respecto al eje Y "frontal"):
    const float angulo = std::atan2(centro.x(), centro.y());

    const float k = 0.8f;
    float w = std::clamp(k * angulo, -0.6f, 0.6f);

    // Si está alineado, cruza
    if (std::abs(angulo) < 0.10f)
    {
        localised = false;                    // al cruzar pierdes localización
        qInfo() << "[ORIENT_TO_DOOR] aligned -> CROSS_DOOR";
        return {STATE::CROSS_DOOR, 0.f, 0.f};
    }

    return {STATE::ORIENT_TO_DOOR, 0.f, w};
}

SpecificWorker::RetVal SpecificWorker::cross_door(const RoboCompLidar3D::TPoints &points)
{
    static int ticks_in_cross = 0;
    static float traveled_mm = 0.f;   // estimación simple de avance acumulado
    static int door_lost_count = 0;
    static int clear_count = 0;

    ticks_in_cross++;

    // ---- TIMEOUT duro anti-bucle ----
    constexpr int MAX_TICKS_CROSS = 250;   // ~5s si 20Hz
    if(ticks_in_cross > MAX_TICKS_CROSS)
    {
        qWarning() << "[CROSS_DOOR] TIMEOUT -> forcing LOCALISE";
        ticks_in_cross = 0;
        traveled_mm = 0.f;
        door_lost_count = 0;
        clear_count = 0;

        localised = false;
        relocalised_once = false;
        relocalised_room = -1;
        return {STATE::LOCALISE, 0.f, 0.f};
    }

    // ---- Seguridad frontal ----
    float front_min = std::numeric_limits<float>::max();
    for(const auto &p : points)
        if(std::fabs(p.phi) < 0.20f)
            front_min = std::min(front_min, p.r);

    if(front_min < 650.f)
    {
        // si estás muy cerca, retrocede un poco sin girar tanto (para no "escaparte" a la derecha)
        return {STATE::CROSS_DOOR, -120.f, 0.20f};
    }

    // ---- Control: avanzar + rotación limitada ----
    float adv = 420.f;
    float rot = 0.f;

    const auto doors = door_detector.doors();
    if(!doors.empty())
    {
        // puerta más frontal
        const auto it = std::ranges::min_element(doors, [](const auto &a, const auto &b)
        {
            return std::fabs(a.center_angle()) < std::fabs(b.center_angle());
        });

        float ang = it->center_angle();
        // rotación MUY limitada para no irse hacia paredes
        rot = std::clamp(0.6f * ang, -0.25f, 0.25f);
        door_lost_count = 0;
    }
    else
    {
        // sin puerta detectada: contamos pérdida
        door_lost_count++;
        rot = 0.f;
        adv = 350.f;
    }

    // ---- integrar distancia recorrida (aprox) ----
    // Si tu compute es 50ms (20Hz), cada tick recorre adv*0.05
    // No tenemos dt exacto aquí; usamos un dt fijo razonable:
    constexpr float DT = 0.05f;   // 50 ms
    traveled_mm += std::max(0.f, adv) * DT;

    // ---- Condición de salida REAL: haber avanzado lo suficiente + perder puerta ----
    // Esto evita salir antes de cruzar y evita quedarse para siempre.
    constexpr float MIN_TRAVEL_TO_DECLARE_INSIDE = 1200.f;  // 1.2m atravesando y despejando
    constexpr int DOOR_LOST_N = 6;

    if(traveled_mm > MIN_TRAVEL_TO_DECLARE_INSIDE && door_lost_count >= DOOR_LOST_N)
    {
        // Clear extra (salir del umbral)
        clear_count++;
        if(clear_count < 10)
            return {STATE::CROSS_DOOR, 300.f, 0.f};

        // reset contadores
        ticks_in_cross = 0;
        traveled_mm = 0.f;
        door_lost_count = 0;
        clear_count = 0;

        // ---- TU LÓGICA ORIGINAL: conexión ----
        // (la dejo igual que en tu cross_door original)
        auto in_range_room = [this](int r)
        {
            return r >= 0 && r < static_cast<int>(nominal_rooms.size());
        };
        auto in_range_door = [this](int r, int d)
        {
            if (r < 0 || r >= static_cast<int>(nominal_rooms.size())) return false;
            if (d < 0 || d >= static_cast<int>(nominal_rooms[r].doors.size())) return false;
            return true;
        };

        if(!in_range_door(current_room, current_door))
        {
            qCritical() << "[CROSS_DOOR] invalid current_room/current_door"
                        << current_room << current_door;
            localised = false;
            return {STATE::LOCALISE, 0.f, 0.f};
        }

        const auto &leaving_door = nominal_rooms[current_room].doors[current_door];
        int next_room_idx = leaving_door.connects_to_room;
        int next_door_idx = leaving_door.connects_to_door;

        if(in_range_room(next_room_idx) &&
           nominal_rooms[next_room_idx].visited &&
           in_range_door(next_room_idx, next_door_idx))
        {
            current_room = next_room_idx;
            room_index   = current_room;
            current_door = next_door_idx;

            const auto &entering_door = nominal_rooms[current_room].doors[current_door];
            Eigen::Vector2f door_center = (entering_door.p1_global + entering_door.p2_global) * 0.5f;

            Eigen::Vector2f inward = -door_center;
            float n = inward.norm();
            if (n > 1e-3f) inward /= n;

            const float angle = std::atan2(inward.x(), inward.y());

            robot_pose.setIdentity();
            robot_pose.translate(door_center + inward * 500.f);
            robot_pose.rotate(angle);

            localised = true;
            return {STATE::GOTO_ROOM_CENTER, 0.f, 0.f};
        }

        nominal_rooms[current_room].doors[current_door].visited = true;
        door_crossing = DoorCrossing{current_room, current_door};

        localised = false;
        relocalised_once = false;
        relocalised_room = -1;

        return {STATE::LOCALISE, 0.f, 0.f};
    }

    return {STATE::CROSS_DOOR, adv, rot};
}




void SpecificWorker::choose_next_door(int current_room)
{
    std::size_t i = 0;
    for (Door &door : nominal_rooms[current_room].doors)
    {
        if (door.visited == false)
        {
            current_door = static_cast<int>(i);
            break;
        }
        i++;
    }
    if (i >= nominal_rooms[current_room].doors.size())
    {
        for (auto &door : nominal_rooms[current_room].doors)
            door.visited = false;
        current_door = 0;
    }
}

/*SpecificWorker::RetVal SpecificWorker::TURN_method(const Corners &corners)
{
   //////////////////////////////////////////////////////////////////////
    // 1) Buscar parche de color (ANEXO 1)  -> NO MNIST
    //////////////////////////////////////////////////////////////////////
    const auto &[success, room_index, left_right] =
        image_processor.check_colour_patch_in_image(camera360rgb_proxy, this->label_img);

    if (success)
    {
        current_room = room_index;

        //////////////////////////////////////////////////////////////////////
        // 2) Refrescar pose "en frame de sala" (match sin transformar corners)
        //////////////////////////////////////////////////////////////////////
        if (const auto res = update_robot_pose(current_room, corners, robot_pose, false); res.has_value())
            robot_pose = res.value().first;
        else
            // Si no ha podido ajustar, sigue girando suavemente hacia el lado del parche
            return {STATE::TURN, 0.0f, left_right * params.RELOCAL_ROT_SPEED / 2.0f};

        //////////////////////////////////////////////////////////////////////
        // 3) Guardar puertas en la sala si es la primera vez que la visitamos
        //////////////////////////////////////////////////////////////////////
        if (not nominal_rooms[current_room].visited)
        {
            nominal_rooms[current_room].name = image_processor.room_name_from_index(current_room);

            auto doors = door_detector.doors();
            if (doors.empty())
            {
                qWarning() << __FUNCTION__ << "empty doors while localised. keep turning";
                return {STATE::TURN, 0.0f, left_right * params.RELOCAL_ROT_SPEED};
            }

            // Proyectar las puertas (medidas en robot frame) a la pared más cercana en room frame
            for (auto &d : doors)
            {
                d.p1_global = nominal_rooms[current_room].get_projection_of_point_on_closest_wall(robot_pose * d.p1);
                d.p2_global = nominal_rooms[current_room].get_projection_of_point_on_closest_wall(robot_pose * d.p2);
            }

            // Guardar puertas "memoradas" en la sala
            nominal_rooms[current_room].doors = doors;

            // Elegir puerta inicial
            current_door = choose_next_door(current_room);

            // Opcional (recomendado): “enganchar” la puerta nominal elegida con la puerta local más cercana
            // para que durante el approach (GOTO_DOOR / ORIENT) se seleccione bien.
            if (current_door >= 0 && current_door < (int)nominal_rooms[current_room].doors.size())
            {
                const auto dn = nominal_rooms[current_room].doors[current_door];
                const auto ds = door_detector.doors();
                if (not ds.empty())
                {
                    const auto sd = std::ranges::min_element(ds,
                        [dn, this](const auto &a, const auto &b)
                        {
                            return (a.center() - robot_pose.inverse() * dn.center_global()).norm() <
                                   (b.center() - robot_pose.inverse() * dn.center_global()).norm();
                        });

                    // Actualiza nominal con valores locales (robot frame) para seguimiento inmediato
                    nominal_rooms[current_room].doors[current_door].p1 = sd->p1;
                    nominal_rooms[current_room].doors[current_door].p2 = sd->p2;
                }
            }

            nominal_rooms[current_room].visited = true;
        }

        //////////////////////////////////////////////////////////////////////
        // 4) Finalizar tracking de cruce de puerta (si vienes de CROSS_DOOR)
        //////////////////////////////////////////////////////////////////////
        // Esto solo tiene sentido si ya tienes DoorCrossing implementado como en el enunciado.
        // Si no lo tienes todavía, puedes comentar este bloque.
        door_crossing.set_entering_data(current_room, nominal_rooms);
        if (door_crossing.valid)
        {
            // actualiza conexiones en ambas salas
            nominal_rooms[door_crossing.leaving_room_index]
                .doors[door_crossing.leaving_door_index]
                .connects_to_door = door_crossing.entering_door_index;

            nominal_rooms[door_crossing.leaving_room_index]
                .doors[door_crossing.leaving_door_index]
                .connects_to_room = door_crossing.entering_room_index;

            nominal_rooms[current_room]
                .doors[door_crossing.entering_door_index]
                .visited = true;

            nominal_rooms[current_room]
                .doors[door_crossing.entering_door_index]
                .connects_to_door = door_crossing.leaving_door_index;

            nominal_rooms[current_room]
                .doors[door_crossing.entering_door_index]
                .connects_to_room = door_crossing.leaving_room_index;

            door_crossing.valid = false;
        }

        //////////////////////////////////////////////////////////////////////
        // 5) Ya estamos localizados -> salir hacia GOTO_DOOR
        //////////////////////////////////////////////////////////////////////
        localised = true;
        return {STATE::GOTO_DOOR, 0.0f, 0.0f};
    }

    //////////////////////////////////////////////////////////////////////
    // Si no detecta parche: seguir girando (sin bloquear el hilo)
    //////////////////////////////////////////////////////////////////////
    return {STATE::TURN, 0.0f, left_right * params.RELOCAL_ROT_SPEED};
}
*/
SpecificWorker::RetVal SpecificWorker::TURN_method(const Corners &corners)
{
    door_crossing.track_entering_door(door_detector.doors());

    // --- Ajustes observados en tus logs ---
    const int CENTER_TARGET = camera_width / 2;  // ~900 si width ~1800
    constexpr int   CENTER_TOL    = 50;
    constexpr float KP            = 0.0015f;
    constexpr float WMAX          = 0.6f;
    constexpr float SEARCH_W      = 0.25f;
    constexpr float MICRO_W       = 0.10f;

    // --- Debounce ---
    static int last_room = -1;
    static int stable_count = 0;
    constexpr int STABLE_N = 4;

    ROBOCOMPMNIST::MNISTResult mnist_result;
    try
    {
        mnist_result = mnist_proxy->getNumber();
        qInfo() << "[TURN] raw number:" << mnist_result.number << "center:" << mnist_result.center;
    }
    catch (const std::exception &e)
    {
        qInfo() << "[TURN] MNIST exception:" << e.what();
        stable_count = 0; last_room = -1;
        return {STATE::TURN, 0.f, SEARCH_W};
    }

    // 1) sin ROI -> búsqueda
    if (mnist_result.center < 0)
    {
        stable_count = 0; last_room = -1;
        return {STATE::TURN, 0.f, SEARCH_W};
    }

    // 2) Normaliza center (a veces viene muy grande: 1300+)
    int c = mnist_result.center;   // píxeles reales [0..camera_width]
    qInfo() << "[TURN] center(px):" << c << " target:" << CENTER_TARGET;


    // 3) Control para centrar cartel
    const int error = c - CENTER_TARGET;
    if (std::abs(error) > CENTER_TOL)
    {
        stable_count = 0;

        float w = KP * static_cast<float>(error);
        w = std::clamp(w, -WMAX, WMAX);

        // Si gira al revés, invierte:
        // w = -w;

        return {STATE::TURN, 0.f, w};
    }

    // 4) Ya está centrado -> mapear habitación (solo 0 y 1 reales)
    int raw = mnist_result.number;

    // MAPEO: 0 real a veces 6, 1 real a veces 2 o 3
    int room = -1;
    if      (raw == 0) room = 0;
    else if (raw == 1) room = 1;
    else if (raw == 6) room = 0;
    else if (raw == 2) room = 1;
    else if (raw == 3) room = 1;
    else room = -1;

    qInfo() << "[TURN] mapped room:" << room;

    // Si centrado pero número raro: micro giro para mejorar clasificación
    if (room < 0)
    {
        stable_count = 0; last_room = -1;
        return {STATE::TURN, 0.f, MICRO_W};
    }

    // 5) Debounce sobre room
    if (room == last_room) stable_count++;
    else { last_room = room; stable_count = 1; }

    if (stable_count < STABLE_N)
        return {STATE::TURN, 0.f, 0.f};  // quieto para estabilizar

    // 6) Ya tenemos sala estable -> intentamos match con corners
    room_index = room;
    current_room = room_index;
    nominal_rooms[current_room].visited = true;   // ✅ clave para saber que ya estuviste

    const auto m = hungarian.match(corners, nominal_rooms[room_index].corners());
    if (m.empty() || m.size() < 3)
    {
        qWarning() << "[TURN] match failed. m.size()=" << m.size();
        stable_count = 0; last_room = -1;
        return {STATE::TURN, 0.f, MICRO_W};
    }

    const auto max_error_iter = std::ranges::max_element(
        m, [](const auto &a, const auto &b) { return std::get<2>(a) < std::get<2>(b); });

    const auto max_match_error = std::get<2>(*max_error_iter);
    qInfo() << "[TURN] max_match_error=" << max_match_error
            << " threshold=" << params.RELOCAL_DONE_MATCH_MAX_ERROR;

    if (max_match_error > params.RELOCAL_DONE_MATCH_MAX_ERROR)
    {
        stable_count = 0; last_room = -1;
        return {STATE::TURN, 0.f, MICRO_W};
    }

    // 7) update pose (tu update_robot_pose exige >=4 matches)
    if (const auto res = update_robot_pose(room_index, corners, robot_pose, false); res.has_value())
        robot_pose = res.value().first;
    else
    {
        qWarning() << "[TURN] update_robot_pose failed (need >=4 matched corners)";
        stable_count = 0; last_room = -1;
        return {STATE::TURN, 0.f, MICRO_W};
    }

    // 8) puertas
    nominal_rooms[room_index].name = image_processor.room_name_from_index(room_index);

    auto doorsy = door_detector.doors();
    if (doorsy.empty())
    {
        qWarning() << "[TURN] doors empty";
        stable_count = 0; last_room = -1;
        return {STATE::TURN, 0.f, MICRO_W};
    }

    for (auto &d : doorsy)
    {
        d.p1_global = nominal_rooms[room_index].get_projection_of_point_on_closest_wall(
            robot_pose.cast<float>() * d.p1.cast<float>());
        d.p2_global = nominal_rooms[room_index].get_projection_of_point_on_closest_wall(
            robot_pose.cast<float>() * d.p2.cast<float>());
    }
    nominal_rooms[room_index].doors = doorsy;

    // 9) Enlazar cruce de puerta (si vienes de CROSS_DOOR y door_crossing está implementado)
    door_crossing.set_entering_data(current_room, nominal_rooms);
    if (door_crossing.valid)
    {
        nominal_rooms[door_crossing.leaving_room_index]
            .doors[door_crossing.leaving_door_index]
            .connects_to_door = door_crossing.entering_door_index;

        nominal_rooms[door_crossing.leaving_room_index]
            .doors[door_crossing.leaving_door_index]
            .connects_to_room = door_crossing.entering_room_index;

        nominal_rooms[current_room]
            .doors[door_crossing.entering_door_index]
            .connects_to_door = door_crossing.leaving_door_index;

        nominal_rooms[current_room]
            .doors[door_crossing.entering_door_index]
            .connects_to_room = door_crossing.leaving_room_index;

        // Marca la puerta por la que entraste como visitada (para ir a la otra)
        if (door_crossing.entering_door_index >= 0 &&
            door_crossing.entering_door_index < static_cast<int>(nominal_rooms[current_room].doors.size()))
        {
            nominal_rooms[current_room].doors[door_crossing.entering_door_index].visited = true;
            current_door = door_crossing.entering_door_index;
        }

        door_crossing.valid = false;
    }

    // 10) Elegir siguiente puerta (la no visitada)
    choose_next_door(current_room);

    // 11) salir de TURN
    localised = true;
    stable_count = 0; last_room = -1;
    qInfo() << "[TURN] LOCALIZED -> GOTO_DOOR";
    return {STATE::GOTO_DOOR, 0.f, 0.f};
}


SpecificWorker::RetVal SpecificWorker::IDLE_method()
{
    return {};
}

SpecificWorker::RetVal SpecificWorker::goto_room_center(const RoboCompLidar3D::TPoints &points)
{
    Eigen::Vector2f target_robot;

    if(localised)
    {
        // centro nominal en sala (0,0) expresado en frame robot
        target_robot = robot_pose.inverse() * Eigen::Vector2f(0.f, 0.f);
    }
    else
    {
        auto center = center_estimator.estimate(points);
        if(!center.has_value())
            return {STATE::LOCALISE, 0.f, 0.f};

        target_robot = center.value().cast<float>();
    }

    // ----- si ya estás en el centro -----
    if(target_robot.norm() < params.RELOCAL_CENTER_EPS)
    {
        if(localised)
        {
            // ya sabes dónde estás: NO TURN, ve directo a la puerta
            current_room = room_index;
            choose_next_door(current_room);
            return {STATE::GOTO_DOOR, 0.f, 0.f};
        }
        else
        {
            // no localizado: toca leer cartel
            return {STATE::TURN, 0.f, 0.f};
        }
    }

    auto [v, w] = robot_controller(target_robot);
    door_crossing.track_entering_door(door_detector.doors());
    return {STATE::GOTO_ROOM_CENTER, v, w};
}



SpecificWorker::RetVal SpecificWorker::localise(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene)
{
    Q_UNUSED(scene);

    // reset “suave”: si prefieres no machacar pose cada ciclo, quita estas 2 líneas
    robot_pose.setIdentity();
    robot_pose.translate(Eigen::Vector2f(0.0f, 0.0f));

    localised = false;

    if (const auto center = center_estimator.estimate(points); center.has_value())
    {
        if (center.value().norm() > params.RELOCAL_CENTER_EPS)
            return {STATE::GOTO_ROOM_CENTER, 0.0f, 0.0f};

        // ya estás en centro -> pasa a TURN para buscar cartel
        return {STATE::TURN, 0.0f, 0.0f};
    }

    qWarning() << __FUNCTION__ << "Not able to estimate room center from walls, turning to search.";
    return {STATE::LOCALISE, 0.0f, 0.25f};   // giro suave para mejorar geometría
}

std::tuple<SpecificWorker::STATE, float, float> SpecificWorker::state_machine(STATE state, const RoboCompLidar3D::TPoints &filter_data, const Corners &corners)
{
    switch (state)
    {
    case STATE::LOCALISE:
        return localise(filter_data, &viewer->scene);
    case STATE::IDLE:
        return IDLE_method();
    case STATE::GOTO_DOOR:
        return goto_door();
    case STATE::TURN:
        return TURN_method(corners);
    case STATE::ORIENT_TO_DOOR:
        return orient_to_door();
    case STATE::GOTO_ROOM_CENTER:
        return goto_room_center(filter_data);
    case STATE::CROSS_DOOR:
        return cross_door(filter_data);
    }
    return {};
}

std::tuple<float, float> SpecificWorker::robot_controller(const Eigen::Vector2f &target)
{
    const auto dist = target.norm();
    if (dist < params.RELOCAL_CENTER_EPS)
        return {0.f, 0.f};

    const auto theta = std::atan2(target.x(), target.y());
    float rot = 0.5f * theta;
    rot = std::clamp(rot, -0.6f, 0.6f);

    const float angle_break = std::exp((-theta * theta) / (M_PI / 6.f));
    float adv = 1000.f * angle_break;
    adv = std::clamp(adv, 0.f, 800.f);

    return {adv, rot};
}

RoboCompLidar3D::TPoints SpecificWorker::read_data()
{
    RoboCompLidar3D::TData data;
    try
    {
        data = lidar3d_proxy->getLidarDataWithThreshold2d("helios", 12000, 2);
    }
    catch (const Ice::Exception &e)
    {
        std::cout << e << " "
                  << "Connection to Lidar failed" << std::endl;
        return {};
    }

    // filter data from 3D to 2D
    RoboCompLidar3D::TPoints filter_data;
    if (const auto filter_data_ = data_filter(data.points); filter_data_.has_value())
        filter_data = filter_data_.value();
    else
    {
        qWarning() << "filter_data_.has_value() is false";
        return {};
    }

    return filter_data;
}

void SpecificWorker::draw_lidar(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene)
{
    static std::vector<QGraphicsItem *> items;
    for (auto i : items)
    {
        scene->removeItem(i);
        delete i;
    }
    items.clear();

    auto color = QColor(Qt::green);
    auto brush = QBrush(QColor(Qt::green));
    for (const auto &p : points)
    {
        auto item = scene->addRect(-50, -50, 100, 100, color, brush);
        item->setPos(p.x, p.y);
        items.push_back(item);
    }

    // draw estimated center (if available)
    auto center = room_detector.estimate_center_from_walls();
    if (center.has_value())
    {
        auto center_item = scene->addEllipse(-150, -150, 300, 300, QPen(Qt::cyan), QBrush(Qt::cyan));
        center_item->setPos(center.value().x(), center.value().y());
        items.push_back(center_item);

        auto text_item = scene->addText(QString("Center\nx=%1 y=%2").arg(center.value().x()).arg(center.value().y()));
        text_item->setDefaultTextColor(Qt::cyan);
        text_item->setPos(center.value().x() + 20, center.value().y() + 20);
        items.push_back(text_item);
    }
}

std::expected<int, std::string> SpecificWorker::closest_lidar_index_to_given_angle(const RoboCompLidar3D::TPoints &points, float angle)
{
    auto res = std::ranges::find_if(points, [angle](auto &a)
                                   { return a.phi > angle; });
    if (res != std::end(points))
        return static_cast<int>(std::distance(std::begin(points), res));
    else
        return std::unexpected("No closest value found in method <closest_lidar_index_to_given_angle>");
}

std::optional<RoboCompLidar3D::TPoints> SpecificWorker::data_filter(const RoboCompLidar3D::TPoints &puntos)
{
    if (puntos.empty())
        return {};

    RoboCompLidar3D::TPoints salida;
    salida.reserve(puntos.size());

    for (auto &&[angle, group] : iter::groupby(puntos, [](const auto &p)
                                             {
        float factor = std::pow(10.0f, 2);
        return std::floor(p.phi * factor) / factor; }))
    {
        auto min_r = std::min_element(std::begin(group), std::end(group), [](const auto &p1, const auto &p2)
                                      { return p1.r < p2.r; });
        salida.emplace_back(*min_r);
    }
    return salida;
}

std::optional<std::pair<Eigen::Affine2f, float>>
SpecificWorker::update_robot_pose(int room_index, const Corners &corners, const Eigen::Affine2f &r_pose, bool transform_corners)
{
    Match match;
    if (transform_corners)
        match = hungarian.match(corners, nominal_rooms[room_index].transform_corners_to(r_pose.inverse()));
    else
        match = hungarian.match(corners, nominal_rooms[room_index].corners());

    if (match.empty() or match.size() < 4)
        return {};

    const auto max_error_iter = std::ranges::max_element(match, [](const auto &a, const auto &b)
                                                        { return std::get<2>(a) < std::get<2>(b); });
    const auto max_match_error = std::get<2>(*max_error_iter);

    Eigen::MatrixXd W(match.size() * 2, 3);
    Eigen::VectorXd b(match.size() * 2);
    for (auto &&[i, m] : match | iter::enumerate)
    {
        auto &[meas_c, nom_c, _] = m;
        auto &[p_meas, __, ___] = meas_c;
        auto &[p_nom, ____, _____] = nom_c;

        b(2 * i) = p_nom.x() - p_meas.x();
        b(2 * i + 1) = p_nom.y() - p_meas.y();
        W.block<1, 3>(2 * i, 0) << 1.0, 0.0, -p_meas.y();
        W.block<1, 3>(2 * i + 1, 0) << 0.0, 1.0, p_meas.x();
    }

    const Eigen::Vector3d r = (W.transpose() * W).inverse() * W.transpose() * b;
    if (r.array().isNaN().any())
    {
        qWarning() << __FUNCTION__ << "NaN values in r";
        return {};
    }

    auto r_pose_copy = r_pose;
    r_pose_copy.translate(Eigen::Vector2f(static_cast<float>(r(0)), static_cast<float>(r(1))));
    r_pose_copy.rotate(static_cast<float>(r[2]));
    return {{r_pose_copy, max_match_error}};
}

void SpecificWorker::emergency()
{
    std::cout << "Emergency worker" << std::endl;
}

void SpecificWorker::restore()
{
    std::cout << "Restore worker" << std::endl;
}

int SpecificWorker::startup_check()
{
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, QCoreApplication::instance(), SLOT(quit()));
    return 0;
}