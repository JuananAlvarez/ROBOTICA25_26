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

#include "specificworker.h"

#include <algorithm>
#include <cmath>
#include <cppitertools/enumerate.hpp>
#include <cppitertools/groupby.hpp>
#include <iostream>
#include <ranges>

// Qt
#include <QCoreApplication>
#include <QPen>
#include <QTimer>

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
    Doors doors = door_detector.doors();
    if (doors.empty())
    {
        qInfo() << __FUNCTION__ << "No doors detected";
        return {STATE::GOTO_DOOR, 0.f, 0.f};
    }

    // target door is the current nominal door in GLOBAL
    const Door &nominal_door = nominal_rooms[room_index].doors[current_door];

    const auto target_it = std::ranges::min_element(doors, [nominal_door, this](const auto &a, const auto &b)
                                                   {
        return (a.center() - robot_pose.inverse().cast<float>() * nominal_door.global_center()).norm() <
               (b.center() - robot_pose.inverse().cast<float>() * nominal_door.global_center()).norm(); });

    Door target_door = *target_it;

    target_door.p1_global = nominal_rooms[room_index].get_projection_of_point_on_closest_wall(robot_pose.cast<float>() * target_door.p1.cast<float>());
    target_door.p2_global = nominal_rooms[room_index].get_projection_of_point_on_closest_wall(robot_pose.cast<float>() * target_door.p2.cast<float>());

    // door center in ROBOT frame
    Eigen::Vector2f centro = target_door.center();
    const float k_rot = 1.0f;
    const float angulo = std::atan2(centro.x(), centro.y());
    const float dist = centro.norm();

    if (dist < 600.f)
        return {STATE::ORIENT_TO_DOOR, 0.f, 0.f};

    const float vrot = k_rot * angulo;
    const float brake = std::exp(-angulo * angulo / (M_PI / 10.f));
    const float adv = 500.f * brake;
    return {STATE::GOTO_DOOR, adv, vrot};
}

SpecificWorker::RetVal SpecificWorker::orient_to_door()
{
    const auto doorsy = door_detector.doors();
    if (doorsy.empty())
        return {};

    const auto sd = std::ranges::min_element(doorsy, [](const auto &a, const auto &b)
                                            { return std::fabs(a.center_angle()) < std::fabs(b.center_angle()); });

    const auto centro = sd->center();
    const float k = 0.5f;
    const float angulo = std::atan2(centro.x(), centro.y());

    if (std::abs(angulo) < 0.1f)
    {
        localised = false;
        return {STATE::CROSS_DOOR, 0.5f, 0.0f};
    }
    return {STATE::ORIENT_TO_DOOR, 0.0f, k * angulo};
}

SpecificWorker::RetVal SpecificWorker::cross_door(const RoboCompLidar3D::TPoints &points)
{
    Q_UNUSED(points);

    static int contador = 0;
    contador++;

    // mantener avance un rato (tu comportamiento)
    if (contador < 40)
        return {STATE::CROSS_DOOR, 500.f, 0.f};

    contador = 0;

    // ✅ NO borres memoria de puertas aquí
    // ❌ QUITA esto si lo tienes:
    // nominal_rooms[room_index].doors = door_detector.doors();

    // puerta por la que salimos
    const auto &leaving_door = nominal_rooms[current_room].doors[current_door];
    int next_room_idx = leaving_door.connects_to_room;

    // ✅ si la puerta ya sabe a qué sala lleva y esa sala ya se visitó -> relocalización instantánea
    if (next_room_idx >= 0 && nominal_rooms[next_room_idx].visited)
    {
        int next_door_idx = leaving_door.connects_to_door;

        current_room = next_room_idx;
        room_index   = current_room;   // tu goto_door usa room_index
        current_door = next_door_idx;
        // Compute robot pose based on the door in the new room frame.
        const auto &entering_door = nominal_rooms[next_room_idx].doors[next_door_idx];
        Eigen::Vector2f door_center = (entering_door.p1_global + entering_door.p2_global) * 0.5f;

        // dirección desde la puerta hacia el centro (0,0)
        Eigen::Vector2f inward = -door_center;
        float n = inward.norm();
        if (n > 1e-3f) inward /= n;

        // orientar el robot hacia el centro
        const float angle = std::atan2(inward.x(), inward.y());

        robot_pose.setIdentity();
        robot_pose.translate(door_center + inward * 500.f);  // 500mm dentro hacia el centro real
        robot_pose.rotate(angle);

        current_room = next_room_idx;
        current_door = next_door_idx;
        localised = true;

        return {STATE::GOTO_ROOM_CENTER, 0.f, 0.f};

    }
    else
    {
        // Sala desconocida: inicializa tracking para enlazar puertas cuando vuelvas a localizar
        door_crossing = DoorCrossing{current_room, current_door};
        nominal_rooms[current_room].doors[current_door].visited = true;

        localised = false;
        relocalised_once = false;
        relocalised_room = -1;

        return {STATE::LOCALISE, 0.f, 0.f};
    }
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
    // 0) Capturar imagen y detectar parche
    const auto img = camera360rgb_proxy->getROI(0, 0, 0, 0, 0, 0);

    const auto [success, room_idx, left_right] =
        image_processor.check_colour_patch_in_image(img, this->label_img);

    if (success)
    {
        // 1) Set sala actual (sincroniza indices)
        current_room = room_idx;
        room_index   = current_room;   // tu goto_door usa room_index

        // 2) Refrescar pose
        if (const auto res = update_robot_pose(current_room, corners, robot_pose, false); res.has_value())
            robot_pose = res.value().first;
        else
            return {STATE::TURN, 0.0f, left_right * params.RELOCAL_ROT_SPEED / 2.0f};

        // 3) Guardar puertas solo la primera vez (memoria)
        if (not nominal_rooms[current_room].visited)
        {
            nominal_rooms[current_room].name = image_processor.room_name_from_index(current_room);

            auto doors = door_detector.doors();
            if (doors.empty())
            {
                qWarning() << __FUNCTION__ << "empty doors while localised. keep turning";
                return {STATE::TURN, 0.0f, left_right * params.RELOCAL_ROT_SPEED};
            }

            // Proyectar p1/p2 a paredes en frame de sala
            for (auto &d : doors)
            {
                d.p1_global = nominal_rooms[current_room]
                                  .get_projection_of_point_on_closest_wall(robot_pose * d.p1);
                d.p2_global = nominal_rooms[current_room]
                                  .get_projection_of_point_on_closest_wall(robot_pose * d.p2);
            }

            // Guardar puertas en memoria
            nominal_rooms[current_room].doors = doors;

            // Elegir puerta a usar
            choose_next_door(current_room);

            // Enganchar nominal con puerta local más cercana (seguimiento inmediato)
            if (current_door >= 0 && current_door < (int)nominal_rooms[current_room].doors.size())
            {
                const auto dn = nominal_rooms[current_room].doors[current_door];
                const auto ds = door_detector.doors();
                if (!ds.empty())
                {
                    const auto sd = std::ranges::min_element(
                        ds, [dn, this](const auto &a, const auto &b)
                        {
                            return (a.center() - robot_pose.inverse() * dn.global_center()).norm() <
                                   (b.center() - robot_pose.inverse() * dn.global_center()).norm();
                        });

                    nominal_rooms[current_room].doors[current_door].p1 = sd->p1;
                    nominal_rooms[current_room].doors[current_door].p2 = sd->p2;
                }
            }

            nominal_rooms[current_room].visited = true;
        }

        // 4) Finalizar tracking del cruce (si venías de sala nueva)
        door_crossing.set_entering_data(current_room, nominal_rooms);

        if (door_crossing.valid)
        {
            // puerta de salida (sala anterior)
            nominal_rooms[door_crossing.leaving_room_index]
                .doors[door_crossing.leaving_door_index]
                .connects_to_door = door_crossing.entering_door_index;

            nominal_rooms[door_crossing.leaving_room_index]
                .doors[door_crossing.leaving_door_index]
                .connects_to_room = door_crossing.entering_room_index;

            // puerta de entrada (sala actual)
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

        // 5) Ya localizado -> ir a por puerta
        localised = true;
        return {STATE::GOTO_DOOR, 0.0f, 0.0f};
    }

    // si no detecta parche -> seguir girando
    return {STATE::TURN, 0.0f, left_right * params.RELOCAL_ROT_SPEED};
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
        // centro nominal exacto en sala (0,0) expresado en frame robot
        target_robot = robot_pose.inverse() * Eigen::Vector2f(0.f, 0.f);
    }
    else
    {
        auto center = center_estimator.estimate(points);
        if(!center.has_value())
            return {STATE::GOTO_ROOM_CENTER, 0.f, 0.f};

        target_robot = center.value().cast<float>();
    }

    if(target_robot.norm() < params.RELOCAL_CENTER_EPS)
        return {STATE::TURN, 0.f, 0.f};

    auto [v, w] = robot_controller(target_robot);

    door_crossing.track_entering_door(door_detector.doors());
    return {STATE::GOTO_ROOM_CENTER, v, w};
}



SpecificWorker::RetVal SpecificWorker::localise(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene)
{
    Q_UNUSED(scene);
    robot_pose.setIdentity();
    robot_pose.translate(Eigen::Vector2f(0.0f, 0.0f));
    localised = false;

    if (const auto center = center_estimator.estimate(points); center.has_value())
    {
        if (center.value().norm() > params.RELOCAL_CENTER_EPS)
            return {STATE::GOTO_ROOM_CENTER, 0.0f, 0.0f};
        if (center.value().norm() < params.RELOCAL_CENTER_EPS)
            return {STATE::TURN, 0.0f, 0.0f};
    }
    qWarning() << __FUNCTION__ << "Not able to estimate room center from walls, continue localising.";
    return {STATE::LOCALISE, 0.0f, 0.0f};
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
    const float rot = 0.5f * theta;
    const float angle_break = std::exp((-theta * theta) / (M_PI / 6.f));
    const float adv = 1000.f * angle_break;
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