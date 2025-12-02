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
#include "specificworker.h"

#include <iostream>
#include <qcolor.h>
#include <QLoggingCategory>
#include <QtMath>
#include <cppitertools/groupby.hpp>

#include <algorithm>
#include <optional>
#include <cmath>
#include <limits>
#include <QLabel>

SpecificWorker::SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check) : GenericWorker(configLoader, tprx)
{
	setupUi(this);
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		#ifdef HIBERNATION_ENABLED
			hibernationChecker.start(500);
		#endif

		
		// Example statemachine:
		/***
		//Your definition for the statesmachine (if you dont want use a execute function, use nullptr)
		states["CustomState"] = std::make_unique<GRAFCETStep>("CustomState", period, 
															std::bind(&SpecificWorker::customLoop, this),  // Cyclic function
															std::bind(&SpecificWorker::customEnter, this), // On-enter function
															std::bind(&SpecificWorker::customExit, this)); // On-exit function

		//Add your definition of transitions (addTransition(originOfSignal, signal, dstState))
		states["CustomState"]->addTransition(states["CustomState"].get(), SIGNAL(entered()), states["OtherState"].get());
		states["Compute"]->addTransition(this, SIGNAL(customSignal()), states["CustomState"].get()); //Define your signal in the .h file under the "Signals" section.

		//Add your custom state
		statemachine.addState(states["CustomState"].get());
		***/

		statemachine.setChildMode(QState::ExclusiveStates);
		statemachine.start();

		auto error = statemachine.errorString();
		if (error.length() > 0){
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

    if(startup_check_flag)
    {
        startup_check();
        return;
    }

    // =========================
    // 1) VIEWER IZQUIERDO (LIDAR)
    // =========================
    viewer = new AbstractGraphicViewer(this->frame, params.GRID_MAX_DIM);

    if(frame->layout())
        frame->layout()->addWidget(viewer);
    else
        viewer->setGeometry(frame->rect());

    viewer->scene.setSceneRect(params.GRID_MAX_DIM);
    viewer->fitInView(params.GRID_MAX_DIM);

    auto [r, _1] = viewer->add_robot(params.ROBOT_WIDTH,
                                     params.ROBOT_LENGTH,
                                     0, 100,
                                     QColor("Blue"));
    robot_draw = r;

    // ===================================================
    // 2) VIEWER DERECHO — SIEMPRE USA current_room_id = 0
    // ===================================================
    QRectF room_rect = nominal_rooms[current_room_id].rect();

    viewer_room = new AbstractGraphicViewer(this->frame_room, room_rect);

    if(frame_room->layout())
        frame_room->layout()->addWidget(viewer_room);
    else
        viewer_room->setGeometry(frame_room->rect());

    // Configurar escena
    viewer_room->scene.setSceneRect(room_rect);
    viewer_room->fitInView(room_rect, Qt::KeepAspectRatio);

    viewer_room->scene.addRect(room_rect, QPen(Qt::black, 20));

    auto [rr, _2] = viewer_room->add_robot(
        params.ROBOT_WIDTH,
        params.ROBOT_LENGTH,
        0, 100,
        QColor("Blue")
    );
    robot_room_draw = rr;

    // =========================
    // 3) Pose inicial en la primera habitación
    // =========================
    robot_pose.setIdentity();
    robot_pose.translate(Eigen::Vector2d(0, 0));

    // =========================
    // 4) Gráfica de error de matching
    // =========================
    TimeSeriesPlotter::Config cfg;
    cfg.title = "Maximum Match Error Over Time";
    cfg.yAxisLabel = "Error (mm)";
    cfg.timeWindowSeconds = 15.0;
    cfg.autoScaleY = false;
    cfg.yMin = 0;
    cfg.yMax = 1000;

    time_series_plotter = std::make_unique<TimeSeriesPlotter>(frame_plot_error, cfg);
    match_error_graph = time_series_plotter->addGraph("", Qt::blue);

    show();
}

void SpecificWorker::compute()
{
    // =============================
    // 1) Obtener y filtrar datos LiDAR
    // =============================
    RoboCompLidar3D::TPoints lidar_points = read_data();
    if (lidar_points.empty())
        return;

    // =============================
    // 2) Extraer esquinas y estimar centro
    // =============================
    const auto& [detected_corners, wall_segments] =
        room_detector.compute_corners(lidar_points, &viewer->scene);

    std::optional<Eigen::Vector2d> estimated_center =
        room_detector.estimate_center_from_walls(wall_segments);

    draw_lidar(lidar_points, estimated_center, &viewer->scene);

    // =============================
    // 3) Transformar ESQUINAS NOMINALES según habitación actual
    // =============================
    const auto nominal_robot_frame =
        nominal_rooms[current_room_id].transform_corners_to(robot_pose.inverse());

    Match associations = hungarian.match(detected_corners, nominal_robot_frame);

    // =============================
    // 4) Error de correspondencia
    // =============================
    float max_error = 99999.f;

    if (!associations.empty())
    {
        const auto worst = std::max_element(
            associations.begin(), associations.end(),
            [](auto& a, auto& b) { return std::get<2>(a) < std::get<2>(b); });

        max_error = std::get<2>(*worst);

        if (time_series_plotter)
            time_series_plotter->addDataPoint(match_error_graph, max_error);
    }

    // =============================
    // 5) Localización válida
    // =============================
    localised =
        (associations.size() >= 3 &&
            max_error < params.RELOCAL_DONE_MATCH_MAX_ERROR);

    if (localised)
        update_robot_pose(detected_corners, associations);

    // =============================
    // 6) Proceso de la máquina de estados
    // =============================
    auto [next_state, adv, rot] =
        process_state(lidar_points, detected_corners, wall_segments,
            associations, viewer);

    state = next_state;

    // =============================
    // 7) Enviar comando al robot real
    // =============================
    move_robot(adv, rot, max_error);

    // =============================
    // 8) SI HAY CAMBIO DE HABITACIÓN → Redibujar frame derecho
    // =============================
    if (need_redraw_room)
    {
        need_redraw_room = false;

        viewer_room->scene.clear();

        QRectF room_rect = nominal_rooms[current_room_id].rect();
        viewer_room->scene.setSceneRect(room_rect);
        viewer_room->fitInView(room_rect, Qt::KeepAspectRatio);

        viewer_room->scene.addRect(room_rect, QPen(Qt::black, 20));

        auto [rr, __] = viewer_room->add_robot(
            params.ROBOT_WIDTH,
            params.ROBOT_LENGTH,
            0, 100,
            QColor("Blue"));

        robot_room_draw = rr;
    }

    // =============================
    // 9) Actualizar robot en frame derecho
    // =============================
    robot_room_draw->setPos(robot_pose.translation().x(),
        robot_pose.translation().y());

    double theta = std::atan2(robot_pose.linear()(1, 0),
        robot_pose.linear()(0, 0));

    robot_room_draw->setRotation(qRadiansToDegrees(theta));

    // =============================
    // 10) ACTUALIZACIÓN UI — NUEVO BLOQUE
    // =============================
    // Valores de pose
    double ui_x = robot_pose.translation().x();
    double ui_y = robot_pose.translation().y();
    double ui_angle = std::atan2(robot_pose.linear()(1, 0), robot_pose.linear()(0, 0)); // radianes

    // adv y rot vienen de process_state (las variables locales adv, rot están a mano arriba)
    // Nota: en tu compute() adv y rot son las variables de la tupla retornada por process_state.
    // Ya están en el scope: adv, rot.

    // Actualizar LCDs (si están conectados en el .ui)
    // En tu .ui los nombres son: lcdNumber_x, lcdNumber_y, lcdNumber_angle, lcdNumber_adv, lcdNumber_rot
    if (lcdNumber_x)    lcdNumber_x->display(ui_x);
    if (lcdNumber_y)    lcdNumber_y->display(ui_y);
    if (lcdNumber_angle) lcdNumber_angle->display(ui_angle * 180.0 / M_PI); // grados

    if (lcdNumber_adv)  lcdNumber_adv->display(adv);
    if (lcdNumber_rot)  lcdNumber_rot->display(rot);

    // Estado textual
    if (label_state)
        label_state->setText(getStateName(state));

    // Habitación — actualizamos solo si el widget existe en la UI.
    QLabel* lblRoom = this->findChild<QLabel*>("label_room");
    if (lblRoom)
        lblRoom->setText(QString::number(current_room_id));
    // =============================

    // =============================
    // 11) Actualización UI (gráfica)
    // =============================
    if (time_series_plotter)
        time_series_plotter->update();

    last_time = std::chrono::high_resolution_clock::now();
}
QString SpecificWorker::getStateName(STATE st) const
{
	switch (st) {
	case STATE::IDLE:               return "IDLE";
	case STATE::LOCALISE:           return "LOCALISE";
	case STATE::GOTO_DOOR:          return "GOTO_DOOR";
	case STATE::ORIENT_TO_DOOR:     return "ORIENT_TO_DOOR";
	case STATE::GOTO_ROOM_CENTER:   return "GOTO_ROOM_CENTER";
	case STATE::TURN:               return "TURN";
	case STATE::CROSS_DOOR:         return "CROSS_DOOR";
	case STATE::UPDATE_POSE:        return "UPDATE_POSE";
	default:                        return "UNKNOWN";
	}
}

Eigen::Vector3d SpecificWorker::solve_pose(const Corners &corners, const Match &match)
{
	if (match.empty())
		return Eigen::Vector3d::Zero();

	// W y b como en tu compute antiguo
	Eigen::MatrixXd W(match.size() * 2, 3);
	Eigen::VectorXd b(match.size() * 2);

	std::size_t i = 0;
	for (const auto &m : match)
	{
		const auto &[meas_c, nom_c, cost] = m;
		const auto &[p_meas, __1, __2] = meas_c;
		const auto &[p_nom, __3, __4] = nom_c;

		b(2 * i)     = p_nom.x() - p_meas.x();
		b(2 * i + 1) = p_nom.y() - p_meas.y();

		W.block<1, 3>(2 * i, 0)     << 1.0, 0.0, -p_meas.y();
		W.block<1, 3>(2 * i + 1, 0) << 0.0, 1.0,  p_meas.x();

		++i;
	}

	// Resolver el sistema normal WᵀW r = Wᵀ b
	Eigen::Matrix3d H = W.transpose() * W;
	Eigen::Vector3d g = W.transpose() * b;
	Eigen::Vector3d r = H.ldlt().solve(g);

	if (r.array().isNaN().any())
		return Eigen::Vector3d::Zero();

	// r = (Δx, Δy, Δθ) → corrección incremental
	// La convertimos en pose absoluta sumando a la pose actual
	Eigen::Vector2d current_t = robot_pose.translation();
	double current_theta = std::atan2(robot_pose.linear()(1,0),
									  robot_pose.linear()(0,0));

	Eigen::Vector3d new_pose;
	new_pose.head<2>() = current_t + r.head<2>();
	new_pose.z() = current_theta + r(2);

	return new_pose;
}


// ----------------------------------------------------------
// Metodo Read_data (este metodo llama a la maquina de estados del robot y le pasa la velocidad al robot)
//
// ----------------------------------------------------------
RoboCompLidar3D::TPoints SpecificWorker::read_data()
{
	// ==========================================
	// 1) Obtener datos del LiDAR con protección
	// ==========================================
	RoboCompLidar3D::TPoints raw_points;

	try
	{
		const auto lidar_packet =
			lidar3d_proxy->getLidarDataWithThreshold2d(
				params.LIDAR_NAME_LOW,
				12000,
				1);

		raw_points = lidar_packet.points;
	}
	catch (const Ice::Exception &ex)
	{
		qWarning() << "[Lidar3D] Error al leer datos:" << ex.what();
		return {};
	}

	// Si no hay puntos, no seguimos procesando
	if (raw_points.empty())
		return {};

	// ==========================================
	// 2) Filtrar: quedarnos con el punto más cercano por cada phi
	// ==========================================
	auto collapsed =
		filter_same_phi(raw_points);

	// ==========================================
	// 3) Eliminar puntos aislados (versión suave)
	// ==========================================
	auto clustered =
		filter_isolated_points(collapsed, 200.f);

	// ==========================================
	// 4) Filtro crítico de puerta
	// ==========================================
	auto doorway_filtered =
		door_detector.filter_points(clustered, &viewer->scene);

	return doorway_filtered;
}

RoboCompLidar3D::TPoints
SpecificWorker::filter_same_phi(const RoboCompLidar3D::TPoints &points)
{
	// Si no hay puntos, devolvemos un contenedor vacío
	if (points.empty())
		return {};

	RoboCompLidar3D::TPoints result;
	result.reserve(points.size());

	// Factor de cuantización del ángulo (≈0.01 rad)
	constexpr float phi_precision = 1e2f;

	// Agrupar por phi redondeado
	auto quantizer = [](const auto &p)
	{
		const float scaled = p.phi * phi_precision;
		return std::floor(scaled) / phi_precision;
	};

	for (auto &&group : iter::groupby(points, quantizer))
	{
		auto &bucket = group.second;

		// Seleccionar el punto más cercano dentro del grupo
		auto nearest_it = std::min_element(
			bucket.begin(), bucket.end(),
			[](const auto &lhs, const auto &rhs)
			{
				return lhs.r < rhs.r;
			});

		if (nearest_it != bucket.end())
			result.push_back(*nearest_it);
	}

	return result;
}
RoboCompLidar3D::TPoints
SpecificWorker::filter_isolated_points(const RoboCompLidar3D::TPoints &points, float d)
{
	// versión mínima: no filtra nada todavía
	Q_UNUSED(d);
	return points;
}

// ----------------------------------------------------------
// Este modulo acutaliza la posicion del robot en el segundo frame
// ----------------------------------------------------------



// ----------------------------------------------------------
// Estado FORWARD → Avanza hacia adelante si el camino está libre.
// Si detecta un obstáculo cerca (r < 700 mm), pasa a estado TURN.
// ----------------------------------------------------------



// ----------------------------------------------------------
// Estado TURN → El robot gira en su lugar hasta que el frente esté despejado.
// Cuando despeja, decide aleatoriamente si continuar en FORWARD o pasar a FOLLOW_WALL.
// ----------------------------------------------------------
SpecificWorker::RetVal
SpecificWorker::turn(const Corners &corners)
{
    Q_UNUSED(corners);

    // ----------------------------------------------------------
    // FASE 1 — Detectar parche rojo mediante la cámara 360
    // ----------------------------------------------------------
    if (!red_patch_detected)
    {
        const auto [patch_visible, unused_spin] =
            rc::ImageProcessor::check_red_patch_in_image(
                camera360rgb_proxy,
                QColor("red"),
                nullptr,
                1500);   // umbral mínimo de píxeles rojos

        Q_UNUSED(unused_spin);

        // Si aún no se percibe el parche → seguir girando
        if (!patch_visible)
        {
            return RetVal{STATE::TURN, 0.f, 0.3f};   // giro constante siempre en el mismo signo
        }

        // Parche detectado → pasar a fase de puerta
        red_patch_detected = true;
        qInfo() << "TURN: RED PATCH FOUND → switching to LIDAR-based door alignment phase";
    }

    // ----------------------------------------------------------
    // FASE 2 — Girar hasta alinear la puerta según LIDAR
    // ----------------------------------------------------------
    auto maybe_door = door_detector.get_current_door();

    const float tolerance = 0.10f;    // ≈6 grados
    float advance_cmd = 0.f;
    float rotation_cmd = 0.6f;        // giro constante mientras buscamos la puerta

    if (maybe_door)
    {
        const Door &door = *maybe_door;
        Eigen::Vector2f center = door.center();               // centro de la puerta en frame robot
        float d     = center.norm();
        float angle = std::atan2(center.x(), center.y());     // signo según izquierda/derecha

        // Si ya apuntamos directamente a la puerta → preparar cruce recto
        if (std::fabs(angle) < tolerance)
        {
            constexpr float extra_distance_mm = 2000.f;       // “overtravel” para entrar en la segunda sala

            door_travel_target_mm = d + extra_distance_mm;
            crossing_door   = false;   // permitir inicialización en cross_door()
            crossed_door    = false;

            qInfo() << "TURN: Door ALIGNED | dist =" << d
                    << "| travel_target =" << door_travel_target_mm
                    << "→ switching to CROSS_DOOR";

            return RetVal{STATE::CROSS_DOOR, 0.f, 0.f};
        }

        // Puerta detectada pero no alineada → continuar giro
        qInfo() << "TURN: adjusting toward door | angle:" << angle << "| dist:" << d;
    }
    else
    {
        qInfo() << "TURN: rotating — no door seen yet";
    }

    // Seguir girando mientras no se cumpla la alineación
    return RetVal{STATE::TURN, advance_cmd, rotation_cmd};
}



// ----------------------------------------------------------
// Estado FOLLOW_WALL → Mantiene una distancia constante respecto a una pared lateral.
// ----------------------------------------------------------


// ----------------------------------------------------------
// Estado SPIRAL → El robot avanza describiendo una espiral creciente.
// La rotación aumenta poco a poco, simulando un patrón de búsqueda.
// ----------------------------------------------------------


////////////////////////////////////////////////////////////////////////////////////////////////////////////////7


void SpecificWorker::emergency()
{
    std::cout << "Emergency worker" << std::endl;
    //emergencyCODE
    //
    //if (SUCCESSFUL) //The componet is safe for continue
    //  emmit goToRestore()
}


//Execute one when exiting to emergencyState
void SpecificWorker::restore()
{
    std::cout << "Restore worker" << std::endl;
    //restoreCODE
    //Restore emergency component

}


int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, QCoreApplication::instance(), SLOT(quit()));
	return 0;
}



void SpecificWorker::draw_lidar(const RoboCompLidar3D::TPoints &points,
								std::optional<Eigen::Vector2d> center,
								QGraphicsScene *scene)
{
	if (scene == nullptr)
		return;

	// Mantener referencias a los items dibujados anteriormente
	static std::vector<QGraphicsItem*> rendered_items;

	// --------------------------------------------------------
	// 1) Eliminar cualquier dibujo anterior de la escena
	// --------------------------------------------------------
	for (auto *item : rendered_items)
	{
		scene->removeItem(item);
		delete item;
	}
	rendered_items.clear();

	// --------------------------------------------------------
	// 2) Dibujar los puntos del LiDAR
	// --------------------------------------------------------
	const QPen point_pen(QColor("LightGreen"), 10);

	for (const auto &pt : points)
	{
		auto box = scene->addRect(-25, -25, 50, 50, point_pen);
		box->setPos(pt.x, pt.y);
		rendered_items.emplace_back(box);
	}

	// --------------------------------------------------------
	// 3) Dibujar el centro estimado (si existe)
	// --------------------------------------------------------
	if (center)
	{
		QPen mark_pen(Qt::red);
		mark_pen.setWidth(15);

		auto mark = scene->addEllipse(-50, -50, 100, 100, mark_pen);
		mark->setPos(center->x(), center->y());

		rendered_items.emplace_back(mark);
	}
}


SpecificWorker::RetVal
SpecificWorker::process_state(const RoboCompLidar3D::TPoints &data,
							  const Corners &corners,
							  const Lines   &lines,
							  const Match   &match,
							  AbstractGraphicViewer *viewer)
{
	// Estos parámetros no se usan dentro del método (por ahora)
	Q_UNUSED(match);
	Q_UNUSED(corners);
	Q_UNUSED(viewer);

	// ==========================================================
	// Selector principal de comportamiento según el estado actual
	// ==========================================================
	switch (state)
	{
	case STATE::LOCALISE:
	case STATE::GOTO_ROOM_CENTER:
		// Movimiento hacia el centro estimado de la sala
		return goto_room_center(data, lines);

	case STATE::TURN:
		// Giro buscando parche rojo / puerta alineada
		return turn(corners);

	case STATE::CROSS_DOOR:
		// Avance recto durante una distancia predefinida
		return cross_door(data);

	case STATE::IDLE:
		// Mantener parado
		return RetVal{STATE::IDLE, 0.f, 0.f};

	default:
		// Estado desconocido → volver a LOCALISE
		return RetVal{STATE::LOCALISE, 0.f, 0.f};
	}
}
SpecificWorker::RetVal
SpecificWorker::localise(const Match &match)
{
	Q_UNUSED(match);
	return {STATE::LOCALISE, 0.f, 0.f};
}
SpecificWorker::RetVal
SpecificWorker::goto_door(const RoboCompLidar3D::TPoints &points)
{
    Q_UNUSED(points);   // La detección ya se realizó en filter_points()

    // ---------------------------------------------------------
    // 1) ¿Hay puerta detectada recientemente?
    // ---------------------------------------------------------
    auto maybe_door = door_detector.get_current_door();

    if (!maybe_door)
    {
        // No hay puerta -> girar suave hasta encontrarla
        qInfo() << "GOTO_DOOR: No door detected -> rotating in place";

        const float adv = 0.f;
        const float rot = 0.3f;

        return RetVal{STATE::GOTO_DOOR, adv, rot};
    }

    // ---------------------------------------------------------
    // 2) Puerta encontrada: calcular su centro en frame ROBOT
    // ---------------------------------------------------------
    const Door &door     = *maybe_door;
    Eigen::Vector2f pos  = door.center();       // (x lateral, y frontal)
    float distance       = pos.norm();
    float angle_to_door  = std::atan2(pos.x(), pos.y());

    // ---------------------------------------------------------
    // 3) Comprobar si ya estamos alineados
    // ---------------------------------------------------------
    constexpr float angular_tolerance = 0.05f;   // ≈ 3 grados

    if (std::fabs(angle_to_door) < angular_tolerance)
    {
        qInfo() << "GOTO_DOOR: alignment achieved. dist =" << distance
                << "angle =" << angle_to_door;

        // Quieto mirando a la puerta
        return {STATE::IDLE, 0.f, 0.f};
    }

    // ---------------------------------------------------------
    // 4) Alineación proporcional hacia el centro de la puerta
    // ---------------------------------------------------------
    const float gain_rot = 0.8f;
    float rot_cmd = gain_rot * angle_to_door;

    // Limitar la velocidad de giro
    rot_cmd = std::clamp(rot_cmd,
                         -params.RELOCAL_ROT_SPEED,
                         params.RELOCAL_ROT_SPEED);

    qInfo() << "GOTO_DOOR: turning toward door | angle:" << angle_to_door
            << " dist:" << distance
            << " rot_cmd:" << rot_cmd;

    return RetVal{STATE::GOTO_DOOR, 0.f, rot_cmd};
}
SpecificWorker::RetVal
SpecificWorker::orient_to_door(const RoboCompLidar3D::TPoints &points)
{
	Q_UNUSED(points);
	return {STATE::IDLE, 0.f, 0.f};   // este estado ni se usa ya
}
SpecificWorker::RetVal
SpecificWorker::cross_door(const RoboCompLidar3D::TPoints &points)
{
	Q_UNUSED(points);

	// 1) Primera entrada al estado
	if (!crossing_door)
	{
		crossing_door = true;
		cross_door_start = std::chrono::high_resolution_clock::now();

		qInfo() << "CROSS_DOOR: Begin crossing. travel_target (mm) ="
				<< door_travel_target_mm;
	}

	// 2) Tiempo transcurrido
	const auto now = std::chrono::high_resolution_clock::now();
	float elapsed_sec =
		std::chrono::duration_cast<std::chrono::milliseconds>(now - cross_door_start).count() / 1000.f;

	float travelled_mm = params.CROSS_DOOR_SPEED * elapsed_sec;

	// 3) Aún no se llegó
	if (travelled_mm < door_travel_target_mm)
		return RetVal{STATE::CROSS_DOOR, params.CROSS_DOOR_SPEED, 0.f};

	// 4) CRUCE COMPLETADO
	crossing_door      = false;
	crossed_door       = true;
	red_patch_detected = false;

	// ========= 🔥 AQUI CAMBIA DE HABITACIÓN 🔥 =========
	current_room_id = (current_room_id + 1) % nominal_rooms.size();

	robot_pose.setIdentity();     // nueva sala → pose reiniciada
	need_redraw_room = true;      // pedir redibujado del frame derecho

	qInfo() << "CROSS_DOOR: Completed crossing. Now entering room"
			<< current_room_id;

	// Nueva localización en la habitación nueva
	return RetVal{STATE::GOTO_ROOM_CENTER, 0.f, 0.f};
}


SpecificWorker::RetVal
SpecificWorker::goto_room_center(const RoboCompLidar3D::TPoints &points,
                                 const Lines &lines)
{
    Q_UNUSED(points);

    // ----------------------------------------------------------
    // 1) Calcular el centro geométrico de la sala según las paredes detectadas
    // ----------------------------------------------------------
    auto maybe_center = room_detector.estimate_center_from_walls(lines);

    // ----------------------------------------------------------
    // 2) Si no podemos estimar el centro → girar hasta encontrar geometría válida
    // ----------------------------------------------------------
    if (!maybe_center)
    {
        const float adv_cmd = 0.f;
        const float rot_cmd = 0.35f;  // giro de barrido constante

        qInfo() << "GOTO_ROOM_CENTER: no center detected → rotating to explore room";

        return RetVal{STATE::GOTO_ROOM_CENTER, adv_cmd, rot_cmd};
    }

    // ----------------------------------------------------------
    // 3) Transformar el centro en coordenadas del robot
    // ----------------------------------------------------------
    Eigen::Vector2d center_world = *maybe_center;
    float cx = static_cast<float>(center_world.x());  // lateral
    float cy = static_cast<float>(center_world.y());  // frontal

    // ----------------------------------------------------------
    // 4) Dibujar punto del centro para depuración
    // ----------------------------------------------------------
    static QGraphicsEllipseItem *center_marker = nullptr;

    if (center_marker)
    {
        viewer->scene.removeItem(center_marker);
        delete center_marker;
    }

    center_marker =
        viewer->scene.addEllipse(-100, -100, 200, 200,
                                 QPen(Qt::red, 3),
                                 QBrush(Qt::red, Qt::SolidPattern));

    center_marker->setPos(center_world.x(), center_world.y());

    // ----------------------------------------------------------
    // 5) Evaluar distancia al centro
    // ----------------------------------------------------------
    float distance = std::hypot(cx, cy);

    // Si estamos suficientemente cerca, pasar al estado TURN
    if (distance < params.RELOCAL_CENTER_EPS)
    {
        relocal_centered = true;

        qInfo() << "GOTO_ROOM_CENTER: center reached → switching to TURN";

        return RetVal{STATE::TURN, 0.f, 0.f};
    }

    // ----------------------------------------------------------
    // 6) Cálculo del ángulo hacia el centro
    // ----------------------------------------------------------
    float angle_to_center = std::atan2(cx, cy);

    // ----------------------------------------------------------
    // 7) Control proporcional para orientar el robot
    // ----------------------------------------------------------
    constexpr float k_rot = 0.5f;   // ganancia angular
    float rot_cmd = k_rot * angle_to_center;

    // ----------------------------------------------------------
    // 8) Freno (campana gaussiana) para suavizar la velocidad de avance
    // ----------------------------------------------------------
    float gaussian = std::exp(-(angle_to_center * angle_to_center) /
                               (static_cast<float>(M_PI) / 10.f));

    float adv_cmd = params.RELOCAL_MAX_ADV * gaussian;

    // ----------------------------------------------------------
    // 9) Depuración
    // ----------------------------------------------------------
    qInfo() << "CENTER TARGET | cx:" << cx << "cy:" << cy
            << "| dist:" << distance
            << "| adv:" << adv_cmd
            << "| rot:" << rot_cmd;

    // ----------------------------------------------------------
    // 10) Ejecutar movimiento hacia el centro
    // ----------------------------------------------------------
    return RetVal{STATE::GOTO_ROOM_CENTER, adv_cmd, rot_cmd};
}

std::tuple<float, float>
SpecificWorker::robot_controller(const Eigen::Vector2f &target)
{
    // El vector objetivo está en coordenadas del robot:
    //  x → frente, y → izquierda
    const float x = target.x();
    const float y = target.y();

    // -----------------------------------------------------
    // 1) Distancia al objetivo
    // -----------------------------------------------------
    float distance = std::hypot(x, y);

    // Si prácticamente estamos encima del objetivo, no moverse
    if (distance < 1e-3f)
        return {0.f, 0.f};

    // -----------------------------------------------------
    // 2) Ángulo relativo hacia el objetivo
    // -----------------------------------------------------
    float heading = std::atan2(y, x);

    // Si estamos dentro del margen de tolerancia del centro, parada
    if (distance < params.RELOCAL_CENTER_EPS)
        return {0.f, 0.f};

    // -----------------------------------------------------
    // 3) Caso especial: objetivo detrás del robot
    //    cos(heading) < 0 → ángulo > 90° o < -90°
    // -----------------------------------------------------
    if (std::cos(heading) < 0.f)
    {
        float rot_sign = (heading >= 0.f) ? 1.f : -1.f;
        float rot_cmd  = rot_sign * params.RELOCAL_ROT_SPEED;
        return {0.f, rot_cmd};   // solo giro, sin avance
    }

    // -----------------------------------------------------
    // 4) Control angular proporcional
    // -----------------------------------------------------
    constexpr float k_rot = 1.0f;          // ganancia P
    float rot_cmd = k_rot * heading;

    // Limitar velocidad angular
    rot_cmd = std::clamp(rot_cmd,
                         -params.RELOCAL_ROT_SPEED,
                          params.RELOCAL_ROT_SPEED);

    // -----------------------------------------------------
    // 5) Control del avance mediante campana gaussiana
    // -----------------------------------------------------
    const float max_forward_speed = params.RELOCAL_MAX_ADV;     // ej: 300 mm/s
    const float sigma = static_cast<float>(M_PI) / 4.f;          // ≈45°
    const float ang2  = heading * heading;

    // factor de frenado entre 0 y 1
    float gaussian_factor =
        std::exp(-(ang2) / (2.f * sigma * sigma));

    float adv_cmd = max_forward_speed * gaussian_factor;

    // -----------------------------------------------------
    // 6) Velocidades finales
    // -----------------------------------------------------
    return {adv_cmd, rot_cmd};
}


bool SpecificWorker::update_robot_pose(const Corners &corners, const Match &match)
{
	if(match.empty()) return false;

	// estimar pose usando las ecuaciones de la Actividad 2
	Eigen::Vector3d new_pose = solve_pose(corners, match);

	// filtro suave
	robot_pose.translation() =
		0.9 * robot_pose.translation() +
		0.1 * new_pose.head<2>();

	Eigen::Rotation2Dd R(new_pose.z());
	robot_pose.linear() = R.toRotationMatrix();

	return true;
}
std::expected<int, std::string>
SpecificWorker::closest_lidar_index_to_given_angle(const auto &points, float angle)
{
	if (points.empty())
		return std::unexpected("No points available");

	int   selected_index = -1;
	float best_distance  = std::numeric_limits<float>::max();

	// ----------------------------------------------------------
	// Buscar el punto cuyo φ esté más cerca del ángulo objetivo
	// ----------------------------------------------------------
	for (std::size_t i = 0; i < points.size(); ++i)
	{
		float diff = std::fabs(points[i].phi - angle);

		if (diff < best_distance)
		{
			best_distance  = diff;
			selected_index = static_cast<int>(i);
		}
	}

	// Si no se encontró ninguno (caso teórico)
	if (selected_index < 0)
		return std::unexpected("Closest angle not found");

	return selected_index;
}

SpecificWorker::RetVal
SpecificWorker::update_pose(const Corners &corners, const Match &match)
{
	Q_UNUSED(corners);
	Q_UNUSED(match);

	// Aún no implementado: en este estado el robot permanece quieto.
	return RetVal{STATE::UPDATE_POSE, 0.f, 0.f};
}
void SpecificWorker::move_robot(float adv, float rot, float max_match_error)
{
	Q_UNUSED(max_match_error);

	// === AÑADIR: Odometría simple ===
	float dt = 0.1f;  // tu periodo es 100ms

	double theta = std::atan2(robot_pose.linear()(1,0),
							  robot_pose.linear()(0,0));

	// Integrar avance (Y) y rotación
	if (!localised)   // Solo si NO tenemos SLAM/LIDAR fiable
	{
		float dx = adv * std::sin(theta) * dt;
		float dy = adv * std::cos(theta) * dt;

		robot_pose.translation().x() += dx;
		robot_pose.translation().y() += dy;

		theta += rot * dt;
		Eigen::Rotation2Dd R(theta);
		robot_pose.linear() = R.toRotationMatrix();
	}

	// === Restante código igual ===
	const auto t_now = std::chrono::high_resolution_clock::now();
	const long t_stamp = std::chrono::duration_cast<std::chrono::milliseconds>(
							 t_now.time_since_epoch())
							 .count();

	std::tuple<float, float, float, long> command{0.f, adv, rot, t_stamp};
	commands_buffer.put(std::move(command));
	last_velocities = command;

	try
	{
		omnirobot_proxy->setSpeedBase(0.f, adv, rot);
	}
	catch (const Ice::Exception &ex)
	{
		qWarning() << "[OmniRobot] Error sending speed command:" << ex.what();
	}
}


/**************************************/
// From the RoboCompDifferentialRobot you can call this methods:
// RoboCompDifferentialRobot::void this->differentialrobot_proxy->correctOdometer(int x, int z, float alpha)
// RoboCompDifferentialRobot::void this->differentialrobot_proxy->getBasePose(int x, int z, float alpha)
// RoboCompDifferentialRobot::void this->differentialrobot_proxy->getBaseState(RoboCompGenericBase::TBaseState state)
// RoboCompDifferentialRobot::void this->differentialrobot_proxy->resetOdometer()
// RoboCompDifferentialRobot::void this->differentialrobot_proxy->setOdometer(RoboCompGenericBase::TBaseState state)
// RoboCompDifferentialRobot::void this->differentialrobot_proxy->setOdometerPose(int x, int z, float alpha)
// RoboCompDifferentialRobot::void this->differentialrobot_proxy->setSpeedBase(float adv, float rot)
// RoboCompDifferentialRobot::void this->differentialrobot_proxy->stopBase()

/**************************************/
// From the RoboCompDifferentialRobot you can use this types:
// RoboCompDifferentialRobot::TMechParams

/**************************************/
// From the RoboCompLaser you can call this methods:
// RoboCompLaser::TLaserData this->laser_proxy->getLaserAndBStateData(RoboCompGenericBase::TBaseState bState)
// RoboCompLaser::LaserConfData this->laser_proxy->getLaserConfData()
// RoboCompLaser::TLaserData this->laser_proxy->getLaserData()

/**************************************/
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData