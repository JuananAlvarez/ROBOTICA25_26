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
#include <cppitertools/groupby.hpp>
#include <cmath>
#include <algorithm>
#include <cppitertools/enumerate.hpp>


SpecificWorker::SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check) : GenericWorker(configLoader, tprx)
{
	this->startup_check_flag = startup_check;
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
    std::cout << "initialize worker" << std::endl;

    //initializeCODE
	this->setupUi(this);

    /////////GET PARAMS, OPEND DEVICES....////////
    //int period = configLoader.get<int>("Period.Compute") //NOTE: If you want get period of compute use getPeriod("compute")
    //std::string device = configLoader.get<std::string>("Device.name")
    this->dimensions = QRectF(-6000, -3000, 12000, 6000);
    viewer = new AbstractGraphicViewer(this->frame, this->dimensions);

    this->resize(900,450);
    this->show();
    const auto rob = viewer->add_robot(ROBOT_LENGTH, ROBOT_LENGTH, 0, 190, QColor("Blue"));
    robot_polygon = std::get<0>(rob);



	viewer_room = new AbstractGraphicViewer(this->frame_room, params.GRID_MAX_DIM);
	auto [rr, re] = viewer_room->add_robot(params.ROBOT_WIDTH, params.ROBOT_LENGTH, 0, 100, QColor("Blue"));
	room_draw_robot = rr;

	// draw room in viewer_room
	viewer_room->scene.addRect(params.GRID_MAX_DIM, QPen(Qt::black, 30));
	viewer_room->show();

	// initialise robot pose
	robot_pose.setIdentity();
	robot_pose.translate(Eigen::Vector2d(0.0,0.0));

 //    connect(viewer, &AbstractGraphicViewer::new_mouse_coordinates, this, &SpecificWorker::new_target_slot);
	// connect(room_viewer, &AbstractGraphicViewer::new_mouse_coordinates, this, &SpecificWorker::new_target_slot);
	qInfo() << "ENTRANDO COMPUTE";
}




//compute() ahora NO controla la navegación, solo la localización.
void SpecificWorker::compute()
{
	qInfo() << "ENTRANDO COMPUTE";
	//auto filtered_data = read_data();
	//if (!filtered_data.has_value()) return;

	//El filtered_data lo que hace es darle los puntos al robot para que pueda moverse
	//	esto se lo pasamos la state machine para que ya ejecute los estados correspondientes.
    //stateMachine(filtered_data);

	//ERROR EN THTETA, TENEMOS QUE ARREGALRLO

	//updateRobotInRoom(x,y, theta * 180 /M_PI);
	 // 1️⃣ Leer los datos del LIDAR
    auto filtered_data = read_data();
    if (!filtered_data.has_value() or filtered_data->empty())
        return;

    // 2️⃣ Detectar las esquinas medidas desde el LIDAR
    Corners measured_corners = room_detector.compute_corners(filtered_data.value(), &viewer->scene);

    if (measured_corners.empty())
        return;

    // 3️⃣ Transformar las esquinas nominales al frame del robot
    //    (queremos comparar esquinas medidas por el robot con las nominales)
    Corners nominal_corners_in_robot = room.transform_corners_to(robot_pose.inverse());

    // 4️⃣ Emparejar las esquinas medidas con las nominales usando Hungarian
    //    El metodo Hungarian::match(corners1, corners2) suele devolver un vector de tuplas (measured, nominal, cost)
    auto match = hungarian.match(measured_corners, nominal_corners_in_robot);

    if (match.empty())
        return;

    // 5️⃣ Construir matrices W y b
    Eigen::MatrixXd W(match.size() * 2, 3);
    Eigen::VectorXd b(match.size() * 2);

    for (auto &&[i, m] : iter::enumerate(match))
    {
        auto &[meas_c, nom_c, _] = m;
        auto &[p_meas, __, ___]  = meas_c;
        auto &[p_nom, ____, _____] = nom_c;

        b(2 * i)     = p_nom.x() - p_meas.x();
        b(2 * i + 1) = p_nom.y() - p_meas.y();

        W.block<1, 3>(2 * i, 0)     << 1.0, 0.0, -p_meas.y();
        W.block<1, 3>(2 * i + 1, 0) << 0.0, 1.0,  p_meas.x();
    }

    // 6️⃣ Resolver por pseudoinversa: r = (WᵀW)^(-1) Wᵀ b
    const Eigen::Vector3d r = (W.transpose() * W).inverse() * W.transpose() * b;
    if (r.array().isNaN().any())
        return;

    qInfo() << "Pose correction (x,y,phi): " << r(0) << "," << r(1) << "," << r(2);

    // 7️⃣ Actualizar la pose del robot
    robot_pose.translate(Eigen::Vector2d(r(0), r(1)));
    robot_pose.rotate(r(2));

    // 8️⃣ Actualizar la posición en el visor
    room_draw_robot->setPos(robot_pose.translation().x(), robot_pose.translation().y());
    double angle = std::atan2(robot_pose.rotation()(1, 0), robot_pose.rotation()(0, 0));
    room_draw_robot->setRotation(angle * 180 / M_PI);  // convertir a grados

    qInfo() << "--------------------";

}


// ----------------------------------------------------------
// Metodo movimientos del robot
//
// ----------------------------------------------------------

std::tuple<State, float, float> SpecificWorker::stateMachine(const std::optional<RoboCompLidar3D::TPoints> &filtered_data)
{

	if (!filtered_data.has_value() || filtered_data->empty())
		return {State::IDLE, 0.0f, 0.0f};

	std::tuple<State, float, float> result;  // State, adv, rot
	// State machine
	switch(state)
	{
	case State::IDLE:
		// Pasamos a FORWARD por defecto
		result = {State::FORWARD, 0.0f, 0.0f};
		break;

	case State::FORWARD:
		result = FORWARD_method(filtered_data);
		break;

	case State::TURN:
		result = TURN_method(filtered_data);
		break;

	case State::SFO:
		result = FOLLOW_WALL_method(filtered_data);
		break;

	case State::SPIRAL:
		result = SPIRAL_method(filtered_data);
		break;

	default:
		result = {State::IDLE, 0.0f, 0.0f};
		break;
	}

	 //6️⃣ Enviar velocidades al robot
	float adv = std::get<1>(result);
	float rot = std::get<2>(result);


	try
	{ omnirobot_proxy->setSpeedBase(0.0, adv, rot);}
	 catch (const Ice::Exception &e){ std::cout << e.what() << std::endl;}

	state = std::get<0>(result);
	return result;
}

// ----------------------------------------------------------
// Metodo Read_data (este metodo llama a la maquina de estados del robot y le pasa la velocidad al robot)
//
// ----------------------------------------------------------
std::optional<RoboCompLidar3D::TPoints> SpecificWorker::read_data()
{
	RoboCompLidar3D::TData data;
	try
	{
		// 1️⃣ Leer datos 2D del LIDAR
		data = lidar3d_proxy->getLidarDataWithThreshold2d("helios", 12000, 1);

		if (data.points.empty())
			return std::nullopt;   // 🔹 Devuelve "sin datos"
	}
	catch (const Ice::Exception &e)
	{
		std::cout << "Error al leer LIDAR: " << e << std::endl;
		return std::nullopt;  // 🔹 También devuelve vacío si hay error
	}

	// 2️⃣ Filtrar los puntos
	const auto filtered_data_ = filter_min_distance_cppitertools(data.points);
	if (!filtered_data_.has_value())
		return std::nullopt;   // 🔹 Si no hay puntos válidos, devuelve vacío

	RoboCompLidar3D::TPoints filtered_data = filtered_data_.value();

	// 3️⃣ Dibujar los puntos en la ventana Qt
	draw_lidar(filtered_data, &viewer->scene);

	// 4️⃣ Buscar el obstáculo más cercano
	int offset = filtered_data.size()/2 -10;
	auto min_it = std::min_element(filtered_data.begin()+offset,
			filtered_data.end()-offset,[](const auto& a, const auto& b){ return a.r < b.r; });

	// 4️⃣ Devolver los datos filtrados (para la máquina de estados)
	return filtered_data;  // ✅ Devuelve los puntos procesados
}

// ----------------------------------------------------------
// Este modulo acutaliza la posicion del robot en el segundo frame
// ----------------------------------------------------------
void SpecificWorker::updateRobotInRoom (float x, float y, float alpha)
{
	if (!robot_draw) return;

	robot_draw->setPos(x, y);
	robot_draw->setRotation(alpha);


}



// ----------------------------------------------------------
// Estado FORWARD → Avanza hacia adelante si el camino está libre.
// Si detecta un obstáculo cerca (r < 700 mm), pasa a estado TURN.
// ----------------------------------------------------------
std::tuple<State, float, float> SpecificWorker::FORWARD_method(const std::optional<RoboCompLidar3D::TPoints> &filtered_data)
{
	if (!filtered_data.has_value() || filtered_data->empty())
		return {State::IDLE, 0.0f, 0.0f};

	auto &points = filtered_data.value();

	// 🔎 Detectar obstáculos por sectores según phi
	auto front_it = std::min_element(points.begin(), points.end(),
		[](const auto &a, const auto &b){
			return (std::abs(a.phi) < 0.25 ? a.r : 1e6) < (std::abs(b.phi) < 0.25 ? b.r : 1e6);
		});

	auto left_it = std::min_element(points.begin(), points.end(),
	[](const RoboCompLidar3D::TPoint &a, const RoboCompLidar3D::TPoint &b){
		return (a.phi < 0 && a.phi > -1.57 ? a.r : 1e6) < (b.phi < 0 && b.phi > -1.57 ? b.r : 1e6);
	});

	auto right_it = std::min_element(points.begin(), points.end(),
		[](const RoboCompLidar3D::TPoint &a, const RoboCompLidar3D::TPoint &b){
			return (a.phi > 0 && a.phi < 1.57 ? a.r : 1e6) < (b.phi > 0 && b.phi < 1.57 ? b.r : 1e6);
		});


	auto back_it = std::min_element(points.begin(), points.end(),
		[](const auto &a, const auto &b){
			return (std::abs(std::abs(a.phi) - M_PI) < 0.4 ? a.r : 1e6) < (std::abs(std::abs(b.phi) - M_PI) < 0.4 ? b.r : 1e6);
		});

	// 🚫 Si hay obstáculo al frente → pasar a TURN
	if (front_it->r < 900)
{
    // Elegir dirección de giro aleatoria
    bool turn_right = (std::rand() % 2 == 0);
    float rot = turn_right ? 0.6f : -0.6f;

    qInfo() << "[FORWARD] Obstáculo al frente → TURN"
            << (turn_right ? "(derecha)" : "(izquierda)");

    return {State::TURN, 0.0f, rot};
}

	// ✅ Si hay espacio libre en todas las direcciones y no ha hecho el espiral todavía
	if (!spiral_done && front_it->r > 1400 && left_it->r > 1400 && right_it->r > 1400 && back_it->r > 1400)
	{
		//spiral_done = true;  // 🔒 Solo una vez
		qInfo() << "[FORWARD] Zona completamente despejada → cambio a SPIRAL";
		return {State::SPIRAL, 500.0f, 0.5f};
	}

	// 🚶 Si no hay nada especial → seguir avanzando

	return {State::FORWARD, 1000.0f, 0.0f};
}


// ----------------------------------------------------------
// Estado TURN → El robot gira en su lugar hasta que el frente esté despejado.
// Cuando despeja, decide aleatoriamente si continuar en FORWARD o pasar a FOLLOW_WALL.
// ----------------------------------------------------------
std::tuple<State, float, float> SpecificWorker::TURN_method(const std::optional<RoboCompLidar3D::TPoints> &filtered_data)
{
    static auto turn_start_time = std::chrono::steady_clock::now();
	static float turn_duracion= 0.0f;
	static bool inicialized = false;

	if(!filtered_data.has_value() || filtered_data->empty()){
	return {State::IDLE,0.0f, 0.0f};
	}

	if(!inicialized){
		turn_duracion = 1.0f + static_cast<float>(std::rand() % 2001) /1000.0f;
		turn_start_time = std::chrono::steady_clock::now();
		inicialized = true;
		qInfo() << "Turn iniciado con "<<turn_duracion<< "segundos";
	}
	float elapsed = std::chrono::duration<float>(std::chrono::steady_clock::now() - turn_start_time).count();
	if(elapsed >= turn_duracion){
	inicialized= false;

	int r = std::rand() % 10;
	if(r<5){
	qInfo() << "Turn -> Follow_wall";
	return {State::SFO, 800.0f, 0.0f};
	}
	else{
	qInfo() << "Turn -> Forward";
	return {State::FORWARD, 1000.0f, 0.0f};
    }
	}

    // Si sigue bloqueado → continuar girando
    return {State::TURN, 0.0f, 0.6f};
}



// ----------------------------------------------------------
// Estado FOLLOW_WALL → Mantiene una distancia constante respecto a una pared lateral.
// ----------------------------------------------------------
std::tuple<State, float, float> SpecificWorker::FOLLOW_WALL_method(const std::optional<RoboCompLidar3D::TPoints> &filtered_data)
{
   static bool inicialized = false;
	auto &points = filtered_data.value();
	if(!filtered_data.has_value() || filtered_data->empty()){
		return {State::IDLE,0.0f, 0.0f};
	}

   if(inicialized){
      auto left_it = std::min_element(points.begin(), points.end(),[](const auto& a, const auto& b )
		{return (a.phi < 0 ? a.r : 1e6) < (b.phi < 0 ? b.r : 1e6);});
   	  auto right_it = std::min_element(points.begin(), points.end(),[](const auto& a, const auto& b )
		  {return (a.phi > 0 ? a.r : 1e6) < (b.phi > 0 ? b.r : 1e6);});

	follow_right= (right_it->r < left_it->r);
	inicialized = true;

    qInfo() << "[FOLLOW_WALL] iniciado en pared de "<<(follow_right ? "D" : "I")
				<< "distancia L:" <<left_it->r << "R: "<<right_it->r;
	}

	auto min_front = std::min_element(points.begin(), points.end(),[](const auto& a, const auto& b){
			return(std::abs(a.phi) <0.2 ? a.r : 1e6) < (std::abs(b.phi) <0.2 ? b.r : 1e6);});

	if(min_front->r < 900){
     inicialized = false;
	follow_right= !follow_right;
	 float rot = follow_right ? 0.6f : -0.6f;
	 qInfo() << "Follow wall pasando a turn por obstaculo";
	return {State::TURN, 1000.0f, rot};
	}



	auto wall_it = follow_right ? std::min_element(points.begin(), points.end(),[](const auto& a, const auto& b )
		{return (a.phi < 0 ? a.r : 1e6) < (b.phi < 0 ? b.r : 1e6);}) :
								  std::min_element(points.begin(), points.end(),[](const auto& a, const auto& b )
		{return (a.phi > 0 ? a.r : 1e6) < (b.phi > 0 ? b.r : 1e6);});

   float dist = wall_it->r;
   float target_dist = 800.0f;
   float error = dist - target_dist;

   float rot = std::clamp(error / 1000.0f, -0.2f, 0.2f);
	if(!follow_right) rot = -rot;

	float adv = 600.0f;




    return {State::SFO, adv, rot};
}

// ----------------------------------------------------------
// Estado SPIRAL → El robot avanza describiendo una espiral creciente.
// La rotación aumenta poco a poco, simulando un patrón de búsqueda.
// ----------------------------------------------------------
std::tuple<State, float, float> SpecificWorker::SPIRAL_method(const std::optional<RoboCompLidar3D::TPoints> &filtered_data)
{
    if (!filtered_data.has_value() || filtered_data->empty())
        return {State::IDLE, 0.0f, 0.0f};

    auto &points = filtered_data.value();

    // ⚙️ Variables estáticas para mantener el giro entre llamadas
    static float rot = 0.7f;          // giro base
    static float adv = 700.0f;        // velocidad lineal
    static auto last_update = std::chrono::steady_clock::now();

    // 🔄 Incrementamos lentamente el radio de la espiral (menor rotación = más abierto)
    auto now = std::chrono::steady_clock::now();
    float elapsed = std::chrono::duration<float>(now - last_update).count();

    if (elapsed > 0.2f)  // cada 200 ms
    {
    	rot += 0.02f;
    	if (rot > 0.9f)
    		rot = 0.1f;
        last_update = now;
    }

    // 🚨 Detectar obstáculos cercanos frontales
    auto front_it = std::min_element(points.begin(), points.end(),
                                     [](const auto &a, const auto &b){
                                         return (std::abs(a.phi) < 0.3 ? a.r : 1e6) < (std::abs(b.phi) < 0.3 ? b.r : 1e6);
                                     });

    if (front_it != points.end() && front_it->r < 800) // obstáculo frontal
    {
        qInfo() << "[SPIRAL] Obstáculo detectado, pasando a TURN";
        return {State::TURN, 0.0f, 0.6f};
    }

    // 🧱 Detectar si hay pared lateral (izquierda o derecha)
    auto left_it = std::min_element(points.begin(), points.end(),
                                    [](const auto &a, const auto &b){ return (a.phi < 0 ? a.r : 1e6) < (b.phi < 0 ? b.r : 1e6); });
    auto right_it = std::min_element(points.begin(), points.end(),
                                     [](const auto &a, const auto &b){ return (a.phi > 0 ? a.r : 1e6) < (b.phi > 0 ? b.r : 1e6); });

    if (left_it->r < 1000 || right_it->r < 1000)
    {
        follow_right = (right_it->r < left_it->r);
        qInfo() << "[SPIRAL] Detectada pared → pasando a FOLLOW_WALL (lado:" << (follow_right ? "derecha" : "izquierda") << ")";
        return {State::SFO, 800.0f, 0.0f};
    }

    // 🌀 Movimiento normal en espiral
    qInfo() << "[SPIRAL] Avanzando en espiral (adv:" << adv << ", rot:" << rot << ")";
    return {State::SPIRAL, adv, rot};
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////7
std::optional<RoboCompLidar3D::TPoints> SpecificWorker::filter_min_distance_cppitertools(const RoboCompLidar3D::TPoints& points)
{
	if (points.empty())
		return {};

	RoboCompLidar3D::TPoints result;
	result.reserve(points.size());

	for (auto&& [angle, group] : iter::groupby(points, [](const auto& p)
		{ float multiplier = std::pow(10.0f, 2); return std::floor(p.phi * multiplier) / multiplier; }))
	{
		auto min_ele = std::min_element(std::begin(group), std::end(group),
									   [](const auto& a, const auto& b){ return a.r < b.r; });

		// 🔹 Mantén solo los puntos frontales (-90° a +90°)
		result.emplace_back(*min_ele);

	}

	// 🔹 Ordenar los puntos de más cercano a más lejano
	// std::sort(result.begin(), result.end(),
	// 		  [](const auto& a, const auto& b){ return a.r < b.r; });

	return result;
}


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

void SpecificWorker::new_target_slot(QPointF point)
{
	qInfo() << "New target";
}

void SpecificWorker::draw_lidar(const RoboCompLidar3D::TPoints &points, QGraphicsScene* scene)
{
	static std::vector<QGraphicsItem*> draw_points;
	for (const auto &p : draw_points)
	{
		scene->removeItem(p);
		delete p;
	}
	draw_points.clear();

	const QColor color("LightGreen");
	const QPen pen(color, 10);
	//const QBrush brush(color, Qt::SolidPattern);
	for (const auto &p : points)
	{
		const auto dp = scene->addRect(-25, -25, 50, 50, pen);
		dp->setPos(p.x, p.y);
		draw_points.push_back(dp);   // add to the list of points to be deleted next time
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