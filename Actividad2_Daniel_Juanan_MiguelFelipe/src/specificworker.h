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
#include <vector>
#include <tuple>
#include <abstract_graphic_viewer/abstract_graphic_viewer.h>
#include "common_types.h"
#include <optional>
#include "hungarian.h"
#include "room_detector.h"
#include "ransac_line_detector.h"

enum class State {IDLE, FORWARD, TURN, SFO, SPIRAL};
/**
 * \brief Class SpecificWorker implements the core functionality of the component.
 */
class SpecificWorker : public GenericWorker
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


	/**
     * \brief Destructor for SpecificWorker.
     */
	~SpecificWorker();


public slots:

	/**
	 * \brief Initializes the worker one time.
	 */
	void initialize();

	/**
	 * \brief Main compute loop of the worker.
	 */
	void compute();

	/**
	 * \brief Handles the emergency state loop.
	 */
	void emergency();

	/**
	 * \brief Restores the component from an emergency state.
	 */
	void restore();

    /**
     * \brief Performs startup checks for the component.
     * \return An integer representing the result of the checks.
     */
	int startup_check();

    void new_target_slot(QPointF point);

private:

	QRectF dimensions;
	AbstractGraphicViewer *viewer;
	const int ROBOT_LENGTH = 400;
	QGraphicsPolygonItem *robot_polygon;

	AbstractGraphicViewer *room_viewer;
	QRectF room_dimensions;
	QGraphicsPolygonItem *robot_draw;

	std::optional<RoboCompLidar3D::TPoints> filter_min_distance_cppitertools(const RoboCompLidar3D::TPoints& points);
	void draw_lidar(const RoboCompLidar3D::TPoints &points, QGraphicsScene* scene);
	/**
     * \brief Flag indicating whether startup checks are enabled.
     */
	State state = State::FORWARD;
	std::tuple<State, float, float> FORWARD_method(const std::optional<RoboCompLidar3D::TPoints> &filtered_data);
	std::tuple<State, float, float> TURN_method(const std::optional<RoboCompLidar3D::TPoints> &filtered_data);
	std::tuple<State, float, float> FOLLOW_WALL_method(const std::optional<RoboCompLidar3D::TPoints> &filtered_data);
	std::tuple<State, float, float> SPIRAL_method(const std::optional<RoboCompLidar3D::TPoints> &filtered_data);
	std::tuple<State, float, float> stateMachine(const std::optional<RoboCompLidar3D::TPoints> &filtered_data);
	std::optional<RoboCompLidar3D::TPoints> read_data();

	void updateRobotInRoom (float x, float y, float alpha);
	bool startup_check_flag;
	bool follow_right = true;  // true = sigue pared derecha, false = izquierda
	bool spiral_done = false;
	rc::Hungarian hungarian;
	rc::Room_Detector room_detector;
	AbstractGraphicViewer *viewer_room;
	Eigen::Affine2d robot_pose;
	QGraphicsPolygonItem *room_draw_robot;
	struct NominalRoom
	{
		float width; //  mm
		float length;
		Corners corners;
		explicit NominalRoom(const float width_=10000.f, const float length_=5000.f, Corners  corners_ = {}) : width(width_), length(length_), corners(std::move(corners_)){};
		Corners transform_corners_to(const Eigen::Affine2d &transform) const  // for room to robot pass the inverse of robot_pose
		{
			Corners transformed_corners;
			for(const auto &[p, _, __] : corners)
			{
				auto ep = Eigen::Vector2d{p.x(), p.y()};
				Eigen::Vector2d tp = transform * ep;
				transformed_corners.emplace_back(QPointF{static_cast<float>(tp.x()), static_cast<float>(tp.y())}, 0.f, 0.f);
			}
			return transformed_corners;
		}
	};
	NominalRoom room{10000.f, 5000.f,
				{{QPointF{-5000.f, -2500.f}, 0.f, 0.f},
					   {QPointF{5000.f, -2500.f}, 0.f, 0.f},
					   {QPointF{5000.f, 2500.f}, 0.f, 0.f},
					   {QPointF{-5000.f, 2500.f}, 0.f, 0.f}}};

	struct Params
	{
		QRectF GRID_MAX_DIM;     // Tamaño máximo del grid (la habitación)
		float ROBOT_WIDTH;       // Ancho del robot en mm
		float ROBOT_LENGTH;      // Largo del robot en mm

		Params()
		{
			GRID_MAX_DIM = QRectF(-5000, -2500, 10000, 5000); // habitación de 10x5 metros
			ROBOT_WIDTH = 400.0f;   // mm
			ROBOT_LENGTH = 500.0f;  // mm
		}
	};
	Params params;



signals:
	//void customSignal();
};

#endif