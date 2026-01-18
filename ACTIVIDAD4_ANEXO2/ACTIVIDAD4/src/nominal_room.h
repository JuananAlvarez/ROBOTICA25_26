#pragma once
#include <QPointF>
#include <QRectF>
#include <QString>          // <-- AÑADIDO
#include <Eigen/Dense>
#include <vector>
#include "common_types.h"

/*
 *Cambios 9-12
*✅ PASO 2 — Guardar correctamente las puertas en cada habitación (NominalRoom)
 **/
struct NominalRoom
{
    QString name;           // <-- AÑADIDO (nombre de la habitación)
    bool visited = false;
    float width;            // mm
    float length;
    Doors doors;            // Lista de puertas detectadas y proyectadas en esta room

    explicit NominalRoom(const float width_=10000.f,
                         const float length_=5000.f,
                         Corners corners_ = {},
                         const QString &name_ = {}) :
        name(name_),
        width(width_),
        length(length_)
    {
        (void)corners_; // por ahora no se usa
    }

    [[nodiscard]] Corners corners() const
    {
        // compute corners from width and length
        return {
            {QPointF{-width/2.f, -length/2.f}, 0.f, 0.f},
            {QPointF{ width/2.f, -length/2.f}, 0.f, 0.f},
            {QPointF{ width/2.f,  length/2.f}, 0.f, 0.f},
            {QPointF{-width/2.f,  length/2.f}, 0.f, 0.f}
        };
    }

    [[nodiscard]] QRectF rect() const
    {
        return QRectF{-width/2.f, -length/2.f, width, length};
    }

    [[nodiscard]] Corners transform_corners_to(const Eigen::Affine2f &transform) const   // float
  {
        Corners transformed_corners;
        transformed_corners.reserve(corners().size());

        for(const auto &[p, _, __] : corners())
        {
            Eigen::Vector2f ep{p.x(), p.y()};
            Eigen::Vector2f tp = transform * ep;
            transformed_corners.emplace_back(QPointF{tp.x(), tp.y()}, 0.f, 0.f);
        }
        return transformed_corners;
  }

    // =====================================================================================
    // APARTADO C — MÉTODOS NUEVOS PARA EL TASK IV
    // CAMBIOS 9/12
    // =====================================================================================

    // ---------------------------------------------------------------
    // Devuelve las paredes de la habitación como líneas paramétricas
    // ---------------------------------------------------------------
    Walls get_walls() const
    {
        Walls walls;

        const auto cs = corners();
        const size_t N = cs.size();
        walls.reserve(N);

        for (size_t i = 0; i < N; i++)
        {
            const auto &c1 = cs[i];
            const auto &c2 = cs[(i + 1) % N];

            const QPointF &q1 = std::get<0>(c1);
            const QPointF &q2 = std::get<0>(c2);

            Eigen::Vector2f p1(q1.x(), q1.y());
            Eigen::Vector2f p2(q2.x(), q2.y());

            Eigen::ParametrizedLine<float,2> line =
                Eigen::ParametrizedLine<float,2>::Through(p1, p2);

            walls.emplace_back(line, static_cast<int>(i), c1, c2);
        }

        return walls;
    }

    // ---------------------------------------------------------------
    // Devuelve la pared más cercana a un punto en coordenadas room
    // ---------------------------------------------------------------
    Wall get_closest_wall_to_point(const Eigen::Vector2f &p) const
    {
        Walls walls = get_walls();

        auto it = std::ranges::min_element(
            walls,
            [&](const Wall &w1, const Wall &w2)
            {
                const auto &l1 = std::get<0>(w1);
                const auto &l2 = std::get<0>(w2);
                return l1.distance(p) < l2.distance(p);
            });

        return *it;
    }

    // ---------------------------------------------------------------
    // Proyecta un punto sobre la pared más cercana
    // ---------------------------------------------------------------
    Eigen::Vector2f get_projection_of_point_on_closest_wall(const Eigen::Vector2f &p) const
    {
        Wall w = get_closest_wall_to_point(p);
        const auto &line = std::get<0>(w);
        return line.projection(p);
    }

private:
    // Guardamos esquinas originales en caso de transformación
    Corners _corners;
};
