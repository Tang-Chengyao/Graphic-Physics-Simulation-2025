#pragma once

#include <utility>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <glm/glm.hpp>

namespace VCX::Labs::GettingStarted {
    struct MassSpringSystem {
        struct Spring {
            std::pair<std::size_t, std::size_t> AdjIdx;
            float                               RestLength;
        };

        std::vector<glm::vec3> Positions;
        std::vector<glm::vec3> Velocities;
        std::vector<int>       Fixed;
        float                  Mass { 1 };

        std::vector<Spring> Springs;
        float               Stiffness { 100 };
        float               Damping { .2f };
        float               Gravity { .3f };

        void AddParticle(glm::vec3 const & position, glm::vec3 const & velocity = glm::vec3(0)) {
            Positions.push_back(position);
            Velocities.push_back(velocity);
            Fixed.push_back(false);
        }

        void AddSpring(std::size_t const adjIdx0, std::size_t const adjIdx1, float const restLength = -1) {
            Springs.push_back({
                .AdjIdx { adjIdx0, adjIdx1 },
                .RestLength { restLength < 0 ? glm::length(Positions[adjIdx0] - Positions[adjIdx1]) : restLength }
            });
        }

        void AdvanceMassSpringSystem(float const dt) {
            auto const                         nDoFs = int(Positions.size()) * 3;
            Eigen::SparseMatrix<float>         matLinearized(nDoFs, nDoFs);
            std::vector<Eigen::Triplet<float>> coefficients;

            const auto AddBlock = [&](int const p0, int const p1, glm::mat3 const & block) {
                for (int i = 0; i < 3; i++)
                    for (int j = 0; j < 3; j++)
                        coefficients.emplace_back(p0 * 3 + i, p1 * 3 + j, block[j][i]);
            };

            for (int i = 0; i < nDoFs; i++) coefficients.emplace_back(i, i, Mass);

            for (auto const & spring : Springs) {
                auto const p0 = spring.AdjIdx.first;
                auto const p1 = spring.AdjIdx.second;
                if (Fixed[p0] || Fixed[p1]) continue;
                glm::vec3 const x01   = Positions[p1] - Positions[p0];
                glm::vec3 const e01   = glm::normalize(x01);
                glm::mat3 const block = Damping * glm::outerProduct(e01, e01) * dt;
                AddBlock(p0, p0, -block);
                AddBlock(p0, p1, block);
                AddBlock(p1, p0, block);
                AddBlock(p1, p1, -block);
            }

            matLinearized.setFromTriplets(coefficients.begin(), coefficients.end());

            std::vector<glm::vec3> forces(Positions.size(), glm::vec3(0, -Gravity, 0) * Mass);
            for (std::size_t i = 0; i < Positions.size(); i++) {
                if (Fixed[i]) forces[i] = glm::vec3(0);
            }
            for (auto const & spring : Springs) {
                auto const      p0  = spring.AdjIdx.first;
                auto const      p1  = spring.AdjIdx.second;
                glm::vec3 const x01 = Positions[p1] - Positions[p0];
                glm::vec3 const v01 = Velocities[p1] - Velocities[p0];
                glm::vec3 const e01 = glm::normalize(x01);
                glm::vec3       f   = (Stiffness * (glm::length(x01) - spring.RestLength) + Damping * glm::dot(v01, e01)) * e01;
                if (! Fixed[p0]) forces[p0] += f;
                if (! Fixed[p1]) forces[p1] -= f;
            }

            auto vecVelocities = Eigen::Map<Eigen::VectorXf, Eigen::Aligned>(reinterpret_cast<float *>(Velocities.data()), nDoFs);
            auto vecForces     = Eigen::Map<Eigen::VectorXf, Eigen::Aligned>(reinterpret_cast<float *>(forces.data()), nDoFs);

            Eigen::VectorXf rhsLinearized = matLinearized * vecVelocities + vecForces * dt;

            for (auto const & spring : Springs) {
                auto const p0 = spring.AdjIdx.first;
                auto const p1 = spring.AdjIdx.second;
                if (Fixed[p0] || Fixed[p1]) continue;
                glm::vec3 const x01    = Positions[p1] - Positions[p0];
                glm::vec3 const e01    = glm::normalize(x01);
                float const     length = glm::length(x01);
                glm::mat3 const block  = Stiffness * ((spring.RestLength / length - 1) * glm::mat3(1) - spring.RestLength / length * glm::outerProduct(e01, e01)) * dt * dt;
                AddBlock(p0, p0, -block);
                AddBlock(p0, p1, block);
                AddBlock(p1, p0, block);
                AddBlock(p1, p1, -block);
            }

            matLinearized.setFromTriplets(coefficients.begin(), coefficients.end());
            auto solver   = Eigen::SimplicialLLT<Eigen::SparseMatrix<float>>(matLinearized);
            vecVelocities = solver.solve(rhsLinearized);

            for (std::size_t i = 0; i < Positions.size(); i++) {
                Positions[i] += Velocities[i] * dt;
            }
        }
    };
} // namespace VCX::Labs::GettingStarted
