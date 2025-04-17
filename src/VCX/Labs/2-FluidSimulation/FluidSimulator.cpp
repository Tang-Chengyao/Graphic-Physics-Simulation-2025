#include "FluidSimulator.h"

using namespace VCX::Labs::Fluid;

void Simulator::integrateParticles(float timeStep) {
    for (int i { 0 }; i < m_iNumSpheres; i++) {
        m_particleVel[i] += timeStep * gravity;
        m_particlePos[i] += timeStep * m_particleVel[i];
    }
}

void Simulator::buildHashTable() {
    // 重置哈希表索引
    m_hashtableindex.assign(m_iNumCells + 1, 0); // +1 是为了方便计算范围

    int n = m_iCellY * m_iCellZ;
    int m = m_iCellZ;
    // 统计每个网格的粒子数量
    for (int p = 0; p < m_iNumSpheres; p++) {
        glm::ivec3 gridIdx = posToGridIndex(m_particlePos[p]);
        int        index   = gridIdx.x * n + gridIdx.y * m + gridIdx.z;
        m_hashtableindex[index + 1]++; // 从 index=1 开始累加
    }

    // 计算前缀和，确定每个网格的粒子在 m_hashtable 中的范围
    for (int i = 1; i <= m_iNumCells; i++) {
        m_hashtableindex[i] += m_hashtableindex[i - 1];
    }

    // 填充粒子索引到 m_hashtable
    m_hashtable.resize(m_iNumSpheres);
    std::vector<int> count(m_iNumCells, 0); // 临时计数器
    for (int p = 0; p < m_iNumSpheres; p++) {
        glm::ivec3 gridIdx  = posToGridIndex(m_particlePos[p]);
        int        index    = gridIdx.x * n + gridIdx.y * m + gridIdx.z;
        int        offset   = m_hashtableindex[index] + count[index];
        m_hashtable[offset] = p;
        count[index]++;
    }
}

void Simulator::pushParticlesApart(int numIters) {
    for (int iter = 0; iter < numIters; iter++) {
        buildHashTable(); // 每次迭代前更新哈希表

        for (int pi = 0; pi < m_iNumSpheres; pi++) {
            glm::ivec3 gridIdx = posToGridIndex(m_particlePos[pi]);

            // 遍历当前网格及其相邻 26 个网格
            for (int di = -1; di <= 1; di++) {
                for (int dj = -1; dj <= 1; dj++) {
                    for (int dk = -1; dk <= 1; dk++) {
                        int ni = gridIdx.x + di;
                        int nj = gridIdx.y + dj;
                        int nk = gridIdx.z + dk;

                        // 检查网格是否合法
                        if (ni < 0 || ni >= m_iCellX || nj < 0 || nj >= m_iCellY || nk < 0 || nk >= m_iCellZ)
                            continue;

                        int cellIdx = ni * m_iCellY * m_iCellZ + nj * m_iCellZ + nk;
                        int start   = m_hashtableindex[cellIdx];
                        int end     = m_hashtableindex[cellIdx + 1];

                        // 检查该网格中的所有粒子
                        for (int idx = start; idx < end; idx++) {
                            int pj = m_hashtable[idx];
                            if (pi == pj) continue; // 跳过自身

                            glm::vec3 d       = m_particlePos[pj] - m_particlePos[pi];
                            float     lenth_d = glm::length(d);
                            if (lenth_d < 2 * m_particleRadius) {
                                glm::vec3 s = (2 * m_particleRadius - lenth_d) * (d / lenth_d);
                                m_particlePos[pj] += s;
                                m_particlePos[pi] -= s;
                            }
                        }
                    }
                }
            }
        }
    }
}

void Simulator::handleParticleCollisions(glm::vec3 obstaclePos, float obstacleRadius, glm::vec3 obstacleVel) {
    for (int p = 0; p < m_iNumSpheres; p++) {
        if (m_particlePos[p].x < xmin + m_h + m_particleRadius) {
            m_particlePos[p].x = xmin + m_h + m_particleRadius;
            m_particleVel[p].x = 0.0f;
        }
        if (m_particlePos[p].y < ymin + m_h + m_particleRadius) {
            m_particlePos[p].y = ymin + m_h + m_particleRadius;
            m_particleVel[p].y = 0.0f;
        }
        if (m_particlePos[p].z < zmin + m_h + m_particleRadius) {
            m_particlePos[p].z = zmin + m_h + m_particleRadius;
            m_particleVel[p].z = 0.0f;
        }
        if (m_particlePos[p].x > xmax - m_h - m_particleRadius) {
            m_particlePos[p].x = xmax - m_h - m_particleRadius;
            m_particleVel[p].x = 0.0f;
        }
        if (m_particlePos[p].y > ymax - m_h - m_particleRadius) {
            m_particlePos[p].y = ymax - m_h - m_particleRadius;
            m_particleVel[p].y = 0.0f;
        }
        if (m_particlePos[p].z > zmax - m_h - m_particleRadius) {
            m_particlePos[p].z = zmax - m_h - m_particleRadius;
            m_particleVel[p].z = 0.0f;
        }
        // 障碍物处理
        glm::vec3 d        = m_particlePos[p] - obstaclePos;
        float     length_d = glm::length(d);
        glm::vec3 n        = d / length_d;
        if (length_d < obstacleRadius + m_particleRadius) {
            glm::vec3 s = (obstacleRadius + m_particleRadius - length_d) * n;
            m_particlePos[p] += s;
        }
    }
}
void Simulator::updateParticleDensity() {
    m_particleDensity.clear();
    m_particleDensity.resize(m_iNumCells, 0.0f);

    int n = m_iCellY * m_iCellZ;
    int m = m_iCellZ;
    for (int p = 0; p < m_iNumSpheres; p++) {
        float xp0      = m_particlePos[p].x - xmin;
        float yp0      = m_particlePos[p].y - xmin;
        float zp0      = m_particlePos[p].z - zmin;
        float xp0_half = xp0 - m_h / 2;
        float yp0_half = yp0 - m_h / 2;
        float zp0_half = zp0 - m_h / 2;
        int   i_half   = static_cast<int>(xp0_half / m_h);
        int   j_half   = static_cast<int>(yp0_half / m_h);
        int   k_half   = static_cast<int>(zp0_half / m_h);

        float Dx = xp0_half - i_half * m_h;
        float Dy = yp0_half - j_half * m_h;
        float Dz = zp0_half - k_half * m_h;

        const float w[8] = {
            (1 - Dx) * (1 - Dy) * (1 - Dz),
            (1 - Dx) * (1 - Dy) * Dz,
            (1 - Dx) * Dy * (1 - Dz),
            (1 - Dx) * Dy * Dz,
            Dx * (1 - Dy) * (1 - Dz),
            Dx * (1 - Dy) * Dz,
            Dx * Dy * (1 - Dz),
            Dx * Dy * Dz
        };

        const int di[8] = { 0, 0, 0, 0, 1, 1, 1, 1 };
        const int dj[8] = { 0, 0, 1, 1, 0, 0, 1, 1 };
        const int dk[8] = { 0, 1, 0, 1, 0, 1, 0, 1 };

        for (int idx = 0; idx < 8; idx++) {
            int index = (i_half + di[idx]) * n + (j_half + dj[idx]) * m + (k_half + dk[idx]);
            m_particleDensity[index] += w[idx];
        }
    }
}

void Simulator::transferVelocities(bool toGrid, float flipRatio) {
    const int n = m_iCellY * m_iCellZ;
    const int m = m_iCellZ;
    if (toGrid) {
        std::vector<glm::vec3> m_momentum; // 网格动量
        std::vector<float>     m_mass;     // 网格质量
        m_momentum.clear();
        m_momentum.resize(m_iNumCells, glm::vec3(0.0f));
        m_mass.clear();
        m_mass.resize(m_iNumCells, 0.0f);

        auto P2G = [&](int i, int j, int k, float Dx, float Dy, float Dz, int p, int mode) {
            const float w[8] = {
                (1 - Dx) * (1 - Dy) * (1 - Dz),
                (1 - Dx) * (1 - Dy) * Dz,
                (1 - Dx) * Dy * (1 - Dz),
                (1 - Dx) * Dy * Dz,
                Dx * (1 - Dy) * (1 - Dz),
                Dx * (1 - Dy) * Dz,
                Dx * Dy * (1 - Dz),
                Dx * Dy * Dz
            };
            const int di[8] = { 0, 0, 0, 0, 1, 1, 1, 1 };
            const int dj[8] = { 0, 0, 1, 1, 0, 0, 1, 1 };
            const int dk[8] = { 0, 1, 0, 1, 0, 1, 0, 1 };

            for (int idx = 0; idx < 8; idx++) {
                int index = (i + di[idx]) * n + (j + dj[idx]) * m + (k + dk[idx]);
                m_mass[index] += w[idx];

                if (mode == 0) {
                    m_momentum[index].x += w[idx] * m_particleVel[p].x;
                } else if (mode == 1) {
                    m_momentum[index].y += w[idx] * m_particleVel[p].y;
                } else {
                    m_momentum[index].z += w[idx] * m_particleVel[p].z;
                }
            }
        };

        for (int p { 0 }; p < m_iNumSpheres; p++) { // 遍历particles
            float xp0      = m_particlePos[p].x - xmin;
            float yp0      = m_particlePos[p].y - xmin;
            float zp0      = m_particlePos[p].z - zmin;
            float xp0_half = xp0 - m_h / 2;
            float yp0_half = yp0 - m_h / 2;
            float zp0_half = zp0 - m_h / 2;

            int i      = static_cast<int>(xp0 / m_h);
            int j      = static_cast<int>(yp0 / m_h);
            int k      = static_cast<int>(zp0 / m_h);
            int i_half = static_cast<int>(xp0_half / m_h);
            int j_half = static_cast<int>(yp0_half / m_h);
            int k_half = static_cast<int>(zp0_half / m_h);

            float Dx      = xp0 - i * m_h;
            float Dy      = yp0 - j * m_h;
            float Dz      = zp0 - k * m_h;
            float Dx_half = xp0_half - i_half * m_h;
            float Dy_half = yp0_half - j_half * m_h;
            float Dz_half = zp0_half - k_half * m_h;

            P2G(i, j_half, k_half, Dx, Dy_half, Dz_half, p, 0); // uVel
            P2G(i_half, j, k_half, Dx_half, Dy, Dz_half, p, 1); // vVel
            P2G(i_half, j_half, k, Dx_half, Dy_half, Dz, p, 2); // wVel
        }

        for (int i { 0 }; i < m_iCellX; i++) {
            for (int j { 0 }; j < m_iCellY; j++) {
                for (int k { 0 }; k < m_iCellZ; k++) { // update m_vel and label m_type
                    int index = i * n + j * m + k;
                    if (i == 0 || i >= m_iCellX - 2 || j == 0 || j >= m_iCellY - 2 || k == 0 || k >= m_iCellZ - 2) { // 边界
                        m_type[index] = 2;                                                                           // solid
                        m_vel[index]  = glm::vec3(0.0f);
                    } else {
                        if (m_mass[index] > 0.0f) { // has liquid
                            m_type[index] = 1;
                            m_vel[index]  = m_momentum[index] / m_mass[index];
                        } else { // air
                            m_type[index] = 0;
                            m_vel[index]  = glm::vec3(0.0f);
                        }
                    }
                }
            }
        }
    } else {
        std::vector<glm::vec3> m_PicMomentum;
        std::vector<float>     m_particleMass;
        std::vector<glm::vec3> m_FlipDeltaMomentum;
        m_PicMomentum.clear();
        m_PicMomentum.resize(m_iNumSpheres, glm::vec3(0.0f));
        m_particleMass.clear();
        m_particleMass.resize(m_iNumSpheres, 0.0f);
        m_FlipDeltaMomentum.clear();
        m_FlipDeltaMomentum.resize(m_iNumSpheres, glm::vec3(0.0f));

        auto G2P = [&](int i, int j, int k, float Dx, float Dy, float Dz, int p, int mode) {
            const float w[8] = {
                (1 - Dx) * (1 - Dy) * (1 - Dz),
                (1 - Dx) * (1 - Dy) * Dz,
                (1 - Dx) * Dy * (1 - Dz),
                (1 - Dx) * Dy * Dz,
                Dx * (1 - Dy) * (1 - Dz),
                Dx * (1 - Dy) * Dz,
                Dx * Dy * (1 - Dz),
                Dx * Dy * Dz
            };
            const int di[8] = { 0, 0, 0, 0, 1, 1, 1, 1 };
            const int dj[8] = { 0, 0, 1, 1, 0, 0, 1, 1 };
            const int dk[8] = { 0, 1, 0, 1, 0, 1, 0, 1 };

            for (int idx = 0; idx < 8; idx++) {
                int index = (i + di[idx]) * n + (j + dj[idx]) * m + (k + dk[idx]);
                m_particleMass[p] += w[idx];

                if (mode == 0) {
                    m_PicMomentum[p].x += w[idx] * m_vel[index].x;
                    m_FlipDeltaMomentum[p].x += w[idx] * (m_vel[index].x - m_pre_vel[index].x);
                } else if (mode == 1) {
                    m_PicMomentum[p].y += w[idx] * m_vel[index].y;
                    m_FlipDeltaMomentum[p].y += w[idx] * (m_vel[index].y - m_pre_vel[index].y);
                } else {
                    m_PicMomentum[p].z += w[idx] * m_vel[index].z;
                    m_FlipDeltaMomentum[p].z += w[idx] * (m_vel[index].z - m_pre_vel[index].z);
                }
            }
        };

        for (int p { 0 }; p < m_iNumSpheres; p++) {
            float xp0      = m_particlePos[p].x - xmin;
            float yp0      = m_particlePos[p].y - xmin;
            float zp0      = m_particlePos[p].z - zmin;
            float xp0_half = xp0 - m_h / 2;
            float yp0_half = yp0 - m_h / 2;
            float zp0_half = zp0 - m_h / 2;

            int i      = static_cast<int>(xp0 / m_h);
            int j      = static_cast<int>(yp0 / m_h);
            int k      = static_cast<int>(zp0 / m_h);
            int i_half = static_cast<int>(xp0_half / m_h);
            int j_half = static_cast<int>(yp0_half / m_h);
            int k_half = static_cast<int>(zp0_half / m_h);

            float Dx      = xp0 - i * m_h;
            float Dy      = yp0 - j * m_h;
            float Dz      = zp0 - k * m_h;
            float Dx_half = xp0_half - i_half * m_h;
            float Dy_half = yp0_half - j_half * m_h;
            float Dz_half = zp0_half - k_half * m_h;

            G2P(i, j_half, k_half, Dx, Dy_half, Dz_half, p, 0);
            G2P(i_half, j, k_half, Dx_half, Dy, Dz_half, p, 1);
            G2P(i_half, j_half, k, Dx_half, Dy_half, Dz, p, 2);
        }

        for (int p { 0 }; p < m_iNumSpheres; p++) { // 更新 m_particleVel
            m_particleVel[p] = (1 - flipRatio) * m_PicMomentum[p] / m_particleMass[p] + flipRatio * (m_particleVel[p] + m_FlipDeltaMomentum[p] / m_particleMass[p]);
        }
    }
}

void Simulator::solveIncompressibility(int numIters, float dt, float overRelaxation, bool compensateDrift) { // Gauss-Seidel
    const int n = m_iCellY * m_iCellZ;
    const int m = m_iCellZ;
    m_pre_vel   = m_vel; // 复制一份 m_vel
    for (int iter = 0; iter < numIters; iter++) {
        for (int i = 0; i < m_iCellX; i++) {
            for (int j = 0; j < m_iCellY; j++) {
                for (int k = 0; k < m_iCellZ; k++) {
                    const int index = i * n + j * m + k;
                    if (m_type[index] == 1) {                                                                                                      // liquidus
                        float d = m_vel[index + n].x - m_vel[index].x + m_vel[index + m].y - m_vel[index].y + m_vel[index + 1].z - m_vel[index].z; // 该网格的散度
                        d *= overRelaxation;
                        if (compensateDrift) {
                            if (m_particleDensity[index] > m_particleRestDensity) {
                                d -= ko * (m_particleDensity[index] - m_particleRestDensity);
                            }
                        }
                        float s = m_s[index + n] + m_s[index - n] + m_s[index + m] + m_s[index - m] + m_s[index + 1] + m_s[index - 1];
                        m_vel[index].x += d * m_s[index - n] / s;
                        m_vel[index + n].x += -d * m_s[index + n] / s;
                        m_vel[index].y += d * m_s[index - m] / s;
                        m_vel[index + m].y += -d * m_s[index + m] / s;
                        m_vel[index].z += d * m_s[index - 1] / s;
                        m_vel[index + 1].z += -d * m_s[index + 1] / s;
                    }
                }
            }
        }
    }
}

void Simulator::updateParticleColors() {}