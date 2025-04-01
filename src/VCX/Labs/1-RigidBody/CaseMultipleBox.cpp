#include "Labs/1-RigidBody/CaseMultipleBox.h"
#include "Engine/app.h"
#include "Labs/Common/ImGuiHelper.h"
#include <fcl/narrowphase/collision.h>

namespace VCX::Labs::RigidBody {

    CaseMultipleBox::CaseMultipleBox():
        _program(
            Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/flat.vert"), Engine::GL::SharedShader("assets/shaders/flat.frag") })) {
        // 初始化所有wall
        const std::vector<glm::vec3> wallPositions {
            glm::vec3(0.0f, 0.0f, -10.0f),
            glm::vec3(10.0f, 0.0f, -5.0f),
            glm::vec3(0.0f, 10.0f, -5.0f),
            glm::vec3(-10.0f, 0.0f, -5.0f),
            glm::vec3(0.0f, -10.0f, -5.0f)
        };
        const std::vector<glm::vec2> wallDims {
            glm::vec2(20.0f, 20.0f),
            glm::vec2(20.0f, 10.0f),
            glm::vec2(20.0f, 10.0f),
            glm::vec2(20.0f, 10.0f),
            glm::vec2(20.0f, 10.0f)
        };
        const std::vector<glm::vec3> wallTangent1 {
            glm::vec3(1.0f, 0.0f, 0.0f),
            glm::vec3(0.0f, 1.0f, 0.0f),
            glm::vec3(-1.0f, 0.0f, 0.0f),
            glm::vec3(0.0f, -1.0f, 0.0f),
            glm::vec3(1.0f, 0.0f, 0.0f)
        };
        const std::vector<glm::vec3> wallTangent2 {
            glm::vec3(0.0f, 1.0f, 0.0f),
            glm::vec3(0.0f, 0.0f, -1.0f),
            glm::vec3(0.0f, 0.0f, -1.0f),
            glm::vec3(0.0f, 0.0f, -1.0f),
            glm::vec3(0.0f, 0.0f, -1.0f),
        };
        for (int id { 0 }; id < 5; id++) { // 生成_walls列表
            _walls.push_back(Wall(
                id,
                wallPositions[id],
                wallDims[id],
                wallTangent1[id],
                wallTangent2[id]));
        }
        for (auto & wall : _walls) { // 墙的位置固定不动，可以在initialize时就把顶点位置上传给渲染器
            auto span_bytes = Engine::make_span_bytes<glm::vec3>(wall.VerticesPosition);
            wall.LineItem.UpdateVertexBuffer("position", span_bytes);
            wall.TriangleItem.UpdateVertexBuffer("position", span_bytes);
        }

        // 初始化所有box
        float Lx = _walls[0].dim.x;
        float Ly = _walls[0].dim.y;
        float boxSize { 2.0f };
        float horizontalSpacing { 1.0f };
        float verticalSpacing { 1.5f };
        int   Nx = static_cast<int>((Lx - boxSize - 2 * horizontalSpacing) / (boxSize + horizontalSpacing)) + 1;
        int   Ny = static_cast<int>((Ly - boxSize - 2 * horizontalSpacing) / (boxSize + horizontalSpacing)) + 1;

        for (int id { 0 }; id < _numBoxes; id++) {
            int   k = id / (Nx * Ny);
            int   j = (id - k * Nx * Ny) / Nx;
            int   i = id - k * Nx * Ny - j * Nx;
            float x = (-Lx / 2.0f + horizontalSpacing + boxSize / 2.0f) + i * (boxSize + horizontalSpacing);
            float y = (-Ly / 2.0f + horizontalSpacing + boxSize / 2.0f) + j * (boxSize + horizontalSpacing);
            float z = (_walls[0].position.z + verticalSpacing + boxSize / 2.0f) + k * (boxSize + verticalSpacing);
            Box   box(id, 1.0f, glm::vec3(boxSize), glm::vec3(x, y, z), glm::sphericalRand(0.5f), glm::sphericalRand(1.0f));
            float Ek  = 0.5f * box.mass * glm::dot(box.v, box.v) + 0.5f * glm::dot(box.w, box.I_world * box.w);
            box.color = glm::vec4(MapColor(Ek), box.color[3]);
            _boxes.push_back(std::move(box));
        }
        _cameraManager.AutoRotate = false;
        _cameraManager.Save(_camera);
    }

    void CaseMultipleBox::OnSetupPropsUI() {
        if (ImGui::CollapsingHeader("System", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::SliderFloat("Gravity: ", &_gravity, 0.0f, 10.0f);
            ImGui::SliderFloat("Wall-Box Elasticity: ", &_c, 0.0f, 1.0f);
            ImGui::SliderFloat("Box-Box Recovery Coefficient: ", &_muN, 0.0f, 1.0f);
            ImGui::SliderFloat("Box-Box Friction Coefficient: ", &_muT, 0.0f, 1.0f);
        }
        ImGui::Spacing();
        if (_isPaused) {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.0f, 0.7f, 0.0f, 1.0f)); // 绿色
            if (ImGui::Button("Continue", ImVec2(120, 40))) {
                _isPaused = false;
            }
            ImGui::PopStyleColor();
        } else {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.7f, 0.0f, 0.0f, 1.0f)); // 红色
            if (ImGui::Button("Stop", ImVec2(120, 40))) {
                _isPaused = true;
            }
            ImGui::PopStyleColor();
        }
    }

    void CaseMultipleBox::UpdatePhysics(float dt) {
        for (int id { 0 }; id < _numBoxes; id++) {
            for (const auto & wall : _walls) {
                HandleCollisionWallBox(wall, _boxes[id]);
            }
            if (id < _numBoxes - 1) {
                for (int jd = id + 1; jd < _numBoxes; jd++) {
                    HandleCollisionBoxBox(_boxes[id], _boxes[jd]);
                }
            }
            // 常规物理更新
            _boxes[id].w += dt * glm::inverse(_boxes[id].I_world) * (-glm::cross(_boxes[id].w, _boxes[id].I_world * _boxes[id].w));
            _boxes[id].v += dt * _gravity * glm::vec3(0.0f, 0.0f, -1.0f);
            _boxes[id].position += dt * _boxes[id].v;
            _boxes[id].q       = glm::normalize(glm::exp(0.5f * dt * glm::quat(0.0f, _boxes[id].w)) * _boxes[id].q);
            glm::mat3 R        = glm::mat3_cast(_boxes[id].q);
            _boxes[id].I_world = R * _boxes[id].I * glm::transpose(R);
            float Ek           = 0.5f * _boxes[id].mass * glm::dot(_boxes[id].v, _boxes[id].v) + 0.5f * glm::dot(_boxes[id].w, _boxes[id].I_world * _boxes[id].w);
            _boxes[id].color   = glm::vec4(MapColor(Ek), _boxes[id].color[3]);
        }
    }

    Common::CaseRenderResult CaseMultipleBox::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        // apply mouse control first
        OnProcessMouseControl(_cameraManager.getMouseMove());

        if (! _isPaused) {
            UpdatePhysics(Engine::GetDeltaTime());
        }

        // rendering
        _frame.Resize(desiredSize);

        _cameraManager.Update(_camera);
        _program.GetUniforms().SetByName("u_Projection", _camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
        _program.GetUniforms().SetByName("u_View", _camera.GetViewMatrix());

        gl_using(_frame);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_LINE_SMOOTH);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glLineWidth(.5f);

        glDepthMask(GL_TRUE);
        for (auto & box : _boxes) {
            std::vector<glm::vec3> VertsPosition;
            glm::mat3              R = glm::mat3_cast(box.q);
            for (const auto & vertexPosition : box.VerticesPosition) {
                VertsPosition.push_back(glm::vec3(box.position + R * vertexPosition));
            }
            auto span_bytes = Engine::make_span_bytes<glm::vec3>(VertsPosition);
            box.TriangleItem.UpdateVertexBuffer("position", span_bytes);
            box.LineItem.UpdateVertexBuffer("position", span_bytes);
            _program.GetUniforms().SetByName("u_Color", box.color);
            box.TriangleItem.Draw({ _program.Use() });
            box.LineItem.Draw({ _program.Use() });
        }

        glDepthMask(GL_FALSE);
        for (const auto & wall : _walls) {
            _program.GetUniforms().SetByName("u_Color", wall.color);
            wall.TriangleItem.Draw({ _program.Use() });
            wall.LineItem.Draw({ _program.Use() });
        }
        glDepthMask(GL_TRUE);

        glLineWidth(1.f);
        glPointSize(1.f);
        glDisable(GL_LINE_SMOOTH);

        return Common::CaseRenderResult {
            .Fixed     = false,
            .Flipped   = true,
            .Image     = _frame.GetColorAttachment(),
            .ImageSize = desiredSize,
        };
    }

    void CaseMultipleBox::OnProcessInput(ImVec2 const & pos) {
        _cameraManager.ProcessInput(_camera, pos);
    }

    void CaseMultipleBox::OnProcessMouseControl(glm::vec3 mouseDelta) {
        float movingScale = 0.1f;
    }

    void CaseMultipleBox::HandleCollisionBoxBox(Box & box1, Box & box2) {                                               // 2撞向1
        if (glm::length(box1.position - box2.position) < 0.5f * glm::length(box1.dim) + 0.5f * glm::length(box2.dim)) { // 足够近才调用fcl包检测
            using CollisionGeometryPtr_t = std::shared_ptr<fcl::CollisionGeometry<float>>;
            CollisionGeometryPtr_t       geometry1(new fcl::Box<float>(box1.dim.x, box1.dim.y, box1.dim.z));
            CollisionGeometryPtr_t       geometry2(new fcl::Box<float>(box2.dim.x, box2.dim.y, box2.dim.z));
            fcl::Transform3f             transform1(Eigen::Translation3f(Eigen::Vector3f(box1.position.x, box1.position.y, box1.position.z)) * Eigen::Quaternionf(box1.q.w, box1.q.x, box1.q.y, box1.q.z));
            fcl::Transform3f             transform2(Eigen::Translation3f(Eigen::Vector3f(box2.position.x, box2.position.y, box2.position.z)) * Eigen::Quaternionf(box2.q.w, box2.q.x, box2.q.y, box2.q.z));
            fcl::CollisionObject<float>  obj1(geometry1, transform1);
            fcl::CollisionObject<float>  obj2(geometry2, transform2);
            fcl::CollisionRequest<float> collisionRequest(8, true);
            fcl::CollisionResult<float>  collisionResult;
            fcl::collide(&obj1, &obj2, collisionRequest, collisionResult);
            if (collisionResult.isCollision()) {
                std::vector<fcl::Contact<float>> contacts;
                collisionResult.getContacts(contacts);
                glm::vec3 collisionPosition { 0.0f, 0.0f, 0.0f };
                glm::vec3 collisionNormal(0.0f, 0.0f, 0.0f);
                float     collisionDepth { 0.0f };

                int counter { 0 };
                // 对所有碰撞点取平均
                for (const auto & contact : contacts) {
                    counter += 1;
                    collisionPosition += glm::vec3(contact.pos[0], contact.pos[1], contact.pos[2]);
                    collisionNormal += glm::vec3(contact.normal[0], contact.normal[1], contact.normal[2]);
                    collisionDepth += contact.penetration_depth;
                }
                static_cast<float>(counter);
                collisionPosition /= counter;
                collisionDepth /= counter;
                collisionNormal = glm::normalize(collisionNormal);
                // 先解决穿透问题
                box2.position += collisionDepth * collisionNormal;
                collisionPosition += collisionDepth * collisionNormal;
                glm::vec3 Rr1                     = collisionPosition - box1.position;
                glm::vec3 Rr2                     = collisionPosition - box2.position;
                glm::vec3 collisionPointVelocity1 = box1.v + glm::cross(box1.w, Rr1);
                glm::vec3 collisionPointVelocity2 = box2.v + glm::cross(box2.w, Rr2);
                glm::vec3 v_rel                   = collisionPointVelocity2 - collisionPointVelocity1;
                float     dotProduct              = glm::dot(collisionNormal, v_rel);
                if (dotProduct < 0) { // 有碰撞发生
                    // 处理碰撞
                    glm::vec3 v_rel_N     = dotProduct * collisionNormal;
                    glm::vec3 v_rel_T     = v_rel - v_rel_N;
                    glm::vec3 v_rel_N_new = -_muN * v_rel_N;
                    float     alpha       = fmax(1.0f - _muT * (1 + _muN) * glm::length(v_rel_N) / glm::length(v_rel_T), 0.0f);
                    glm::vec3 v_rel_T_new = alpha * v_rel_T;
                    glm::vec3 v_rel_new   = v_rel_N_new + v_rel_T_new;

                    glm::mat3 r1(
                        0.0f, -Rr1.z, Rr1.y, Rr1.z, 0.0f, -Rr1.x, -Rr1.y, Rr1.x, 0.0f);
                    glm::mat3 I_world1     = box1.I_world;
                    glm::mat3 I_world_inv1 = glm::inverse(I_world1);
                    glm::mat3 K1           = 1.0f / box1.mass * glm::mat3(1.0f) - r1 * I_world_inv1 * r1;

                    glm::mat3 r2(
                        0.0f, -Rr2.z, Rr2.y, Rr2.z, 0.0f, -Rr2.x, -Rr2.y, Rr2.x, 0.0f);
                    glm::mat3 I_world2     = box2.I_world;
                    glm::mat3 I_world_inv2 = glm::inverse(I_world2);
                    glm::mat3 K2           = 1.0f / box2.mass * glm::mat3(1.0f) - r2 * I_world_inv1 * r2;
                    glm::mat3 K            = K1 + K2;
                    glm::vec3 j            = glm::inverse(K) * (v_rel_new - v_rel);
                    box1.v += 1.0f / box1.mass * (-j);
                    box1.w += I_world_inv1 * glm::cross(Rr1, -j);
                    box2.v += 1.0f / box2.mass * (+j);
                    box2.w += I_world_inv2 * glm::cross(Rr2, +j);
                }
            }
        }
    }

    void CaseMultipleBox::HandleCollisionWallBox(const Wall & wall, Box & box) {
        glm::vec3 n = wall.normal;
        glm::mat3 R = glm::mat3_cast(box.q);
        int       counter { 0 };
        glm::vec3 collisionPosition { 0.0f, 0.0f, 0.0f };
        float     collisionDepth { 0.0f };
        for (const auto & vertexPosition : box.VerticesPosition) {
            glm::vec3 VertPosition       = box.position + R * vertexPosition;
            glm::vec3 VertVelocity       = box.v + glm::cross(box.w, R * vertexPosition);
            float     dotProductPosition = glm::dot(n, VertPosition - wall.position);
            if (dotProductPosition < 0) {
                counter += 1;
                collisionPosition += VertPosition;
                collisionDepth += -dotProductPosition;
            }
        }
        if (counter > 0) { // 有穿透发生，取平均等效为一个穿透点
            static_cast<float>(counter);
            collisionPosition /= counter;
            collisionDepth /= counter;
            // 修正穿透
            box.position += collisionDepth * n;
            collisionPosition += collisionDepth * n;
            glm::vec3 Rr                 = collisionPosition - box.position;
            glm::vec3 collisionVelocity  = box.v + glm::cross(box.w, Rr);
            float     dotProductVelocity = glm::dot(n, collisionVelocity);
            if (dotProductVelocity < 0) { // 有碰撞发生
                glm::mat3 I_world     = box.I_world;
                glm::mat3 I_world_inv = glm::inverse(I_world);
                float     Jn          = -(1.0f + _c) * glm::dot(n, collisionVelocity) / (1.0f / box.mass + glm::dot(n, glm::cross(I_world_inv * glm::cross(Rr, n), Rr)));
                glm::vec3 J           = Jn * n;
                box.v += J / box.mass;
                box.w += I_world_inv * glm::cross(Rr, J);
            }
        }
    }

    glm::vec3 CaseMultipleBox::MapColor(float E, float Emax) {
        float t = glm::clamp(E / Emax, 0.0f, 1.0f);
        return glm::vec3(t, 0.0f, 1 - t);
    }
} // namespace VCX::Labs::RigidBody