#include "Labs/1-RigidBody/CaseTwoBox.h"
#include "Engine/app.h"
#include "Labs/Common/ImGuiHelper.h"
#include <fcl/narrowphase/collision.h>

namespace VCX::Labs::RigidBody {

    CaseTwoBox::CaseTwoBox():
        _program(
            Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/flat.vert"), Engine::GL::SharedShader("assets/shaders/flat.frag") })),
        box1(1, 1.0f, glm::vec3(1.0f, 2.0f, 3.0f), glm::vec3(-5.0f, 0.0f, 0.0f)),
        box2(2, 1.0f, glm::vec3(1.0f, 2.0f, 3.0f), glm::vec3(5.0f, 0.0f, 0.0f)) {
        _cameraManager.AutoRotate = false;
        _cameraManager.Save(_camera);
    }

    void CaseTwoBox::OnSetupPropsUI() {
        if (ImGui::CollapsingHeader("Box1", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Text("Position##Box1: (%.2f, %.2f, %.2f)", box1.position.x, box1.position.y, box1.position.z);
            ImGui::Text("Orientation##Box1: (%.2f, %.2f, %.2f, %.2f)", box1.q.w, box1.q.x, box1.q.y, box1.q.z);
            ImGui::SliderFloat3("Velocity##Box1", glm::value_ptr(box1.v), -5.0f, 5.0f);
            ImGui::SliderFloat3("Angular Velocity##Box1", glm::value_ptr(box1.w), -5.0f, 5.0f);
        }
        if (ImGui::CollapsingHeader("Box2", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Text("Position##Box2: (%.2f, %.2f, %.2f)", box2.position.x, box2.position.y, box2.position.z);
            ImGui::Text("Orientation##Box2: (%.2f, %.2f, %.2f, %.2f)", box2.q.w, box2.q.x, box2.q.y, box2.q.z);
            ImGui::SliderFloat3("Velocity##Box2", glm::value_ptr(box2.v), -5.0f, 5.0f);
            ImGui::SliderFloat3("Angular Velocity##Box2", glm::value_ptr(box2.w), -5.0f, 5.0f);
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

    void CaseTwoBox::HandleCollision(Box & box1, Box & box2) { // 2撞向1
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
            collisionNormal                   = glm::normalize(collisionNormal);
            glm::vec3 Rr1                     = collisionPosition - box1.position;
            glm::vec3 Rr2                     = collisionPosition - box2.position;
            glm::vec3 collisionPointVelocity1 = box1.v + glm::cross(box1.w, Rr1);
            glm::vec3 collisionPointVelocity2 = box2.v + glm::cross(box2.w, Rr2);
            glm::vec3 v_rel                   = collisionPointVelocity2 - collisionPointVelocity1;
            float     dotProduct              = glm::dot(collisionNormal, v_rel);
            if (dotProduct < 0) { // 等效碰撞点在互相靠近
                // 处理碰撞
                glm::vec3 v_rel_N     = dotProduct * collisionNormal;
                glm::vec3 v_rel_T     = v_rel - v_rel_N;
                glm::vec3 v_rel_N_new = -_muN * v_rel_N;
                float     alpha       = fmax(1.0f - _muT * (1 + _muN) * glm::length(v_rel_N) / glm::length(v_rel_T), 0.0f);
                glm::vec3 v_rel_T_new = alpha * v_rel_T;
                glm::vec3 v_rel_new   = v_rel_N_new + v_rel_T_new;

                glm::mat3 r1(
                    0.0f, -Rr1.z, Rr1.y, Rr1.z, 0.0f, -Rr1.x, -Rr1.y, Rr1.x, 0.0f);
                glm::mat3 R1           = glm::mat3_cast(box1.q);
                glm::mat3 I_world1     = R1 * box1.I * glm::transpose(R1);
                glm::mat3 I_world_inv1 = glm::inverse(I_world1);
                glm::mat3 K1           = 1.0f / box1.mass * glm::mat3(1.0f) - r1 * I_world_inv1 * r1;

                glm::mat3 r2(
                    0.0f, -Rr2.z, Rr2.y, Rr2.z, 0.0f, -Rr2.x, -Rr2.y, Rr2.x, 0.0f);
                glm::mat3 R2           = glm::mat3_cast(box2.q);
                glm::mat3 I_world2     = R2 * box2.I * glm::transpose(R2);
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

    void CaseTwoBox::UpdatePhysics(float dt) {
        HandleCollision(box1, box2);
        glm::mat3 R1           = glm::mat3_cast(box1.q);
        glm::mat3 I_world1     = R1 * box1.I * glm::transpose(R1);
        glm::mat3 I_world_inv1 = glm::inverse(I_world1);
        box1.w += dt * I_world_inv1 * (-glm::cross(box1.w, I_world1 * box1.w));
        box1.position += dt * box1.v;
        box1.q                 = glm::normalize(glm::exp(0.5f * dt * glm::quat(0.0f, box1.w)) * box1.q);
        glm::mat3 R2           = glm::mat3_cast(box2.q);
        glm::mat3 I_world2     = R2 * box2.I * glm::transpose(R2);
        glm::mat3 I_world_inv2 = glm::inverse(I_world2);
        box2.w += dt * I_world_inv2 * (-glm::cross(box2.w, I_world2 * box2.w));
        box2.position += dt * box2.v;
        box2.q = glm::normalize(glm::exp(0.5f * dt * glm::quat(0.0f, box2.w)) * box2.q);
    }

    Common::CaseRenderResult CaseTwoBox::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
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
        glLineWidth(.5f);

        std::vector<glm::vec3> VertsPosition1;
        glm::mat3              R1 = glm::mat3_cast(box1.q);
        for (const auto & vertexPosition : box1.VerticesPosition) {
            VertsPosition1.push_back(box1.position + R1 * vertexPosition);
        }
        auto span_bytes1 = Engine::make_span_bytes<glm::vec3>(VertsPosition1);

        _program.GetUniforms().SetByName("u_Color", box1.color);
        box1.TriangleItem.UpdateVertexBuffer("position", span_bytes1);
        box1.TriangleItem.Draw({ _program.Use() });
        _program.GetUniforms().SetByName("u_Color", glm::vec4(1.f, 1.f, 1.f, 1.f));
        box1.LineItem.UpdateVertexBuffer("position", span_bytes1);
        box1.LineItem.Draw({ _program.Use() });

        std::vector<glm::vec3> VertsPosition2;
        glm::mat3              R2 = glm::mat3_cast(box2.q);
        for (const auto & vertexPosition : box2.VerticesPosition) {
            VertsPosition2.push_back(box2.position + R2 * vertexPosition);
        }
        auto span_bytes2 = Engine::make_span_bytes<glm::vec3>(VertsPosition2);

        _program.GetUniforms().SetByName("u_Color", box2.color);
        box2.TriangleItem.UpdateVertexBuffer("position", span_bytes2);
        box2.TriangleItem.Draw({ _program.Use() });
        _program.GetUniforms().SetByName("u_Color", glm::vec4(1.f, 1.f, 1.f, 1.f));
        box2.LineItem.UpdateVertexBuffer("position", span_bytes2);
        box2.LineItem.Draw({ _program.Use() });

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

    void CaseTwoBox::OnProcessInput(ImVec2 const & pos) {
        _cameraManager.ProcessInput(_camera, pos);
    }

    void CaseTwoBox::OnProcessMouseControl(glm::vec3 mouseDelta) {
        float movingScale = 0.1f;
    }

} // namespace VCX::Labs::RigidBody
