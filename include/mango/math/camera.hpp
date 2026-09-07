/*
    MANGO Multimedia Development Platform
    Copyright (C) 2012-2026 Twilight Finland 3D Oy Ltd. All rights reserved.
*/
#pragma once

#include <algorithm>
#include <cmath>

#include <mango/math/matrix.hpp>

namespace mango::math
{

    // Per-frame camera controls. Device-agnostic: fill from keyboard, gamepad, etc.
    // NOTE: use (0.0f) / explicit components for float32x3 — bare `{}` does not
    // reliably zero simd vectors and will inject NaNs into the integrator.
    struct CameraInput
    {
        float32x3 move { 0.0f, 0.0f, 0.0f };    // local X/Y/Z, typically in [-1, 1]
        float32x3 rotate { 0.0f, 0.0f, 0.0f };  // pitch / yaw / roll, typically in [-1, 1]
        float speed = 1.0f;   // relative scale (e.g. 5 for boost, 0.2 for precision)
    };

    // Tuning for the damped integrator. Defaults match ~60 Hz legacy feel.
    struct CameraControllerConfig
    {
        float damping = 80.0f * 0.04082199598598729f;
        float moveAccel = 0.16f * 60.0f;
        float rotateAccel = 0.025f * 60.0f;
    };

    // Damped rigid-body integrator: local impulses → velocity → transform.
    // Columns: right, up, forward, origin (+Z ahead).
    struct CameraController
    {
        CameraControllerConfig config;
        Matrix4x4 transform { 1.0f };
        float32x3 velocity { 0.0f, 0.0f, 0.0f };
        float32x3 rollrate { 0.0f, 0.0f, 0.0f };
        float moveScale = 1.0f;

        explicit CameraController(const CameraControllerConfig& config = {})
            : config(config)
        {
        }

        const Matrix4x4& getTransform() const
        {
            return transform;
        }

        void update(CameraInput input, float dt)
        {
            // Spiral-of-death guard: a hitch must not fling the camera.
            dt = std::min(dt, 0.1f);
            if (dt <= 0.0f)
            {
                return;
            }

            const float speed = moveScale * input.speed;

            velocity.x += input.move.x * speed * config.moveAccel * dt;
            velocity.y += input.move.y * speed * config.moveAccel * dt;
            velocity.z += input.move.z * speed * config.moveAccel * dt;

            rollrate.x += input.rotate.x * config.rotateAccel * dt;
            rollrate.y += input.rotate.y * config.rotateAccel * dt;
            rollrate.z += input.rotate.z * config.rotateAccel * dt;

            const Matrix4x4 rotate = Matrix4x4::rotateXYZ(rollrate.x * dt, rollrate.y * dt, rollrate.z * dt);
            const Matrix4x4 translate = Matrix4x4::translate(velocity.x * dt, velocity.y * dt, velocity.z * dt);
            transform = rotate * translate * transform;

            const float decay = std::exp(-config.damping * dt);
            velocity *= decay;
            rollrate *= decay;
        }
    };

} // namespace mango::math
