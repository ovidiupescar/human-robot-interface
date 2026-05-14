#pragma once

enum class FaceState {
    STANDBY    = 0,  // Baseline Pulse — concentric cyan rings breathing
    PROCESSING = 1,  // Kinetic Activation — sawtooth waveform
    SPEAKING   = 2,  // Warm Wave — sine layers orange/magenta, brow arcs up
    AGGRESSIVE = 3,  // Holistic Contradiction — red spikes, brow arcs down
};

struct FaceParams {
    FaceState state = FaceState::STANDBY;
    float     amplitude = 0.0f;  // 0.0–1.0, speech loudness for SPEAKING
    float     transition = 1.0f; // 0.0–1.0, interpolation progress (1=done)
};
