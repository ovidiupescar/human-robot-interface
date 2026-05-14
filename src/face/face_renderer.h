#pragma once

#include <Arduino_GFX_Library.h>
#include "face_states.h"

class FaceRenderer {
public:
    void init(Arduino_GFX* gfx, uint16_t w, uint16_t h);
    void setState(FaceState state, float amplitude = 0.0f);
    void setAmplitude(float amplitude);
    void tick(uint32_t delta_ms);

    // 0..1 pulse value matching face animation energy
    float getEnergy() const;

private:
    Arduino_GFX* _gfx = nullptr;
    uint16_t*    _fb   = nullptr;   // primary PSRAM framebuffer
    uint16_t*    _fb2  = nullptr;   // secondary buffer for transitions

    uint16_t _w  = 466;
    uint16_t _h  = 466;
    uint16_t _cx = 233;
    uint16_t _cy = 233;

    FaceState _current   = FaceState::STANDBY;
    FaceState _prev      = FaceState::STANDBY;
    float     _amplitude = 0.0f;
    float     _phase_ms  = 0.0f;

    // Transition
    float     _transition    = 1.0f;   // 0=prev, 1=current (done)
    float     _transitionSpd = 0.0f;   // per-ms increment

    // Speaking syllable simulation
    float     _syllable_target  = 0.5f;
    float     _syllable_current = 0.5f;
    uint32_t  _syllable_next_ms = 0;
    uint32_t  _syllable_seed    = 12345;

    // Framebuffer drawing primitives
    void _fbClear(uint16_t color);
    void _fbPixel(int x, int y, uint16_t color);
    void _fbFillCircle(int cx, int cy, int r, uint16_t color);
    void _fbHLine(int x, int y, int w, uint16_t color);
    void _fbLine(int x0, int y0, int x1, int y1, uint16_t color);
    void _fbFlush();

    void _drawState(FaceState state);
    void _drawTransition_StandbyToProcessing(float t);
    void _drawTransition_ProcessingToSpeaking(float t);
    void _drawTransition_SpeakingToAggressive(float t);
    void _drawTransition_WaveToStandby(float t, bool fromAggressive);
    void _drawStandby();
    void _drawStandbyRing(int r, int ring_w, uint8_t peak_g, uint8_t peak_b);
    void _drawProcessing();
    void _drawSpeaking();
    void _drawAggressive();

    void  _drawBrows(bool arcsUp);
    void  _drawBrowsWithAlpha(bool arcsUp, float alpha);
    float _breathe(float period_ms, float min_val, float max_val);
};
