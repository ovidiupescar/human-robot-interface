#include "face_renderer.h"
#include <math.h>
#include <string.h>

static inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint16_t)(r & 0xF8) << 8) |
           ((uint16_t)(g & 0xFC) << 3) |
           ((uint16_t)(b) >> 3);
}

#define CLR_BLACK   0x0000
#define CLR_WHITE   0xFFFF

// ---- Framebuffer primitives ----

void FaceRenderer::_fbClear(uint16_t color) {
    if (color == 0) {
        memset(_fb, 0, _w * _h * sizeof(uint16_t));
    } else {
        for (int i = 0; i < _w * _h; i++) _fb[i] = color;
    }
}

void FaceRenderer::_fbPixel(int x, int y, uint16_t color) {
    if (x >= 0 && x < _w && y >= 0 && y < _h) {
        _fb[y * _w + x] = color;
    }
}

void FaceRenderer::_fbHLine(int x, int y, int w, uint16_t color) {
    if (y < 0 || y >= _h) return;
    if (x < 0) { w += x; x = 0; }
    if (x + w > _w) w = _w - x;
    if (w <= 0) return;
    uint16_t* row = &_fb[y * _w + x];
    for (int i = 0; i < w; i++) row[i] = color;
}

void FaceRenderer::_fbFillCircle(int cx, int cy, int r, uint16_t color) {
    if (r <= 0) return;
    for (int dy = -r; dy <= r; dy++) {
        int dx = (int)(sqrtf((float)(r * r - dy * dy)) + 0.5f);
        _fbHLine(cx - dx, cy + dy, dx * 2 + 1, color);
    }
}

void FaceRenderer::_fbLine(int x0, int y0, int x1, int y1, uint16_t color) {
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    while (true) {
        _fbPixel(x0, y0, color);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

void FaceRenderer::_fbFlush() {
    _gfx->draw16bitRGBBitmap(0, 0, _fb, _w, _h);
}

// ---- Public API ----

void FaceRenderer::init(Arduino_GFX* gfx, uint16_t w, uint16_t h) {
    _gfx = gfx;
    _w = w;
    _h = h;
    _cx = w / 2;
    _cy = h / 2;

    // Allocate two framebuffers in PSRAM for crossfade transitions
    size_t fb_size = w * h * sizeof(uint16_t);
    _fb = (uint16_t*)ps_malloc(fb_size);
    _fb2 = (uint16_t*)ps_malloc(fb_size);
    if (_fb && _fb2) {
        Serial.printf("[face] PSRAM framebuffers OK (2x %dKB)\n", fb_size / 1024);
    } else {
        Serial.println("[face] PSRAM alloc FAILED!");
    }
}

void FaceRenderer::setState(FaceState state, float amplitude) {
    if (state != _current) {
        _prev = _current;
        _current = state;
        _transition = 0.0f;
        _transitionSpd = 1.0f / 800.0f;  // 800ms transition
        Serial.printf("[face] transition %d -> %d\n", (int)_prev, (int)_current);
    }
    _amplitude = constrain(amplitude, 0.0f, 1.0f);
}

void FaceRenderer::setAmplitude(float amplitude) {
    _amplitude = constrain(amplitude, 0.0f, 1.0f);
}

void FaceRenderer::_drawState(FaceState state) {
    switch (state) {
        case FaceState::STANDBY:    _drawStandby();    break;
        case FaceState::PROCESSING: _drawProcessing(); break;
        case FaceState::SPEAKING:   _drawSpeaking();   break;
        case FaceState::AGGRESSIVE: _drawAggressive(); break;
    }
}

void FaceRenderer::tick(uint32_t delta_ms) {
    if (!_fb || !_fb2) return;

    _phase_ms += (float)delta_ms;

    // Advance transition
    if (_transition < 1.0f) {
        _transition += _transitionSpd * (float)delta_ms;
        if (_transition >= 1.0f) {
            _transition = 1.0f;
            Serial.printf("[face] transition done -> %d\n", (int)_current);
        }
    }

    if (_transition >= 1.0f) {
        // No transition — draw current state directly
        _fbClear(CLR_BLACK);
        _drawState(_current);
        _fbFlush();
    } else {
        float t = _transition;
        t = t * t * (3.0f - 2.0f * t);  // smoothstep

        _fbClear(CLR_BLACK);

        // Morphing transitions
        if (_prev == FaceState::STANDBY && _current == FaceState::PROCESSING) {
            _drawTransition_StandbyToProcessing(t);
        } else if (_prev == FaceState::PROCESSING && _current == FaceState::STANDBY) {
            _drawTransition_StandbyToProcessing(1.0f - t);
        } else if (_prev == FaceState::PROCESSING && _current == FaceState::SPEAKING) {
            _drawTransition_ProcessingToSpeaking(t);
        } else if (_prev == FaceState::SPEAKING && _current == FaceState::PROCESSING) {
            _drawTransition_ProcessingToSpeaking(1.0f - t);
        } else if (_prev == FaceState::SPEAKING && _current == FaceState::AGGRESSIVE) {
            _drawTransition_SpeakingToAggressive(t);
        } else if (_prev == FaceState::AGGRESSIVE && _current == FaceState::SPEAKING) {
            _drawTransition_SpeakingToAggressive(1.0f - t);
        } else if (_prev == FaceState::SPEAKING && _current == FaceState::STANDBY) {
            _drawTransition_WaveToStandby(t, false);
        } else if (_prev == FaceState::AGGRESSIVE && _current == FaceState::STANDBY) {
            _drawTransition_WaveToStandby(t, true);
        } else {
            _drawState(_current);
        }

        _fbFlush();
    }
}

float FaceRenderer::getEnergy() const {
    switch (_current) {
        case FaceState::STANDBY: {
            // Slow breath matching ring animation (4000ms period)
            float s = sinf(_phase_ms / 4000.0f * 2.0f * (float)M_PI);
            return 0.5f + 0.5f * s;  // 0..1
        }
        case FaceState::PROCESSING:
        case FaceState::SPEAKING:
        case FaceState::AGGRESSIVE:
            return _syllable_current;  // already 0..1
    }
    return 0.5f;
}

float FaceRenderer::_breathe(float period_ms, float min_val, float max_val) {
    float t = sinf(_phase_ms / period_ms * 2.0f * (float)M_PI);
    return min_val + (max_val - min_val) * (t * 0.5f + 0.5f);
}

void FaceRenderer::_drawBrows(bool arcsUp) {
    int left_cx  = _cx - _cx * 2 / 5;
    int right_cx = _cx + _cx * 2 / 5;
    int brow_y   = _cy - _cy / 2;
    int brow_w   = _cx / 4;
    int arc_h    = brow_w / 2;

    int dir = arcsUp ? -1 : 1;

    for (int side = 0; side < 2; side++) {
        int cx = (side == 0) ? left_cx : right_cx;
        int inner_dir = (side == 0) ? 1 : -1;  // which side slopes down

        for (int thickness = 0; thickness < 5; thickness++) {
            for (int px = -brow_w; px <= brow_w; px++) {
                float t = (float)px / (float)brow_w;  // -1..1

                float curve;
                if (arcsUp) {
                    curve = cosf(t * (float)M_PI * 0.5f);  // smooth arc
                } else {
                    // Sharp V: slopes down toward nose (inner edge lower)
                    curve = (float)px * inner_dir / (float)brow_w;  // linear slope
                }

                int py = brow_y + dir * (int)(curve * arc_h) + thickness;

                if (arcsUp) {
                    // Gradient: orange at center -> magenta at edges
                    float gt = fabsf(t);  // 0 at center, 1 at edges
                    uint8_t r = (uint8_t)(255.0f - 25.0f * gt);
                    uint8_t g = (uint8_t)(180.0f - 150.0f * gt);
                    uint8_t b = (uint8_t)(30.0f + 150.0f * gt);
                    _fbPixel(cx + px, py, rgb565(r, g, b));
                } else {
                    // Angry brows: amber/orange, sharp V angle
                    float gt = fabsf(t);
                    uint8_t r = (uint8_t)(255.0f - 30.0f * gt);
                    uint8_t g = (uint8_t)(160.0f - 80.0f * gt);
                    uint8_t b = (uint8_t)(20.0f);
                    _fbPixel(cx + px, py, rgb565(r, g, b));
                }
            }
        }
    }
}

void FaceRenderer::_drawBrowsWithAlpha(bool arcsUp, float alpha) {
    int left_cx  = _cx - _cx * 2 / 5;
    int right_cx = _cx + _cx * 2 / 5;
    int brow_y   = _cy - _cy / 2;
    int brow_w   = _cx / 4;
    int arc_h    = brow_w / 2;
    int dir = arcsUp ? -1 : 1;

    for (int side = 0; side < 2; side++) {
        int cx = (side == 0) ? left_cx : right_cx;
        int inner_dir = (side == 0) ? 1 : -1;
        for (int thickness = 0; thickness < 5; thickness++) {
            for (int px = -brow_w; px <= brow_w; px++) {
                float t = (float)px / (float)brow_w;
                float curve;
                if (arcsUp) {
                    curve = cosf(t * (float)M_PI * 0.5f);
                } else {
                    curve = (float)px * inner_dir / (float)brow_w;
                }
                int py = brow_y + dir * (int)(curve * arc_h) + thickness;
                float gt = fabsf(t);
                uint8_t r, g, b;
                if (arcsUp) {
                    r = (uint8_t)((255.0f - 25.0f * gt) * alpha);
                    g = (uint8_t)((180.0f - 150.0f * gt) * alpha);
                    b = (uint8_t)((30.0f + 150.0f * gt) * alpha);
                } else {
                    r = (uint8_t)((255.0f - 30.0f * gt) * alpha);
                    g = (uint8_t)((160.0f - 80.0f * gt) * alpha);
                    b = (uint8_t)(20.0f * alpha);
                }
                _fbPixel(cx + px, py, rgb565(r, g, b));
            }
        }
    }
}

// ---- State renderers ----

void FaceRenderer::_drawStandbyRing(int r, int ring_w, uint8_t peak_g, uint8_t peak_b) {
    if (r <= 0) return;

    // Glitch: pseudo-random timing using sine-based hash
    // Two overlapping cycles (3.7s and 5.3s) create irregular pattern
    float g1 = sinf(_phase_ms / 3700.0f * 2.0f * (float)M_PI);
    float g2 = sinf(_phase_ms / 5300.0f * 2.0f * (float)M_PI);
    float glitch_trigger = g1 * g2;  // only high when both align
    bool glitching = glitch_trigger > 0.85f;
    float glitch_strength = 0.0f;
    float glitch_phase = 0.0f;
    float glitch_band_center = 0.0f;
    float glitch_band_width = 0.0f;
    if (glitching) {
        float intensity = (glitch_trigger - 0.85f) / 0.15f;  // 0..1
        glitch_strength = intensity * 8.0f;
        glitch_phase = _phase_ms / 25.0f;
        // Affect only a band of the circle (not all scanlines)
        glitch_band_center = sinf(_phase_ms / 1700.0f) * 0.6f;  // -0.6..0.6 of radius
        glitch_band_width = 0.25f + intensity * 0.2f;  // 25-45% of height
    }

    uint16_t color = rgb565(0, peak_g, peak_b);

    if (!glitching) {
        // Normal clean circle
        _fbFillCircle(_cx, _cy, r, color);
        int inner = r - ring_w;
        if (inner > 0) _fbFillCircle(_cx, _cy, inner, CLR_BLACK);
    } else {
        // Distorted: draw per-scanline with horizontal offset
        int r_inner = r - ring_w;
        if (r_inner < 0) r_inner = 0;

        for (int dy = -r; dy <= r; dy++) {
            int py = _cy + dy;
            if (py < 0 || py >= _h) continue;

            int dx_outer = (int)(sqrtf(fmaxf(0, (float)(r * r - dy * dy))));
            int dx_inner = 0;
            if (r_inner > 0 && abs(dy) < r_inner) {
                dx_inner = (int)(sqrtf(fmaxf(0, (float)(r_inner * r_inner - dy * dy))));
            }

            // Only distort scanlines within the glitch band
            float norm_dy = (float)dy / (float)r;  // -1..1
            float band_dist = fabsf(norm_dy - glitch_band_center) / glitch_band_width;
            float local_strength = (band_dist < 1.0f) ? (1.0f - band_dist) * glitch_strength : 0.0f;
            int offset = (int)(sinf((float)dy * 0.2f + glitch_phase) * local_strength);

            // Left arc
            for (int px = _cx - dx_outer + offset; px < _cx - dx_inner + offset; px++) {
                _fbPixel(px, py, color);
            }
            // Right arc
            for (int px = _cx + dx_inner + 1 + offset; px <= _cx + dx_outer + offset; px++) {
                _fbPixel(px, py, color);
            }
        }
    }
}

void FaceRenderer::_drawStandby() {
    float breathe = _breathe(4000.0f, 0.75f, 1.0f);

    // Fade trails: draw dimmer copies at slightly offset radii
    // Trail behind = where the ring was a moment ago
    float trail1 = _breathe(4000.0f - 300.0f, 0.75f, 1.0f);  // slightly behind in phase
    float trail2 = _breathe(4000.0f - 600.0f, 0.75f, 1.0f);
    float trail3 = _breathe(4000.0f - 900.0f, 0.75f, 1.0f);

    // Outer ring trails (dim to dimmer)
    _drawStandbyRing((int)(185 * trail3), 25, 40, 45);
    _drawStandbyRing((int)(185 * trail2), 27, 70, 80);
    _drawStandbyRing((int)(185 * trail1), 28, 110, 120);
    // Outer ring main
    _drawStandbyRing((int)(185 * breathe), 30, 200, 210);

    // Inner ring trails
    _drawStandbyRing((int)(80 * trail3), 20, 45, 50);
    _drawStandbyRing((int)(80 * trail2), 22, 80, 90);
    _drawStandbyRing((int)(80 * trail1), 23, 120, 130);
    // Inner ring main
    _drawStandbyRing((int)(80 * breathe), 25, 210, 230);
}

void FaceRenderer::_drawProcessing() {
    int wave_cy = _cy;

    // Gentle syllable-like pulsing — slower than speaking
    if (_phase_ms >= _syllable_next_ms) {
        _syllable_seed = _syllable_seed * 1103515245 + 12345;
        float rnd = (float)((_syllable_seed >> 16) & 0x7FFF) / 32767.0f;
        _syllable_target = 0.3f + rnd * 0.7f;
        uint32_t interval = 150 + ((_syllable_seed >> 8) & 0xFF) % 200;
        _syllable_next_ms = (uint32_t)_phase_ms + interval;
    }
    _syllable_current += (_syllable_target - _syllable_current) * 0.2f;

    float t = _phase_ms / 90.0f;
    int max_spike_h = (int)(_cy * 0.85f);

    for (int x = 0; x < _w; x++) {
        float xn = (float)x / _w;

        // Harmonics — medium complexity, between standby calm and speaking energy
        float wave = sinf(xn * 8.0f * (float)M_PI + t)
                   + 0.7f * sinf(xn * 14.0f * (float)M_PI + t * 1.8f)
                   + 0.5f * sinf(xn * 21.0f * (float)M_PI + t * 3.0f)
                   + 0.3f * sinf(xn * 30.0f * (float)M_PI + t * 4.5f)
                   + 0.2f * sinf(xn * 40.0f * (float)M_PI + t * 6.2f);

        // Square the wave before abs — makes peaks sharper and valleys deeper
        float raw = wave / 2.5f;
        float height = fabsf(raw);
        // Boost peaks, keep valleys low — more contrast
        height = height * height * 2.0f;
        height = fminf(height, 1.0f);
        height *= _syllable_current;
        height = 0.06f + height * 0.94f;

        int spike_h = (int)(height * max_spike_h);
        if (spike_h < 2) spike_h = 2;

        // Mirrored vertical bars — cyan/blue palette
        for (int dy = -spike_h; dy <= spike_h; dy++) {
            int py = wave_cy + dy;
            if (py < 0 || py >= _h) continue;

            float dist = (float)abs(dy) / (float)spike_h;

            uint8_t r, g, b;
            if (dist < 0.25f) {
                // White-cyan core
                float core = 1.0f - dist / 0.25f;
                r = (uint8_t)(180 * core);
                g = (uint8_t)(255 * (0.5f + 0.5f * core));
                b = (uint8_t)(255);
            } else {
                // Blue-cyan fade to dark
                float fade = 1.0f - (dist - 0.25f) / 0.75f;
                fade = fade * fade;
                // Gradient: cyan near center -> blue at edges
                float color_mix = dist;
                r = (uint8_t)(0);
                g = (uint8_t)((220 - 150 * color_mix) * fade);
                b = (uint8_t)((255 - 50 * color_mix) * fade);
            }

            _fbPixel(x, py, rgb565(r, g, b));
        }
    }
}

void FaceRenderer::_drawSpeaking() {
    int band_half = 50 + (int)(_amplitude * 15.0f);
    int wave_cy = _cy + 30;

    // Syllable rhythm: change amplitude target every 100-250ms
    if (_phase_ms >= _syllable_next_ms) {
        _syllable_seed = _syllable_seed * 1103515245 + 12345;  // LCG
        float rnd = (float)((_syllable_seed >> 16) & 0x7FFF) / 32767.0f;
        _syllable_target = 0.3f + rnd * 0.7f;  // 0.3..1.0
        uint32_t interval = 100 + ((_syllable_seed >> 8) & 0xFF) % 150;  // 100-250ms
        _syllable_next_ms = (uint32_t)_phase_ms + interval;
    }
    _syllable_current += (_syllable_target - _syllable_current) * 0.25f;

    float t = _phase_ms / 120.0f;
    float syllable = 0.25f + _syllable_current * 0.55f;

    for (int x = 0; x < _w; x++) {
        float xn = (float)x / _w;  // 0..1

        // Gaussian envelope: tallest at center, tapers at edges
        float env = expf(-((xn - 0.5f) * (xn - 0.5f)) / (2.0f * 0.09f));

        // Standing harmonics — vibrate in place, no horizontal travel
        float wave = sinf(xn * 4.0f * (float)M_PI) * sinf(t)
                   + 0.6f * sinf(xn * 7.0f * (float)M_PI) * sinf(t * 1.7f + 0.8f)
                   + 0.4f * sinf(xn * 11.0f * (float)M_PI) * sinf(t * 3.1f + 1.5f)
                   + 0.25f * sinf(xn * 16.0f * (float)M_PI) * sinf(t * 5.3f + 0.3f);

        wave *= env * syllable;

        int amp_px = (int)(_cy * 0.35f);  // contained
        int yc = wave_cy + (int)(wave * amp_px);

        for (int dy = -band_half; dy <= band_half; dy++) {
            int py = yc + dy;
            if (py < 0 || py >= _h) continue;

            // Smooth edge fade
            float edge = 1.0f - (float)abs(dy) / (float)band_half;
            if (edge <= 0.0f) continue;
            edge = sqrtf(edge);  // soft fade, not harsh

            // Global vertical position determines color (not relative to wave)
            // Top of screen = warm orange, bottom = magenta/pink
            float screen_vt = (float)py / (float)_h;  // 0=top, 1=bottom

            // Orange (255,180,30) at top -> Magenta (230,30,180) at bottom
            float r = (255.0f + (230.0f - 255.0f) * screen_vt) * edge;
            float g = (180.0f + (30.0f - 180.0f) * screen_vt) * edge;
            float b = (30.0f + (180.0f - 30.0f) * screen_vt) * edge;

            _fbPixel(x, py, rgb565((uint8_t)r, (uint8_t)g, (uint8_t)b));
        }
    }

    _drawBrows(true);
}

void FaceRenderer::_drawAggressive() {
    int wave_cy = _cy + 25;

    // Syllable pulsing
    if (_phase_ms >= _syllable_next_ms) {
        _syllable_seed = _syllable_seed * 1103515245 + 12345;
        float rnd = (float)((_syllable_seed >> 16) & 0x7FFF) / 32767.0f;
        _syllable_target = 0.4f + rnd * 0.6f;
        uint32_t interval = 60 + ((_syllable_seed >> 8) & 0xFF) % 100;
        _syllable_next_ms = (uint32_t)_phase_ms + interval;
    }
    _syllable_current += (_syllable_target - _syllable_current) * 0.35f;

    float t = _phase_ms / 80.0f;
    int max_spike_h = (int)(_cy * 0.85f);  // spikes almost full screen height

    // Per-column: compute spike height, draw mirrored vertical bar
    for (int x = 0; x < _w; x++) {
        float xn = (float)x / _w;

        // Jagged harmonics — high frequency, sharp peaks
        float wave = sinf(xn * 10.0f * (float)M_PI + t)
                   + 0.9f * sinf(xn * 17.0f * (float)M_PI + t * 1.9f)
                   + 0.7f * sinf(xn * 23.0f * (float)M_PI + t * 3.1f)
                   + 0.5f * sinf(xn * 31.0f * (float)M_PI + t * 4.7f)
                   + 0.4f * sinf(xn * 41.0f * (float)M_PI + t * 6.3f)
                   + 0.3f * sinf(xn * 53.0f * (float)M_PI + t * 8.1f);

        // Absolute value — mirrored spikes
        float height = fabsf(wave) / 2.5f;
        height *= _syllable_current;
        // Ensure minimum visible base
        height = 0.08f + height * 0.92f;

        int spike_h = (int)(height * max_spike_h);
        if (spike_h < 2) spike_h = 2;  // minimum bar

        // Draw mirrored: above and below center line
        for (int dy = -spike_h; dy <= spike_h; dy++) {
            int py = wave_cy + dy;
            if (py < 0 || py >= _h) continue;

            // Distance from center determines color: white core -> red edges
            float dist = (float)abs(dy) / (float)spike_h;

            uint8_t r, g, b;
            if (dist < 0.3f) {
                // White-hot core
                float core = 1.0f - dist / 0.3f;
                r = 255;
                g = (uint8_t)(200 * core + 40 * (1.0f - core));
                b = (uint8_t)(180 * core);
            } else {
                // Red fade to dark
                float fade = 1.0f - (dist - 0.3f) / 0.7f;
                fade = fade * fade;  // sharper falloff
                r = (uint8_t)(255 * fade);
                g = (uint8_t)(20 * fade);
                b = (uint8_t)(10 * fade);
            }

            _fbPixel(x, py, rgb565(r, g, b));
        }
    }

    _drawBrows(false);
}

void FaceRenderer::_drawTransition_StandbyToProcessing(float t) {
    // t=0: standby rings, t=1: processing waveform
    // Phase 1 (t 0->0.5): rings collapse vertically, squash into horizontal band
    // Phase 2 (t 0.5->1): flat line grows waveform spikes

    float wave_cy = _cy;

    if (t < 0.5f) {
        // Phase 1: rings squash vertically
        float squash = t / 0.5f;  // 0->1
        squash = squash * squash;  // ease in

        float breathe = _breathe(4000.0f, 0.75f, 1.0f);

        // Draw rings but compressed vertically
        // As squash increases, circles become ellipses then flat lines
        float v_scale = 1.0f - squash * 0.97f;  // 1.0 -> 0.03

        int base_radii[] = {185, 80};
        int ring_ws[] = {30, 25};
        uint8_t gs[] = {200, 210};
        uint8_t bs[] = {210, 230};

        for (int ring = 0; ring < 2; ring++) {
            int r = (int)(base_radii[ring] * breathe);
            int rw = ring_ws[ring];
            uint16_t color = rgb565(0, gs[ring], bs[ring]);

            // Draw squashed ring as ellipse using per-scanline fill
            int r_inner = r - rw;
            if (r_inner < 0) r_inner = 0;

            int max_dy = (int)(r * v_scale);
            if (max_dy < 1) max_dy = 1;

            for (int dy = -max_dy; dy <= max_dy; dy++) {
                int py = (int)wave_cy + dy;
                if (py < 0 || py >= _h) continue;

                // Map dy back to circle space
                float circle_dy = (float)dy / v_scale;
                if (fabsf(circle_dy) > r) continue;

                int dx_outer = (int)(sqrtf(fmaxf(0, (float)(r * r) - circle_dy * circle_dy)));
                int dx_inner = 0;
                if (r_inner > 0 && fabsf(circle_dy) < r_inner) {
                    dx_inner = (int)(sqrtf(fmaxf(0, (float)(r_inner * r_inner) - circle_dy * circle_dy)));
                }

                // Left arc
                _fbHLine(_cx - dx_outer, py, dx_outer - dx_inner, color);
                // Right arc
                _fbHLine(_cx + dx_inner + 1, py, dx_outer - dx_inner, color);
            }
        }
    } else {
        // Phase 2: waveform grows from flat
        float grow = (t - 0.5f) / 0.5f;  // 0->1
        grow = grow * grow * (3.0f - 2.0f * grow);  // smoothstep

        float time = _phase_ms / 90.0f;
        int max_spike_h = (int)(_cy * 0.85f * grow);  // grows from 0

        // Start with syllable engine
        if (_phase_ms >= _syllable_next_ms) {
            _syllable_seed = _syllable_seed * 1103515245 + 12345;
            float rnd = (float)((_syllable_seed >> 16) & 0x7FFF) / 32767.0f;
            _syllable_target = 0.3f + rnd * 0.7f;
            uint32_t interval = 150 + ((_syllable_seed >> 8) & 0xFF) % 200;
            _syllable_next_ms = (uint32_t)_phase_ms + interval;
        }
        _syllable_current += (_syllable_target - _syllable_current) * 0.2f;

        for (int x = 0; x < _w; x++) {
            float xn = (float)x / _w;
            float env = expf(-((xn - 0.5f) * (xn - 0.5f)) / (2.0f * 0.09f));

            float wave = sinf(xn * 8.0f * (float)M_PI + time)
                       + 0.7f * sinf(xn * 14.0f * (float)M_PI + time * 1.8f)
                       + 0.5f * sinf(xn * 21.0f * (float)M_PI + time * 3.0f)
                       + 0.3f * sinf(xn * 30.0f * (float)M_PI + time * 4.5f)
                       + 0.2f * sinf(xn * 40.0f * (float)M_PI + time * 6.2f);

            float raw = wave / 2.5f;
            float height = fabsf(raw);
            height = height * height * 2.0f;
            height = fminf(height, 1.0f);
            height *= _syllable_current;
            // Minimum shrinks as grow increases (starts as a line, grows to full)
            float min_h = 0.15f * (1.0f - grow) + 0.06f * grow;
            height = min_h + height * (1.0f - min_h);

            int spike_h = (int)(height * max_spike_h);
            if (spike_h < 1) spike_h = 1;

            for (int dy = -spike_h; dy <= spike_h; dy++) {
                int py = (int)wave_cy + dy;
                if (py < 0 || py >= _h) continue;

                float dist = (float)abs(dy) / (float)spike_h;

                uint8_t r, g, b;
                if (dist < 0.25f) {
                    float core = 1.0f - dist / 0.25f;
                    r = (uint8_t)(180 * core);
                    g = (uint8_t)(255 * (0.5f + 0.5f * core));
                    b = (uint8_t)(255);
                } else {
                    float fade = 1.0f - (dist - 0.25f) / 0.75f;
                    fade = fade * fade;
                    float color_mix = dist;
                    r = 0;
                    g = (uint8_t)((220 - 150 * color_mix) * fade);
                    b = (uint8_t)((255 - 50 * color_mix) * fade);
                }

                _fbPixel(x, py, rgb565(r, g, b));
            }
        }
    }
}

void FaceRenderer::_drawTransition_ProcessingToSpeaking(float t) {
    // t=0: processing (cyan mirrored waveform), t=1: speaking (orange ribbon)
    // Morph: same waveform shape, color shifts, symmetry breaks, brows appear

    int wave_cy = _cy + (int)(30.0f * t);  // shift down as speaking is lower

    // Syllable engine
    if (_phase_ms >= _syllable_next_ms) {
        _syllable_seed = _syllable_seed * 1103515245 + 12345;
        float rnd = (float)((_syllable_seed >> 16) & 0x7FFF) / 32767.0f;
        _syllable_target = 0.3f + rnd * 0.7f;
        uint32_t interval = 120 + ((_syllable_seed >> 8) & 0xFF) % 180;
        _syllable_next_ms = (uint32_t)_phase_ms + interval;
    }
    _syllable_current += (_syllable_target - _syllable_current) * 0.25f;

    float time = _phase_ms / 100.0f;

    // Symmetry fades: t=0 mirrored, t=1 single wave
    // Spike height lerps between processing and speaking amounts
    float proc_amp = _cy * 0.85f;
    float speak_amp = _cy * 0.35f;
    float max_h = proc_amp + (speak_amp - proc_amp) * t;

    // Band thickness grows as we move to speaking ribbon
    int speak_band = 50;
    float band_mix = t;

    for (int x = 0; x < _w; x++) {
        float xn = (float)x / _w;

        // Processing envelope fades out, speaking has none
        float proc_env = expf(-((xn - 0.5f) * (xn - 0.5f)) / (2.0f * 0.09f));
        float env = proc_env * (1.0f - t) + 1.0f * t;  // full width at t=1

        // Harmonics: sharp processing harmonics morph into smoother speaking
        float proc_wave = sinf(xn * 8.0f * (float)M_PI + time)
                        + 0.7f * sinf(xn * 14.0f * (float)M_PI + time * 1.8f)
                        + 0.5f * sinf(xn * 21.0f * (float)M_PI + time * 3.0f)
                        + 0.3f * sinf(xn * 30.0f * (float)M_PI + time * 4.5f);

        float speak_wave = sinf(xn * 4.0f * (float)M_PI + time * 1.2f)
                         + 0.6f * sinf(xn * 7.0f * (float)M_PI + time * 1.7f)
                         + 0.4f * sinf(xn * 11.0f * (float)M_PI + time * 3.1f);

        float wave = proc_wave * (1.0f - t) + speak_wave * t;
        wave *= env * _syllable_current;

        // Processing: mirrored bars. Speaking: offset wave with band.
        // Blend between the two approaches
        float wave_val = wave * 0.4f;
        int yc = wave_cy + (int)(wave_val * max_h);

        // Height: processing uses abs spike, speaking uses band
        float proc_h_raw = fabsf(wave) / 2.5f;
        proc_h_raw = proc_h_raw * proc_h_raw * 2.0f;
        proc_h_raw = fminf(proc_h_raw, 1.0f);
        int proc_spike = (int)(proc_h_raw * max_h * _syllable_current);
        if (proc_spike < 2) proc_spike = 2;

        int band_h = (int)(speak_band * band_mix);
        int total_h = proc_spike + (int)((band_h - proc_spike) * t);
        if (total_h < 2) total_h = 2;

        for (int dy = -total_h; dy <= total_h; dy++) {
            int py = yc + dy;
            if (py < 0 || py >= _h) continue;

            float dist = (float)abs(dy) / (float)total_h;

            // Color: lerp from cyan to orange/magenta
            float edge;
            if (dist < 0.25f) {
                edge = 1.0f;
            } else {
                edge = 1.0f - (dist - 0.25f) / 0.75f;
                edge = edge * edge;
            }
            if (edge <= 0.01f) continue;

            float screen_vt = (float)py / (float)_h;

            // Cyan: r=0, g=220, b=255  ->  Orange: r=255, g=180, b=30
            // lerp by t
            float cr = (0.0f + 255.0f * t) * edge;
            float cg = (220.0f + (180.0f - 220.0f) * t) * edge;
            float cb = (255.0f + (30.0f - 255.0f) * t) * edge;

            // At speaking end, add vertical gradient
            if (t > 0.3f) {
                float grad_t = (t - 0.3f) / 0.7f;  // 0..1
                // Shift bottom toward magenta
                float mag_r = 230.0f * edge;
                float mag_g = 20.0f * edge;
                float mag_b = 200.0f * edge;
                float vt_mix = screen_vt * grad_t;
                cr = cr * (1.0f - vt_mix) + mag_r * vt_mix;
                cg = cg * (1.0f - vt_mix) + mag_g * vt_mix;
                cb = cb * (1.0f - vt_mix) + mag_b * vt_mix;
            }

            _fbPixel(x, py, rgb565((uint8_t)cr, (uint8_t)cg, (uint8_t)cb));
        }
    }

    // Brows fade in during second half
    if (t > 0.5f) {
        float brow_alpha = (t - 0.5f) / 0.5f;
        // Draw brows with reduced brightness
        int left_cx  = _cx - _cx * 2 / 5;
        int right_cx = _cx + _cx * 2 / 5;
        int brow_y   = _cy - _cy / 2;
        int brow_w   = _cx / 4;
        int arc_h    = brow_w / 2;

        for (int side = 0; side < 2; side++) {
            int cx = (side == 0) ? left_cx : right_cx;
            for (int thickness = 0; thickness < 5; thickness++) {
                for (int px = -brow_w; px <= brow_w; px++) {
                    float bt = (float)px / (float)brow_w;
                    float curve = cosf(bt * (float)M_PI * 0.5f);
                    int py = brow_y - (int)(curve * arc_h) + thickness;
                    float gt = fabsf(bt);
                    uint8_t r = (uint8_t)((255.0f - 25.0f * gt) * brow_alpha);
                    uint8_t g = (uint8_t)((180.0f - 150.0f * gt) * brow_alpha);
                    uint8_t b = (uint8_t)((30.0f + 150.0f * gt) * brow_alpha);
                    _fbPixel(cx + px, py, rgb565(r, g, b));
                }
            }
        }
    }
}

void FaceRenderer::_drawTransition_SpeakingToAggressive(float t) {
    int wave_cy = _cy + (int)(30.0f * (1.0f - t) + 25.0f * t);

    if (_phase_ms >= _syllable_next_ms) {
        _syllable_seed = _syllable_seed * 1103515245 + 12345;
        float rnd = (float)((_syllable_seed >> 16) & 0x7FFF) / 32767.0f;
        _syllable_target = 0.3f + rnd * 0.7f;
        uint32_t interval = (uint32_t)(120 - 40 * t) + ((_syllable_seed >> 8) & 0xFF) % 150;
        _syllable_next_ms = (uint32_t)_phase_ms + interval;
    }
    _syllable_current += (_syllable_target - _syllable_current) * (0.25f + 0.1f * t);

    float time = _phase_ms / (120.0f - 40.0f * t);
    float speak_max = _cy * 0.35f;
    float aggr_max = _cy * 0.85f;
    float max_h = speak_max + (aggr_max - speak_max) * t;
    int speak_band = 50;

    for (int x = 0; x < _w; x++) {
        float xn = (float)x / _w;

        float speak_w = sinf(xn * 4.0f * (float)M_PI + time * 1.2f)
                      + 0.6f * sinf(xn * 7.0f * (float)M_PI + time * 1.7f)
                      + 0.4f * sinf(xn * 11.0f * (float)M_PI + time * 3.1f);

        float aggr_w = sinf(xn * 10.0f * (float)M_PI + time)
                     + 0.9f * sinf(xn * 17.0f * (float)M_PI + time * 1.9f)
                     + 0.7f * sinf(xn * 23.0f * (float)M_PI + time * 3.1f)
                     + 0.5f * sinf(xn * 31.0f * (float)M_PI + time * 4.7f)
                     + 0.4f * sinf(xn * 41.0f * (float)M_PI + time * 6.3f);

        float wave = speak_w * (1.0f - t) + aggr_w * t;
        wave *= _syllable_current;

        float speak_offset = wave * 0.4f * speak_max * (1.0f - t);
        int yc = wave_cy + (int)speak_offset;

        float aggr_h = fabsf(wave) / 2.5f;
        aggr_h = aggr_h * aggr_h * 2.0f;
        aggr_h = fminf(aggr_h, 1.0f);
        aggr_h = 0.08f + aggr_h * 0.92f;
        int aggr_spike = (int)(aggr_h * max_h);

        int total_h = (int)(speak_band * (1.0f - t) + aggr_spike * t);
        if (total_h < 2) total_h = 2;

        for (int dy = -total_h; dy <= total_h; dy++) {
            int py = yc + dy;
            if (py < 0 || py >= _h) continue;

            float dist = (float)abs(dy) / (float)total_h;
            float soft = sqrtf(fmaxf(0, 1.0f - dist));
            float sharp = (dist < 0.3f) ? 1.0f : fmaxf(0, 1.0f - (dist-0.3f)/0.7f);
            sharp = sharp * sharp;
            float edge = soft * (1.0f - t) + sharp * t;
            if (edge <= 0.01f) continue;

            float screen_vt = (float)py / (float)_h;

            float sr = (255.0f + (230.0f - 255.0f) * screen_vt) * edge;
            float sg = (180.0f + (30.0f - 180.0f) * screen_vt) * edge;
            float sb = (30.0f + (180.0f - 30.0f) * screen_vt) * edge;

            float ar, ag, ab;
            if (dist < 0.3f) {
                float core = 1.0f - dist / 0.3f;
                ar = 255 * edge; ag = (200*core + 40*(1-core)) * edge; ab = 180*core * edge;
            } else {
                ar = 255 * edge; ag = 20 * edge; ab = 10 * edge;
            }

            uint8_t r = (uint8_t)(sr * (1.0f - t) + ar * t);
            uint8_t g = (uint8_t)(sg * (1.0f - t) + ag * t);
            uint8_t b = (uint8_t)(sb * (1.0f - t) + ab * t);
            _fbPixel(x, py, rgb565(r, g, b));
        }
    }

    // Brows morph: curved up -> V down, gradient -> amber
    int left_cx  = _cx - _cx * 2 / 5;
    int right_cx = _cx + _cx * 2 / 5;
    int brow_y   = _cy - _cy / 2;
    int brow_w   = _cx / 4;
    int arc_h    = brow_w / 2;

    for (int side = 0; side < 2; side++) {
        int cx = (side == 0) ? left_cx : right_cx;
        int inner_dir = (side == 0) ? 1 : -1;

        for (int thickness = 0; thickness < 5; thickness++) {
            for (int px = -brow_w; px <= brow_w; px++) {
                float bt = (float)px / (float)brow_w;
                float curve_up = cosf(bt * (float)M_PI * 0.5f);
                float curve_v = (float)px * inner_dir / (float)brow_w;
                float dir_f = -1.0f + 2.0f * t;  // -1 -> +1
                float curve = curve_up * (1.0f - t) + curve_v * t;
                int py = brow_y + (int)(dir_f * curve * arc_h) + thickness;

                float gt = fabsf(bt);
                uint8_t r = (uint8_t)((255.0f - 25.0f * gt) * (1.0f-t) + (255.0f - 30.0f * gt) * t);
                uint8_t g = (uint8_t)((180.0f - 150.0f * gt) * (1.0f-t) + (160.0f - 80.0f * gt) * t);
                uint8_t b = (uint8_t)((30.0f + 150.0f * gt) * (1.0f-t) + 20.0f * t);
                _fbPixel(cx + px, py, rgb565(r, g, b));
            }
        }
    }
}

void FaceRenderer::_drawTransition_WaveToStandby(float t, bool fromAggressive) {
    // t=0: wave state (speaking or aggressive), t=1: standby rings
    // Phase 1 (t 0->0.5): wave spikes shrink to flat line, color shifts to cyan
    // Phase 2 (t 0.5->1): flat line expands into rings

    int wave_cy = _cy + (int)(28.0f * (1.0f - t));  // drift back to center

    if (t < 0.5f) {
        // Phase 1: wave shrinks, color shifts to cyan
        float shrink = t / 0.5f;  // 0->1
        shrink = shrink * shrink;

        if (_phase_ms >= _syllable_next_ms) {
            _syllable_seed = _syllable_seed * 1103515245 + 12345;
            float rnd = (float)((_syllable_seed >> 16) & 0x7FFF) / 32767.0f;
            _syllable_target = 0.3f + rnd * 0.7f;
            uint32_t interval = 120 + ((_syllable_seed >> 8) & 0xFF) % 150;
            _syllable_next_ms = (uint32_t)_phase_ms + interval;
        }
        _syllable_current += (_syllable_target - _syllable_current) * 0.25f;

        float time = _phase_ms / 100.0f;
        float start_max = fromAggressive ? _cy * 0.85f : _cy * 0.35f;
        float max_h = start_max * (1.0f - shrink);  // shrinks to 0
        if (max_h < 2) max_h = 2;

        for (int x = 0; x < _w; x++) {
            float xn = (float)x / _w;

            float wave;
            if (fromAggressive) {
                wave = sinf(xn * 10.0f * (float)M_PI + time)
                     + 0.9f * sinf(xn * 17.0f * (float)M_PI + time * 1.9f)
                     + 0.7f * sinf(xn * 23.0f * (float)M_PI + time * 3.1f)
                     + 0.5f * sinf(xn * 31.0f * (float)M_PI + time * 4.7f);
            } else {
                wave = sinf(xn * 4.0f * (float)M_PI + time * 1.2f)
                     + 0.6f * sinf(xn * 7.0f * (float)M_PI + time * 1.7f)
                     + 0.4f * sinf(xn * 11.0f * (float)M_PI + time * 3.1f);
            }
            wave *= _syllable_current;

            float height;
            if (fromAggressive) {
                height = fabsf(wave) / 2.5f;
                height = height * height * 2.0f;
                height = fminf(height, 1.0f);
                height = 0.08f + height * 0.92f;
            } else {
                height = 0.5f + fabsf(wave) * 0.15f;
            }

            int spike_h = (int)(height * max_h);
            if (spike_h < 1) spike_h = 1;

            int yc = wave_cy;

            for (int dy = -spike_h; dy <= spike_h; dy++) {
                int py = yc + dy;
                if (py < 0 || py >= _h) continue;

                float dist = (float)abs(dy) / (float)spike_h;
                float edge = sqrtf(fmaxf(0, 1.0f - dist));
                if (edge <= 0.01f) continue;

                // Color shifts toward cyan
                uint8_t r, g, b;
                if (fromAggressive) {
                    r = (uint8_t)(255 * (1.0f - shrink) * edge);
                    g = (uint8_t)((40 + 180 * shrink) * edge);
                    b = (uint8_t)((10 + 245 * shrink) * edge);
                } else {
                    float screen_vt = (float)py / (float)_h;
                    float sr = 255.0f + (230.0f - 255.0f) * screen_vt;
                    float sg = 180.0f + (30.0f - 180.0f) * screen_vt;
                    float sb = 30.0f + (180.0f - 30.0f) * screen_vt;
                    r = (uint8_t)((sr * (1.0f - shrink) + 0 * shrink) * edge);
                    g = (uint8_t)((sg * (1.0f - shrink) + 210 * shrink) * edge);
                    b = (uint8_t)((sb * (1.0f - shrink) + 230 * shrink) * edge);
                }

                _fbPixel(x, py, rgb565(r, g, b));
            }
        }

        // Brows fade out (only if from speaking)
        if (!fromAggressive && shrink < 0.8f) {
            float brow_alpha = 1.0f - shrink / 0.8f;
            _drawBrowsWithAlpha(true, brow_alpha);
        }
        if (fromAggressive && shrink < 0.8f) {
            float brow_alpha = 1.0f - shrink / 0.8f;
            _drawBrowsWithAlpha(false, brow_alpha);
        }

    } else {
        // Phase 2: flat line expands into rings
        float expand = (t - 0.5f) / 0.5f;  // 0->1
        expand = expand * expand * (3.0f - 2.0f * expand);  // smoothstep

        float breathe = _breathe(4000.0f, 0.75f, 1.0f);
        float v_scale = 0.03f + expand * 0.97f;  // 0.03 (flat) -> 1.0 (full circle)

        int base_radii[] = {185, 80};
        int ring_ws[] = {30, 25};
        uint8_t gs[] = {200, 210};
        uint8_t bs[] = {210, 230};

        for (int ring = 0; ring < 2; ring++) {
            int r = (int)(base_radii[ring] * breathe);
            int rw = ring_ws[ring];
            uint16_t color = rgb565(0, gs[ring], bs[ring]);

            int r_inner = r - rw;
            if (r_inner < 0) r_inner = 0;

            int max_dy = (int)(r * v_scale);
            if (max_dy < 1) max_dy = 1;

            for (int dy = -max_dy; dy <= max_dy; dy++) {
                int py = _cy + dy;
                if (py < 0 || py >= _h) continue;

                float circle_dy = (float)dy / v_scale;
                if (fabsf(circle_dy) > r) continue;

                int dx_outer = (int)(sqrtf(fmaxf(0, (float)(r * r) - circle_dy * circle_dy)));
                int dx_inner = 0;
                if (r_inner > 0 && fabsf(circle_dy) < r_inner) {
                    dx_inner = (int)(sqrtf(fmaxf(0, (float)(r_inner * r_inner) - circle_dy * circle_dy)));
                }

                _fbHLine(_cx - dx_outer, py, dx_outer - dx_inner, color);
                _fbHLine(_cx + dx_inner + 1, py, dx_outer - dx_inner, color);
            }
        }
    }
}
