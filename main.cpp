// trumpet — a 3D trumpet you can play with the PC keyboard.
// OpenGL 1.x fixed-function (no loader needed) + GLFW for the window + miniaudio for sound.
//
// VALVE mode (default):  A S D F G H = blow on partial 2,3,4,5,6,8
//                        J K L       = valves 1, 2, 3   (real Bb trumpet fingerings)
// PIANO mode (Tab):      Z S X D C V G B H N J M ,  = C4..C5
//                        Q 2 W 3 E R 5 T 6 Y 7 U I  = C5..C6   (valves animate the right fingering)
//                        Up / Down arrows = octave shift (Down once: low row = C3..C4, so E3 is the C key)
// Mouse drag = orbit, scroll = zoom, Esc = quit.

#define NOMINMAX  // stop windows.h from defining min/max macros that break std::min
#define MINIAUDIO_IMPLEMENTATION
#include "miniaudio.h"
#include <GLFW/glfw3.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

static const float PI = 3.14159265358979f;

// ---------------------------------------------------------------- math
struct V3 { float x, y, z; };
static V3 operator+(V3 a, V3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
static V3 operator-(V3 a, V3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
static V3 operator*(V3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
static float dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static V3 cross(V3 a, V3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
static V3 norm(V3 a) { float l = std::sqrt(dot(a, a)); return l > 1e-8f ? a * (1.f / l) : V3{0, 1, 0}; }

// ---------------------------------------------------------------- mesh
struct Mesh {
    std::vector<float> v;          // x y z nx ny nz
    std::vector<unsigned> idx;
    void vert(V3 p, V3 n) { v.insert(v.end(), {p.x, p.y, p.z, n.x, n.y, n.z}); }
    unsigned count() const { return (unsigned)(v.size() / 6); }
    void draw() const {
        if (idx.empty()) return;
        glVertexPointer(3, GL_FLOAT, 6 * sizeof(float), v.data());
        glNormalPointer(GL_FLOAT, 6 * sizeof(float), v.data() + 3);
        glDrawElements(GL_TRIANGLES, (GLsizei)idx.size(), GL_UNSIGNED_INT, idx.data());
    }
};

static void gridIndices(Mesh& m, unsigned base, int rings, int sides) {
    for (int i = 0; i < rings - 1; i++)
        for (int j = 0; j < sides; j++) {
            unsigned a = base + i * (sides + 1) + j, b = a + 1, c = a + (sides + 1), d = c + 1;
            m.idx.insert(m.idx.end(), {a, b, c, b, d, c});  // CCW seen from outside
        }
}

static float len(V3 a) { return std::sqrt(dot(a, a)); }

// Straight segments between the points, with every corner rounded off (radius rad).
// Two 90-degree corners close together make a nice U-bend, like real trumpet tubing.
static std::vector<V3> roundedPath(const std::vector<V3>& p, float rad) {
    std::vector<V3> out{p[0]};
    for (size_t i = 1; i + 1 < p.size(); i++) {
        V3 a = p[i - 1], c = p[i], b = p[i + 1];
        float r = std::min({rad, len(a - c) * 0.5f, len(b - c) * 0.5f});
        V3 s = c + norm(a - c) * r, e = c + norm(b - c) * r;
        out.push_back(s);
        for (int k = 1; k <= 14; k++) {
            float t = k / 14.f, u = 1 - t;
            out.push_back(s * (u * u) + c * (2 * u * t) + e * (t * t));
        }
    }
    out.push_back(p.back());
    return out;
}

// Total length of a tube's centre line (so a radius function can work in real distances).
static float pathLength(const std::vector<V3>& corners, float bend) {
    std::vector<V3> p = roundedPath(corners, bend);
    float L = 0;
    for (size_t i = 1; i < p.size(); i++) L += len(p[i] - p[i - 1]);
    return L;
}

// A tube swept along a path; radius(u) gets u = 0..1 by arc length.
// maxStep > 0 adds rings along straight runs (needed when the radius changes along them, e.g. a bell flare).
template <class RadiusFn>
static void addTube(Mesh& m, const std::vector<V3>& corners, float bend, RadiusFn radius, int sides = 24, float maxStep = 0) {
    std::vector<V3> raw = roundedPath(corners, bend), pts{raw[0]};
    for (size_t i = 1; i < raw.size(); i++) {
        int n = maxStep > 0 ? std::max(1, (int)std::ceil(len(raw[i] - raw[i - 1]) / maxStep)) : 1;
        for (int k = 1; k <= n; k++) pts.push_back(raw[i - 1] + (raw[i] - raw[i - 1]) * ((float)k / n));
    }
    int N = (int)pts.size();
    std::vector<float> arc(N, 0.f), rad(N);
    for (int i = 1; i < N; i++) arc[i] = arc[i - 1] + len(pts[i] - pts[i - 1]);
    for (int i = 0; i < N; i++) rad[i] = radius(arc[i] / arc[N - 1]);

    V3 T = norm(pts[1] - pts[0]);
    V3 up = std::fabs(T.y) < 0.9f ? V3{0, 1, 0} : V3{1, 0, 0};
    V3 Nn = norm(cross(cross(T, up), T));
    unsigned base = m.count();
    for (int i = 0; i < N; i++) {
        V3 Ti = norm(i == 0 ? pts[1] - pts[0] : i == N - 1 ? pts[N - 1] - pts[N - 2] : pts[i + 1] - pts[i - 1]);
        Nn = norm(Nn - Ti * dot(Nn, Ti));  // parallel transport
        V3 B = cross(Ti, Nn);
        float r = rad[i];
        int i0 = std::max(i - 1, 0), i1 = std::min(i + 1, N - 1);
        float drds = arc[i1] > arc[i0] ? (rad[i1] - rad[i0]) / (arc[i1] - arc[i0]) : 0.f;  // how fast the tube widens
        for (int j = 0; j <= sides; j++) {
            float a = 2 * PI * j / sides;
            V3 d = Nn * std::cos(a) + B * std::sin(a);
            m.vert(pts[i] + d * r, norm(d - Ti * drds));
        }
    }
    gridIndices(m, base, N, sides);
}

// A surface of revolution: profile is (distance along axis, radius).
struct P2 { float a, r; };
static void addLathe(Mesh& m, V3 origin, V3 axis, const std::vector<P2>& prof, int sides = 32) {
    axis = norm(axis);
    V3 ref = std::fabs(axis.y) < 0.9f ? V3{0, 1, 0} : V3{1, 0, 0};
    V3 U = norm(cross(axis, ref)), W = cross(axis, U);
    unsigned base = m.count();
    int n = (int)prof.size();
    for (int i = 0; i < n; i++) {
        const P2& A = prof[std::max(i - 1, 0)];
        const P2& B = prof[std::min(i + 1, n - 1)];
        float da = B.a - A.a, dr = B.r - A.r, l = std::sqrt(da * da + dr * dr);
        if (l < 1e-8f) { da = 1; dr = 0; l = 1; }
        float nr = da / l, na = -dr / l;
        for (int j = 0; j <= sides; j++) {
            float t = 2 * PI * j / sides;
            V3 d = U * std::cos(t) + W * std::sin(t);
            m.vert(origin + axis * prof[i].a + d * prof[i].r, d * nr + axis * na);
        }
    }
    gridIndices(m, base, n, sides);
}

static void addCylinder(Mesh& m, V3 origin, V3 axis, float a0, float a1, float r) {
    addLathe(m, origin, axis, {{a0, 0}, {a0, r}});       // bottom cap
    addLathe(m, origin, axis, {{a0, r}, {a1, r}});       // side
    addLathe(m, origin, axis, {{a1, r}, {a1, 0}});       // top cap
}

// ---------------------------------------------------------------- the trumpet
// Side view, laid out like a real Bb trumpet: mouthpiece on the left (-x), bell on the right (+x), y up.
// Valve casings sit at z = 0; the leadpipe and slides run on the near side (+z), the bell section on the far side (-z).
// Air path: mouthpiece -> leadpipe -> main tuning slide (front U, under the bell) -> valves
//           -> back bow (big U at the player's end) -> bell pipe -> bell.
// Slides: 1st points back toward the player, 2nd is a short stub, 3rd points forward under the main slide.
static const float VALVE_X[3] = {-0.43f, 0.0f, 0.43f};  // valve 1 is nearest the player
static const float CASING_R = 0.2f;
static const float CAP_TOP = 0.97f;        // top of the valve caps; the felt sits here
static const float PISTON_TRAVEL = 0.245f;   // pressed: the button comes to rest on the felt

struct Trumpet { Mesh brass, nickel, felt, stem, pearl; };

static Trumpet buildTrumpet() {
    Trumpet t;
    Mesh& m = t.brass;
    const float SIDE_NEAR = 0.31f, SIDE_FAR = -0.31f;  // tubing runs just beside the casings
    const float INSIDE = 0.05f;                  // where a tube disappears into a casing
    auto r = [](float v) { return [v](float) { return v; }; };

    // mouthpiece: short, with a proper cup and rim (silver), then the brass receiver
    // profile runs from the bottom of the cup, up the inside to the rim, then down the outside of the shank
    addLathe(t.nickel, {-3.4f, 0.62f, SIDE_NEAR}, {1, 0, 0},
             {{0.15f, 0}, {0.145f, 0.018f}, {0.12f, 0.022f}, {0.09f, 0.038f}, {0.055f, 0.06f}, {0.025f, 0.08f},
              {0.008f, 0.092f}, {-0.005f, 0.1f}, {-0.012f, 0.112f}, {-0.015f, 0.128f}, {-0.012f, 0.145f},
              {-0.004f, 0.157f}, {0.01f, 0.162f}, {0.04f, 0.163f}, {0.08f, 0.158f}, {0.13f, 0.14f}, {0.2f, 0.105f},
              {0.26f, 0.085f}, {0.3f, 0.083f}, {0.33f, 0.09f}, {0.37f, 0.088f}, {0.4f, 0.072f}, {0.62f, 0.068f}}, 56);
    addLathe(t.felt, {-3.4f, 0.62f, SIDE_NEAR}, {1, 0, 0}, {{0.142f, 0}, {0.142f, 0.02f}}, 24);  // dark throat hole
    addCylinder(m, {-2.8f, 0.62f, SIDE_NEAR}, {1, 0, 0}, 0, 0.26f, 0.105f);

    // Heights: both big U-bends are slim (top and bottom tubes only a few tube-widths apart) and identical;
    // the leadpipe (near side) and bell pipe (far side) run side by side at the top, just under the valve caps.
    const float TOP = 0.7f, BOT = -0.1f, UR = (TOP - BOT) / 2;

    // leadpipe -> main tuning slide (slim U) -> bottom tube back into valve 3
    addTube(m, {{-2.58f, 0.62f, SIDE_NEAR}, {2.6f, TOP, SIDE_NEAR}, {2.6f, BOT, SIDE_NEAR}, {0.5f, BOT, SIDE_NEAR},
                {0.5f, BOT, INSIDE}},
            UR, [](float u) { return 0.078f + 0.022f * std::min(1.f, u * 1.6f); });

    // out of valve 1 -> slim back bow at the player's end -> bell pipe along the top -> bell flare (far side).
    // The pipe and the flare are ONE continuous surface, so there is no joint and no seam.
    // Flare: stays narrow for a long way, opens fast at the end.
    const float L = 2.9f, r0 = 0.1f, R = 1.05f, k = 0.95f, BELL_X = 1.2f;
    const float x0 = L / (std::pow(R / r0, 1.f / k) - 1.f), b = R * std::pow(x0, k);
    const std::vector<V3> bellPath{{VALVE_X[0], BOT, -INSIDE}, {VALVE_X[0], BOT, SIDE_FAR}, {-2.3f, BOT, SIDE_FAR},
                                   {-2.3f, TOP, SIDE_FAR}, {BELL_X + L, TOP, SIDE_FAR}};
    const float bellLen = pathLength(bellPath, UR);
    addTube(m, bellPath, UR, [=](float u) {
        float a = L - bellLen * (1 - u);          // distance past the start of the flare
        return a <= 0 ? r0 : b * std::pow(L - a + x0, -k);
    }, 72, 0.012f);
    // rolled rim at the lip
    addLathe(m, {BELL_X, TOP, SIDE_FAR}, {1, 0, 0},
             {{L, R}, {L + 0.025f, R + 0.02f}, {L + 0.02f, R + 0.045f}, {L - 0.015f, R + 0.045f}}, 72);

    // Slides sit at the height of the bottom tubes, straddling that line, running beside them (not inside the U's).
    const float SL_HI = BOT + 0.15f, SL_LO = BOT - 0.15f;
    // 1st slide: back toward the player, beside the back bow's bottom tube (near side)
    addTube(m, {{VALVE_X[0], SL_HI, INSIDE}, {VALVE_X[0], SL_HI, SIDE_NEAR}, {-1.35f, SL_HI, SIDE_NEAR}, {-1.35f, SL_LO, SIDE_NEAR},
                {VALVE_X[0], SL_LO, SIDE_NEAR}, {VALVE_X[0], SL_LO, INSIDE}},
            0.15f, r(0.085f));
    // 2nd slide: short U sticking straight out to the side of the middle valve, level with the 1st and 3rd slides
    addTube(m, {{VALVE_X[1], SL_HI, 0.1f}, {VALVE_X[1], SL_HI, 0.72f}, {VALVE_X[1], SL_LO, 0.72f}, {VALVE_X[1], SL_LO, 0.1f}},
            0.15f, r(0.085f));
    // 3rd slide: forward toward the bell on the far side, behind the main slide, under the bell pipe
    addTube(m, {{0.5f, SL_HI, -INSIDE}, {0.5f, SL_HI, SIDE_FAR}, {2.1f, SL_HI, SIDE_FAR}, {2.1f, SL_LO, SIDE_FAR},
                {0.5f, SL_LO, SIDE_FAR}, {0.5f, SL_LO, -INSIDE}},
            0.15f, r(0.085f));

    // finger hook on the leadpipe
    addTube(t.nickel, {{0.95f, 0.72f, SIDE_NEAR}, {0.95f, 0.98f, SIDE_NEAR}, {1.18f, 0.98f, SIDE_NEAR}, {1.18f, 0.88f, SIDE_NEAR}}, 0.09f, r(0.024f), 12);

    // valve casings: brass body, silver top caps, brass bottom caps, dark felt pad on top
    for (float x : VALVE_X) {
        addCylinder(m, {x, 0, 0}, {0, 1, 0}, -0.58f, 0.86f, CASING_R);   // casings hang a little below the tubes
        addLathe(m, {x, 0, 0}, {0, 1, 0},                                                    // bottom cap, soft chamfers
                 {{-0.7f, 0}, {-0.7f, CASING_R - 0.01f}, {-0.68f, CASING_R + 0.02f}, {-0.58f, CASING_R + 0.02f},
                  {-0.53f, CASING_R}}, 40);
        addLathe(t.nickel, {x, 0, 0}, {0, 1, 0},                                             // top cap with a shallow pocket
                 {{0.8f, CASING_R}, {0.85f, CASING_R + 0.02f}, {0.95f, CASING_R + 0.02f}, {CAP_TOP, CASING_R + 0.005f},
                  {CAP_TOP, 0.155f}, {CAP_TOP - 0.01f, 0.155f}, {CAP_TOP - 0.01f, 0}}, 40);
        addLathe(t.felt, {x, 0, 0}, {0, 1, 0}, {{CAP_TOP - 0.002f, 0.155f}, {CAP_TOP - 0.002f, 0.065f}}, 40);   // felt, flush in the pocket
        addLathe(t.nickel, {x, 0, 0}, {0, 1, 0}, {{CAP_TOP - 0.002f, 0.065f}, {CAP_TOP - 0.002f, 0.045f}}, 40); // ring around the stem hole
    }

    // moving piston parts, built relative to the cap top: stem, silver button rim, pearl insert
    addCylinder(t.stem, {0, 0, 0}, {0, 1, 0}, -0.1f, 0.27f, 0.042f);
    addLathe(t.stem, {0, 0, 0}, {0, 1, 0},
             {{0.25f, 0}, {0.25f, 0.155f}, {0.26f, 0.165f}, {0.33f, 0.165f}, {0.345f, 0.15f}, {0.345f, 0.128f}}, 40);
    addLathe(t.pearl, {0, 0, 0}, {0, 1, 0},
             {{0.33f, 0.13f}, {0.345f, 0.128f}, {0.358f, 0.11f}, {0.366f, 0.07f}, {0.369f, 0}}, 40);
    return t;
}

// ---------------------------------------------------------------- audio
struct Synth {
    std::atomic<float> target{233.f};
    std::atomic<bool> gate{false};
    float sr = 48000, freq = 233, env = 0, held = 0, vib = 0, lp = 0, breath = 0;
    float ph[2] = {0, 0};
    bool wasOn = false;
    unsigned rng = 12345;

    // small Freeverb-style reverb: 4 damped combs + 2 allpasses
    std::vector<float> comb[4], ap[2];
    int ci[4] = {0, 0, 0, 0}, ai[2] = {0, 0};
    float damp[4] = {0, 0, 0, 0};

    void init(float sampleRate) {
        sr = sampleRate;
        const int cl[4] = {1557, 1617, 1491, 1422}, al[2] = {556, 225};
        float k = sr / 44100.f;
        for (int i = 0; i < 4; i++) comb[i].assign((size_t)(cl[i] * k), 0.f);
        for (int i = 0; i < 2; i++) ap[i].assign((size_t)(al[i] * k), 0.f);
    }

    float noise() { rng = rng * 1664525u + 1013904223u; return (rng >> 9) * (1.f / 8388608.f) * 2.f - 1.f; }

    float reverb(float x) {
        float out = 0;
        for (int i = 0; i < 4; i++) {
            float y = comb[i][ci[i]];
            damp[i] = y * 0.65f + damp[i] * 0.35f;  // darker tail
            comb[i][ci[i]] = x + damp[i] * 0.70f;  // shorter tail
            if (++ci[i] >= (int)comb[i].size()) ci[i] = 0;
            out += y;
        }
        out *= 0.25f;
        for (int i = 0; i < 2; i++) {
            float b = ap[i][ai[i]];
            ap[i][ai[i]] = out + b * 0.5f;
            if (++ai[i] >= (int)ap[i].size()) ai[i] = 0;
            out = b - out;
        }
        return out;
    }

    // Band-limited brass tone: a sum of clean harmonics (no aliasing = no fizz).
    // bright 0..1 controls how slowly the harmonics fall off.
    float tone(float& phase, float f, float bright) {
        phase += f / sr;
        if (phase >= 1) phase -= 1;
        float th = 2 * PI * phase, c = std::cos(th), sk = std::sin(th), sPrev = 0;
        float roll = 0.7f - 0.35f * bright, out = 0, total = 0;
        for (int k = 1; k <= 24; k++) {
            float fk = f * k;
            if (fk > 9000.f) break;
            float a = std::exp(-roll * (k - 1));
            if (k == 1) a *= 0.75f;                          // trumpets have a slightly weak fundamental
            if (fk > 4000.f) a *= 1.f - (fk - 4000.f) / 5000.f;  // gently tame the top end
            out += a * sk;
            total += a;
            float sn = 2 * c * sk - sPrev;  // sin(k+1) from sin(k), sin(k-1)
            sPrev = sk;
            sk = sn;
        }
        return out / std::max(total, 1e-3f);
    }

    void render(float* out, unsigned n) {
        if (comb[0].empty()) init(sr);
        float tgt = target.load();
        bool g = gate.load();
        float aC = 1 - std::exp(-1 / (0.05f * sr));
        float rC = 1 - std::exp(-1 / (0.14f * sr));
        float glC = 1 - std::exp(-1 / (0.03f * sr));
        for (unsigned i = 0; i < n; i++) {
            if (g && !wasOn) { if (env < 0.05f) freq = tgt * 0.97f; held = 0; }  // soft scoop into a fresh note
            wasOn = g;
            env += ((g ? 1.f : 0.f) - env) * (g ? aC : rC);
            freq += (tgt - freq) * glC;
            held += 1 / sr;
            vib += 2 * PI * 5.2f / sr;
            if (vib > 2 * PI) vib -= 2 * PI;
            float vibAmt = std::min(1.f, std::max(0.f, (held - 0.4f) / 0.6f));
            float f = freq * (1 + 0.004f * std::sin(vib) * vibAmt);

            float bright = 0.25f + 0.6f * env;  // blowing harder = brighter, but never harsh
            float x = 0.6f * tone(ph[0], f, bright) + 0.4f * tone(ph[1], f * 1.0035f, bright);  // two voices, ~6 cents apart

            breath += (noise() - breath) * 0.08f;  // soft, low breath noise
            x += breath * 0.06f;

            float fc = std::min(7000.f, f * 7.f);  // warm low-pass that follows the pitch
            lp += (x - lp) * (1 - std::exp(-2 * PI * fc / sr));

            float dry = lp * env * 0.45f;
            out[i] = dry + reverb(dry) * 0.07f;  // just a touch of room
        }
    }
};

static void audioCallback(ma_device* d, void* out, const void*, ma_uint32 n) {
    ((Synth*)d->pUserData)->render((float*)out, n);
}

// ---------------------------------------------------------------- input
static bool keyDown[GLFW_KEY_LAST + 1];
static std::vector<int> noteOrder;  // held note keys, last = newest
static bool pianoMode = false;
static int octave = 0;  // piano-mode octave shift, -2..+1: the low row starts at C2, C3, C4 or C5
static float yaw = 28, pitch = 12, dist = 12;
static bool dragging = false;
static double lastX, lastY;

static int partialFor(int key) {
    switch (key) {
        case GLFW_KEY_A: return 2; case GLFW_KEY_S: return 3; case GLFW_KEY_D: return 4;
        case GLFW_KEY_F: return 5; case GLFW_KEY_G: return 6; case GLFW_KEY_H: return 8;
    }
    return 0;
}

static int pianoNote(int key) {
    switch (key) {
        case GLFW_KEY_Z: return 60; case GLFW_KEY_S: return 61; case GLFW_KEY_X: return 62;
        case GLFW_KEY_D: return 63; case GLFW_KEY_C: return 64; case GLFW_KEY_V: return 65;
        case GLFW_KEY_G: return 66; case GLFW_KEY_B: return 67; case GLFW_KEY_H: return 68;
        case GLFW_KEY_N: return 69; case GLFW_KEY_J: return 70; case GLFW_KEY_M: return 71;
        case GLFW_KEY_COMMA: return 72;
        case GLFW_KEY_Q: return 72; case GLFW_KEY_2: return 73; case GLFW_KEY_W: return 74;
        case GLFW_KEY_3: return 75; case GLFW_KEY_E: return 76; case GLFW_KEY_R: return 77;
        case GLFW_KEY_5: return 78; case GLFW_KEY_T: return 79; case GLFW_KEY_6: return 80;
        case GLFW_KEY_Y: return 81; case GLFW_KEY_7: return 82; case GLFW_KEY_U: return 83;
        case GLFW_KEY_I: return 84;
    }
    return -1;
}

static bool isNoteKey(int key) { return pianoMode ? pianoNote(key) >= 0 : partialFor(key) > 0; }

// Bb trumpet fingering for a concert MIDI note (pedal Bb2 = 46).
static void fingering(int midi, bool v[3]) {
    static const int parts[] = {2, 3, 4, 5, 6, 8, 9, 10};
    static const bool tab[7][3] = {{0,0,0},{0,1,0},{1,0,0},{1,1,0},{0,1,1},{1,0,1},{1,1,1}};
    int off = midi - 46, best = 99;
    while (off < 6) off += 12;   // any note works: fold it into the range the fingering chart covers
    while (off > 40) off -= 12;
    for (int n : parts) {
        int s = (int)std::lround(12 * std::log2((float)n)) - off;
        if (s >= 0 && s <= 6 && s < best) best = s;
    }
    for (int i = 0; i < 3; i++) v[i] = best <= 6 ? tab[best][i] : false;
}

static std::string noteName(int midi) {
    static const char* names[] = {"C","C#","D","Eb","E","F","F#","G","Ab","A","Bb","B"};
    return std::string(names[((midi % 12) + 12) % 12]) + std::to_string(midi / 12 - 1);
}

static void keyCb(GLFWwindow* w, int key, int, int action, int) {
    if (key < 0 || key > GLFW_KEY_LAST || action == GLFW_REPEAT) return;
    if (action == GLFW_PRESS) {
        if (key == GLFW_KEY_ESCAPE) glfwSetWindowShouldClose(w, 1);
        if (key == GLFW_KEY_TAB) { pianoMode = !pianoMode; noteOrder.clear(); }
        if (key == GLFW_KEY_UP) octave = std::min(octave + 1, 1);   // highest start: C5
        if (key == GLFW_KEY_DOWN) octave = std::max(octave - 1, -2);  // lowest start: C2
        keyDown[key] = true;
        if (isNoteKey(key)) noteOrder.push_back(key);
    } else {
        keyDown[key] = false;
        noteOrder.erase(std::remove(noteOrder.begin(), noteOrder.end(), key), noteOrder.end());
    }
}
static void mouseBtnCb(GLFWwindow* w, int b, int action, int) {
    if (b == GLFW_MOUSE_BUTTON_LEFT) { dragging = action == GLFW_PRESS; glfwGetCursorPos(w, &lastX, &lastY); }
}
static void cursorCb(GLFWwindow*, double x, double y) {
    if (dragging) {
        yaw += (float)(x - lastX) * 0.4f;
        pitch = std::max(-89.f, std::min(89.f, pitch + (float)(y - lastY) * 0.4f));
    }
    lastX = x; lastY = y;
}
static void scrollCb(GLFWwindow*, double, double dy) { dist = std::max(3.f, std::min(25.f, dist * (float)std::pow(0.9, dy))); }

// ---------------------------------------------------------------- realistic metal shading
// A small GLSL shader makes the metal look real: it reflects a made-up photo studio (softboxes,
// strip lights, a dark floor), gets brighter at grazing angles (Fresnel), and is tone-mapped like
// a camera. The few GL 2.0 functions it needs are loaded through GLFW, so no extra library.
// If shaders aren't available it falls back to the old fixed-function lighting.
#if defined(_WIN32)
#define GLCALL __stdcall
#else
#define GLCALL
#endif
#ifndef GL_FRAGMENT_SHADER
#define GL_FRAGMENT_SHADER 0x8B30
#define GL_VERTEX_SHADER 0x8B31
#define GL_COMPILE_STATUS 0x8B81
#define GL_LINK_STATUS 0x8B82
#endif
static GLuint (GLCALL* pCreateShader)(GLenum);
static void (GLCALL* pShaderSource)(GLuint, GLsizei, const char* const*, const GLint*);
static void (GLCALL* pCompileShader)(GLuint);
static void (GLCALL* pGetShaderiv)(GLuint, GLenum, GLint*);
static void (GLCALL* pGetShaderInfoLog)(GLuint, GLsizei, GLsizei*, char*);
static GLuint (GLCALL* pCreateProgram)();
static void (GLCALL* pAttachShader)(GLuint, GLuint);
static void (GLCALL* pLinkProgram)(GLuint);
static void (GLCALL* pGetProgramiv)(GLuint, GLenum, GLint*);
static void (GLCALL* pUseProgram)(GLuint);
static GLint (GLCALL* pGetUniformLocation)(GLuint, const char*);
static void (GLCALL* pUniform3f)(GLint, float, float, float);
static void (GLCALL* pUniform1f)(GLint, float);

static const char* VERT_SRC = R"(
#version 120
centroid varying vec3 vPos, vNrm, vObj;  // centroid: anti-aliased edge pixels sample inside the surface
void main() {
    vec4 p = gl_ModelViewMatrix * gl_Vertex;
    vPos = p.xyz;
    vNrm = gl_NormalMatrix * gl_Normal;
    vObj = gl_Vertex.xyz;
    gl_Position = gl_ProjectionMatrix * p;
}
)";

static const char* FRAG_SRC = R"(
#version 120
centroid varying vec3 vPos, vNrm, vObj;  // centroid: anti-aliased edge pixels sample inside the surface
uniform vec3 uBase;    // metal colour (linear)
uniform float uRough;  // 0 = mirror, higher = softer reflections
uniform float uWear;   // 0 = brand new, 1 = well played
uniform float uKind;   // 0 = metal, 1 = mother-of-pearl, 2 = felt
uniform vec3 uSeed;    // shifts the pearl pattern so each button looks different

float hash(vec3 p) {   // sin-free hash: stable on every GPU, no blocky artifacts
    p = fract(p * 0.1031);
    p += dot(p, p.zyx + 31.32);
    return fract((p.x + p.y) * p.z);
}
float noise(vec3 p) {
    vec3 i = floor(p), f = fract(p);
    f = f * f * (3.0 - 2.0 * f);
    return mix(mix(mix(hash(i), hash(i + vec3(1,0,0)), f.x), mix(hash(i + vec3(0,1,0)), hash(i + vec3(1,1,0)), f.x), f.y),
               mix(mix(hash(i + vec3(0,0,1)), hash(i + vec3(1,0,1)), f.x), mix(hash(i + vec3(0,1,1)), hash(i + vec3(1,1,1)), f.x), f.y), f.z);
}

// The "studio" the metal reflects, as a function of direction (camera space: +y up, +z toward the viewer).
vec3 studio(vec3 r, float w) {
    vec3 c = mix(vec3(0.02, 0.019, 0.02), vec3(0.24, 0.22, 0.2), smoothstep(-0.2, 0.9, r.y));      // walls
    c = mix(mix(vec3(0.03, 0.022, 0.015), vec3(0.16, 0.12, 0.08), smoothstep(-1.0, -0.2, r.y)), c,
            smoothstep(-0.12, 0.04, r.y));                                                          // warm floor sweep
    float a = atan(r.x, r.z);
    float tall = smoothstep(-0.9, -0.6, r.y) * (1.0 - smoothstep(0.6, 0.8, r.y));
    vec3 D = normalize(vec3(-0.55, 0.75, 0.35));                                                   // key softbox: upper left
    c += vec3(4.0, 3.9, 3.7) * smoothstep(0.62 - w, 0.84 + w, dot(r, D));
    c += vec3(0.45, 0.44, 0.42) * smoothstep(0.55, 0.95, r.y);                                      // weak top fill
    c += vec3(3.0, 2.95, 2.85) * (1.0 - smoothstep(0.12, 0.12 + 2.0 * w, abs(a - 0.7))) * tall;     // tall strip, front right
    c += vec3(1.8, 1.75, 1.7) * (1.0 - smoothstep(0.08, 0.08 + 2.0 * w, abs(a + 0.55))) * tall;     // thinner strip, front left
    c += vec3(0.5, 0.47, 0.43) * (1.0 - smoothstep(0.0, 0.5, abs(a))) * tall;                       // soft fill behind the camera
    return c;
}

void main() {
    // --- aged lacquer: worn-through patches, mottled colour, gentle unevenness.
    // Every fine detail fades to its average when it gets too small on screen, so nothing sparkles.
    float fb = noise(vObj * 1.6) * 0.5 + noise(vObj * 3.7) * 0.3 + noise(vObj * 8.3) * 0.2;
    float worn = smoothstep(0.42, 0.78, fb);                               // lacquer worn through to raw brass
    float mott = noise(vObj * 14.0 + 3.1) * 0.6 + noise(vObj * 31.0) * 0.4;
    mott = mix(0.5, mott, clamp(1.0 - fwidth(mott) * 4.0, 0.0, 1.0));      // colour mottling
    vec3 wob = vec3(noise(vObj * 5.0), noise(vObj * 5.0 + 17.3), noise(vObj * 5.0 + 41.9)) - 0.5;

    vec3 N = normalize(vNrm);
    if (!gl_FrontFacing) N = -N;                       // inside of the bell
    N = normalize(N + wob * 0.03 * uWear);
    vec3 V = normalize(-vPos);
    float nv = dot(N, V);
    if (nv < 0.05) N = normalize(N + V * (0.05 - nv));   // edge pixels: never let the surface face away from the camera
    float ndv = max(dot(N, V), 0.0);
    vec3 R = reflect(-V, N);

    vec3 base = uBase * mix(vec3(1.0), mix(vec3(1.0, 0.97, 0.86), vec3(0.9, 0.8, 0.74), mott), uWear);  // olive-gold to rosy
    base = mix(base, uBase * vec3(0.6, 0.46, 0.3), worn * 0.55 * uWear);   // raw, oxidised brass where the lacquer is gone
    float rough = uRough + (0.15 * worn + 0.06 * mott) * uWear;

    vec3 col;
    float spread = 0.03 + rough + 0.35 * pow(1.0 - ndv, 3.0);   // softer reflections toward the edges: no thin sparkly slivers
    vec3 L = normalize(vec3(-0.55, 0.75, 0.35)), H = normalize(L + V);   // same direction as the key softbox
    if (uKind < 0.5) {
        vec3 F = base * (1.0 + 0.15 * pow(1.0 - ndv, 3.0));   // edges only slightly brighter, and stay metal-coloured
        col = F * studio(R, spread) * (0.1 / (0.1 + spread * 0.9));    // blurrier reflections are also dimmer
        col += F * pow(max(dot(N, H), 0.0), mix(300.0, 25.0, clamp(rough * 3.0, 0.0, 1.0))) * 0.6;
        col += base * (0.05 + 0.05 * uWear) * (0.35 * studio(N, 0.8) + 0.4 * max(dot(N, L), 0.0) + 0.2);  // satin body: aged metal scatters a bit more
    } else if (uKind < 1.5) {
        // mother-of-pearl: swirling layered bands with a rainbow shimmer that shifts with the viewing angle
        vec3 p = vObj * 22.0 + uSeed;
        vec2 q = vec2(noise(p), noise(p + 5.2));
        float sw = noise(p * 0.6 + vec3(q * 3.0, 0.0));                // domain-warped swirl
        float bands = sw * 7.0;
        float aa = clamp(1.0 - fwidth(bands) * 1.5, 0.0, 1.0);        // fade fine bands when far away: no shimmer
        float layer = mix(0.5, 0.5 + 0.5 * sin(bands * 6.2831), aa);
        float t = sw * 1.3 + ndv * 0.9 + layer * 0.25;
        vec3 irid = 0.5 + 0.5 * cos(6.2831 * (t + vec3(0.0, 0.33, 0.67)));
        vec3 body = base * mix(vec3(1.0, 0.95, 0.9), irid, 0.32 + 0.18 * layer) * (0.84 + 0.2 * layer);
        col = body * (0.32 * studio(N, 0.6) + 0.28 * max(dot(N, L), 0.0) + 0.05);
        float fr = 0.05 + 0.25 * pow(1.0 - ndv, 5.0);
        col += fr * studio(R, 0.04) * 0.9;                             // polished, glossy surface
    } else {
        // felt: fine fibres and clumps, with a soft velvety sheen at grazing angles
        vec3 p = vObj * 140.0;
        float f1 = noise(p), f2 = noise(p * 2.3 + 11.0);
        float aa = clamp(1.0 - fwidth(f1) * 2.0, 0.0, 1.0);           // average the fibres out when far away
        float fib = mix(0.5, f1 * 0.6 + f2 * 0.4, aa);
        float clump = noise(vObj * 30.0);
        vec3 body = base * (0.55 + 0.9 * fib) * (0.85 + 0.3 * clump);
        col = body * (0.6 * studio(N, 0.8) + 0.6 * max(dot(N, L), 0.0) + 0.15);
        col += vec3(0.05) * (pow(1.0 - ndv, 2.0) * 0.5 + 0.08) * (0.5 + fib);
    }
    if (!gl_FrontFacing) col *= 0.75;

    col *= 0.6;   // exposure: overall brightness of the lighting
    col = clamp((col * (2.51 * col + 0.03)) / (col * (2.43 * col + 0.59) + 0.14), 0.0, 1.0);  // filmic tone map
    gl_FragColor = vec4(pow(col, vec3(1.0 / 2.2)), 1.0);
}
)";

static GLuint shaderProg = 0;
static GLint uBaseLoc = -1, uRoughLoc = -1, uWearLoc = -1, uKindLoc = -1, uSeedLoc = -1;

static GLuint compileShader(GLenum type, const char* src) {
    GLuint s = pCreateShader(type);
    pShaderSource(s, 1, &src, nullptr);
    pCompileShader(s);
    GLint ok = 0;
    pGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[2048];
        pGetShaderInfoLog(s, sizeof log, nullptr, log);
        std::fprintf(stderr, "Shader error:\n%s\n", log);
        return 0;
    }
    return s;
}

static void initShading() {
#define LOAD(var, name) var = (decltype(var))glfwGetProcAddress(name)
    LOAD(pCreateShader, "glCreateShader"); LOAD(pShaderSource, "glShaderSource");
    LOAD(pCompileShader, "glCompileShader"); LOAD(pGetShaderiv, "glGetShaderiv");
    LOAD(pGetShaderInfoLog, "glGetShaderInfoLog"); LOAD(pCreateProgram, "glCreateProgram");
    LOAD(pAttachShader, "glAttachShader"); LOAD(pLinkProgram, "glLinkProgram");
    LOAD(pGetProgramiv, "glGetProgramiv"); LOAD(pUseProgram, "glUseProgram");
    LOAD(pGetUniformLocation, "glGetUniformLocation"); LOAD(pUniform3f, "glUniform3f");
    LOAD(pUniform1f, "glUniform1f");
#undef LOAD
    if (!pCreateShader || !pUseProgram || !pUniform1f) return;  // very old GPU: keep fixed-function
    GLuint vs = compileShader(GL_VERTEX_SHADER, VERT_SRC), fs = compileShader(GL_FRAGMENT_SHADER, FRAG_SRC);
    if (!vs || !fs) return;
    GLuint p = pCreateProgram();
    pAttachShader(p, vs);
    pAttachShader(p, fs);
    pLinkProgram(p);
    GLint ok = 0;
    pGetProgramiv(p, GL_LINK_STATUS, &ok);
    if (!ok) return;
    shaderProg = p;
    uBaseLoc = pGetUniformLocation(p, "uBase");
    uRoughLoc = pGetUniformLocation(p, "uRough");
    uWearLoc = pGetUniformLocation(p, "uWear");
    uKindLoc = pGetUniformLocation(p, "uKind");
    uSeedLoc = pGetUniformLocation(p, "uSeed");
}

// Sets a metal: colour + roughness for the shader, or the old-style material as a fallback.
enum Kind { METAL = 0, PEARL = 1, FELT = 2 };
static void material(float r, float g, float b, float rough, float wear, Kind kind = METAL) {
    if (shaderProg) {
        pUniform3f(uBaseLoc, r, g, b);
        pUniform1f(uRoughLoc, rough);
        pUniform1f(uWearLoc, wear);
        pUniform1f(uKindLoc, (float)kind);
        return;
    }
    float d[] = {r, g, b, 1}, s[] = {1, 1, 0.9f, 1};
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, d);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, s);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 90);
}

int main() {
    if (!glfwInit()) return 1;
    glfwWindowHint(GLFW_SAMPLES, 8);
    GLFWwindow* win = glfwCreateWindow(1280, 720, "Trumpet", nullptr, nullptr);
    if (!win) { glfwTerminate(); return 1; }
    glfwMakeContextCurrent(win);
    glfwSwapInterval(1);
    glfwSetKeyCallback(win, keyCb);
    glfwSetMouseButtonCallback(win, mouseBtnCb);
    glfwSetCursorPosCallback(win, cursorCb);
    glfwSetScrollCallback(win, scrollCb);

    Synth synth;
    ma_device device;
    ma_device_config cfg = ma_device_config_init(ma_device_type_playback);
    cfg.playback.format = ma_format_f32;
    cfg.playback.channels = 1;
    cfg.sampleRate = 48000;
    cfg.dataCallback = audioCallback;
    cfg.pUserData = &synth;
    bool audioOk = ma_device_init(nullptr, &cfg, &device) == MA_SUCCESS;
    if (audioOk) { synth.init((float)device.sampleRate); ma_device_start(&device); }
    else std::fprintf(stderr, "Could not open audio device - running silent.\n");

    Trumpet trumpet = buildTrumpet();
    initShading();
    if (shaderProg) pUseProgram(shaderProg);
    std::printf(shaderProg ? "Realistic shading on.\n" : "Shaders unavailable - using simple lighting.\n");

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1);
    glEnable(GL_NORMALIZE);
    glShadeModel(GL_SMOOTH);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);  // lights the inside of the bell
    float amb[] = {0.18f, 0.16f, 0.14f, 1};
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, amb);
    float l0d[] = {1.0f, 0.97f, 0.9f, 1}, l1d[] = {0.35f, 0.4f, 0.5f, 1}, spec[] = {1, 1, 1, 1}, none[] = {0, 0, 0, 1};
    glLightfv(GL_LIGHT0, GL_DIFFUSE, l0d);
    glLightfv(GL_LIGHT0, GL_SPECULAR, spec);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, l1d);
    glLightfv(GL_LIGHT1, GL_SPECULAR, none);
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_NORMAL_ARRAY);

    float press[3] = {0, 0, 0};
    std::string lastTitle;
    double prevT = glfwGetTime();

    while (!glfwWindowShouldClose(win)) {
        double now = glfwGetTime();
        float dt = (float)(now - prevT);
        prevT = now;

        // ---- work out what is being played
        bool valves[3] = {false, false, false}, playing = false;
        int midi = 0;
        float freq = 0;
        if (!pianoMode) {
            valves[0] = keyDown[GLFW_KEY_J]; valves[1] = keyDown[GLFW_KEY_K]; valves[2] = keyDown[GLFW_KEY_L];
            int semis = (valves[0] ? 2 : 0) + (valves[1] ? 1 : 0) + (valves[2] ? 3 : 0);
            if (!noteOrder.empty()) {
                int n = partialFor(noteOrder.back());
                freq = 116.54f * n * std::pow(2.f, -semis / 12.f);
                midi = (int)std::lround(46 + 12 * std::log2((float)n)) - semis;
                playing = true;
            }
        } else if (!noteOrder.empty()) {
            midi = pianoNote(noteOrder.back()) + 12 * octave;
            freq = 440.f * std::pow(2.f, (midi - 69) / 12.f);
            fingering(midi, valves);
            playing = true;
        }
        if (playing) synth.target = freq;
        synth.gate = playing;

        std::string title = std::string("Trumpet  |  ") +
            (pianoMode ? "PIANO mode: Z-M / Q-I rows, Up/Down octave (low row starts at C" + std::to_string(4 + octave) + ")" : "VALVE mode: A S D F G H blow, J K L valves") +
            "  |  Tab switches  |  " + (playing ? noteName(midi) : "-");
        if (title != lastTitle) { glfwSetWindowTitle(win, title.c_str()); lastTitle = title; }

        float k = std::min(1.f, dt * 25.f);
        for (int i = 0; i < 3; i++) press[i] += ((valves[i] ? 1.f : 0.f) - press[i]) * k;

        // ---- draw
        int w, h;
        glfwGetFramebufferSize(win, &w, &h);
        if (w == 0 || h == 0) { glfwPollEvents(); continue; }
        glViewport(0, 0, w, h);
        glClearColor(0.07f, 0.07f, 0.09f, 1);  // dark slate background
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        float nearZ = 0.1f, top = nearZ * std::tan(40.f * PI / 360.f), aspect = (float)w / h;
        glFrustum(-top * aspect, top * aspect, -top, top, nearZ, 100.0);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        float l0p[] = {0.4f, 0.8f, 1.0f, 0}, l1p[] = {-0.7f, -0.3f, 0.4f, 0};  // fixed to the camera
        glLightfv(GL_LIGHT0, GL_POSITION, l0p);
        glLightfv(GL_LIGHT1, GL_POSITION, l1p);
        glTranslatef(0, 0, -dist);
        glRotatef(pitch, 1, 0, 0);
        glRotatef(yaw, 0, 1, 0);
        glTranslatef(-0.35f, -0.25f, 0);  // centre of the horn

        material(0.78f, 0.56f, 0.22f, 0.22f, 1.0f);  // old lacquered brass with real hours on it: satin, worn, mottled
        trumpet.brass.draw();
        material(0.85f, 0.84f, 0.81f, 0.14f, 0.35f);  // silver: mouthpiece, valve caps, hook
        trumpet.nickel.draw();
        material(0.018f, 0.017f, 0.017f, 0.7f, 0.0f, FELT);  // black felt pads (and the mouthpiece throat)
        trumpet.felt.draw();
        for (int i = 0; i < 3; i++) {
            glPushMatrix();
            glTranslatef(VALVE_X[i], CAP_TOP - press[i] * PISTON_TRAVEL, 0);
            material(0.85f, 0.84f, 0.81f, 0.1f, 0.4f);  // silver stem + button rim
            trumpet.stem.draw();
            if (shaderProg) pUniform3f(uSeedLoc, i * 3.7f, i * 1.9f, i * 5.3f);  // each button gets its own pattern
            material(0.93f, 0.91f, 0.87f, 0.15f, 0.0f, PEARL);  // mother-of-pearl inlay
            trumpet.pearl.draw();
            glPopMatrix();
        }

        glfwSwapBuffers(win);
        glfwPollEvents();
    }

    if (audioOk) ma_device_uninit(&device);
    glfwTerminate();
    return 0;
}
