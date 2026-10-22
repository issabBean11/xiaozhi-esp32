// Bean Bean: live anime kitty face (golden glowing eyes, pink ear outlines, blush, tiny "w" mouth).
// Drawn with LVGL shapes, animated with little springs at ~30 fps. Reacts to talking, moods, taps, tilt and shakes.
#include "bean_face.h"

#include "application.h"

#include <esp_log.h>
#include <esp_random.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>
#include <string.h>

#define TAG "BeanFace"

namespace bean {

// ----------------------------------------------------------------------------- motion sensor
static std::atomic<bool> g_imu_ok{false};
static std::atomic<bool> g_shaken{false};
static std::atomic<bool> g_lifted{false};
static std::atomic<bool> g_upside{false};
static std::atomic<int64_t> g_last_motion{0};
static volatile float g_tilt_x = 0, g_tilt_y = 0;
static volatile float g_dyn_x = 0, g_dyn_y = 0;   // quick jolts (for wobble)

static void ImuTask(void* arg) {
    auto dev = (i2c_master_dev_handle_t)arg;
    auto wr = [&](uint8_t r, uint8_t v) { uint8_t b[2] = {r, v}; return i2c_master_transmit(dev, b, 2, 50); };
    wr(0x02, 0x40);   // CTRL1: register auto-increment
    wr(0x03, 0x16);   // CTRL2: accel +-4g, ~125 Hz
    wr(0x08, 0x01);   // CTRL7: accel on
    vTaskDelay(pdMS_TO_TICKS(30));
    float gx = 0, gy = 0, gz = 1;
    int64_t spikes[4] = {0, 0, 0, 0};
    int spike_i = 0;
    int64_t last_spike = 0;
    float bx = 0, by = 0, bz = 0;          // resting "upright" gravity, learned at start
    int settle = 0, calm = 0, busy = 0;
    while (true) {
        uint8_t reg = 0x35, d[6];
        if (i2c_master_transmit_receive(dev, &reg, 1, d, 6, 50) == ESP_OK) {
            float ax = (int16_t)(d[0] | d[1] << 8) / 8192.0f;
            float ay = (int16_t)(d[2] | d[3] << 8) / 8192.0f;
            float az = (int16_t)(d[4] | d[5] << 8) / 8192.0f;
            gx = gx * 0.85f + ax * 0.15f; gy = gy * 0.85f + ay * 0.15f; gz = gz * 0.85f + az * 0.15f;
            float mag = sqrtf(ax * ax + ay * ay + az * az);
            int64_t now = esp_timer_get_time();
            float dev = fabsf(mag - 1.0f);
            if (settle < 25) {                     // first second: learn how she normally stands
                bx += ax / 25; by += ay / 25; bz += az / 25; settle++;
            } else {
                // tilt relative to her normal pose
                g_tilt_x = gx - bx; g_tilt_y = gy - by;
                g_dyn_x = ax - gx; g_dyn_y = ay - gy;
                // upside down: gravity points the opposite way from normal
                float gn = sqrtf(gx * gx + gy * gy + gz * gz) + 1e-3f;
                float dot = (gx * bx + gy * by + gz * bz) / gn;
                g_upside = dot < -0.55f;
                // picked up: a gentle lift after sitting still
                if (dev > 0.08f) { g_last_motion = now; busy++; calm = 0; } else { calm++; if (calm > 10) busy = 0; }
                if (busy == 4 && dev < 0.9f && calm == 0) g_lifted = true;
            }
            if (fabsf(mag - 1.0f) > 0.9f && now - last_spike > 90000) {
                last_spike = now;
                spikes[spike_i++ & 3] = now;
                int recent = 0;
                for (auto s : spikes) if (now - s < 1200000) recent++;
                if (recent >= 3) { g_shaken = true; for (auto& s : spikes) s = 0; }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(40));
    }
}

void StartImu(i2c_master_bus_handle_t bus) {
    static const uint8_t ADDRS[] = {0x6B, 0x6A};
    for (uint8_t addr : ADDRS) {
        if (i2c_master_probe(bus, addr, 50) != ESP_OK) continue;
        i2c_device_config_t cfg = {};
        cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        cfg.device_address = addr;
        cfg.scl_speed_hz = 400000;
        i2c_master_dev_handle_t dev;
        if (i2c_master_bus_add_device(bus, &cfg, &dev) != ESP_OK) continue;
        uint8_t reg = 0x00, who = 0;
        i2c_master_transmit_receive(dev, &reg, 1, &who, 1, 50);
        ESP_LOGI(TAG, "QMI8658 at 0x%02x, id 0x%02x", addr, who);
        g_imu_ok = true;
        xTaskCreate(ImuTask, "bean_imu", 3072, dev, 3, nullptr);
        return;
    }
    ESP_LOGW(TAG, "No motion sensor found");
}

// ----------------------------------------------------------------------------- moods
enum EyeMode { OPEN, HAPPY, CLOSED, XEYE, HEART, STAR, SPIRAL, WINK };
enum MouthMode { M_W, M_SMILE, M_BIG, M_FROWN, M_O, M_TONGUE, M_FLAT };
enum { SPARK = 1, TEARS = 2, SWEAT = 4, ANGER = 8, ZZZ = 16, QUES = 32, BANG = 64, HATCH = 128, STREAM = 256, EARWIG = 512, ASYM = 1024 };
static const uint32_t GOLD = 0xFFBE46, PINK = 0xFF78BE, ORANGE = 0xFF7A3C;

struct Mood {
    const char* name; EyeMode eye; float size, pupil, lid, slant, lx, ly; MouthMode mouth; float blush; uint32_t fx; uint32_t color;
};
static const Mood MOODS[] = {
    {"neutral",     OPEN,   1.00, 1.00, 0.00,  0.0,   0,   0, M_W,      0.6, 0,                      GOLD},
    {"happy",       HAPPY,  1.00, 1.00, 0.00,  0.0,   0,   0, M_SMILE,  1.0, SPARK | EARWIG,         GOLD},
    {"laughing",    XEYE,   1.00, 1.00, 0.00,  0.0,   0,   0, M_BIG,    1.2, SPARK | EARWIG,         GOLD},
    {"funny",       WINK,   1.00, 1.00, 0.00,  0.0,   0,   0, M_TONGUE, 1.0, SPARK,                  GOLD},
    {"sad",         OPEN,   0.95, 1.15, 0.12, -0.8,   0,   6, M_FROWN,  0.5, TEARS,                  GOLD},
    {"angry",       OPEN,   0.95, 0.80, 0.18,  1.0,   0,   0, M_FROWN,  0.3, ANGER,                  ORANGE},
    {"crying",      CLOSED, 1.00, 1.00, 0.00,  0.0,   0,   0, M_BIG,    0.8, STREAM,                 GOLD},
    {"loving",      HEART,  1.00, 1.00, 0.00,  0.0,   0,   0, M_W,      1.6, SPARK | EARWIG,         PINK},
    {"embarrassed", XEYE,   0.90, 1.00, 0.00,  0.0,   0,   0, M_FLAT,   2.0, HATCH | SWEAT,          GOLD},
    {"surprised",   OPEN,   1.15, 0.60, 0.00,  0.0,   0,  -4, M_O,      0.5, BANG,                   GOLD},
    {"shocked",     OPEN,   1.22, 0.40, 0.00,  0.0,   0,  -6, M_BIG,    0.3, BANG | SWEAT,           GOLD},
    {"thinking",    OPEN,   0.95, 0.90, 0.22,  0.0,  22, -16, M_FLAT,   0.5, QUES,                   GOLD},
    {"winking",     WINK,   1.00, 1.00, 0.00,  0.0,   0,   0, M_SMILE,  1.0, SPARK,                  GOLD},
    {"cool",        OPEN,   1.00, 0.90, 0.42,  0.0,   0,   0, M_FLAT,   0.4, SPARK,                  GOLD},
    {"relaxed",     CLOSED, 1.00, 1.00, 0.00,  0.0,   0,   0, M_W,      1.0, 0,                      GOLD},
    {"delicious",   HAPPY,  1.00, 1.00, 0.00,  0.0,   0,   0, M_TONGUE, 1.2, SPARK,                  GOLD},
    {"kissy",       CLOSED, 1.00, 1.00, 0.00,  0.0,   0,   0, M_O,      1.6, 0,                      GOLD},
    {"confident",   STAR,   1.00, 1.00, 0.00,  0.0,   0,   0, M_SMILE,  0.8, SPARK,                  GOLD},
    {"sleepy",      OPEN,   1.00, 1.00, 0.62,  0.0,   0,   4, M_FLAT,   0.6, ZZZ,                    GOLD},
    {"silly",       XEYE,   1.00, 1.00, 0.00,  0.0,   0,   0, M_TONGUE, 1.2, SWEAT | EARWIG,         GOLD},
    {"confused",    OPEN,   1.00, 0.90, 0.10,  0.0, -10,  -6, M_FLAT,   0.5, QUES | SWEAT | ASYM,    GOLD},
    {"petted",      HAPPY,  1.05, 1.00, 0.00,  0.0,   0,   0, M_SMILE,  1.9, SPARK | EARWIG | HATCH, GOLD},
    {"dizzy",       SPIRAL, 1.00, 1.00, 0.00,  0.0,   0,   0, M_O,      0.6, SWEAT,                  GOLD},
};
static const int MOOD_COUNT = sizeof(MOODS) / sizeof(MOODS[0]);
static int MoodIndex(const char* n) {
    for (int i = 0; i < MOOD_COUNT; i++) if (n && strcmp(MOODS[i].name, n) == 0) return i;
    return -1;
}

// ----------------------------------------------------------------------------- body transform
// Everything is designed on a 480x480 canvas, then drawn smaller (a little blob) and moved around
// the screen by the wobble physics.
static const float G_S = 0.70f;          // kitty size on screen
static float G_OX = 0, G_OY = 0;         // where the blob has rolled to
static float TX(float x) { return 240 + (x - 240) * G_S + G_OX; }
static float TY(float y) { return 250 + (y - 250) * G_S + G_OY; }
static int SW(int w) { int v = lroundf(w * G_S); return v < 1 ? 1 : v; }
static lv_obj_t *s_glow, *s_body, *s_earf[2], *s_eari[2];
static lv_obj_t *s_torso_glow, *s_torso, *s_tail_glow, *s_tail, *s_paw_glow[2], *s_paw[2], *s_star[2];
static lv_point_precise_t s_earf_pts[2][3], s_eari_pts[2][3];

// Fur themes: 0 = black kitty, 1 = white kitty, 2 = pastel pink kitty
struct Theme { uint32_t fur, rim, ear_in, mouth, line_eye, blush, text; };
static const Theme THEMES[3] = {
    {0x1C1824, 0xA996E6, 0xFF8DB6, 0xFF9EC4, 0xFFBE46, 0xFF5C93, 0xC8BEFF},   // black, lavender glow
    {0xFFF9FC, 0xFFB3D0, 0xFF8DB6, 0x9C3D63, 0xC9731A, 0xFF7BA6, 0x7A5CC8},   // white, pink rim
    {0xFFC9DD, 0xFF8DB6, 0xFF86B0, 0x8E3459, 0xB8610F, 0xFF5C93, 0x7A5CC8},   // pastel pink
};
static int g_theme = 0;
static std::atomic<int> g_pending_theme{-1};
#define FUR (THEMES[g_theme].fur)
#define RIMC (THEMES[g_theme].rim)
#define LINE_EYE (THEMES[g_theme].line_eye)   // pastel pink kitty with a soft light rim

// ----------------------------------------------------------------------------- tiny LVGL helpers
static lv_obj_t* Blob(lv_obj_t* p, uint32_t color, lv_opa_t opa = LV_OPA_COVER) {
    lv_obj_t* o = lv_obj_create(p);
    lv_obj_remove_style_all(o);
    lv_obj_set_style_bg_opa(o, opa, 0);
    lv_obj_set_style_bg_color(o, lv_color_hex(color), 0);
    lv_obj_set_style_radius(o, LV_RADIUS_CIRCLE, 0);
    lv_obj_remove_flag(o, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_remove_flag(o, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(o, LV_OBJ_FLAG_HIDDEN);
    return o;
}
static lv_obj_t* Stroke(lv_obj_t* p, uint32_t color, int width) {
    lv_obj_t* l = lv_line_create(p);
    lv_obj_set_style_line_width(l, SW(width), 0);
    lv_obj_set_style_line_color(l, lv_color_hex(color), 0);
    lv_obj_set_style_line_rounded(l, true, 0);
    lv_obj_set_pos(l, 0, 0);
    lv_obj_remove_flag(l, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(l, LV_OBJ_FLAG_HIDDEN);
    return l;
}
static lv_obj_t* Curve(lv_obj_t* p, uint32_t color, int width) {
    lv_obj_t* a = lv_arc_create(p);
    lv_obj_remove_style(a, nullptr, LV_PART_KNOB);
    lv_obj_set_style_bg_opa(a, LV_OPA_TRANSP, 0);
    lv_obj_set_style_arc_opa(a, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_pad_all(a, 0, 0);
    lv_obj_set_style_arc_color(a, lv_color_hex(color), LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(a, SW(width), LV_PART_INDICATOR);
    lv_obj_set_style_arc_rounded(a, true, LV_PART_INDICATOR);
    lv_arc_set_rotation(a, 0);
    lv_arc_set_bg_angles(a, 0, 360);
    lv_obj_remove_flag(a, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(a, LV_OBJ_FLAG_HIDDEN);
    return a;
}
static lv_obj_t* Text(lv_obj_t* p, const char* s, uint32_t color) {
    lv_obj_t* l = lv_label_create(p);
    lv_label_set_text(l, s);
    lv_obj_set_style_text_color(l, lv_color_hex(color), 0);
    lv_obj_remove_flag(l, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(l, LV_OBJ_FLAG_HIDDEN);
    return l;
}
static void Show(lv_obj_t* o, bool on) {
    bool hidden = lv_obj_has_flag(o, LV_OBJ_FLAG_HIDDEN);
    if (on && hidden) lv_obj_remove_flag(o, LV_OBJ_FLAG_HIDDEN);
    else if (!on && !hidden) lv_obj_add_flag(o, LV_OBJ_FLAG_HIDDEN);
}
static void BoxRaw(lv_obj_t* o, float x, float y, float w, float h) {
    int ix = lroundf(x), iy = lroundf(y), iw = lroundf(w < 1 ? 1 : w), ih = lroundf(h < 1 ? 1 : h);
    if (lv_obj_get_style_x(o, LV_PART_MAIN) != ix || lv_obj_get_style_y(o, LV_PART_MAIN) != iy) lv_obj_set_pos(o, ix, iy);
    if (lv_obj_get_style_width(o, LV_PART_MAIN) != iw || lv_obj_get_style_height(o, LV_PART_MAIN) != ih) lv_obj_set_size(o, iw, ih);
}
static void Box(lv_obj_t* o, float x, float y, float w, float h) {
    int ix = lroundf(TX(x)), iy = lroundf(TY(y)), iw = lroundf(w * G_S < 1 ? 1 : w * G_S), ih = lroundf(h * G_S < 1 ? 1 : h * G_S);
    if (lv_obj_get_style_x(o, LV_PART_MAIN) != ix || lv_obj_get_style_y(o, LV_PART_MAIN) != iy) lv_obj_set_pos(o, ix, iy);
    if (lv_obj_get_style_width(o, LV_PART_MAIN) != iw || lv_obj_get_style_height(o, LV_PART_MAIN) != ih) lv_obj_set_size(o, iw, ih);
}
static void Circle(lv_obj_t* o, float cx, float cy, float d) { Box(o, cx - d / 2, cy - d / 2, d, d); }
static void Line(lv_obj_t* l, lv_point_precise_t* store, const float* xy, int n) {
    bool diff = false;
    for (int i = 0; i < n; i++) {
        lv_value_precise_t x = lroundf(TX(xy[i * 2])), y = lroundf(TY(xy[i * 2 + 1]));
        if (store[i].x != x || store[i].y != y) { store[i].x = x; store[i].y = y; diff = true; }
    }
    if (diff || lv_line_get_point_count(l) != (uint32_t)n) lv_line_set_points(l, store, n);
}
static void ArcAt(lv_obj_t* a, float cx, float cy, float d, int a0, int a1) {
    Circle(a, cx, cy, d);
    if (lv_arc_get_angle_start(a) != a0 || lv_arc_get_angle_end(a) != a1) lv_arc_set_angles(a, a0, a1);
}
static void Color(lv_obj_t* o, uint32_t c) {
    lv_color_t col = lv_color_hex(c);
    if (!lv_color_eq(lv_obj_get_style_bg_color(o, LV_PART_MAIN), col)) lv_obj_set_style_bg_color(o, col, 0);
}
static void LineW(lv_obj_t* o, int w) { w = SW(w); if (lv_obj_get_style_line_width(o, LV_PART_MAIN) != w) lv_obj_set_style_line_width(o, w, 0); }
static void LineC(lv_obj_t* o, uint32_t c) {
    lv_color_t col = lv_color_hex(c);
    if (!lv_color_eq(lv_obj_get_style_line_color(o, LV_PART_MAIN), col)) lv_obj_set_style_line_color(o, col, 0);
}
static void Radius(lv_obj_t* o, int r) { if (lv_obj_get_style_radius(o, LV_PART_MAIN) != r) lv_obj_set_style_radius(o, r, 0); }
static void Pos(lv_obj_t* o, float x, float y) {
    int ix = lroundf(TX(x)), iy = lroundf(TY(y));
    if (lv_obj_get_style_x(o, LV_PART_MAIN) != ix || lv_obj_get_style_y(o, LV_PART_MAIN) != iy) lv_obj_set_pos(o, ix, iy);
}
static void Spring(float& p, float& v, float target, float k = 0.16f, float damp = 0.30f) {
    v += (target - p) * k; v *= (1.0f - damp); p += v;
}
static uint32_t Rgb(float r, float g, float b) { return ((uint32_t)r << 16) | ((uint32_t)g << 8) | (uint32_t)b; }
static uint32_t Rand(uint32_t lo, uint32_t hi) { return lo + esp_random() % (hi - lo + 1); }

// ----------------------------------------------------------------------------- build
void Face::BuildEye(Eye& e) {
    e.halo2 = Blob(root_, 0xFFFFFF, 60);
    e.halo1 = Blob(root_, GOLD, 90);
    e.iris = Blob(root_, GOLD);
    lv_obj_set_style_bg_grad_dir(e.iris, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_border_width(e.iris, 4, 0);
    lv_obj_set_style_border_color(e.iris, lv_color_hex(0x9A5208), 0);
    e.pupil = Blob(root_, 0x3A1A02);
    e.hl1 = Blob(root_, 0xFFFFFF);
    e.hl2 = Blob(root_, 0xFFFFFF);
    e.hl3 = Blob(root_, 0xFFF4C8);
    e.arc = Curve(root_, LINE_EYE, 13);
    e.l1 = Stroke(root_, LINE_EYE, 13);
    e.l2 = Stroke(root_, GOLD, 13);
    e.hc1 = Blob(root_, PINK);
    e.hc2 = Blob(root_, PINK);
    e.star_v = Blob(root_, 0xFFAA1E);
    e.star_h = Blob(root_, 0xFFAA1E);
    e.lid = Stroke(root_, FUR, 60);
    lv_obj_set_style_line_rounded(e.lid, false, 0);
}

void Face::Create(lv_obj_t* parent) {
    root_ = lv_obj_create(parent);
    lv_obj_remove_style_all(root_);
    lv_obj_set_size(root_, 480, 480);
    lv_obj_set_pos(root_, 0, 0);
    lv_obj_remove_flag(root_, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(root_, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(root_, ClickCb, LV_EVENT_CLICKED, this);

    s_tail_glow = Curve(root_, RIMC, 40);
    s_tail = Curve(root_, FUR, 28);
    s_torso_glow = Blob(root_, RIMC);
    s_torso = Blob(root_, FUR);
    for (int i = 0; i < 2; i++) { s_paw_glow[i] = Blob(root_, RIMC); s_paw[i] = Blob(root_, FUR); }
    Show(s_tail_glow, true); Show(s_tail, true); Show(s_torso_glow, true); Show(s_torso, true);
    for (int i = 0; i < 2; i++) { Show(s_paw_glow[i], true); Show(s_paw[i], true); }
    s_glow = Blob(root_, RIMC);
    for (int i = 0; i < 2; i++) { s_earf[i] = Stroke(root_, RIMC, 62); }
    s_body = Blob(root_, FUR);
    for (int i = 0; i < 2; i++) { s_eari[i] = Stroke(root_, FUR, 50); lv_obj_set_style_line_color(s_eari[i], lv_color_hex(FUR), 0); }
    ear_l_ = Stroke(root_, 0xFF86B0, 18);
    ear_r_ = Stroke(root_, 0xFF86B0, 18);
    Show(s_glow, true); Show(s_body, true);
    for (int i = 0; i < 2; i++) { Show(s_earf[i], true); Show(s_eari[i], true); }
    s_star[0] = Blob(root_, 0xFFC93C); s_star[1] = Blob(root_, 0xFFC93C);
    Show(s_star[0], true); Show(s_star[1], true);
    for (auto& e : eye_) BuildEye(e);
    for (int s = 0; s < 2; s++) {
        blush_[s] = Blob(root_, 0xFF5C93, 120);
        for (int i = 0; i < 3; i++) hatch_[s][i] = Stroke(root_, 0xE0457E, 4);
    }
    w_l_ = Curve(root_, 0x8E3459, 6);
    w_r_ = Curve(root_, 0x8E3459, 6);
    frown_ = Curve(root_, 0x8E3459, 6);
    flat_ = Stroke(root_, 0x8E3459, 6);
    mouth_ = Blob(root_, 0x7A1232);
    lv_obj_set_style_radius(mouth_, 18, 0);
    lv_obj_set_style_border_width(mouth_, 4, 0);
    lv_obj_set_style_border_color(mouth_, lv_color_hex(0x8E3459), 0);
    tongue_in_ = Blob(mouth_, 0xFF6E96);
    lv_obj_remove_flag(tongue_in_, LV_OBJ_FLAG_HIDDEN);
    tongue_out_ = Blob(root_, 0xFF6E96);
    lv_obj_set_style_radius(tongue_out_, 9, 0);
    for (int i = 0; i < 4; i++) { spark_[i][0] = Blob(root_, 0xFFB020); spark_[i][1] = Blob(root_, 0xFFB020); }
    for (auto& t : tear_) t = Blob(root_, 0x78C8FF);
    sweat_ = Blob(root_, 0x8CD2FF);
    for (auto& a : anger_) a = Curve(root_, 0xFF3C50, 5);
    zzz_[0] = Text(root_, "z", 0x7A5CC8); zzz_[1] = Text(root_, "Z", 0x7A5CC8); zzz_[2] = Text(root_, "Z", 0x7A5CC8);
    qmark_ = Text(root_, "?", 0x7A5CC8);
    bang_ = Text(root_, "!", 0xFF5064);

    Show(ear_l_, true); Show(ear_r_, true); Show(blush_[0], true); Show(blush_[1], true);
    if (g_pending_theme.load() < 0) g_pending_theme = g_theme;
    lv_timer_create(TickCb, 33, this);
}

static void ApplyTheme(lv_obj_t* const* lids, lv_obj_t* ear_l, lv_obj_t* ear_r, lv_obj_t* const* mouth_parts, int n_mouth,
                       lv_obj_t* const* blush, lv_obj_t* const* texts, int n_text) {
    const Theme& t = THEMES[g_theme];
    lv_obj_t* rims[4] = {s_glow, s_torso_glow, s_paw_glow[0], s_paw_glow[1]};
    lv_obj_t* furs[4] = {s_body, s_torso, s_paw[0], s_paw[1]};
    for (lv_obj_t* o : rims) lv_obj_set_style_bg_color(o, lv_color_hex(t.rim), 0);
    for (lv_obj_t* o : furs) lv_obj_set_style_bg_color(o, lv_color_hex(t.fur), 0);
    lv_obj_set_style_arc_color(s_tail_glow, lv_color_hex(t.rim), LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(s_tail, lv_color_hex(t.fur), LV_PART_INDICATOR);
    for (int i = 0; i < 2; i++) {
        lv_obj_set_style_line_color(s_earf[i], lv_color_hex(t.rim), 0);
        lv_obj_set_style_line_color(s_eari[i], lv_color_hex(t.fur), 0);
        lv_obj_set_style_line_color(lids[i], lv_color_hex(t.fur), 0);
    }
    lv_obj_set_style_line_color(ear_l, lv_color_hex(t.ear_in), 0);
    lv_obj_set_style_line_color(ear_r, lv_color_hex(t.ear_in), 0);
    for (int i = 0; i < n_mouth; i++) {
        lv_obj_t* o = mouth_parts[i];
        if (lv_obj_check_type(o, &lv_arc_class)) lv_obj_set_style_arc_color(o, lv_color_hex(t.mouth), LV_PART_INDICATOR);
        else if (lv_obj_check_type(o, &lv_line_class)) lv_obj_set_style_line_color(o, lv_color_hex(t.mouth), 0);
        else lv_obj_set_style_border_color(o, lv_color_hex(t.mouth), 0);
    }
    for (int i = 0; i < 2; i++) lv_obj_set_style_bg_color(blush[i], lv_color_hex(t.blush), 0);
    for (int i = 0; i < n_text; i++) lv_obj_set_style_text_color(texts[i], lv_color_hex(t.text), 0);
}

void SetFurTheme(int theme) { if (theme >= 0 && theme < 3) g_pending_theme = theme; }
int GetFurTheme() { return g_theme; }

void Face::SetMood(const char* emotion) {
    int i = MoodIndex(emotion);
    pending_mood_ = i < 0 ? 0 : i;
}
void Face::Poke() { pending_poke_ = true; }
void Face::TickCb(lv_timer_t* t) { static_cast<Face*>(lv_timer_get_user_data(t))->Tick(); }
void Face::ClickCb(lv_event_t* e) { static_cast<Face*>(lv_event_get_user_data(e))->Poke(); }

// ----------------------------------------------------------------------------- draw one eye
void Face::DrawEye(Eye& e, int side, float cx, float cy) {
    const Mood& m = MOODS[mood_];
    EyeMode mode = m.eye;
    if (mode == WINK) mode = side > 0 ? HAPPY : OPEN;
    float asym = (m.fx & ASYM) && side > 0 ? 0.75f : 1.0f;
    float pop = 1.0f + pop_;
    float W = 96 * w_ * asym * pop, Hfull = 132 * h_ * asym * pop;

    // blink
    static const float curve[] = {0.55f, 0.15f, 0.06f, 0.15f, 0.55f, 1.0f};
    float blink = 1.0f;
    if (blink_t_ > 0 && blink_t_ <= 6) blink = curve[blink_t_ - 1];
    float H = Hfull * blink;
    uint32_t col = Rgb(cr_, cg_, cb_);
    uint32_t col_top = Rgb(cr_ * 0.9f, cg_ * 0.62f, cb_ * 0.35f);
    uint32_t col_bot = Rgb(fminf(255, cr_ + 10), fminf(255, cg_ + 40), fminf(255, cb_ + 60));

    bool open = mode == OPEN;
    Show(e.halo2, mode == STAR); Show(e.halo1, open); Show(e.iris, open);
    Show(e.pupil, open && blink > 0.3f); Show(e.hl1, open && blink > 0.3f); Show(e.hl2, open && blink > 0.3f); Show(e.hl3, open && blink > 0.3f);
    Show(e.arc, mode == HAPPY || mode == CLOSED || mode == SPIRAL);
    Show(e.l1, mode == XEYE || mode == HEART); Show(e.l2, mode == HEART);
    Show(e.hc1, mode == HEART); Show(e.hc2, mode == HEART);
    Show(e.star_v, mode == STAR); Show(e.star_h, mode == STAR);

    float px = cx + lx_ * 0.45f, py = cy + ly_ * 0.45f;   // whole eye shifts a bit, pupil shifts more
    if (open) {
        Box(e.halo1, px - W / 2 - 11, py - H / 2 - 11, W + 22, H + 22); Color(e.halo1, col);
        Box(e.iris, px - W / 2, py - H / 2, W, H);
        Color(e.iris, col_top);
        lv_color_t gb = lv_color_hex(col_bot);
        if (!lv_color_eq(lv_obj_get_style_bg_grad_color(e.iris, LV_PART_MAIN), gb)) lv_obj_set_style_bg_grad_color(e.iris, gb, 0);
        float ppx = cx + lx_ * 0.8f, ppy = cy + ly_ * 0.8f + H * 0.05f;
        float pw = W * 0.44f * pupil_, ph = H * 0.52f * pupil_;
        Box(e.pupil, ppx - pw / 2, ppy - ph / 2, pw, ph);
        float tw = 1.0f + 0.12f * sinf(t_ * 0.21f + side);
        Circle(e.hl1, ppx - W * 0.17f, ppy - H * 0.2f, W * 0.34f);
        Circle(e.hl2, ppx + W * 0.19f, ppy + H * 0.17f, W * 0.15f * tw);
        Circle(e.hl3, ppx + W * 0.06f, ppy - H * 0.33f, W * 0.09f * (2.0f - tw));
    }
    if (mode == HAPPY) { float d = W * 0.95f; ArcAt(e.arc, px, py + d * 0.22f, d, 200, 340); }
    if (mode == CLOSED) { float d = W * 0.95f; ArcAt(e.arc, px, py - d * 0.3f, d, 20, 160); }
    if (mode == SPIRAL) { float d = W * 0.9f; int a0 = (t_ * 28) % 360; ArcAt(e.arc, px, py, d, a0, a0 + 290); }
    if (mode == XEYE || mode == HEART) LineC(e.l1, mode == HEART ? PINK : LINE_EYE);
    if (mode == XEYE) {
        float s = W * 0.4f, dir = side < 0 ? 1.0f : -1.0f;   // left eye ">", right eye "<"
        float xy[6] = {px - dir * s, py - s * 0.85f, px + dir * s * 0.7f, py, px - dir * s, py + s * 0.85f};
        LineW(e.l1, 13);
        Line(e.l1, e.p1, xy, 3);
    }
    if (mode == HEART) {
        float s = W * (1.0f + 0.08f * sinf(t_ * 0.35f));
        Circle(e.hc1, px - s * 0.23f, py - s * 0.12f, s * 0.52f);
        Circle(e.hc2, px + s * 0.23f, py - s * 0.12f, s * 0.52f);
        float xy[6] = {px - s * 0.34f, py + s * 0.02f, px, py + s * 0.36f, px + s * 0.34f, py + s * 0.02f};
        LineW(e.l1, lroundf(s * 0.3f));
        Line(e.l1, e.p1, xy, 3);
        float xy2[4] = {px, py - s * 0.05f, px, py + s * 0.22f};
        LineW(e.l2, lroundf(s * 0.42f));
        LineC(e.l2, PINK);
        Line(e.l2, e.p2, xy2, 2);
        Circle(e.hl1, px - s * 0.28f, py - s * 0.2f, s * 0.14f); Show(e.hl1, true);
    }
    if (mode == STAR) {
        float s = W * (1.05f + 0.12f * sinf(t_ * 0.3f + side));
        Box(e.star_v, px - s * 0.11f, py - s * 0.55f, s * 0.22f, s * 1.1f);
        Box(e.star_h, px - s * 0.55f, py - s * 0.11f, s * 1.1f, s * 0.22f);
        Box(e.halo2, px - s * 0.3f, py - s * 0.3f, s * 0.6f, s * 0.6f); Color(e.halo2, col);
    }
    // eyelid (angry / sad slant, sleepy / smug cut)
    bool lid = open && (lid_ > 0.02f || fabsf(slant_) > 0.05f);
    Show(e.lid, lid);
    if (lid) {
        float cut = py - H / 2 + lid_ * H;
        float inner = cut + slant_ * H * 0.32f, outer = cut - slant_ * H * 0.12f;
        float xo = px - side * (W / 2 + 40), xi = px + side * (W / 2 + 40);
        float xy[4] = {xo, outer - 30, xi, inner - 30};
        Line(e.lid, e.lid_pts, xy, 2);
    }
}

// ----------------------------------------------------------------------------- frame
void Face::Tick() {
    t_++;
    int th = g_pending_theme.exchange(-1);
    if (th >= 0) {
        g_theme = th;
        lv_obj_t* lids[2] = {eye_[0].lid, eye_[1].lid};
        lv_obj_t* mouth_parts[5] = {w_l_, w_r_, frown_, flat_, mouth_};
        lv_obj_t* texts[4] = {zzz_[0], zzz_[1], zzz_[2], qmark_};
        ApplyTheme(lids, ear_l_, ear_r_, mouth_parts, 5, blush_, texts, 4);
        for (auto& e : eye_) lv_obj_set_style_arc_color(e.arc, lv_color_hex(LINE_EYE), LV_PART_INDICATOR);
    }
    int pm = pending_mood_.exchange(-1);
    if (pm >= 0 && pm != base_mood_) { base_mood_ = pm; vpop_ += 0.10f; }
    auto state0 = Application::GetInstance().GetDeviceState();
    bool idle0 = state0 == kDeviceStateIdle;
    static uint32_t last_voice = 0, last_active = 0;
    auto voice = [&](const char* what) {   // let her react out loud (not too often)
        if (idle0 && (last_voice == 0 || t_ - last_voice > 1500)) {   // ~50 s cooldown
            last_voice = t_;
            Application::GetInstance().WakeWordInvoke(what);
        }
    };
    if (!idle0 || esp_timer_get_time() - g_last_motion < 1500000) last_active = t_;
    if (pending_poke_.exchange(false)) {
        mood_ = MoodIndex("petted"); mood_until_ = t_ + 50; vpop_ += 0.14f; last_active = t_;
        voice("Bean bean is petting your head! React happy and purry in one tiny sentence.");
    }
    if (g_shaken.exchange(false)) {
        mood_ = MoodIndex("dizzy"); mood_until_ = t_ + 85; vpop_ += 0.2f; last_active = t_;
        voice("Bean bean just shook you really hard! React dizzy and silly in one tiny sentence.");
    }
    if (g_lifted.exchange(false) && !mood_until_) {
        mood_ = MoodIndex("surprised"); mood_until_ = t_ + 30; vpop_ += 0.15f; last_active = t_;
    }
    if (g_upside.load()) { mood_ = MoodIndex("shocked"); mood_until_ = t_ + 5; last_active = t_; }
    if (mood_until_ && t_ >= mood_until_) {
        if (MOODS[mood_].eye == SPIRAL) { mood_ = MoodIndex("laughing"); mood_until_ = t_ + 40; }
        else mood_until_ = 0;
    }
    if (!mood_until_) {
        mood_ = base_mood_;
        uint32_t quiet = t_ - last_active;
        if (idle0 && base_mood_ == 0) {
            if (quiet > 5400) mood_ = MoodIndex("relaxed");        // ~3 min: dozing
            else if (quiet > 2700) mood_ = MoodIndex("sleepy");    // ~1.5 min: getting sleepy
        }
    }
    const Mood& m = MOODS[mood_];

    auto state = Application::GetInstance().GetDeviceState();
    bool speaking = state == kDeviceStateSpeaking;
    bool listening = state == kDeviceStateListening;
    bool idle = state == kDeviceStateIdle;

    // springs
    float size = m.size * (listening ? 1.06f : 1.0f);
    Spring(w_, vw_, size); Spring(h_, vh_, size);
    Spring(lid_, vlid_, m.lid, 0.12f, 0.35f); Spring(slant_, vslant_, m.slant, 0.12f, 0.35f);
    Spring(pupil_, vpupil_, m.pupil); Spring(pop_, vpop_, 0.0f, 0.22f, 0.18f);
    blush_amt_ += (m.blush - blush_amt_) * 0.12f;
    uint32_t tc = m.color;
    cr_ += (((tc >> 16) & 255) - cr_) * 0.15f; cg_ += (((tc >> 8) & 255) - cg_) * 0.15f; cb_ += ((tc & 255) - cb_) * 0.15f;

    // where to look: mood, idle glances, tilt
    if (t_ >= next_glance_) {
        bool center = Rand(0, 2) == 0;
        glance_x_ = center ? 0 : (float)Rand(0, 44) - 22; glance_y_ = center ? 0 : (float)Rand(0, 24) - 12;
        next_glance_ = t_ + Rand(55, 150);
    }
    float tx = m.lx, ty = m.ly;
    if (m.lx == 0 && m.ly == 0 && !speaking) { tx = glance_x_; ty = glance_y_; }
    if (g_imu_ok && (fabsf(g_tilt_x) > 0.15f || fabsf(g_tilt_y) > 0.15f)) {
        tx = fmaxf(-30, fminf(30, -g_tilt_y * 45)); ty = fmaxf(-20, fminf(20, g_tilt_x * 35));
    }
    Spring(lx_, vlx_, tx, 0.14f, 0.32f); Spring(ly_, vly_, ty, 0.14f, 0.32f);

    // blinking (never while eyes are closed shapes)
    if (blink_t_ > 0) { if (++blink_t_ > 6) blink_t_ = 0; }
    else if (t_ >= next_blink_) {
        blink_t_ = 1;
        next_blink_ = t_ + (Rand(0, 4) == 0 ? 9 : Rand(70, 190));   // sometimes a double blink
    }

    // ---- wobble physics: the blob rolls with tilt and sloshes when shaken
    {
        static float vx = 0, vy = 0, sq = 0;
        // only her HEAD moves: it leans with tilt and bobbles fast when shaken; the body stays put
        const float room_x = 46, room_y = 26;
        float tgx = 0, tgy = 0;
        if (g_imu_ok) {
            tgx = fmaxf(-room_x, fminf(room_x, -g_tilt_y * 70));
            tgy = fmaxf(-room_y, fminf(room_y, g_tilt_x * 40));
            vx += -g_dyn_y * 45; vy += g_dyn_x * 30;
        }
        if (MOODS[mood_].eye == SPIRAL) { vx += (float)Rand(0, 20) - 10; vy += (float)Rand(0, 10) - 5; }
        if (pop_ > 0.05f) vy -= pop_ * 10;                // little hop on mood changes / pets
        vx += (tgx - G_OX) * 0.25f; vy += (tgy - G_OY) * 0.25f;
        vx *= 0.74f; vy *= 0.74f;
        G_OX += vx; G_OY += vy;
        if (fabsf(G_OX) > room_x) { G_OX = copysignf(room_x, G_OX); sq = fminf(0.10f, fabsf(vx) * 0.015f); vx = -vx * 0.45f; }
        if (fabsf(G_OY) > room_y) { G_OY = copysignf(room_y, G_OY); sq = fminf(0.10f, fabsf(vy) * 0.015f); vy = -vy * 0.45f; }
        if (fabsf(vx) < 0.3f && fabsf(vy) < 0.3f && fabsf(tgx - G_OX) < 0.8f && fabsf(tgy - G_OY) < 0.8f) { G_OX = tgx; G_OY = tgy; vx = vy = 0; }
        sq *= 0.8f;
        float speed = fminf(0.05f, sqrtf(vx * vx + vy * vy) * 0.004f);
        float br = 0;                                           // big shapes stay still (keeps the frame rate high)
        float hox = G_OX, hoy = G_OY; G_OX = 0; G_OY = 0;       // body, paws and tail don't move
        float tw = 290, th2 = 190, ty0 = 410;                   // sitting body
        Box(s_torso, 240 - tw / 2, ty0 - th2 / 2, tw, th2);
        Box(s_torso_glow, 240 - tw / 2 - 7, ty0 - th2 / 2 - 7, tw + 14, th2 + 14);
        for (int i = 0; i < 2; i++) {
            float px = 240 + (i ? 52 : -52), py = 488;
            Box(s_paw[i], px - 40, py - 26, 80, 52);
            Box(s_paw_glow[i], px - 46, py - 32, 92, 64);
        }
        float swish = ((MOODS[mood_].fx & EARWIG) || MOODS[mood_].eye == SPIRAL) ? 26 * sinf(t_ * 0.35f) : 12 * sinf(t_ * 0.08f);
        int a0 = 300 + (int)swish, a1 = 420 + (int)swish;      // tail curls up the right side
        ArcAt(s_tail_glow, 372, 400, 170, a0, a1);
        ArcAt(s_tail, 372, 400, 170, a0 + 3, a1 - 3);
        G_OX = hox; G_OY = hoy;
        float hw = 350 * (1 + sq + speed), hh = 320 * (1 - sq * 0.8f);   // head (moves)
        float hy = 200 + (320 - hh) / 2;
        Box(s_body, 240 - hw / 2, hy - hh / 2, hw, hh);
        Box(s_glow, 240 - hw / 2 - 7, hy - hh / 2 - 7, hw + 14, hh + 14);
        float st = 18 + 5 * fabsf(sinf(t_ * 0.12f)) + ((MOODS[mood_].fx & SPARK) ? 6 : 0);   // forehead star
        Box(s_star[0], 240 - st * 0.22f, 92 + br - st, st * 0.44f, st * 2);
        Box(s_star[1], 240 - st, 92 + br - st * 0.22f, st * 2, st * 0.44f);
    }

    float bob = speaking ? 4 * fabsf(sinf(t_ * 0.45f)) : 0;
    float breathe = 3 * sinf(t_ * 0.07f) - bob;
    float cy = 200 + breathe;
    DrawEye(eye_[0], -1, 240 - 84, cy);
    DrawEye(eye_[1], +1, 240 + 84, cy);

    // ears (wiggle when happy)
    float wig = (m.fx & EARWIG) ? 6 * sinf(t_ * 0.6f) : ((t_ % 160) < 8 ? 3 * sinf(t_ * 1.2f) : 0);
    float ef[2][6] = {{96, 130, 96 - wig, 0 + fabsf(wig), 182, 66}, {384, 130, 384 + wig, 0 + fabsf(wig), 298, 66}};
    for (int i = 0; i < 2; i++) { Line(s_earf[i], s_earf_pts[i], ef[i], 3); Line(s_eari[i], s_eari_pts[i], ef[i], 3); }
    float el[6] = {112, 112, 108 - wig, 30 + fabsf(wig), 160, 80};
    float er[6] = {368, 112, 372 + wig, 30 + fabsf(wig), 320, 80};
    Line(ear_l_, ear_l_pts_, el, 3);
    Line(ear_r_, ear_r_pts_, er, 3);

    // blush + anime hatch lines
    for (int s = 0; s < 2; s++) {
        float bx = s ? 350 : 130, by = 286;
        float bw = 74 + 10 * fminf(1, blush_amt_ - 1);
        Box(blush_[s], bx - bw / 2, by - 15, bw, 30);
        lv_opa_t op = (lv_opa_t)fminf(255, 40 + blush_amt_ * 100);
        if (lv_obj_get_style_bg_opa(blush_[s], LV_PART_MAIN) != op) lv_obj_set_style_bg_opa(blush_[s], op, 0);
        for (int i = 0; i < 3; i++) {
            Show(hatch_[s][i], m.fx & HATCH);
            float x = bx - 22 + i * 18;
            float xy[4] = {x, by + 11, x + 10, by - 11};
            Line(hatch_[s][i], hatch_pts_[s][i], xy, 2);
        }
    }

    // mouth
    MouthMode mm = m.mouth;
    float my = 304 - bob * 0.5f;
    float open_amt = 0;
    if (speaking) {
        open_amt = 0.15f + 0.85f * fabsf(sinf(t_ * 0.85f) * sinf(t_ * 0.23f + 1.0f));
        mm = M_SMILE;
    }
    Show(w_l_, mm == M_W || mm == M_TONGUE); Show(w_r_, mm == M_W || mm == M_TONGUE);
    Show(frown_, mm == M_FROWN); Show(flat_, mm == M_FLAT); Show(tongue_out_, mm == M_TONGUE);
    Show(mouth_, mm == M_SMILE || mm == M_BIG || mm == M_O);
    if (mm == M_W || mm == M_TONGUE) { ArcAt(w_l_, 240 - 13, my - 8, 28, 0, 180); ArcAt(w_r_, 240 + 13, my - 8, 28, 0, 180); }
    if (mm == M_TONGUE) Box(tongue_out_, 240 - 9, my + 2, 18, 18 + 2 * sinf(t_ * 0.3f));
    if (mm == M_FROWN) ArcAt(frown_, 240, my + 16, 44, 205, 335);
    if (mm == M_FLAT) { float xy[4] = {224, my, 256, my}; Line(flat_, flat_pts_, xy, 2); }
    if (mm == M_SMILE || mm == M_BIG || mm == M_O) {
        float mw, mh;
        if (speaking) { mw = 40 + 10 * open_amt; mh = 10 + 30 * open_amt; }
        else if (mm == M_BIG) { mw = 60; mh = 42 + 4 * sinf(t_ * 0.5f); }
        else if (mm == M_O) { mw = 22; mh = 26; }
        else { mw = 46; mh = 24; }
        mouth_open_ += (mh - mouth_open_) * 0.5f;
        Box(mouth_, 240 - mw / 2, my - 6, mw, mouth_open_);
        Radius(mouth_, lroundf(fminf(mw, mouth_open_) / 2 * G_S));
        Show(tongue_in_, mouth_open_ > 20);
        BoxRaw(tongue_in_, mw * 0.22f * G_S, (mouth_open_ - 17) * G_S, (mw * 0.56f - 8) * G_S, 11 * G_S);
    }

    // sparkles
    static const float SP[4][2] = {{58, 176}, {422, 176}, {116, 360}, {364, 360}};
    for (int i = 0; i < 4; i++) {
        bool on = m.fx & SPARK;
        float s = 12 + 12 * fabsf(sinf(t_ * 0.17f + i * 1.7f));
        Show(spark_[i][0], on && s > 13); Show(spark_[i][1], on && s > 13);
        if (on) { Box(spark_[i][0], SP[i][0] - 2.5f, SP[i][1] - s, 5, s * 2); Box(spark_[i][1], SP[i][0] - s, SP[i][1] - 2.5f, s * 2, 5); }
    }
    // tears
    for (int s = 0; s < 2; s++) {
        bool on = (m.fx & STREAM) || ((m.fx & TEARS) && s == 0);
        Show(tear_[s], on);
        if (on) {
            float ex = s ? 240 + 84 + 26 : 240 - 84 - 26;
            if (m.fx & STREAM) { float len = 70 + 20 * sinf(t_ * 0.3f + s); Box(tear_[s], ex - 9, cy + 30, 18, len); }
            else { float f = (t_ * 3 + s * 40) % 110; Box(tear_[s], ex - 7, cy + 50 + f, 14, 20); }
        }
    }
    Show(sweat_, m.fx & SWEAT);
    if (m.fx & SWEAT) Box(sweat_, 386, 92 + 4 * sinf(t_ * 0.25f), 22, 32);
    for (int i = 0; i < 4; i++) {
        Show(anger_[i], m.fx & ANGER);
        if (m.fx & ANGER) {
            float k = 1 + 0.15f * sinf(t_ * 0.5f);
            float ox = (i == 0 || i == 3) ? -12 : 12, oy = i < 2 ? -12 : 12;
            static const int A[4][2] = {{0, 90}, {90, 180}, {180, 270}, {270, 360}};
            ArcAt(anger_[i], 396 + ox * k, 100 + oy * k, 24 * k, A[i][0], A[i][1]);
        }
    }
    for (int i = 0; i < 3; i++) {
        Show(zzz_[i], m.fx & ZZZ);
        if (m.fx & ZZZ) { float q = fmodf(t_ * 0.012f + i / 3.0f, 1.0f); Pos(zzz_[i], 350 + q * 60, 150 - q * 90); }
    }
    Show(qmark_, m.fx & QUES); if (m.fx & QUES) Pos(qmark_, 390, 92 + 5 * sinf(t_ * 0.2f));
    Show(bang_, m.fx & BANG); if (m.fx & BANG) Pos(bang_, 396, 88 - 4 * fabsf(sinf(t_ * 0.4f)));
    (void)idle;
}

#ifdef BEAN_SIM
void SimMotion(float tx, float ty, float dx, float dy, bool shake) {
    g_imu_ok = true; g_tilt_x = tx; g_tilt_y = ty; g_dyn_x = dx; g_dyn_y = dy; if (shake) g_shaken = true;
}
#endif

Face& GetFace() {
    static Face face;
    return face;
}

}  // namespace bean
