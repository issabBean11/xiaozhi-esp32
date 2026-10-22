// Bean Bean: live anime kitty face for the 480x480 AMOLED.
// Everything is drawn with plain LVGL shapes (no images), so it is huge, smooth and costs almost no RAM.
#pragma once

#include <lvgl.h>
#include <driver/i2c_master.h>
#include <atomic>

namespace bean {

// Start the motion sensor (QMI8658) on the shared I2C bus. Safe if the chip is missing.
void StartImu(i2c_master_bus_handle_t bus);

// Fur color: 0 = black, 1 = white, 2 = pastel pink (any thread)
void SetFurTheme(int theme);
int GetFurTheme();

class Face {
public:
    void Create(lv_obj_t* parent);          // call with the display lock held
    void SetMood(const char* emotion);      // any thread
    void Poke();                            // tap reaction (any thread)
    lv_obj_t* root() const { return root_; }

private:
    static void TickCb(lv_timer_t* t);
    static void ClickCb(lv_event_t* e);
    void Tick();

    struct Eye {
        lv_obj_t *halo2, *halo1, *iris, *pupil, *hl1, *hl2, *hl3, *lid;
        lv_obj_t *arc;                       // ^ / closed / spiral
        lv_obj_t *l1, *l2;                   // > < and X strokes, heart tip
        lv_obj_t *hc1, *hc2;                 // heart lobes
        lv_obj_t *star_v, *star_h;           // star sparkle eye
        lv_point_precise_t lid_pts[2], p1[3], p2[3];
    };
    void BuildEye(Eye& e);
    void DrawEye(Eye& e, int side, float cx, float cy);

    lv_obj_t* root_ = nullptr;
    lv_obj_t *ear_l_ = nullptr, *ear_r_ = nullptr;
    lv_point_precise_t ear_l_pts_[3], ear_r_pts_[3];
    Eye eye_[2];
    lv_obj_t *blush_[2], *hatch_[2][3];
    lv_point_precise_t hatch_pts_[2][3][2];
    lv_obj_t *w_l_, *w_r_, *frown_, *mouth_, *tongue_in_, *tongue_out_, *flat_;
    lv_point_precise_t flat_pts_[2];
    lv_obj_t *spark_[4][2];
    lv_obj_t *tear_[2], *sweat_, *anger_[4], *zzz_[3], *qmark_, *bang_;

    // animated state (spring physics)
    float w_ = 1, vw_ = 0, h_ = 1, vh_ = 0, lx_ = 0, vlx_ = 0, ly_ = 0, vly_ = 0;
    float lid_ = 0, vlid_ = 0, slant_ = 0, vslant_ = 0, pupil_ = 1, vpupil_ = 0;
    float blush_amt_ = 0.6f, mouth_open_ = 0, pop_ = 0, vpop_ = 0;
    float cr_ = 255, cg_ = 190, cb_ = 70;
    uint32_t t_ = 0;               // ticks
    uint32_t next_blink_ = 60, blink_t_ = 0, next_glance_ = 90;
    float glance_x_ = 0, glance_y_ = 0;
    uint32_t mood_until_ = 0;      // temporary moods (poke, dizzy)
    int mood_ = 0, base_mood_ = 0;
    std::atomic<int> pending_mood_{-1};
    std::atomic<bool> pending_poke_{false};
};

Face& GetFace();

}  // namespace bean
