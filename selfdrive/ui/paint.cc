#include "selfdrive/ui/paint.h"

#include <cassert>
#include <cmath>

//#define __TEST

#ifdef __APPLE__
#include <OpenGL/gl3.h>
#define NANOVG_GL3_IMPLEMENTATION
#define nvgCreate nvgCreateGL3
#else
#include <GLES3/gl3.h>
#define NANOVG_GLES3_IMPLEMENTATION
#define nvgCreate nvgCreateGLES3
#endif

#define NANOVG_GLES3_IMPLEMENTATION
#include <nanovg_gl.h>
#include <nanovg_gl_utils.h>

#define COLOR_BLACK nvgRGBA(0, 0, 0, 255)
#define COLOR_BLACK_ALPHA(x) nvgRGBA(0, 0, 0, x)
#define COLOR_WHITE nvgRGBA(255, 255, 255, 255)
#define COLOR_WHITE_ALPHA(x) nvgRGBA(255, 255, 255, x)
#define COLOR_RED_ALPHA(x) nvgRGBA(201, 34, 49, x)
#define COLOR_YELLOW nvgRGBA(218, 202, 37, 255)
//#define COLOR_RED nvgRGBA(201, 34, 49, 255)
#define COLOR_RED nvgRGBA(255, 0, 0, 255)
#define COLOR_OCHRE nvgRGBA(218, 111, 37, 255)
#define COLOR_OCHRE_ALPHA(x) nvgRGBA(218, 111, 37, x)
#define COLOR_GREEN nvgRGBA(0, 203, 0, 255)
#define COLOR_GREEN_ALPHA(x) nvgRGBA(0, 153, 0, x)
#define COLOR_BLUE nvgRGBA(0, 0, 255, 255)
#define COLOR_BLUE_ALPHA(x) nvgRGBA(0, 0, 255, x)
#define COLOR_ORANGE nvgRGBA(255, 175, 3, 255)
#define COLOR_ORANGE_ALPHA(x) nvgRGBA(255, 175, 3, x)
#define COLOR_YELLOW_ALPHA(x) nvgRGBA(218, 202, 37, x)
#define COLOR_GREY nvgRGBA(191, 191, 191, 1)

#define BOLD "KaiGenGothicKR-Bold"//"Inter-Bold"//"sans-bold"

#if 0
static void ui_print(UIState *s, int x, int y,  const char* fmt, ... )
{
  char* msg_buf = NULL;
  va_list args;
  va_start(args, fmt);
  vasprintf( &msg_buf, fmt, args);
  va_end(args);
  nvgText(s->vg, x, y, msg_buf, NULL);
}
#endif
static void ui_draw_text(const UIState* s, float x, float y, const char* string, float size, NVGcolor color, const char* font_name, float borderWidth=3.0, float shadowOffset=0.0, NVGcolor borderColor=COLOR_BLACK, NVGcolor shadowColor=COLOR_BLACK) {
    y += 6;
    nvgFontFace(s->vg, font_name);
    nvgFontSize(s->vg, size);
    if (borderWidth > 0.0) {
        //NVGcolor borderColor = COLOR_BLACK;
        nvgFillColor(s->vg, borderColor);
        for (int i = 0; i < 360; i += 45) {
            float angle = i * NVG_PI / 180.0f;
            float offsetX = borderWidth * cos(angle);
            float offsetY = borderWidth * sin(angle);
            nvgText(s->vg, x + offsetX, y + offsetY, string, NULL);
        }
    }
    if (shadowOffset != 0.0) {
        //NVGcolor shadowColor = COLOR_BLACK;
        nvgFillColor(s->vg, shadowColor);
        nvgText(s->vg, x + shadowOffset, y + shadowOffset, string, NULL);
    }
    nvgFillColor(s->vg, color);
    nvgText(s->vg, x, y, string, NULL);
}

static void ui_draw_line(const UIState* s, const QPolygonF& vd, NVGcolor* color, NVGpaint* paint) {
    if (vd.size() == 0) return;

    nvgBeginPath(s->vg);
    nvgMoveTo(s->vg, vd.at(0).x(), vd.at(0).y());
    for (int i = 1; i < vd.size(); i++) {
        nvgLineTo(s->vg, vd.at(i).x(), vd.at(i).y());
    }
    nvgClosePath(s->vg);
    if (color) {
        nvgFillColor(s->vg, *color);
    }
    else if (paint) {
        nvgFillPaint(s->vg, *paint);
    }
    nvgFill(s->vg);
}
static void ui_draw_line2(const UIState* s, float x[], float y[], int size, NVGcolor* color, NVGpaint* paint, float stroke=0.0) {

    nvgBeginPath(s->vg);
    nvgMoveTo(s->vg, x[0], y[0]);
    for (int i = 1; i < size; i++) {
        nvgLineTo(s->vg, x[i], y[i]);
    }
    nvgClosePath(s->vg);
    if (color) {
        nvgFillColor(s->vg, *color);
    }
    else if (paint) {
        nvgFillPaint(s->vg, *paint);
    }
    nvgFill(s->vg);

    if (stroke > 0.0) {
        nvgStrokeColor(s->vg, COLOR_WHITE);
        nvgStrokeWidth(s->vg, stroke);
        nvgStroke(s->vg);
    }

}
static void ui_draw_bsd(const UIState* s, const QPolygonF& vd, NVGcolor* color, bool right) {
    int index = vd.length();

    float x[4], y[4];
    for (int i = 0; i < index/2 - 2; i += 2) {

        if (right) {
            x[0] = vd[i + 0].x();
            y[0] = vd[i + 0].y();
            x[1] = vd[i + 1].x();
            y[1] = vd[i + 1].y();
            x[2] = vd[index - i - 3].x();
            y[2] = vd[index - i - 3].y();
            x[3] = vd[index - i - 2].x();
            y[3] = vd[index - i - 2].y();
        }
        else {
            x[0] = vd[i + 0].x();
            y[0] = vd[i + 0].y();
            x[1] = vd[i + 1].x();
            y[1] = vd[i + 1].y();
            x[2] = vd[index - i - 3].x();
            y[2] = vd[index - i - 3].y();
            x[3] = vd[index - i - 2].x();
            y[3] = vd[index - i - 2].y();
        }
        ui_draw_line2(s, x, y, 4, color, nullptr, 3.0f);
    }

}
#if 0
static void ui_draw_path(const UIState* s) {
    auto plan_position = (*s->sm)["uiPlan"].getUiPlan().getPosition();
    if (plan_position.getX().size() < 33) {
        plan_position = (*s->sm)["modelV2"].getModelV2().getPosition();
    }
    float max_distance = std::clamp(plan_position.getX()[TRAJECTORY_SIZE - 1],
        MIN_DRAW_DISTANCE, MAX_DRAW_DISTANCE);

    auto lead_one = (*s->sm)["radarState"].getRadarState().getLeadOne();
    if (lead_one.getStatus()) {
        const float lead_d = lead_one.getDRel() * 2.;
        max_distance = std::clamp((float)(lead_d - fmin(lead_d * 0.35, 10.)), 0.0f, max_distance);
    }

    // 0~150M까지 display해준다... 
    // 높이시작은 0.8 ~ s->show_z_offset(max_distance)

}
#endif

void ui_draw_image(const UIState* s, const Rect& r, const char* name, float alpha) {
    nvgBeginPath(s->vg);
    NVGpaint imgPaint = nvgImagePattern(s->vg, r.x, r.y, r.w, r.h, 0, s->images.at(name), alpha);
    nvgRect(s->vg, r.x, r.y, r.w, r.h);
    nvgFillPaint(s->vg, imgPaint);
    nvgFill(s->vg);
}
static void ui_draw_circle_image_rotation(const UIState* s, int center_x, int center_y, int radius, const char* image, NVGcolor color, float img_alpha, float angleSteers = 0) {
    const int img_size = radius * 2.0;
    float img_rotation = angleSteers / 180 * 3.141592;
    int ct_pos = -radius;

    nvgBeginPath(s->vg);
    nvgCircle(s->vg, center_x, center_y, radius);
    nvgFillColor(s->vg, color);
    nvgFill(s->vg);
    //ui_draw_image(s, {center_x - (img_size / 2), center_y - (img_size / 2), img_size, img_size}, image, img_alpha);

    nvgSave(s->vg);
    nvgTranslate(s->vg, center_x, center_y);
    nvgRotate(s->vg, -img_rotation);

    ui_draw_image(s, { ct_pos, ct_pos, img_size, img_size }, image, img_alpha);
    nvgRestore(s->vg);
}
void ui_draw_rect(NVGcontext* vg, const Rect& r, NVGcolor color, int width, float radius) {
    nvgBeginPath(vg);
    radius > 0 ? nvgRoundedRect(vg, r.x, r.y, r.w, r.h, radius) : nvgRect(vg, r.x, r.y, r.w, r.h);
    nvgStrokeColor(vg, color);
    nvgStrokeWidth(vg, width);
    nvgStroke(vg);
}
static inline void fill_rect(NVGcontext* vg, const Rect& r, const NVGcolor* color, const NVGpaint* paint, float radius) {
    nvgBeginPath(vg);
    radius > 0 ? nvgRoundedRect(vg, r.x, r.y, r.w, r.h, radius) : nvgRect(vg, r.x, r.y, r.w, r.h);
    if (color) nvgFillColor(vg, *color);
    if (paint) nvgFillPaint(vg, *paint);
    nvgFill(vg);
}
void ui_fill_rect(NVGcontext* vg, const Rect& r, const NVGcolor& color, float radius) {
    fill_rect(vg, r, &color, nullptr, radius);
}
void ui_fill_rect(NVGcontext* vg, const Rect& r, const NVGpaint& paint, float radius) {
    fill_rect(vg, r, nullptr, &paint, radius);
}

static NVGcolor get_tpms_color(float tpms) {
    if (tpms < 5 || tpms > 60) // N/A
        return nvgRGBA(255, 255, 255, 220);
    if (tpms < 31)
        return nvgRGBA(255, 90, 90, 220);
    return nvgRGBA(255, 255, 255, 220);
}

static const char *get_tpms_text(float tpms) {
    if (tpms < 5 || tpms > 60)
        return "  -";

    static char str[32];
    snprintf(str, sizeof(str), "%.0f", round(tpms));
    return str;
}

void DrawApilot::drawLaneLines(const UIState* s) {

    static float pathDrawSeq = 0;


    const UIScene& scene = s->scene;
    SubMaster& sm = *(s->sm);
    NVGcolor color;

    bool left_blindspot = sm["carState"].getCarState().getLeftBlindspot();
    bool right_blindspot = sm["carState"].getCarState().getRightBlindspot();

    // show_lane_info: -1 : all off, 0: path only, 1: path+lane, 2: path+lane+edge
    // lanelines
    if (s->show_lane_info > 0) {
        for (int i = 0; i < std::size(scene.lane_line_vertices); ++i) {
            //color = nvgRGBAf(1.0, 1.0, 1.0, std::clamp<float>(scene.lane_line_probs[i] * 2.0, 0.5, 1.0));
            color = nvgRGBAf(1.0, 1.0, 1.0, (scene.lane_line_probs[i]>0.3)?1.0:0.0);
            ui_draw_line(s, scene.lane_line_vertices[i], &color, nullptr);
        }
    }
    if (s->show_blind_spot) {
#ifdef __TEST
        left_blindspot = right_blindspot  = true;
#endif
        color = nvgRGBA(255, 215, 0, 150);
        if (left_blindspot) ui_draw_bsd(s, scene.lane_barrier_vertices[0], &color, false); // ui_draw_line(s, scene.lane_barrier_vertices[0], &color, nullptr);
        if (right_blindspot) ui_draw_bsd(s, scene.lane_barrier_vertices[1], &color, true); // ui_draw_line(s, scene.lane_barrier_vertices[1], &color, nullptr);
    }

    // road edges
    if (s->show_lane_info > 1) {
        for (int i = 0; i < std::size(scene.road_edge_vertices); ++i) {
            //color = nvgRGBAf(1.0, 0.0, 1.0, std::clamp<float>(3.0 - scene.road_edge_stds[i], 0.0, 1.0));
            color = nvgRGBAf(1.0, 0.0, 1.0, (scene.road_edge_stds[i]<2.0)?1.0:0.0);
            ui_draw_line(s, scene.road_edge_vertices[i], &color, nullptr);
        }
    }
    // paint path
    static bool forward = true;
    if (s->show_lane_info > -1) {
#if 0
        QLinearGradient bg(0, height(), 0, height() / 4);
        float start_hue, end_hue;
        if (sm["controlsState"].getControlsState().getExperimentalMode() || true) {
            const auto& acceleration = sm["modelV2"].getModelV2().getAcceleration();
            float acceleration_future = 0;
            if (acceleration.getZ().size() > 16) {
                acceleration_future = acceleration.getX()[16];  // 2.5 seconds
            }
            start_hue = 60;
            // speed up: 120, slow down: 0
            end_hue = fmax(fmin(start_hue + acceleration_future * 45, 148), 0);

            // FIXME: painter.drawPolygon can be slow if hue is not rounded
            end_hue = int(end_hue * 100 + 0.5) / 100;

            bg.setColorAt(0.0, QColor::fromHslF(start_hue / 360., 0.97, 0.56, 0.4));
            bg.setColorAt(0.5, QColor::fromHslF(end_hue / 360., 1.0, 0.68, 0.35));
            bg.setColorAt(1.0, QColor::fromHslF(end_hue / 360., 1.0, 0.68, 0.0));
        }
        else {
            bg.setColorAt(0.0, QColor::fromHslF(148 / 360., 0.94, 0.51, 0.4));
            bg.setColorAt(0.5, QColor::fromHslF(112 / 360., 1.0, 0.68, 0.35));
            bg.setColorAt(1.0, QColor::fromHslF(112 / 360., 1.0, 0.68, 0.0));
        }
        painter.setBrush(bg);
        painter.drawPolygon(scene.track_vertices);
#else
        if (s->show_path_mode == 0) {
            NVGpaint track_bg;
            float torque_scale = 0.0;
            int red_lvl = fmin(255, torque_scale);
            int green_lvl = fmin(255, 255 - torque_scale);
            track_bg = nvgLinearGradient(s->vg, s->fb_w, s->fb_h, s->fb_w, s->fb_h * .4,
                nvgRGBA(red_lvl, 150, green_lvl, 160), nvgRGBA((int)(0.7 * red_lvl), 150, (int)(0.7 * green_lvl), 30));
            ui_draw_line(s, scene.track_vertices, nullptr, &track_bg);
        }
        else {

            int     track_vertices_len = scene.track_vertices.length();
            //color = nvgRGBA(0, 150, 0, 30);
            int alpha = 120;
            NVGcolor colors[10] = {
                 COLOR_RED_ALPHA(alpha),
                 nvgRGBA(255,153,0, alpha),
                 COLOR_YELLOW_ALPHA(alpha),
                 COLOR_GREEN_ALPHA(alpha),
                 COLOR_BLUE_ALPHA(alpha),
                 nvgRGBA(0,0,128, alpha),
                 nvgRGBA(0x8b,0,0xff, alpha),
                 COLOR_OCHRE_ALPHA(alpha),
                 COLOR_WHITE_ALPHA(alpha),
                 COLOR_BLACK_ALPHA(alpha),
            };
            auto lp = sm["lateralPlan"].getLateralPlan();

            int show_path_mode = (lp.getUseLaneLines())?s->show_path_mode_lane : s->show_path_mode;
            int show_path_color = (lp.getUseLaneLines())?s->show_path_color_lane : s->show_path_color;
            auto    car_state = sm["carState"].getCarState();
            float   accel = car_state.getAEgo();
            float   v_ego = car_state.getVEgoCluster();
            float   v_ego_kph = v_ego * MS_TO_KPH;
#ifdef __TEST
            v_ego_kph = 20.0;
#endif
            float   seq = (1.0 * v_ego_kph / 100.0);
            if (seq < 0.3) seq = 0.3;
            if (accel < -1.0) forward = false;
            if (accel > -0.5) forward = true;

            if (forward) {
                pathDrawSeq += seq;
                if (pathDrawSeq > track_vertices_len / 4 + 3) pathDrawSeq = 0;
            }
            else {
                pathDrawSeq -= seq;
                if (pathDrawSeq < 0) pathDrawSeq = track_vertices_len / 4. + 3;
            }
            float   x[6], y[6];

            switch (show_path_mode) {
            case 1: case 2: case 5: case 6:
                for (int i = 0, color_n = 0; i < track_vertices_len / 2 - 4; i += 2) {
                    x[0] = scene.track_vertices[i].x();
                    y[0] = scene.track_vertices[i].y();
                    x[1] = scene.track_vertices[i + 2].x();
                    y[1] = scene.track_vertices[i + 2].y();
                    x[2] = scene.track_vertices[track_vertices_len - i - 3].x();
                    y[2] = scene.track_vertices[track_vertices_len - i - 3].y();
                    x[3] = scene.track_vertices[track_vertices_len - i - 1].x();
                    y[3] = scene.track_vertices[track_vertices_len - i - 1].y();

                    int draw = false;
                    if ((int)pathDrawSeq == i/2 || (int)pathDrawSeq == i/2-2) draw = true;
                    //if ((int)(pathDrawSeq + 0.5) * 2 == i || (((int)(pathDrawSeq + 0.5) - 2) * 2 == i))  draw = true;
                    if (track_vertices_len / 2 < 8) draw = true;
                    if (show_path_mode == 5 || show_path_mode == 6) draw = true;

                    if (draw) {
                        switch (show_path_mode) {
                        case 2: case 6: ui_draw_line2(s, x, y, 4, &colors[color_n], nullptr, (show_path_color >= 10) ? 2.0 : 0.0); break;
                        default:        ui_draw_line2(s, x, y, 4, &colors[show_path_color % 10], nullptr, (show_path_color >= 10) ? 2.0 : 0.0); break;
                        }
                    }

                    if (i > 1) color_n++;
                    if (color_n > 6) color_n = 0;
                }
                break;
            case 3: case 4: case 7: case 8:
                for (int i = 0, color_n = 0; i < track_vertices_len / 2 - 4; i += 2) {
                    x[0] = scene.track_vertices[i].x();
                    y[0] = scene.track_vertices[i].y();
                    x[1] = scene.track_vertices[i + 2].x();
                    y[1] = scene.track_vertices[i + 2].y();
                    x[2] = (scene.track_vertices[i + 4].x() + scene.track_vertices[track_vertices_len - i - 5].x()) / 2;
                    y[2] = (scene.track_vertices[i + 4].y() + scene.track_vertices[track_vertices_len - i - 5].y()) / 2;
                    x[3] = scene.track_vertices[track_vertices_len - i - 3].x();
                    y[3] = scene.track_vertices[track_vertices_len - i - 3].y();
                    x[4] = scene.track_vertices[track_vertices_len - i - 1].x();
                    y[4] = scene.track_vertices[track_vertices_len - i - 1].y();
                    x[5] = (x[1] + x[3]) / 2;
                    y[5] = (y[1] + y[3]) / 2;

                    int draw = false;
                    if ((int)pathDrawSeq == i / 2 || (int)pathDrawSeq == i / 2 - 2) draw = true;
                    //if ((int)(pathDrawSeq + 0.5) * 2 == i || (((int)(pathDrawSeq + 0.5) - 2) * 2 == i))  draw = true;
                    if (track_vertices_len / 2 < 8) draw = true;
                    if (show_path_mode == 7 || show_path_mode == 8) draw = true;

                    if (draw) {
                        switch (show_path_mode) {
                        case 4: case 8:     ui_draw_line2(s, x, y, 6, &colors[color_n], nullptr, (show_path_color >= 10) ? 2.0 : 0.0); break;
                        default:            ui_draw_line2(s, x, y, 6, &colors[show_path_color % 10], nullptr, (show_path_color >= 10) ? 2.0 : 0.0); break;

                        }
                    }
                    if (i > 1) color_n++;
                    if (color_n > 6) color_n = 0;
                }
                break;
            }
        }
#endif
    }
}
float filter_x = 0.0;
float filter_y = 0.0;

void DrawApilot::drawLeadApilot(const UIState* s) {
    SubMaster& sm = *(s->sm);
    const cereal::ModelDataV2::Reader& model = sm["modelV2"].getModelV2();
    const UIScene& scene = s->scene;
    auto leads = model.getLeadsV3();
#ifndef __TEST
    const cereal::ModelDataV2::LeadDataV3::Reader& lead_data = leads[0];
#endif
    const QPointF& vd = s->scene.lead_vertices[0];
    //bool is_radar = s->scene.lead_radar[0];
    bool no_radar = leads[0].getProb() < .5;
    bool    uiDrawSteeringRotate = s->show_steer_rotate;
    bool    uiDrawPathEnd = s->show_path_end;   // path끝에 표시를 넣을건지..

#ifndef __TEST
    if (!sm.alive("controlsState") || !sm.alive("radarState") || !sm.alive("carControl")) return;
    auto lead_radar = sm["radarState"].getRadarState().getLeadOne();
    auto lead_one = sm["modelV2"].getModelV2().getLeadsV3()[0];
#endif
    auto controls_state = sm["controlsState"].getControlsState();
    auto car_control = sm["carControl"].getCarControl();
    auto car_state = sm["carState"].getCarState();
    int longActiveUser = controls_state.getLongActiveUser();
    int longActiveUserReady = controls_state.getLongActiveUserReady();

    if (s->show_mode == 2) {
        NVGpaint track_bg;
        track_bg = nvgLinearGradient(s->vg, s->fb_w, s->fb_h - 50, s->fb_w, s->fb_h - 350,
            nvgRGBA(0, 0, 0, 250), nvgRGBA(0, 0, 0, 0));
        float x[4], y[4];
        x[0] = 0.0;
        y[0] = s->fb_h;
        x[1] = 0.0;
        y[1] = s->fb_h - 500;
        x[2] = s->fb_w;
        y[2] = s->fb_h - 500;
        x[3] = s->fb_w;
        y[3] = s->fb_h;
        ui_draw_line2(s, x, y, 4, nullptr, &track_bg, 0.0);
    }
    else if (s->show_mode == 3) {
        NVGpaint track_bg;
        track_bg = nvgLinearGradient(s->vg, 0, 0, 0, 150,
            nvgRGBA(0, 0, 0, 250), nvgRGBA(0, 0, 0, 0));
        float x[4], y[4];
        x[0] = 0.0;
        y[0] = 0;
        x[1] = 0.0;
        y[1] = 500;
        x[2] = s->fb_w;
        y[2] = 500;
        x[3] = s->fb_w;
        y[3] = 0;
        ui_draw_line2(s, x, y, 4, nullptr, &track_bg, 0.0);
    }


    // Path의 끝위치를 계산 및 표시
    int     track_vertices_len = scene.track_vertices.length();
    int path_x = s->fb_w / 2;
    int path_y = s->fb_h - 400;
    int path_width = 160;
    int path_bx = path_x;
    //float path_by = path_y;
    //float path_bwidth = path_width;
    {
        if (track_vertices_len >= 10) {
            path_width = scene.track_vertices[track_vertices_len / 2].x() - scene.track_vertices[track_vertices_len / 2 - 1].x();
            int temp_x = (scene.track_vertices[track_vertices_len / 2].x() + scene.track_vertices[track_vertices_len / 2 - 1].x()) / 2.;
            int temp_y = scene.track_vertices[track_vertices_len / 2].y();
            if (temp_x == 0 or temp_y == 0);
            else {
                path_x = temp_x;
                path_y = temp_y;
                //path_bwidth = scene.track_vertices[0].x() - scene.track_vertices[track_vertices_len -1].x();
                path_bx = (scene.track_vertices[0].x() + scene.track_vertices[track_vertices_len - 1].x()) / 2.;
                //path_by = scene.track_vertices[0].y();
                if (uiDrawPathEnd) {
                    ui_fill_rect(s->vg, { path_x - path_width / 2, path_y, path_width, -10 }, COLOR_RED, 5);
                }
            }
        }
    }
    // 과녁을 표시할 위치를 계산
    int icon_size = 256;
#ifdef __TEST
    const int d_rel = 0;
    static int test_seq = 0;
    if (++test_seq > 100) test_seq = 0;
#else
    const int d_rel = lead_data.getX()[0];
#endif
    int x = path_x;
    int y = path_y;
    if (!no_radar) {
        x = std::clamp((float)vd.x(), 550.f, s->fb_w - 550.f);
        y = std::clamp((float)vd.y(), 300.f, s->fb_h - 180.f);
    }

    y -= ((icon_size / 2) - d_rel);
    if (no_radar) {
        //x = path_x;
        x = std::clamp((float)path_x, 300.f, s->fb_w - 300.f);
        y = path_y; // height() - 250;
    }
    if (y > s->fb_h - 400) y = s->fb_h - 400;

    if (s->show_mode == 2) {
        y = s->fb_h - 400;
        x = path_bx;
    }

    filter_x = filter_x * 0.96 + x * 0.04;
    filter_y = filter_y * 0.96 + y * 0.04;
    x = filter_x;
    y = filter_y;
    // 신호등(traffic)그리기.
    // 신호등내부에는 레이더거리, 비젼거리, 정지거리, 신호대기 표시함.
    int circle_size = 160;
    NVGcolor bgColor = nvgRGBA(0, 0, 0, 166);
    const auto lp = sm["longitudinalPlan"].getLongitudinalPlan();
    float stop_dist = 0;
    bool stopping = false;
    auto hud_control = car_control.getHudControl();
#ifdef __TEST
    float radar_dist = 0;
    bool radar_detected = false;
    float vision_dist = 0.0;
#else
    bool radar_detected = lead_radar.getStatus() && lead_radar.getRadar();
    float radar_dist = radar_detected ? lead_radar.getDRel() : 0;
    float vision_dist = lead_one.getProb() > .5 ? (lead_one.getX()[0] - 0) : 0;
#endif
    float disp_dist = (radar_detected) ? radar_dist : vision_dist;
    int brake_hold = car_state.getBrakeHoldActive();
    int soft_hold = (hud_control.getSoftHold()) ? 1 : 0;

    float v_ego = sm["carState"].getCarState().getVEgoCluster();
    float v_ego_kph = v_ego * MS_TO_KPH;
    float cur_speed = v_ego * (s->scene.is_metric ? MS_TO_KPH : MS_TO_MPH);
    bool brake_valid = car_state.getBrakeLights();
    bool bsd_l = car_state.getLeftBlindspot();
    bool bsd_r = car_state.getRightBlindspot();
    // 차로변경/자동턴 표시
    bool showDistInfo = true;
    bool showBg = (disp_dist > 0.0) ? true : false;
    auto lateralPlan = sm["lateralPlan"].getLateralPlan();
    auto desire = lateralPlan.getDesire();
    auto laneChangeDirection = lateralPlan.getLaneChangeDirection();
    auto desireEvent = lateralPlan.getDesireEvent();
    float desireStateTurnLeft = (desire == cereal::LateralPlan::Desire::TURN_LEFT) ? 1 : 0;
    float desireStateTurnRight = (desire == cereal::LateralPlan::Desire::TURN_RIGHT) ? 1 : 0;
    float desireStateLaneChangeLeft = (desire == cereal::LateralPlan::Desire::LANE_CHANGE_LEFT) ? 1 : 0;
    float desireStateLaneChangeRight = (desire == cereal::LateralPlan::Desire::LANE_CHANGE_RIGHT) ? 1 : 0;

    if (desire == cereal::LateralPlan::Desire::NONE) {
        auto meta = sm["modelV2"].getModelV2().getMeta();
        desireStateTurnLeft = meta.getDesireState()[1];
        desireStateTurnRight = meta.getDesireState()[2];
        desireStateLaneChangeLeft = meta.getDesireState()[3];
        desireStateLaneChangeRight = meta.getDesireState()[4];
    }
    bool leftBlinker = car_state.getLeftBlinker();
    bool rightBlinker = car_state.getRightBlinker();
    static int blinkerTimer = 0;
    blinkerTimer = (blinkerTimer + 1) % 16;
    bool blinkerOn = (blinkerTimer <= 16 / 2);
    int trafficState = lp.getTrafficState();
    char    str[128];

    nvgFontFace(s->vg, BOLD);
    nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_BOTTOM);


#ifdef __TEST
    static int _desire = 0.;
    desireStateTurnLeft = 0.0;
    desireStateTurnRight = 0.0;
    desireStateLaneChangeLeft = 0.0;
    desireStateLaneChangeRight = 0.0;

    if (_desire++ > 200) _desire = 0;
    if (_desire > 100) brake_valid = true; else brake_valid = false;
    if (_desire < 50) {
        desireStateTurnLeft = 1.0; 
        trafficState = 0;
    }
    else if (_desire < 100) {
        desireStateTurnRight = 1.0;
        trafficState = 1;
    }
    else if (_desire < 150) {
        desireStateLaneChangeLeft = 1.0;
        trafficState = 2;
    }
    else {
        desireStateLaneChangeRight = 1.0;
        trafficState = 3;
    }
#endif
    if ((desireStateTurnLeft > 0.5) || (desireStateTurnRight > 0.5) || (desireStateLaneChangeLeft > 0.5) || (desireStateLaneChangeRight > 0.5)) {
        showBg = true;
        showDistInfo = false;
    }
    if (true) {  // 핸들표시가 없는경우만 표시, 핸들표시모드인경우, 핸들아이콘으로 표시됨.
#ifdef __TEST
        if (s->show_steer_mode == 2) ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_traffic_green", 1.0f);
#endif
        int trafficMode = 0;
        if (longActiveUser <= 0) trafficMode = 0;  // 크루즈가 꺼져있으면... 신호등을 모두 꺼버려?
        else if (trafficState >= 100) trafficMode = 3; // yellow
        else {
            switch (lp.getTrafficState() % 100) {
            case 0: trafficMode = 0; break;
            case 1: trafficMode = 1;    // red
                stop_dist = lp.getXStop();
                stopping = true;
                if (s->show_steer_mode == 2) ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_traffic_red", 1.0f);
                showBg = true;
                break;
            case 2: trafficMode = 2;
                if (s->show_steer_mode == 2) ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_traffic_green", 1.0f);
                break; // green // 표시안함.
            case 3: trafficMode = 3; showBg = true; break; // yellow
            }
        }
#ifdef __TEST
        static int traffic = 0;
        if (traffic++ > 200) traffic = 0;
        if (traffic < 50) trafficMode = 0;
        else if (traffic < 100) trafficMode = 1;
        else if (traffic < 150) trafficMode = 2;
        else trafficMode = 3;

        if (traffic < 100) { leftBlinker = true; }
        else { rightBlinker = true; }
#endif
        // steer handle 그리기..
        float steer_angle = car_state.getSteeringAngleDeg();
#ifdef __TEST
        static float steer_ang = 0.0;
        steer_ang += 1.0;
        steer_angle = steer_ang;
#endif
        if (s->show_steer_mode == 1) {
            if (uiDrawSteeringRotate) {      // 시간이 많이(3msec)걸려 3번에 한번씩만 그리자..
                switch (trafficMode) {
                case 0: ui_draw_circle_image_rotation(s, x, y, icon_size / 2., "ic_steer_momo", nvgRGBA(0, 0, 0, 0), 0.7f, steer_angle); break;
                case 1: ui_draw_circle_image_rotation(s, x, y, icon_size / 2., "ic_steer_red", nvgRGBA(0, 0, 0, 0), 0.7f, steer_angle); break;
                case 2: ui_draw_circle_image_rotation(s, x, y, icon_size / 2., "ic_steer_green", nvgRGBA(0, 0, 0, 0), 0.7f, steer_angle); break;
                case 3: ui_draw_circle_image_rotation(s, x, y, icon_size / 2., "ic_steer_yellow", nvgRGBA(0, 0, 0, 0), 0.7f, steer_angle); break;
                }
            }
            else ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_steer_momo", 0.7f);
            bgColor = nvgRGBA(0, 0, 0, 160);
        }
        else if (s->show_steer_mode == 0) {            
            if (uiDrawSteeringRotate) {      
                ui_draw_circle_image_rotation(s, x, y, icon_size / 2., "ic_steer_momo", nvgRGBA(0, 0, 0, 0), 0.7f, steer_angle);
            }
            else ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_steer_momo", 0.7f);
            switch (trafficMode) {
            case 0: bgColor = nvgRGBA(0, 0, 0, 90); break;
            case 1: bgColor = nvgRGBA(255, 0, 0, 160); break;
            case 2: bgColor = nvgRGBA(0, 255, 0, 160); break;
            case 3: bgColor = nvgRGBA(255, 255, 0, 160); break;
            }
        }
        else {
            showBg = false;
        }
        if (showBg) {
            nvgBeginPath(s->vg);
            nvgCircle(s->vg, x, y, circle_size/2.);
            nvgFillColor(s->vg, bgColor);
            nvgFill(s->vg);

        }


        // 차로변경, 턴 표시~
        if (true) {            
            if (desireStateTurnLeft > 0.5) ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_turn_l", 1.0f);
            else if (desireStateTurnRight > 0.5) ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_turn_r", 1.0f);
            else if (desireStateLaneChangeLeft > 0.5) ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_lane_change_l", 1.0f);
            else if (desireStateLaneChangeRight > 0.5) ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_lane_change_r", 1.0f);
            if (desireEvent == 57) {
                ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_lane_change_steer", 1.0f);
                ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_lane_change_l", 1.0f);
            }
            else if (desireEvent == 58) {
                ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_lane_change_steer", 1.0f);
                ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_lane_change_r", 1.0f);
            }
            else if (desireEvent == 71) {
                if (laneChangeDirection == cereal::LateralPlan::LaneChangeDirection::LEFT) {
                    ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_lane_change_inhibit", 1.0f);
                    ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_lane_change_l", 1.0f);
                }
                else if (laneChangeDirection == cereal::LateralPlan::LaneChangeDirection::RIGHT) {
                    ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_lane_change_inhibit", 1.0f);
                    ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_lane_change_r", 1.0f);
                }
            }

        }
        // blinker 표시~~
        if (true) {
            if (rightBlinker && blinkerOn) ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_blinker_r", 1.0f);
            if (leftBlinker && blinkerOn) ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_blinker_l", 1.0f);
        }
        // BSD 표시
        if (true) {
            if (bsd_l) ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_bsd_l", 1.0f);
            if (bsd_r) ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, "ic_bsd_r", 1.0f);
        }

#ifdef __TEST
        radar_detected = true;
        disp_dist = 127.0;
        stop_dist = 112.0;
        stopping = true;
#endif

        NVGcolor textColor = COLOR_WHITE;
        NVGcolor borderColor = COLOR_BLACK;

        if (radar_detected) {
#ifdef __TEST
            float radar_rel_speed = 20.0;
            if (test_seq > 50) radar_rel_speed = -20.0;
#else
            float radar_rel_speed = lead_radar.getVRel();
#endif
            sprintf(str, "%.1f km/h", cur_speed + radar_rel_speed * 3.6);
            if (radar_rel_speed < -0.1) {
                textColor = COLOR_RED; 
                borderColor = COLOR_BLACK;
            }
            else {
                textColor = COLOR_GREEN;
                borderColor = COLOR_BLACK;
            }

            nvgFontSize(s->vg, 40);
            if (s->show_steer_mode == 2) {
                int radar_y = (y > s->fb_h - 550) ? s->fb_h - 550 : y - 40;
                if (s->show_mode >= 3) radar_y = y - 145;
                //ui_fill_rect(s->vg, { x - 250 / 2, radar_y - 35, 250, 45 }, bgColor, 15);
                //textColor = COLOR_WHITE;
                ui_draw_text(s, x, radar_y, str, 40, textColor, BOLD, 1.0, 3.0, borderColor, COLOR_BLACK);
            }
            else {
                int radar_y = y - 140;
                //ui_fill_rect(s->vg, { x - 250 / 2, radar_y - 35, 250, 45 }, bgColor, 15);
                //textColor = COLOR_WHITE;
                ui_draw_text(s, x, radar_y, str, 40, textColor, BOLD, 1.0, 3.0, borderColor, COLOR_BLACK);
            }
        }
        ui_draw_image(s, { x - icon_size / 2, y - icon_size / 2, icon_size, icon_size }, (no_radar) ? "ic_radar_no" : (radar_detected) ? "ic_radar" : "ic_radar_vision", 1.0f);

        if (no_radar) {
            if (stop_dist > 0.5 && stopping) {
                if (stop_dist < 10.0) sprintf(str, "%.1f", stop_dist);
                else sprintf(str, "%.0f", stop_dist);
                ui_draw_text(s, x, y+120.0, str, 45, COLOR_WHITE, BOLD);
            }
            else if (longActiveUser > 0 && (stopping || lp.getTrafficState() >= 1000)) {
                if (brake_hold || soft_hold) {
                    //drawTextWithColor(painter, x, y +120, (brake_hold) ? "AUTOHOLD" : "SOFTHOLD", textColor);
                }
                else {
                    sprintf(str, "%s", (lp.getTrafficState() >= 1000) ? "신호오류" : "신호대기");
                    ui_draw_text(s, x, y + 120.0, str, 40, COLOR_WHITE, BOLD);
                }
            }
        }
        else if (disp_dist > 0.0) {
            if (disp_dist < 10.0) sprintf(str, "%.1f", disp_dist);
            else sprintf(str, "%.0f", disp_dist);
            ui_draw_text(s, x, y + 120.0, str, 45, textColor, BOLD);
        }
    }
    // 타겟좌측 : 갭표시
    int myDrivingMode = controls_state.getMyDrivingMode();
    //const auto lp = sm["longitudinalPlan"].getLongitudinalPlan();
    float gap = lp.getCruiseGap();
    //float tFollow = lp.getTFollow();
    int gap1 = controls_state.getLongCruiseGap(); // car_state.getCruiseGap();
#ifdef __TEST
    myDrivingMode = 3;
#endif
    char strDrivingMode[128];
    switch (myDrivingMode)
    {
    case 0: strcpy(strDrivingMode,"GAP"); break;
    case 1: strcpy(strDrivingMode, "연비"); break;// "연비"; break;
    case 2: strcpy(strDrivingMode, "안전"); break;// "안전"; break;
    case 3: strcpy(strDrivingMode, "일반"); break;// "일반"; break;
    case 4: strcpy(strDrivingMode, "고속"); break;// "고속"; break;
    }

    int dxGap = -128 - 10 - 40;
    if(s->show_gap_info >= 0) ui_draw_text(s, x + dxGap + 15, y + 120.0, strDrivingMode, 30, COLOR_WHITE, BOLD);
    dxGap -= 60;
    if (s->show_gap_info > 0) {
#ifdef __TEST
        static int _gap = 0;
        _gap += 10;
        if (_gap > 400) _gap = 0;
        else if (_gap < 100) gap = 1;
        else if (_gap < 200) gap = 2;
        else if (_gap < 300) gap = 3;
        else gap = 4;
#endif
        ui_fill_rect(s->vg, { x + dxGap - 2, (int)(y + 5 + 64), 40, -(int)(std::clamp((float)gap, 0.0f, 4.0f) / 4. * 64) }, COLOR_GREEN, 0);
        ui_draw_rect(s->vg, { x + dxGap, y + 5, 40, 64 / 4 }, COLOR_WHITE, 4, 0);
        ui_draw_rect(s->vg, { x + dxGap, (int)(y + 5 + 64 * 1 / 4.), 40, 64 / 4 }, COLOR_WHITE, 4, 0);
        ui_draw_rect(s->vg, { x + dxGap, (int)(y + 5 + 64 * 2 / 4.), 40, 64 / 4 }, COLOR_WHITE, 4, 0);
        ui_draw_rect(s->vg, { x + dxGap, (int)(y + 5 + 64 * 3 / 4.), 40, 64 / 4 }, COLOR_WHITE, 4, 0);
        ui_draw_text(s, x + dxGap + 20, y+80, "GAP", 25, COLOR_WHITE, BOLD);
    }
    // 갭정보표시 중앙위
    if (s->show_gap_info >= 0) {
        sprintf(str, "%d", gap1);
        ui_draw_text(s, x + dxGap + 15 + 60, y + 60, str, 50, COLOR_WHITE, BOLD);

    }
    // 타겟하단: 롱컨상태표시
    if (true) {
        auto xState = lp.getXState();
        QString qstr;
        if (brake_hold) qstr = "AUTOHOLD";
        else if (longActiveUser > 0) {
            if (xState == cereal::LongitudinalPlan::XState::E2E_STOP) qstr = tr("SIGN DETECTED");
            else if (xState == cereal::LongitudinalPlan::XState::SOFT_HOLD) qstr = "SOFTHOLD";
            else if (xState == cereal::LongitudinalPlan::XState::LEAD) qstr = "LEAD";
            else if (xState == cereal::LongitudinalPlan::XState::E2E_CRUISE) qstr = (v_ego_kph < 80) ? tr("E2ECRUISE") : tr("CRUISE");
            else if (xState == cereal::LongitudinalPlan::XState::CRUISE) qstr = tr("CRUISE");
            else qstr = "UNKNOWN";
        }
        else {
            if (longActiveUserReady > 0) {
                if (xState == cereal::LongitudinalPlan::XState::SOFT_HOLD) qstr = "SOFTHOLD";
                else qstr = tr("CRUISE READY");
            }
            else qstr = tr("MANUAL");
        }
        //ui_fill_rect(s->vg, { x - 250 / 2, y + 140, 250, 45 }, brake_valid? COLOR_RED : COLOR_GREEN, 15);
        if(brake_valid) ui_draw_text(s, x, y + 175, qstr.toStdString().c_str(), 40, COLOR_RED, BOLD, 1.0, 3.0, COLOR_BLACK, COLOR_BLACK);
        else            ui_draw_text(s, x, y + 175, qstr.toStdString().c_str(), 40, COLOR_WHITE, BOLD, 1.0, 3.0, COLOR_BLACK, COLOR_BLACK);

    }
    // Accel표시
    float accel = car_state.getAEgo();
    int dx = 128 + 10;
#ifdef __TEST
    static float accel1 = 0.0;
    accel1 += 0.2;
    if (accel1 > 2.5) accel1 = -2.5;
    accel = accel1;
#endif
    if (s->show_accel > 0) {
        ui_draw_rect(s->vg, { x + dx, y+5, 40, 128 }, COLOR_WHITE, 4, 0);
        ui_fill_rect(s->vg, { x + dx + 2, y + 64 + 5, 36, -(int)(std::clamp((float)accel, -2.0f, 2.0f) / 2. * 64) }, (accel>=0.0)?COLOR_YELLOW:COLOR_RED, 0);
        ui_draw_text(s, x + dx + 20, y + 160, "ACC", 25, COLOR_WHITE, BOLD);
    }

    // RPM표시
    float engineRpm = car_state.getEngineRpm();
    float motorRpm = car_state.getMotorRpm();
#ifdef __TEST
    static float engineRpm1 = 0.0;
    engineRpm1 += 100.0;
    if (engineRpm1 > 4000.0) engineRpm1 = 0.0;
    motorRpm = engineRpm1;
#endif
    if (s->show_accel > 1) {
        dx += 60;
        //str.sprintf("%s: %.0f CHARGE: %.0f%%", (motorRpm > 0.0) ? "MOTOR" : "RPM", (motorRpm > 0.0) ? motorRpm : engineRpm, car_state.getChargeMeter());
        //drawTextWithColor(p, width() - 350, 80, str, textColor);
        //painter.setPen(Qt::NoPen);
        ui_draw_rect(s->vg, { x + dx, y + 5, 40, 128 }, COLOR_WHITE, 4, 0);
        ui_fill_rect(s->vg, { x + dx + 2, y + 128 + 5, 36, -(int)(std::clamp((float)engineRpm>0.0?engineRpm:motorRpm, 0.0f, 4000.0f) / 4000. * 128.0) }, (engineRpm> 0.0) ? COLOR_BLUE : COLOR_GREEN, 0);
        ui_draw_text(s, x + dx + 20, y + 160, "RPM", 25, COLOR_WHITE, BOLD);
    }
    if (s->show_mode == 3) {
        y = -100;
        if (s->fb_w < 1200) x = 650;
        else x = 950;
    }
    else if (s->show_mode == 4) {
        y = 330;
        x = 150;
    }
    else if (s->show_mode == 5) {
        y = 620;
        x = 600;
    }

    // 속도표시
    if (true) {
        const auto road_limit_speed = sm["roadLimitSpeed"].getRoadLimitSpeed();
        const auto car_params = sm["carParams"].getCarParams();

        //bool is_metric = s->scene.is_metric;
        //bool long_control = 1;// scc_smoother.getLongControl();

        // kph
        float applyMaxSpeed = controls_state.getVCruiseOut();// scc_smoother.getApplyMaxSpeed();
        float cruiseMaxSpeed = controls_state.getVCruiseCluster();// scc_smoother.getCruiseMaxSpeed();
        float curveSpeed = controls_state.getCurveSpeed();
        //float xCruiseTarget = lp.getXCruiseTarget() * 3.6;

        //bool is_cruise_set = (cruiseMaxSpeed > 0 && cruiseMaxSpeed < 255);
        //bool is_cruise_set = (applyMaxSpeed > 0 && applyMaxSpeed < 255);
        //int longActiveUser = controls_state.getLongActiveUser();
        int longOverride = car_control.getLongOverride();

        int sccBus = (int)car_params.getSccBus();

        int enabled = controls_state.getEnabled();

        int activeNDA = road_limit_speed.getActive();
        int roadLimitSpeed = road_limit_speed.getRoadLimitSpeed();
        int camLimitSpeed = road_limit_speed.getCamLimitSpeed();
        int camLimitSpeedLeftDist = road_limit_speed.getCamLimitSpeedLeftDist();
        int sectionLimitSpeed = road_limit_speed.getSectionLimitSpeed();
        int sectionLeftDist = road_limit_speed.getSectionLeftDist();

        int limit_speed = 0;
        int left_dist = 0;

        if (camLimitSpeed > 0 && camLimitSpeedLeftDist > 0) {
            limit_speed = camLimitSpeed;
            left_dist = camLimitSpeedLeftDist;
        }
        else if (sectionLimitSpeed > 0 && sectionLeftDist > 0) {
            limit_speed = sectionLimitSpeed;
            left_dist = sectionLeftDist;
        }

        int radar_tracks = Params().getBool("EnableRadarTracks");
        //QString nda_mode_str = QString::fromStdString(Params().get("AutoNaviSpeedCtrl"));
        //int nda_mode = nda_mode_str.toInt();

        sprintf(str, "%s %s %s", (sccBus) ? "SCC2" : "", (activeNDA > 0) ? "NDA" : "", (radar_tracks) ? "RadarTracks" : "");

        //float accel = car_state.getAEgo();
#ifdef __TEST
        static int _ff = 0;
        if (_ff++ > 100) _ff = 0;
        if (_ff > 50) {
            limit_speed = 110;
            left_dist = _ff * 100;
        }
        else {
            roadLimitSpeed = 110;
        }
        cur_speed = 123;
#endif

        if (accel >= 0) {
            //int a = (int)(255.f - (180.f * (accel / 2.f)));
            //a = std::min(a, 255);
            //a = std::max(a, 80);
            //color = QColor(a, a, 255, 230);
        }
        else {
            int a = (int)(255.f - (255.f * (-accel / 3.f)));
            a = std::min(a, 255);
            a = std::max(a, 60);
            //color = QColor(255, a, a, 255);
        }
        if (s->show_conn_info) {
            //ui_draw_text(s, strlen(str) / 2 * 35 / 2 + 50,40, str, 35, COLOR_WHITE, BOLD);
            if (sccBus) ui_draw_image(s, { 30, 20, 120, 54 }, "ic_scc2", 1.0f);
            if (activeNDA) ui_draw_image(s, { 30 + 135, 20, 120, 54 }, "ic_nda", 1.0f);
            if (radar_tracks) ui_draw_image(s, { 30 + 135 * 2, 20, 240, 54 }, "ic_radartracks", 1.0f);
        }

        int bx = x;
        int by = y + 270;

        char speed[128];
        sprintf(speed, "%.0f", cur_speed);
        ui_draw_text(s, bx, by + 50, speed, 120, COLOR_WHITE, BOLD, 3.0f, 8.0f);
        ui_draw_image(s, { bx - 100, by - 60, 350, 150 }, "ic_speed_bg", 1.0f);

        //color = QColor(255, 255, 255, 255);
#ifdef __TEST
        cruiseMaxSpeed = 110;
        enabled = true;
        longActiveUser = 2;
        applyMaxSpeed = 109;
        curveSpeed = 111;
#endif
        if (enabled && (longActiveUser > 0 || (longOverride && blinkerOn))) sprintf(str, "%d", (int)(cruiseMaxSpeed + 0.5));
        else strcpy(str,"--");
        ui_draw_text(s, bx + 170, by + 15, str, 60, COLOR_GREEN, BOLD, 1.0, 5.0, COLOR_BLACK, COLOR_BLACK);
        if (enabled && longActiveUser > 0 && applyMaxSpeed > 0 && applyMaxSpeed != cruiseMaxSpeed) {
            sprintf(str, "%d", (int)(applyMaxSpeed + 0.5));
            ui_draw_text(s, bx + 250, by - 50, str, 50, COLOR_GREEN, BOLD, 1.0, 5.0, COLOR_BLACK, COLOR_BLACK);
        }
        if (true) {
            if (enabled && curveSpeed > 0 && curveSpeed < 150) {
                sprintf(str, "%d", (int)(curveSpeed + 0.5));
                ui_draw_text(s, bx + 140, by + 110, str, 50, COLOR_YELLOW, BOLD, 1.0, 5.0, COLOR_BLACK, COLOR_BLACK);
            }
        }

        bx = x - 200;
        by = y + 250;
        if (s->show_mode == 4) {
            bx = 150;
            by = 380;
        }
        if (limit_speed > 0) {
            nvgBeginPath(s->vg);
            nvgCircle(s->vg, bx, by, 140 / 2);
            nvgFillColor(s->vg, COLOR_WHITE);
            nvgFill(s->vg);
            nvgBeginPath(s->vg);
            nvgCircle(s->vg, bx, by, 130 / 2);
            nvgFillColor(s->vg, COLOR_RED);
            nvgFill(s->vg);
            nvgBeginPath(s->vg);
            nvgCircle(s->vg, bx, by, 110 / 2);
            nvgFillColor(s->vg, COLOR_WHITE);
            nvgFill(s->vg);
            sprintf(str, "%d", limit_speed);
            ui_draw_text(s, bx, by + 25, str, 60, COLOR_BLACK, BOLD, 0.0f, 0.0f);
            if (left_dist > 0) {
                if (left_dist < 1000) sprintf(str, "%d m", left_dist);
                else  sprintf(str, "%.1f km", left_dist / 1000.f);
                ui_draw_text(s, bx, by + 120, str, 40, COLOR_WHITE, BOLD);
            }
        }
        else if (roadLimitSpeed > 0 && roadLimitSpeed < 200) {
            ui_draw_image(s, { bx - 60, by - 50, 120, 150 }, "ic_road_speed", 1.0f);
            sprintf(str, "%d", roadLimitSpeed);
            ui_draw_text(s, bx, by + 75, str, 50, COLOR_BLACK, BOLD, 0.0f, 0.0f);
        }
    }
    // Tpms...
    if (s->show_tpms) {
        int bx = (192 - 24) / 2 + (bdr_s * 2);
        int by = s->fb_h - 280 / 2;
        auto tpms = car_state.getTpms();
        float fl = tpms.getFl();
        float fr = tpms.getFr();
        float rl = tpms.getRl();
        float rr = tpms.getRr();
#ifdef __TEST
        fl = 25;
        fr = 28;
        rl = 32;
        rr = 44;
        //s->show_dm_info = 0;
#endif
        if (s->show_dm_info < 1) {
            int sx = 100;
            int sy = 220;
            bx = 120;
            by = s->fb_h - 100;
            ui_draw_image(s, { bx - sx / 2, by - sy / 2, sx, sy }, "ic_tire", 1.0f);
            //QColor tpmsColor = get_tpms_color(fl);
            //drawTextWithColor(painter, bx - 80, by - 50, get_tpms_text(fl), tpmsColor);
            //tpmsColor = get_tpms_color(fr);
            //drawTextWithColor(painter, bx + 80, by - 50, get_tpms_text(fr), tpmsColor);
            //tpmsColor = get_tpms_color(rl);
            //drawTextWithColor(painter, bx - 80, by + 75, get_tpms_text(rl), tpmsColor);
            //tpmsColor = get_tpms_color(rr);
            //drawTextWithColor(painter, bx + 80, by + 75, get_tpms_text(rr), tpmsColor);
            ui_draw_text(s, bx - 80, by - 50, get_tpms_text(fl), 38, get_tpms_color(fl), BOLD);
            ui_draw_text(s, bx + 80, by - 50, get_tpms_text(fr), 38, get_tpms_color(fr), BOLD);
            ui_draw_text(s, bx - 80, by + 75, get_tpms_text(rl), 38, get_tpms_color(rl), BOLD);
            ui_draw_text(s, bx + 80, by + 75, get_tpms_text(rr), 38, get_tpms_color(rr), BOLD);
        }
        else {
            //int center_x = bx - 30;
            //int center_y = by - 0;
            //int marginX = (int)(rcFont.width() * 3.2f);
            //int marginY = (int)((footer_h / 2 - rcFont.height()) * 0.6f);
            //drawText2(painter, center_x - marginX, center_y - marginY - rcFont.height(), Qt::AlignRight, get_tpms_text(fl), get_tpms_color(fl));
            //drawText2(painter, center_x + marginX, center_y - marginY - rcFont.height(), Qt::AlignLeft, get_tpms_text(fr), get_tpms_color(fr));
            //drawText2(painter, center_x - marginX, center_y + marginY, Qt::AlignRight, get_tpms_text(rl), get_tpms_color(rl));
            //drawText2(painter, center_x + marginX, center_y + marginY, Qt::AlignLeft, get_tpms_text(rr), get_tpms_color(rr));
            ui_draw_text(s, bx - 90, by - 55, get_tpms_text(fl), 38, get_tpms_color(fl), BOLD);
            ui_draw_text(s, bx + 90, by - 55, get_tpms_text(fr), 38, get_tpms_color(fr), BOLD);
            ui_draw_text(s, bx - 90, by + 80, get_tpms_text(rl), 38, get_tpms_color(rl), BOLD);
            ui_draw_text(s, bx + 90, by + 80, get_tpms_text(rr), 38, get_tpms_color(rr), BOLD);
        }

    }
    if (s->show_radar_info) {
        bool disp = false;
        int wStr = 40;
        for (auto const& vrd : s->scene.lead_vertices_ongoing) {
            auto [rx, ry, rd, rv, ry_rel] = vrd;
            disp = true;
            sprintf(str, "%.0f", rv * 3.6);
            wStr = 35 * (strlen(str) + 1);
            ui_fill_rect(s->vg, { (int)(rx - wStr / 2), (int)(ry - 35), wStr, 42 }, COLOR_GREEN, 15);
            ui_draw_text(s, rx, ry, str, 40, COLOR_WHITE, BOLD);
            if (s->show_radar_info == 2) {
                sprintf(str, "%.1f", ry_rel);
                ui_draw_text(s, rx, ry + 40, str, 40, COLOR_WHITE, BOLD);
            }
        }
        for (auto const& vrd : s->scene.lead_vertices_oncoming) {
            auto [rx, ry, rd, rv, ry_rel] = vrd;
            sprintf(str, "%.0f", rv * 3.6);
            wStr = 35 * (strlen(str) + 1);
            ui_fill_rect(s->vg, { (int)rx - wStr / 2, (int)ry - 35, wStr, 42 }, COLOR_RED, 15);
            ui_draw_text(s, rx, ry, str, 40, COLOR_WHITE, BOLD);
            if (s->show_radar_info == 2) {
                sprintf(str, "%.1f", ry_rel);
                ui_draw_text(s, rx, ry + 40, str, 40, COLOR_WHITE, BOLD);
            }
        }
        if (s->show_radar_info == 2) {
            for (auto const& vrd : s->scene.lead_vertices_stopped) {
                auto [rx, ry, rd, rv, ry_rel] = vrd;
                strcpy(str, "*");
                wStr = 35;
                if (true) {
                    ui_fill_rect(s->vg, { (int)rx - wStr / 2, (int)ry - 35, wStr, 42 }, COLOR_BLACK, 15);
                }
            }
        }
    }
    // 시간표시
    if (s->show_datetime) {
        time_t now = time(nullptr);
        struct tm* local = localtime(&now);

        if (s->show_datetime == 1 || s->show_datetime == 2) {
            strftime(str, sizeof(str), "%H:%M", local);
            ui_draw_text(s, 170, 170, str, 100, COLOR_WHITE, BOLD, 3.0f, 8.0f);
        }
        if (s->show_datetime == 1 || s->show_datetime == 3) {
            strftime(str, sizeof(str), "%m-%d-%a", local);
            ui_draw_text(s, 170, 170+70, str, 60, COLOR_WHITE, BOLD, 3.0f, 8.0f);
        }
    }
    v_ego_kph = v_ego_kph;
    brake_valid = brake_valid;
    longActiveUserReady = longActiveUserReady;
}
void DrawApilot::drawDebugText(UIState* s) {
    if (s->fb_w < 1200) return;
    const SubMaster& sm = *(s->sm);
    char  str[128];

    int y = 150;

    const int text_x = 1600;
    const auto live_torque_params = sm["liveTorqueParameters"].getLiveTorqueParameters();

    sprintf(str, "LT[%.0f]:%s (%.4f/%.4f)", live_torque_params.getTotalBucketPoints(), live_torque_params.getLiveValid() ? "ON" : "OFF", live_torque_params.getLatAccelFactorFiltered(), live_torque_params.getFrictionCoefficientFiltered());
    ui_draw_text(s, text_x, y, str, 40, COLOR_WHITE, BOLD, 0.0f, 0.0f);
    //p.drawText(text_x, y + 80, QString::fromStdString(live_torque_params.getDebugText().cStr()));

    //auto controls_state = sm["controlsState"].getControlsState();
    //p.drawText(text_x, y + 160, QString::fromStdString(controls_state.getDebugText2().cStr()));
    //p.drawText(text_x, y + 240, QString::fromStdString(controls_state.getDebugText1().cStr()));
}
DrawApilot::DrawApilot() {

}
DrawApilot* drawApilot;
void ui_draw(UIState *s, int w, int h) {
  // Update intrinsics matrix after possible wide camera toggle change
  if (s->fb_w != w || s->fb_h != h) {
    ui_resize(s, w, h);
  }
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  //glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
  //glBlendFunc(GL_DST_COLOR, GL_ZERO);
  nvgBeginFrame(s->vg, s->fb_w, s->fb_h, 1.0f);
  nvgScissor(s->vg, 0, 0, s->fb_w, s->fb_h);
  drawApilot->drawLaneLines(s);
  drawApilot->drawLeadApilot(s);
  if (s->show_debug) drawApilot->drawDebugText(s);

  //ui_draw_vision(s);
  //dashcam(s);
    //param value
  //nvgFontSize(s->vg, 170);
  //nvgTextAlign(s->vg, NVG_ALIGN_CENTER | NVG_ALIGN_MIDDLE);
  //ui_print(s, s->fb_w/2, s->fb_h/2, "%s", "APILOT");

  nvgResetScissor(s->vg);
  nvgEndFrame(s->vg);
  glDisable(GL_BLEND);
}

void ui_nvg_init(UIState *s) {
  // on EON, we enable MSAA
  s->vg = Hardware::EON() ? nvgCreate(0) : nvgCreate(NVG_ANTIALIAS | NVG_STENCIL_STROKES | NVG_DEBUG);
  //s->vg = nvgCreate(NVG_ANTIALIAS | NVG_STENCIL_STROKES | NVG_DEBUG);
  assert(s->vg);

  // init fonts
  std::pair<const char *, const char *> fonts[] = {
      {"sans-regular", "../assets/fonts/opensans_regular.ttf"},
      {"sans-semibold", "../assets/fonts/opensans_semibold.ttf"},
      {"sans-bold", "../assets/fonts/opensans_bold.ttf"},
      {"KaiGenGothicKR-Normal", "../assets/addon/font/KaiGenGothicKR-Normal.ttf"},
      {"KaiGenGothicKR-Medium", "../assets/addon/font/KaiGenGothicKR-Medium.ttf"},
      {"KaiGenGothicKR-Bold", "../assets/addon/font/KaiGenGothicKR-Bold.ttf"},
      {"Inter-Bold", "../assets/fonts/Inter-Bold.ttf"},
  };
  for (auto [name, file] : fonts) {
    int font_id = nvgCreateFont(s->vg, name, file);
    assert(font_id >= 0);
  }
  // init images
  std::vector<std::pair<const char *, const char *>> images = {
    //{"wheel", "../assets/img_chffr_wheel.png"},
    //{"driver_face", "../assets/img_driver_face.png"},
    //{"speed_bump", "../assets/addon/img/img_speed_bump.png"},
  {"ic_radar", "../assets/images/radar_red.png"},
  {"ic_radar_vision", "../assets/images/radar_vision.png"},
  {"ic_radar_no", "../assets/images/no_radar.png"},
  {"ic_steer_momo", "../assets/images/steer_momo.png"},
  {"ic_steer_red", "../assets/images/steer_red.png"},
  {"ic_steer_green", "../assets/images/steer_green.png"},
  {"ic_steer_yellow", "../assets/images/steer_yellow.png"},
  {"ic_lane_change_l", "../assets/images/lane_change_l.png"},
  {"ic_lane_change_r", "../assets/images/lane_change_r.png"},
  {"ic_lane_change_inhibit", "../assets/images/lane_change_inhibit.png"},
  {"ic_lane_change_steer", "../assets/images/lane_change_steer.png"},
  {"ic_bsd_l", "../assets/images/bsd_l.png"},
  {"ic_bsd_r", "../assets/images/bsd_r.png"},
  {"ic_turn_l", "../assets/images/turn_l.png"},
  {"ic_turn_r", "../assets/images/turn_r.png"},
  {"ic_blinker_l", "../assets/images/blink_l.png"},
  {"ic_blinker_r", "../assets/images/blink_r.png"},
  {"ic_speed_bg", "../assets/images/speed_bg.png"},
  {"ic_traffic_green", "../assets/images/traffic_green.png"},
  {"ic_traffic_red", "../assets/images/traffic_red.png"},
  {"ic_tire", "../assets/images/img_tire.png"},
  {"ic_road_speed", "../assets/images/road_speed.png"},
  {"ic_nda", "../assets/images/img_nda.png"},
  {"ic_navi","../assets/images/img_navi.png"},
  {"ic_scc2", "../assets/images/img_scc2.png"},
  {"ic_radartracks", "../assets/images/img_radartracks.png"},

  };
  for (auto [name, file] : images) {
    s->images[name] = nvgCreateImage(s->vg, file, 1);
    assert(s->images[name] != 0);
  }
  drawApilot = new DrawApilot();
}
void ui_resize(UIState *s, int width, int height) {
  s->fb_w = width;
  s->fb_h = height;
#if 0
  auto intrinsic_matrix = s->wide_camera ? ecam_intrinsic_matrix : fcam_intrinsic_matrix;
  float zoom = ZOOM / intrinsic_matrix.v[0];
  if (s->wide_camera) {
    zoom *= 0.5;
  }

  // Apply transformation such that video pixel coordinates match video
  // 1) Put (0, 0) in the middle of the video
  // 2) Apply same scaling as video
  // 3) Put (0, 0) in top left corner of video
  s->car_space_transform.reset();
  s->car_space_transform.translate(width / 2, height / 2 + y_offset)
      .scale(zoom, zoom)
      .translate(-intrinsic_matrix.v[2], -intrinsic_matrix.v[5]);
#endif
}
