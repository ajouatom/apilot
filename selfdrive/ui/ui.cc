#include "selfdrive/ui/ui.h"

#include <algorithm>
#include <cassert>
#include <cmath>

#include <QtConcurrent>

#include "common/transformations/orientation.hpp"
#include "common/params.h"
#include "common/swaglog.h"
#include "common/util.h"
#include "common/watchdog.h"
#include "system/hardware/hw.h"

#define BACKLIGHT_DT 0.05
#define BACKLIGHT_TS 10.00

// Projects a point in car to space to the corresponding point in full frame
// image space.
static bool calib_frame_to_full_frame(const UIState *s, float in_x, float in_y, float in_z, QPointF *out) {
  const float margin = 500.0f;
  const QRectF clip_region{-margin, -margin, s->fb_w + 2 * margin, s->fb_h + 2 * margin};

  const vec3 pt = (vec3){{in_x, in_y, in_z}};
  const vec3 Ep = matvecmul3(s->scene.wide_cam ? s->scene.view_from_wide_calib : s->scene.view_from_calib, pt);
  const vec3 KEp = matvecmul3(s->scene.wide_cam ? ECAM_INTRINSIC_MATRIX : FCAM_INTRINSIC_MATRIX, Ep);

  // Project.
  QPointF point = s->car_space_transform.map(QPointF{KEp.v[0] / KEp.v[2], KEp.v[1] / KEp.v[2]});
  if (clip_region.contains(point)) {
    *out = point;
    if(std::isnan(point.x()) || std::isnan(point.y())) return false;
    return true;
  }
  return false;
}

int get_path_length_idx(const cereal::XYZTData::Reader &line, const float path_height) {
  const auto line_x = line.getX();
  int max_idx = 0;
  for (int i = 1; i < line_x.size() && line_x[i] <= path_height; ++i) {
    max_idx = i;
  }
  return max_idx;
}

void update_leads(UIState *s, const cereal::RadarState::Reader &radar_state, const cereal::XYZTData::Reader &line) {
    //SubMaster& sm = *(s->sm);
    //auto lead_one = sm["modelV2"].getModelV2().getLeadsV3()[0];    
  float max_distance = s->scene.max_distance;
  int idx = get_path_length_idx(line, max_distance);
  float y = line.getY()[idx];
  float z = line.getZ()[idx];
  for (int i = 0; i < 2; ++i) {
    auto lead_data = (i == 0) ? radar_state.getLeadOne() : radar_state.getLeadTwo();
    if (lead_data.getStatus()) {
      //float z = line.getZ()[get_path_length_idx(line, lead_data.getDRel())];
      z = line.getZ()[get_path_length_idx(line, lead_data.getDRel())];
      calib_frame_to_full_frame(s, lead_data.getDRel(), -lead_data.getYRel(), z + 1.22, &s->scene.lead_vertices[i]);
      //calib_frame_to_full_frame(s, lead_data.getDRel(), (i == 0) ? lead_one.getY()[0] : -lead_data.getYRel(), z + 1.22, &s->scene.lead_vertices[i]);
      s->scene.lead_radar[i] = lead_data.getRadar();
      max_distance = lead_data.getDRel();
      y = -lead_data.getYRel();
    }
    else
      s->scene.lead_radar[i] = false;
      
    calib_frame_to_full_frame(s, max_distance, y - 1.2, z + 1.22, &s->scene.path_end_left_vertices[i]);
    calib_frame_to_full_frame(s, max_distance, y + 1.2, z + 1.22, &s->scene.path_end_right_vertices[i]);
  }

  s->scene.lead_vertices_side.clear();
  for (auto const& rs : { radar_state.getLeadsLeft(), radar_state.getLeadsRight(), radar_state.getLeadsCenter() }) {
      for (auto const& l : rs) {
          lead_vertex_data vd;
          QPointF vtmp;
          z = line.getZ()[get_path_length_idx(line, l.getDRel())];
          calib_frame_to_full_frame(s, l.getDRel(), -l.getYRel(), z + 0.61, &vtmp);
          vd.x = vtmp.x();
          vd.y = vtmp.y();
          vd.d = l.getDRel();
          vd.v = l.getVLeadK();
          vd.y_rel = l.getYRel();
          vd.v_lat = l.getVLat();
          s->scene.lead_vertices_side.push_back(vd);
      }
  }
}
template <class T>
float interp(float x, std::initializer_list<T> x_list, std::initializer_list<T> y_list, bool extrapolate)
{
    std::vector<T> xData(x_list);
    std::vector<T> yData(y_list);
    int size = xData.size();

    int i = 0;
    if (x >= xData[size - 2]) {
        i = size - 2;
    }
    else {
        while (x > xData[i + 1]) i++;
    }
    T xL = xData[i], yL = yData[i], xR = xData[i + 1], yR = yData[i + 1];
    if (!extrapolate) {
        if (x < xL) yR = yL;
        if (x > xR) yL = yR;
    }

    T dydx = (yR - yL) / (xR - xL);
    return yL + dydx * (x - xL);
}
template <class T>
float interp(float x, const T* x_list, const T* y_list, size_t size, bool extrapolate)
{
    int i = 0;
    if (x >= x_list[size - 2]) {
        i = size - 2;
    }
    else {
        while (x > x_list[i + 1]) i++;
    }
    T xL = x_list[i], yL = y_list[i], xR = x_list[i + 1], yR = y_list[i + 1];
    if (!extrapolate) {
        if (x < xL) yR = yL;
        if (x > xR) yL = yR;
    }

    T dydx = (yR - yL) / (xR - xL);
    return yL + dydx * (x - xL);
}
void update_line_data(const UIState* s, const cereal::XYZTData::Reader& line,
    float y_off, float z_off_left, float z_off_right, QPolygonF* pvd, int max_idx, bool allow_invert = true, float y_shift = 0.0) {
    const auto line_x = line.getX(), line_y = line.getY(), line_z = line.getZ();
    QPolygonF left_points, right_points;
    left_points.reserve(max_idx + 1);
    right_points.reserve(max_idx + 1);

    //printf("%.1f,%.1f,%.1f\n", line_x[0], line_y[0], line_z[0]);

    for (int i = 0; i <= max_idx; i++) {
        // highly negative x positions  are drawn above the frame and cause flickering, clip to zy plane of camera
        if (line_x[i] < 0) continue;
        QPointF left, right;
        bool l = calib_frame_to_full_frame(s, line_x[i], line_y[i] - y_off + y_shift, line_z[i] + z_off_left, &left);
        bool r = calib_frame_to_full_frame(s, line_x[i], line_y[i] + y_off + y_shift, line_z[i] + z_off_right, &right);
        if (l && r) {
            // For wider lines the drawn polygon will "invert" when going over a hill and cause artifacts
            if (!allow_invert && left_points.size() && left.y() > left_points.back().y()) {
                continue;
            }
            left_points.push_back(left);
            right_points.push_front(right);
        }
    }
    *pvd = left_points + right_points;
}
void update_line_data2(const UIState* s, const cereal::XYZTData::Reader& line,
    float width_apply, float z_off_start, float z_off_end, QPolygonF* pvd, int max_idx, bool allow_invert = true) {
    const auto line_x = line.getX(), line_y = line.getY(), line_z = line.getZ();
    QPolygonF left_points, right_points;
    left_points.reserve(max_idx + 1);
    right_points.reserve(max_idx + 1);

    for (int i = 0; i <= max_idx; i++) {
        // highly negative x positions  are drawn above the frame and cause flickering, clip to zy plane of camera
        if (line_x[i] < 0) continue;
        float z_off = interp<float>((float)line_x[i], { 0.0f, 100.0 }, { z_off_start, z_off_end }, false);
        float y_off = interp<float>(z_off, { -3.0f, 0.0f, 3.0f }, { 1.5f, 0.5f, 1.5f }, false);
        y_off *= width_apply;

        QPointF left, right;
        bool l = calib_frame_to_full_frame(s, line_x[i], line_y[i] - y_off, line_z[i] + z_off, &left);
        bool r = calib_frame_to_full_frame(s, line_x[i], line_y[i] + y_off, line_z[i] + z_off, &right);
        if (l && r) {
            // For wider lines the drawn polygon will "invert" when going over a hill and cause artifacts
            if (!allow_invert && left_points.size() && left.y() > left_points.back().y()) {
                continue;
            }
            left_points.push_back(left);
            right_points.push_front(right);
        }
    }
    *pvd = left_points + right_points;
}
void update_line_data_dist(const UIState* s, const cereal::XYZTData::Reader& line,
    float width_apply, float z_off_start, float z_off_end, QPolygonF* pvd, float max_dist, bool allow_invert = true) {
    const auto line_x = line.getX(), line_y = line.getY(), line_z = line.getZ();
    QPolygonF left_points, right_points;
    left_points.reserve(40 + 1);
    right_points.reserve(40 + 1);
    float idxs[33], line_xs[33], line_ys[33], line_zs[33];
    float   x_prev = 0;
    for (int i = 0; i < 33; i++) {
        idxs[i] = (float)i;
        if (i>0 && line_x[i] < x_prev) {
            //printf("plan data error.\n");
            line_xs[i] = x_prev;
        }
        else line_xs[i] = line_x[i];
        x_prev = line_xs[i];
        line_ys[i] = line_y[i];
        line_zs[i] = line_z[i]; 
    }

    float   dist = 2.0, dist_dt = 1.;
    bool    exit = false;
    //printf("\ndist = ");
    for (int i = 0; !exit; i++, dist = dist + dist*0.15) {
        dist_dt += (i*0.05);
        if (dist >= max_dist) {
            dist = max_dist;
            exit = true;
        }
        //printf("%.0f ", dist);
        float z_off = interp<float>(dist, { 0.0f, 100.0 }, { z_off_start, z_off_end }, false);
        float y_off = interp<float>(z_off, { -3.0f, 0.0f, 3.0f }, { 1.5f, 0.5f, 1.5f }, false);
        y_off *= width_apply;
        float  idx = interp<float>(dist, line_xs, idxs, 33, false);
        if (idx >= 33) break;
        float line_y1 = interp<float>(idx, idxs, line_ys, 33, false);
        float line_z1 = interp<float>(idx, idxs, line_zs, 33, false);

        QPointF left, right;
        bool l = calib_frame_to_full_frame(s, dist, line_y1 - y_off, line_z1 + z_off, &left);
        bool r = calib_frame_to_full_frame(s, dist, line_y1 + y_off, line_z1 + z_off, &right);
        if (l && r) {
            // For wider lines the drawn polygon will "invert" when going over a hill and cause artifacts
            if (!allow_invert && left_points.size() && left.y() > left_points.back().y()) {
                //printf("invert...\n");
                continue;
            }
            left_points.push_back(left);
            right_points.push_front(right);
        }
        //else printf("range out..\n");
    }
    *pvd = left_points + right_points;
}
float dist_function(float t, float max_dist) {
    float dist = 3.0 * pow(1.2, t);
    return (dist >= max_dist)? max_dist: dist;
}

void update_line_data_dist3(const UIState* s, const cereal::XYZTData::Reader& line,
    float width_apply, float z_off_start, float z_off_end, QPolygonF* pvd, float max_dist, bool allow_invert = true) {
    const auto line_x = line.getX(), line_y = line.getY(), line_z = line.getZ();
    QPolygonF left_points, right_points;
    left_points.reserve(40 + 1);
    right_points.reserve(40 + 1);


    // comma의 데이터는 x,y,z 점들로 이루어짐. 
    float idxs[33], line_xs[33], line_ys[33], line_zs[33];
    float   x_prev = 0;
    for (int i = 0; i < 33; i++) {
        idxs[i] = (float)i;
        if (i > 0 && line_x[i] < x_prev) {
            //printf("plan data error.\n");
            line_xs[i] = x_prev;
        }
        else line_xs[i] = line_x[i];
        x_prev = line_xs[i];
        line_ys[i] = line_y[i];
        line_zs[i] = line_z[i];
    }
    SubMaster& sm = *(s->sm);
    auto lp = sm["lateralPlan"].getLateralPlan();
    //int show_path_color = (lp.getUseLaneLines()) ? s->show_path_color_lane : s->show_path_color;
    int show_path_mode = (lp.getUseLaneLines()) ? s->show_path_mode_lane : s->show_path_mode;
    auto controls_state = sm["controlsState"].getControlsState();
    int longActiveUser = controls_state.getLongActiveUser();
    if (longActiveUser <= 0) {
        show_path_mode = s->show_path_mode_cruise_off;
    }
    auto    car_state = sm["carState"].getCarState();

    //float   accel = car_state.getAEgo();
    float   v_ego = car_state.getVEgoCluster();
    float   v_ego_kph = v_ego * MS_TO_KPH;

    static float    pos_t = 0.0;
    float           pos_t_start = 0.0;
    float           pos_t_max = 24.0;

    //v_ego_kph = 60.;
    float   dt = (v_ego_kph * 0.01);
    float   dt_max = 1.0;
    if (show_path_mode >= 10) dt_max = 0.6;
    if (dt > dt_max) dt = dt_max;
    else {
        if (v_ego_kph < 1) pos_t = 4.0;
        else if (dt < 0.2) dt = 0.2;
    }
    pos_t += dt;
    if (pos_t > pos_t_max) pos_t = pos_t_start + (pos_t - pos_t_max); 

    float   d, t;
    float   draw_t[50];
    int     draw_t_n = 0;
    float   dist = 0.0;

    if (show_path_mode == 9) {
        draw_t[draw_t_n++] = pos_t;
        d = 3.0;
        t = draw_t[draw_t_n - 1];
        draw_t[draw_t_n++] = ((t + d) > pos_t_max) ? (t + d) - pos_t_max : t + d;
        d = 10.0;
        t = draw_t[draw_t_n - 1];
        draw_t[draw_t_n++] = ((t + d) > pos_t_max) ? (t + d) - pos_t_max : t + d;
        d = 3.0;
        t = draw_t[draw_t_n - 1];
        draw_t[draw_t_n++] = ((t + d) > pos_t_max) ? (t + d) - pos_t_max : t + d;
    }
    else if (show_path_mode == 10) {
        draw_t[draw_t_n++] = pos_t;
        for (int i = 0; i < 7; i++) {
            d = 3.0;
            t = draw_t[draw_t_n - 1];
            draw_t[draw_t_n++] = ((t + d) > pos_t_max) ? (t + d) - pos_t_max : t + d;
        }
    }
    else if (show_path_mode == 11) {
        draw_t[draw_t_n++] = pos_t;
        for (int i = 0; i < 5; i++) {
            d = 3.0;
            t = draw_t[draw_t_n - 1];
            draw_t[draw_t_n++] = ((t + d) > pos_t_max) ? (t + d) - pos_t_max : t + d;
        }
    }
    else if (show_path_mode == 12) {
        draw_t[draw_t_n++] = pos_t;
        int n = (int)(v_ego_kph * 0.058 - 0.5);
        if (n < 0) n = 0;
        else if (n > 7) n = 7;
        
        for (int i = 0; i < n; i++) {
            d = 3.0;
            t = draw_t[draw_t_n - 1];
            draw_t[draw_t_n++] = ((t + d) > pos_t_max) ? (t + d) - pos_t_max : t + d;
        }
    }

    int     draw_t_idx = 0;
    float   temp = draw_t[0];
    for (int i = 0; i < draw_t_n; i++) {
        if (draw_t[i] < temp) {
            draw_t_idx = i;
            temp = draw_t[i];
        }
    }
    bool exit = false;
    for (int i = 0; i <= draw_t_n && !exit; i++) {
        if(i==draw_t_n) exit = true;
        else {
            t = draw_t[draw_t_idx];
            draw_t_idx = (draw_t_idx + 1) % draw_t_n;
            if (t < 3.0) continue;
            if (dist_function(t, max_dist) == max_dist) exit = true;
        }
        for (int j = 2; j >= 0; j--) {
            if (exit) dist = dist_function(100, max_dist);
            else dist = dist_function(t - j * 1.0, max_dist);
            float z_off = interp<float>(dist, { 0.0f, 100.0 }, { z_off_start, z_off_end }, false);
            float y_off = interp<float>(z_off, { -3.0f, 0.0f, 3.0f }, { 1.5f, 0.5f, 1.5f }, false);
            y_off *= width_apply;
            float  idx = interp<float>(dist, line_xs, idxs, 33, false);
            if (idx >= 33) {
                printf("index... %.1f\n", idx);
                break;
            }
            float line_y1 = interp<float>(idx, idxs, line_ys, 33, false);
            float line_z1 = interp<float>(idx, idxs, line_zs, 33, false);

            QPointF left, right;
            bool l = calib_frame_to_full_frame(s, dist, line_y1 - y_off, line_z1 + z_off, &left);
            bool r = calib_frame_to_full_frame(s, dist, line_y1 + y_off, line_z1 + z_off, &right);
            if (l && r) {
                // For wider lines the drawn polygon will "invert" when going over a hill and cause artifacts
                //if (!allow_invert && left_points.size() && left.y() > left_points.back().y()) {
                //    printf("iiiii dist=%.1f, t=%.1f, i=%d, j=%d\n", dist, t, i,j);
                    //continue;
                //}
                left_points.push_back(left);
                right_points.push_front(right);
            }
            //else printf("calib_frame_to_full_frame.... error\n");
            if (exit) break;
        }
    }
    *pvd = left_points + right_points;
}

void update_model(UIState *s,
                  const cereal::ModelDataV2::Reader &model,
                  const cereal::UiPlan::Reader &plan) {
  UIScene &scene = s->scene;
  auto plan_position = plan.getPosition();
  if (plan_position.getX().size() < model.getPosition().getX().size()) {
    plan_position = model.getPosition();
  }
  float max_distance = std::clamp(*(plan_position.getX().end() - 1),
                                  MIN_DRAW_DISTANCE, MAX_DRAW_DISTANCE);

  auto lead_one = (*s->sm)["radarState"].getRadarState().getLeadOne();
  if (lead_one.getStatus()) {
      //const float lead_d = lead_one.getDRel() * 2.;
      //max_distance = std::clamp((float)(lead_d - fmin(lead_d * 0.35, 10.)), 0.0f, max_distance);
      const float lead_d = lead_one.getDRel();
      max_distance = std::clamp((float)lead_d, 0.0f, max_distance);
  }
  scene.max_distance = max_distance;

  // update lane lines
  const auto lane_lines = model.getLaneLines();
  const auto lane_line_probs = model.getLaneLineProbs();
  int max_idx = get_path_length_idx(lane_lines[0], max_distance);
  for (int i = 0; i < std::size(scene.lane_line_vertices); i++) {
    scene.lane_line_probs[i] = lane_line_probs[i];
    update_line_data(s, lane_lines[i], 0.07, 0.0, 0.0, &scene.lane_line_vertices[i], max_idx);
  }
  
  // lane barriers for blind spot
#if 0
  int max_distance_barrier =  40;
  int max_idx_barrier = std::min(max_idx, get_path_length_idx(lane_lines[0], max_distance_barrier));
  update_line_data(s, lane_lines[1], 0, -0.05, -0.6, &scene.lane_barrier_vertices[0], max_idx_barrier, false);
  update_line_data(s, lane_lines[2], 0, -0.05, -0.6, &scene.lane_barrier_vertices[1], max_idx_barrier, false);
#else
  int max_idx_barrier = get_path_length_idx(plan_position, 40.0);
  update_line_data(s, plan_position, 0, 1.2 - 0.05, 1.2 - 0.6, &scene.lane_barrier_vertices[0], max_idx_barrier, false, -1.7); // 차선폭을 알면 좋겠지만...
  update_line_data(s, plan_position, 0, 1.2 - 0.05, 1.2 - 0.6, &scene.lane_barrier_vertices[1], max_idx_barrier, false, 1.7);
#endif

  // update road edges
  const auto road_edges = model.getRoadEdges();
  const auto road_edge_stds = model.getRoadEdgeStds();
  int max_idx_road_edge = get_path_length_idx(lane_lines[0], 100);
  for (int i = 0; i < std::size(scene.road_edge_vertices); i++) {
    scene.road_edge_stds[i] = road_edge_stds[i];
    update_line_data(s, road_edges[i], 0.15, 0.0, 0.0, &scene.road_edge_vertices[i], max_idx_road_edge);
  }

  // update path
//  auto lead_one = (*s->sm)["radarState"].getRadarState().getLeadOne();
//  if (lead_one.getStatus()) {
//    const float lead_d = lead_one.getDRel() * 2.;
//    max_distance = std::clamp((float)(lead_d - fmin(lead_d * 0.35, 10.)), 0.0f, max_distance);
//  }
  SubMaster& sm = *(s->sm);
  auto lp = sm["lateralPlan"].getLateralPlan();
  //int show_path_color = (lp.getUseLaneLines()) ? s->show_path_color_lane : s->show_path_color;
  int show_path_mode = (lp.getUseLaneLines()) ? s->show_path_mode_lane : s->show_path_mode;
  auto controls_state = sm["controlsState"].getControlsState();
  int longActiveUser = controls_state.getLongActiveUser();
  if (longActiveUser <= 0) show_path_mode = s->show_path_mode_cruise_off;

  max_idx = get_path_length_idx(plan_position, max_distance);
  if (show_path_mode == 0 || s->show_mode == 0) {
      //update_line_data(s, plan_position, s->show_path_width, s->show_z_offset, s->show_z_offset, &scene.track_vertices, max_idx, false);
      update_line_data2(s, plan_position, s->show_path_width, 0.8, s->show_z_offset, &scene.track_vertices, max_idx);
  }
  else if(show_path_mode < 9 || show_path_mode == 13 || show_path_mode == 14 || show_path_mode == 15)
    update_line_data_dist(s, plan_position, s->show_path_width, 0.8, s->show_z_offset, &scene.track_vertices, max_distance, false);
  else
    update_line_data_dist3(s, plan_position, s->show_path_width, 0.8, s->show_z_offset, &scene.track_vertices, max_distance, false);
}

void update_dmonitoring(UIState *s, const cereal::DriverStateV2::Reader &driverstate, float dm_fade_state, bool is_rhd) {
  UIScene &scene = s->scene;
  const auto driver_orient = is_rhd ? driverstate.getRightDriverData().getFaceOrientation() : driverstate.getLeftDriverData().getFaceOrientation();
  for (int i = 0; i < std::size(scene.driver_pose_vals); i++) {
    float v_this = (i == 0 ? (driver_orient[i] < 0 ? 0.7 : 0.9) : 0.4) * driver_orient[i];
    scene.driver_pose_diff[i] = fabs(scene.driver_pose_vals[i] - v_this);
    scene.driver_pose_vals[i] = 0.8 * v_this + (1 - 0.8) * scene.driver_pose_vals[i];
    scene.driver_pose_sins[i] = sinf(scene.driver_pose_vals[i]*(1.0-dm_fade_state));
    scene.driver_pose_coss[i] = cosf(scene.driver_pose_vals[i]*(1.0-dm_fade_state));
  }

  const mat3 r_xyz = (mat3){{
    scene.driver_pose_coss[1]*scene.driver_pose_coss[2],
    scene.driver_pose_coss[1]*scene.driver_pose_sins[2],
    -scene.driver_pose_sins[1],

    -scene.driver_pose_sins[0]*scene.driver_pose_sins[1]*scene.driver_pose_coss[2] - scene.driver_pose_coss[0]*scene.driver_pose_sins[2],
    -scene.driver_pose_sins[0]*scene.driver_pose_sins[1]*scene.driver_pose_sins[2] + scene.driver_pose_coss[0]*scene.driver_pose_coss[2],
    -scene.driver_pose_sins[0]*scene.driver_pose_coss[1],

    scene.driver_pose_coss[0]*scene.driver_pose_sins[1]*scene.driver_pose_coss[2] - scene.driver_pose_sins[0]*scene.driver_pose_sins[2],
    scene.driver_pose_coss[0]*scene.driver_pose_sins[1]*scene.driver_pose_sins[2] + scene.driver_pose_sins[0]*scene.driver_pose_coss[2],
    scene.driver_pose_coss[0]*scene.driver_pose_coss[1],
  }};

  // transform vertices
  for (int kpi = 0; kpi < std::size(default_face_kpts_3d); kpi++) {
    vec3 kpt_this = default_face_kpts_3d[kpi];
    kpt_this = matvecmul3(r_xyz, kpt_this);
    scene.face_kpts_draw[kpi] = (vec3){{(float)kpt_this.v[0], (float)kpt_this.v[1], (float)(kpt_this.v[2] * (1.0-dm_fade_state) + 8 * dm_fade_state)}};
  }
}

static void update_sockets(UIState *s) {
  s->sm->update(0);
}

static void update_state(UIState *s) {
  SubMaster &sm = *(s->sm);
  UIScene &scene = s->scene;

  if (sm.updated("liveCalibration")) {
    auto live_calib = sm["liveCalibration"].getLiveCalibration();
    auto rpy_list = live_calib.getRpyCalib();
    auto wfde_list = live_calib.getWideFromDeviceEuler();
    Eigen::Vector3d rpy;
    Eigen::Vector3d wfde;
    if (rpy_list.size() == 3) rpy << rpy_list[0], rpy_list[1], rpy_list[2];
    if (wfde_list.size() == 3) wfde << wfde_list[0], wfde_list[1], wfde_list[2];
    Eigen::Matrix3d device_from_calib = euler2rot(rpy);
    Eigen::Matrix3d wide_from_device = euler2rot(wfde);
    Eigen::Matrix3d view_from_device;
    view_from_device << 0, 1, 0,
                        0, 0, 1,
                        1, 0, 0;
    Eigen::Matrix3d view_from_calib = view_from_device * device_from_calib;
    Eigen::Matrix3d view_from_wide_calib = view_from_device * wide_from_device * device_from_calib;
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        scene.view_from_calib.v[i*3 + j] = view_from_calib(i, j);
        scene.view_from_wide_calib.v[i*3 + j] = view_from_wide_calib(i, j);
      }
    }
    scene.calibration_valid = live_calib.getCalStatus() == cereal::LiveCalibrationData::Status::CALIBRATED;
    scene.calibration_wide_valid = wfde_list.size() == 3;
  }
  if (sm.updated("pandaStates")) {
    auto pandaStates = sm["pandaStates"].getPandaStates();
    if (pandaStates.size() > 0) {
      scene.pandaType = pandaStates[0].getPandaType();

      if (scene.pandaType != cereal::PandaState::PandaType::UNKNOWN) {
        scene.ignition = false;
        for (const auto& pandaState : pandaStates) {
          scene.ignition |= pandaState.getIgnitionLine() || pandaState.getIgnitionCan();
        }
      }
    }
  } else if ((s->sm->frame - s->sm->rcv_frame("pandaStates")) > 5*UI_FREQ) {
    scene.pandaType = cereal::PandaState::PandaType::UNKNOWN;
  }
  if (sm.updated("carParams")) {
    scene.longitudinal_control = sm["carParams"].getCarParams().getOpenpilotLongitudinalControl();
  }
  if (sm.updated("wideRoadCameraState")) {
    auto cam_state = sm["wideRoadCameraState"].getWideRoadCameraState();
    float scale = (cam_state.getSensor() == cereal::FrameData::ImageSensor::AR0231) ? 6.0f : 1.0f;
    scene.light_sensor = std::max(100.0f - scale * cam_state.getExposureValPercent(), 0.0f);
  }
  scene.started = sm["deviceState"].getDeviceState().getStarted() && scene.ignition;
}

void ui_update_params(UIState *s) {
  auto params = Params();
  s->scene.is_metric = params.getBool("IsMetric");
  s->scene.map_on_left = params.getBool("NavSettingLeftSide");
  
  static int updateSeq = 0;
  if (updateSeq++ > 100) updateSeq = 0;
  switch(updateSeq) {
  case 0:
      s->scene.is_metric = params.getBool("IsMetric");
      s->scene.map_on_left = params.getBool("NavSettingLeftSide");
      s->show_debug = params.getBool("ShowDebugUI");
      break;
  case 10:
      s->show_datetime = std::atoi(params.get("ShowDateTime").c_str());
      s->show_mode = std::atoi(params.get("ShowHudMode").c_str());
      s->show_steer_rotate = std::atoi(params.get("ShowSteerRotate").c_str());
      break;
  case 20:
      s->show_path_end = std::atoi(params.get("ShowPathEnd").c_str());;
      s->show_accel = std::atoi(params.get("ShowAccelRpm").c_str());;
      s->show_tpms = std::atoi(params.get("ShowTpms").c_str());;
      break;
  case 30:
      s->show_steer_mode = std::atoi(params.get("ShowSteerMode").c_str());;
      s->show_device_stat = std::atoi(params.get("ShowDeviceState").c_str());;
      s->show_conn_info = std::atoi(params.get("ShowConnInfo").c_str());;
      break;
  case 40:
      s->show_lane_info = std::atoi(params.get("ShowLaneInfo").c_str());;
      s->show_blind_spot = std::atoi(params.get("ShowBlindSpot").c_str());;
      s->show_gap_info = std::atoi(params.get("ShowGapInfo").c_str());;
      break;
  case 50:
      s->show_dm_info = std::atoi(params.get("ShowDmInfo").c_str());;
      s->show_radar_info = std::atoi(params.get("ShowRadarInfo").c_str());;
      s->show_z_offset = std::atof(params.get("ShowZOffset").c_str()) / 100.;
      break;
  case 60:
      s->show_path_mode = std::atoi(params.get("ShowPathMode").c_str());;
      s->show_path_color = std::atoi(params.get("ShowPathColor").c_str());;
      s->show_path_mode_lane = std::atoi(params.get("ShowPathModeLane").c_str());;
      break;
  case 70:
      s->show_path_color_lane = std::atoi(params.get("ShowPathColorLane").c_str());;
      s->show_path_width = std::atof(params.get("ShowPathWidth").c_str()) / 100.;
      s->show_plot_mode = std::atoi(params.get("ShowPlotMode").c_str());
      break;
  case 80:
      s->show_path_mode_cruise_off = std::atoi(params.get("ShowPathModeCruiseOff").c_str());;
      s->show_path_color_cruise_off = std::atoi(params.get("ShowPathColorCruiseOff").c_str());;
      break;
  }
 }

void UIState::updateStatus() {
  if (scene.started && sm->updated("controlsState")) {
    auto controls_state = (*sm)["controlsState"].getControlsState();
    auto alert_status = controls_state.getAlertStatus();
    auto state = controls_state.getState();
    if (alert_status == cereal::ControlsState::AlertStatus::USER_PROMPT) {
      status = STATUS_WARNING;
    } else if (alert_status == cereal::ControlsState::AlertStatus::CRITICAL) {
      status = STATUS_ALERT;
    } else if (state == cereal::ControlsState::OpenpilotState::PRE_ENABLED || state == cereal::ControlsState::OpenpilotState::OVERRIDING) {
      status = STATUS_OVERRIDE;
    } else {
        //status = controls_state.getEnabled() ? STATUS_ENGAGED : STATUS_DISENGAGED;
        status = controls_state.getEnabled() ? (controls_state.getLongActiveUser() <= 0 ? STATUS_CRUISE_STOP: STATUS_ENGAGED) : STATUS_DISENGAGED;
    }
  }

  // Handle onroad/offroad transition
  if (scene.started != started_prev || sm->frame == 1) {
    if (scene.started) {
      status = STATUS_DISENGAGED;
      scene.started_frame = sm->frame;
    }
    started_prev = scene.started;
    emit offroadTransition(!scene.started);
  }
}

UIState::UIState(QObject *parent) : QObject(parent) {
  sm = std::make_unique<SubMaster, const std::initializer_list<const char *>>({
    "modelV2", "controlsState", "liveCalibration", "radarState", "deviceState", "roadCameraState",
    "pandaStates", "carParams", "driverMonitoringState", "carState", "liveLocationKalman", "driverStateV2",
    "wideRoadCameraState", "managerState", "navInstruction", "navRoute", "uiPlan",
    "lateralPlan", "longitudinalPlan", "gpsLocationExternal", "carControl", "liveParameters", "roadLimitSpeed",
    "liveTorqueParameters",
  });

  Params params;
  language = QString::fromStdString(params.get("LanguageSetting"));
  auto prime_value = params.get("PrimeType");
  if (!prime_value.empty()) {
    prime_type = static_cast<PrimeType>(std::atoi(prime_value.c_str()));
  }

  // update timer
  timer = new QTimer(this);
  QObject::connect(timer, &QTimer::timeout, this, &UIState::update);
  timer->start(1000 / UI_FREQ);
}

void UIState::update() {
  update_sockets(this);
  update_state(this);
  updateStatus();

  if (sm->frame % UI_FREQ == 0) {
    watchdog_kick(nanos_since_boot());
  }
  emit uiUpdate(*this);
}

void UIState::setPrimeType(PrimeType type) {
    type = PrimeType::LITE;
  if (type != prime_type) {
    bool prev_prime = hasPrime();

    prime_type = type;
    Params().put("PrimeType", std::to_string(prime_type));
    emit primeTypeChanged(prime_type);

    bool prime = hasPrime();
    if (prev_prime != prime) {
      emit primeChanged(prime);
    }
  }
}

Device::Device(QObject *parent) : brightness_filter(BACKLIGHT_OFFROAD, BACKLIGHT_TS, BACKLIGHT_DT), QObject(parent) {
  setAwake(true);
  resetInteractiveTimeout();

  QObject::connect(uiState(), &UIState::uiUpdate, this, &Device::update);
}

void Device::update(const UIState &s) {
  updateBrightness(s);
  updateWakefulness(s);
}

void Device::setAwake(bool on) {
  if (on != awake) {
    awake = on;
    Hardware::set_display_power(awake);
    LOGD("setting display power %d", awake);
    emit displayPowerChanged(awake);
  }
}

void Device::resetInteractiveTimeout(int timeout) {
  if (timeout == -1) {
    timeout = (ignition_on ? 10 : 30);
  }
  interactive_timeout = timeout * UI_FREQ;
}

void Device::updateBrightness(const UIState &s) {
  float clipped_brightness = offroad_brightness;
  if (s.scene.started) {
    clipped_brightness = s.scene.light_sensor;

    // CIE 1931 - https://www.photonstophotos.net/GeneralTopics/Exposure/Psychometric_Lightness_and_Gamma.htm
    if (clipped_brightness <= 8) {
      clipped_brightness = (clipped_brightness / 903.3);
    } else {
      clipped_brightness = std::pow((clipped_brightness + 16.0) / 116.0, 3.0);
    }

    // Scale back to 10% to 100%
    clipped_brightness = std::clamp(100.0f * clipped_brightness, 10.0f, 100.0f);
  }

  int brightness = brightness_filter.update(clipped_brightness);
  if (!awake) {
    brightness = 0;
  }

  if (brightness != last_brightness) {
    if (!brightness_future.isRunning()) {
      brightness_future = QtConcurrent::run(Hardware::set_brightness, brightness);
      last_brightness = brightness;
    }
  }
}

void Device::updateWakefulness(const UIState &s) {
  bool ignition_just_turned_off = !s.scene.ignition && ignition_on;
  ignition_on = s.scene.ignition;

  if (ignition_just_turned_off) {
    resetInteractiveTimeout();
  } else if (interactive_timeout > 0 && --interactive_timeout == 0) {
    emit interactiveTimeout();
  }

  setAwake(s.scene.ignition || interactive_timeout > 0);
}

UIState *uiState() {
  static UIState ui_state;
  return &ui_state;
}

Device *device() {
  static Device _device;
  return &_device;
}
