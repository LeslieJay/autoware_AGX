// Copyright 2026 BYD. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "reverse_parking_planner/reeds_shepp.hpp"

#include <algorithm>
#include <cmath>

namespace reverse_parking_planner
{

namespace
{
// 数学常量与数值容差
const double PI = M_PI;
const double TWO_PI = 2.0 * PI;
const double ZERO = 10 * std::numeric_limits<double>::epsilon();
}  // namespace

// 18种Reeds-Shepp路径类型定义
const SegmentType ReedsSheppPlanner::path_types_[18][5] = {
  {SegmentType::LEFT, SegmentType::RIGHT, SegmentType::LEFT, SegmentType::NOP, SegmentType::NOP},        // 0
  {SegmentType::RIGHT, SegmentType::LEFT, SegmentType::RIGHT, SegmentType::NOP, SegmentType::NOP},       // 1
  {SegmentType::LEFT, SegmentType::RIGHT, SegmentType::LEFT, SegmentType::RIGHT, SegmentType::NOP},      // 2
  {SegmentType::RIGHT, SegmentType::LEFT, SegmentType::RIGHT, SegmentType::LEFT, SegmentType::NOP},      // 3
  {SegmentType::LEFT, SegmentType::RIGHT, SegmentType::STRAIGHT, SegmentType::LEFT, SegmentType::NOP},   // 4
  {SegmentType::RIGHT, SegmentType::LEFT, SegmentType::STRAIGHT, SegmentType::RIGHT, SegmentType::NOP},  // 5
  {SegmentType::LEFT, SegmentType::STRAIGHT, SegmentType::RIGHT, SegmentType::LEFT, SegmentType::NOP},   // 6
  {SegmentType::RIGHT, SegmentType::STRAIGHT, SegmentType::LEFT, SegmentType::RIGHT, SegmentType::NOP},  // 7
  {SegmentType::LEFT, SegmentType::RIGHT, SegmentType::STRAIGHT, SegmentType::RIGHT, SegmentType::NOP},  // 8
  {SegmentType::RIGHT, SegmentType::LEFT, SegmentType::STRAIGHT, SegmentType::LEFT, SegmentType::NOP},   // 9
  {SegmentType::RIGHT, SegmentType::STRAIGHT, SegmentType::RIGHT, SegmentType::LEFT, SegmentType::NOP},  // 10
  {SegmentType::LEFT, SegmentType::STRAIGHT, SegmentType::LEFT, SegmentType::RIGHT, SegmentType::NOP},   // 11
  {SegmentType::LEFT, SegmentType::STRAIGHT, SegmentType::RIGHT, SegmentType::NOP, SegmentType::NOP},    // 12
  {SegmentType::RIGHT, SegmentType::STRAIGHT, SegmentType::LEFT, SegmentType::NOP, SegmentType::NOP},    // 13
  {SegmentType::LEFT, SegmentType::STRAIGHT, SegmentType::LEFT, SegmentType::NOP, SegmentType::NOP},     // 14
  {SegmentType::RIGHT, SegmentType::STRAIGHT, SegmentType::RIGHT, SegmentType::NOP, SegmentType::NOP},   // 15
  {SegmentType::LEFT, SegmentType::RIGHT, SegmentType::STRAIGHT, SegmentType::LEFT, SegmentType::RIGHT}, // 16
  {SegmentType::RIGHT, SegmentType::LEFT, SegmentType::STRAIGHT, SegmentType::RIGHT, SegmentType::LEFT}  // 17
};

/**
 * @brief 构造一条 Reeds-Shepp 路径对象（初始化为无穷长/无效段）。
 */
ReedsSheppPath::ReedsSheppPath()
  : total_length_(std::numeric_limits<double>::max())
{
  // 默认构造表示“尚未找到有效路径”：
  // - 将 5 段全部标记为 NOP
  // - 段长度初始化为 0
  // - total_length_ 初始化为最大值，便于后续在 computeOptimalPath 中比较并更新
  for (int i = 0; i < 5; ++i) {
    types_[i] = SegmentType::NOP;
    lengths_[i] = 0.0;
  }
}

/**
 * @brief 设置路径段类型及各段长度，并更新总长度。
 * @param types 段类型数组（长度至少为 5）。
 * @param t,u,v,w,x 对应各段的长度（允许为负值，表示逆向）。
 */
void ReedsSheppPath::setSegments(const SegmentType* types, double t, double u, double v, double w, double x)
{
  // 该路径最多包含 5 段；其中 `types_` 决定每一段的运动类型（LEFT/RIGHT/STRAIGHT），
  // `lengths_` 决定每一段的弧长/角度大小（允许为负）。
  // 负长度表示该段以倒车执行（即逆向段），总长度比较时使用 abs 进行求和。
  for (int i = 0; i < 5; ++i) {
    types_[i] = types[i];
  }
  lengths_[0] = t;
  lengths_[1] = u;
  lengths_[2] = v;
  lengths_[3] = w;
  lengths_[4] = x;
  // 使用“各段弧长绝对值之和”作为路径总长度（单位在归一化空间下是角度/弧长比例）。
  total_length_ = std::abs(t) + std::abs(u) + std::abs(v) + std::abs(w) + std::abs(x);
}

/**
 * @brief 构造 Reeds-Shepp 路径规划器。
 * @param turning_radius 最小转弯半径（米）。
 */
ReedsSheppPlanner::ReedsSheppPlanner(double turning_radius)
  : turning_radius_(turning_radius)
{
  // turning_radius_ 决定车辆最小转弯半径，也是路径计算中归一化尺度的来源。
}

/**
 * @brief 将角度归一化到区间 [-pi, pi]。
 */
double ReedsSheppPlanner::mod2pi(double x)
{
  // 目的：把任意角度映射到 [-pi, pi]，避免角度在后续 trig 计算中累积误差。
  // 步骤：
  // 1) 用 fmod 对 2*pi 取余得到候选值 v
  // 2) 若 v 超出 [-pi,pi] 则通过加/减 2*pi 修正到目标区间
  double v = std::fmod(x, TWO_PI);
  if (v < -PI) {
    v += TWO_PI;
  } else if (v > PI) {
    v -= TWO_PI;
  }
  return v;
}

/**
 * @brief 将笛卡尔坐标 (x,y) 转为极坐标 (r,theta)。
 */
void ReedsSheppPlanner::polar(double x, double y, double& r, double& theta)
{
  // (r,theta) 对应平面向量 (x,y)：
  // - r：欧式距离（>=0）
  // - theta：atan2 给出的有符号方位角（范围通常为 [-pi,pi]）
  r = std::sqrt(x * x + y * y);
  theta = std::atan2(y, x);
}

/**
 * @brief tauOmega 计算 Reeds-Shepp 路径参数的中间量。
 * @details 依据 Reeds-Shepp 闭式公式中的相关变换计算 tau 与 omega。
 */
void ReedsSheppPlanner::tauOmega(double u, double v, double xi, double eta, double phi,
                                  double& tau, double& omega)
{
  // 这是闭式公式推导中的中间量计算：
  // - delta：u-v 的角度差（并归一化到 [-pi,pi]）
  // - A/B：由 sin/cos 的差构造出的组合项
  // - t1/t2：用于确定 tau 取哪一个分支（避免得到与几何意义不一致的解）
  double delta = mod2pi(u - v);
  double A = std::sin(u) - std::sin(delta);
  double B = std::cos(u) - std::cos(delta) - 1.0;
  double t1 = std::atan2(eta * A - xi * B, xi * A + eta * B);
  double t2 = 2.0 * (std::cos(delta) - std::cos(v) - std::cos(u)) + 3.0;
  // tau 的分支选择：当 t2 < 0 时，tau 取 t1 + pi；否则取 t1。
  // 两者都再做 mod2pi 归一化，保证输出角落在预期范围内。
  tau = (t2 < 0) ? mod2pi(t1 + PI) : mod2pi(t1);
  // omega 由 tau/u/v/phi 的相对关系得到，并同样归一化到 [-pi,pi]。
  omega = mod2pi(tau - u + v - phi);
}

/**
 * @brief Formula 8.1: L+S+L+（左-直-左-左段的对应求解）。
 * @return 若存在可行解则返回 true，并输出 t,u,v。
 */
bool ReedsSheppPlanner::LpSpLp(double x, double y, double phi, double& t, double& u, double& v) const
{
  // L+S+L：首先把 (x,y,phi) 映射到极坐标求解所需的坐标系；
  // 这里的变换体现为 x - sin(phi)、y - 1 + cos(phi)。
  polar(x - std::sin(phi), y - 1.0 + std::cos(phi), u, t);
  // t 的非负约束（含容差）保证该解析分支对应的段长度为“几何可执行”的形式。
  if (t >= -ZERO) {
    // v 与 phi 和 t 的差关系决定最后一段的弧长符号/大小，并通过 mod2pi 保持连续性。
    v = mod2pi(phi - t);
    if (v >= -ZERO) {
      return true;
    }
  }
  return false;
}

/**
 * @brief Formula 8.2: L+S+R+（左-直-右对应求解）。
 * @return 若存在可行解则返回 true，并输出 t,u,v。
 */
bool ReedsSheppPlanner::LpSpRp(double x, double y, double phi, double& t, double& u, double& v) const
{
  double t1, u1;
  // L+S+R：同样先做坐标变换得到 (u1,t1)；
  // 后续需要 sqrt(u1^2 - 4)，因此这里直接对 u1 平方，便于检查阈值。
  polar(x + std::sin(phi), y - 1.0 - std::cos(phi), u1, t1);
  u1 = u1 * u1;
  // 若 u1^2 < 4，则无实数解（acos/sqrt 等计算会失败），直接返回 false。
  if (u1 >= 4.0) {
    double theta;
    // u 来自 sqrt(u1 - 4)，theta 是用于修正 t 的辅助角度。
    u = std::sqrt(u1 - 4.0);
    theta = std::atan2(2.0, u);
    t = mod2pi(t1 + theta);
    // v 表示末段相对目标朝向的角度差。
    v = mod2pi(t - phi);
    if (t >= -ZERO && v >= -ZERO) {
      return true;
    }
  }
  return false;
}

/**
 * @brief Formula 8.3/8.4: L+R-L-（左-右-左-逆向对应求解）。
 * @return 若存在可行解则返回 true，并输出 t,u,v。
 */
bool ReedsSheppPlanner::LpRmL(double x, double y, double phi, double& t, double& u, double& v) const
{
  double xi = x - std::sin(phi);
  double eta = y - 1.0 + std::cos(phi);
  double r, theta;
  polar(xi, eta, r, theta);
  // r<=4 的约束保证 0.25*r 在 asin 的定义域内，从而保证该分支可计算出实数解。
  if (r <= 4.0) {
    // u 为该分支中间段的弧长（符号由公式决定），t/v 由闭式组合式求得。
    u = -2.0 * std::asin(0.25 * r);
    t = mod2pi(theta + 0.5 * u + PI);
    v = mod2pi(phi - t + u);
    if (t >= -ZERO && u <= ZERO) {
      return true;
    }
  }
  return false;
}

/**
 * @brief Formula 8.7: L+R+L-R-（四段中间包含 tauOmega 求解）。
 * @return 若存在可行解则返回 true，并输出 t,u,v。
 */
bool ReedsSheppPlanner::LpRupLumRm(double x, double y, double phi, double& t, double& u, double& v) const
{
  double xi = x + std::sin(phi);
  double eta = y - 1.0 - std::cos(phi);
  double rho = 0.25 * (2.0 + std::sqrt(xi * xi + eta * eta));
  // rho<=1 才能使 acos(rho) 有实数解。
  if (rho <= 1.0) {
    // u 由 acos(rho) 得到，之后用 tauOmega 计算剩余段关键角量 t 与 v。
    u = std::acos(rho);
    tauOmega(u, -u, xi, eta, phi, t, v);
    if (t >= -ZERO && v <= ZERO) {
      return true;
    }
  }
  return false;
}

/**
 * @brief Formula 8.8: L+R-L-R+（四段中间包含 tauOmega 求解）。
 * @return 若存在可行解则返回 true，并输出 t,u,v。
 */
bool ReedsSheppPlanner::LpRumLumRp(double x, double y, double phi, double& t, double& u, double& v) const
{
  double xi = x + std::sin(phi);
  double eta = y - 1.0 - std::cos(phi);
  double rho = (20.0 - xi * xi - eta * eta) / 16.0;
  // rho 必须落在 [0,1] 以保证 acos(rho) 为实数。
  if (rho >= 0.0 && rho <= 1.0) {
    // u 该分支上取负值分支（由公式推导决定）。
    u = -std::acos(rho);
    // 额外约束 u>=-pi/2，用于保证后续 tauOmega 分支选择正确。
    if (u >= -0.5 * PI) {
      tauOmega(u, u, xi, eta, phi, t, v);
      if (t >= -ZERO && v >= -ZERO) {
        return true;
      }
    }
  }
  return false;
}

/**
 * @brief Formula 8.9: L+R-S-L-（CCSC 族中一类闭式求解）。
 * @return 若存在可行解则返回 true，并输出 t,u,v。
 */
bool ReedsSheppPlanner::LpRmSmLm(double x, double y, double phi, double& t, double& u, double& v) const
{
  double xi = x - std::sin(phi);
  double eta = y - 1.0 + std::cos(phi);
  double r, theta;
  polar(xi, eta, r, theta);
  // r>=2 才能计算 sqrt(r^2 - 4)。
  if (r >= 2.0) {
    // rr 为 sqrt(r^2 - 4)，u/t/v 的闭式解均依赖它。
    double rr = std::sqrt(r * r - 4.0);
    u = 2.0 - rr;
    t = mod2pi(theta + std::atan2(rr, -2.0));
    v = mod2pi(phi - 0.5 * PI - t);
    if (t >= -ZERO && u <= ZERO && v <= ZERO) {
      return true;
    }
  }
  return false;
}

/**
 * @brief Formula 8.10: L+R-S-R-（CCSC 族中一类闭式求解）。
 * @return 若存在可行解则返回 true，并输出 t,u,v。
 */
bool ReedsSheppPlanner::LpRmSmRm(double x, double y, double phi, double& t, double& u, double& v) const
{
  double xi = x + std::sin(phi);
  double eta = y - 1.0 - std::cos(phi);
  double r, theta;
  polar(-eta, xi, r, theta);
  // 同样需要 r>=2 才能保证该解析分支有效（sqrt(r^2-4) 的隐含约束）。
  if (r >= 2.0) {
    // 该分支中 t 直接取 theta，u 与 v 由 r 与 phi 的关系得到。
    t = theta;
    u = 2.0 - r;
    v = mod2pi(t + 0.5 * PI - phi);
    if (t >= -ZERO && u <= ZERO && v <= ZERO) {
      return true;
    }
  }
  return false;
}

/**
 * @brief Formula 8.11: L+R-S-L-R+（CCSCC 族中一类闭式求解）。
 * @return 若存在可行解则返回 true，并输出 t,u,v。
 */
bool ReedsSheppPlanner::LpRmSLmRp(double x, double y, double phi, double& t, double& u, double& v) const
{
  double xi = x + std::sin(phi);
  double eta = y - 1.0 - std::cos(phi);
  double r, theta;
  polar(xi, eta, r, theta);
  // r>=2 才能计算 sqrt(r^2 - 4)，否则 u 将无法得到实数解。
  if (r >= 2.0) {
    u = 4.0 - std::sqrt(r * r - 4.0);
    // u<=0（含容差）表示该分支的中间段符号/几何约束满足要求。
    if (u <= ZERO) {
      t = mod2pi(std::atan2((4.0 - u) * xi - 2.0 * eta, -2.0 * xi + (u - 4.0) * eta));
      v = mod2pi(t - phi);
      if (t >= -ZERO && v >= -ZERO) {
        return true;
      }
    }
  }
  return false;
}

/**
 * @brief 在局部归一化坐标系下，计算到目标位姿的最优 Reeds-Shepp 路径。
 * @param x,y,phi 已归一化（除以转弯半径）后的目标相对位姿。
 * @return 最短的 Reeds-Shepp 路径（包含段类型与段长度）。
 */
ReedsSheppPath ReedsSheppPlanner::computeOptimalPath(double x, double y, double phi) const
{
  ReedsSheppPath best_path;
  double t, u, v;
  
  // 在归一化空间（转弯半径为 1 的坐标系）下枚举不同的 Reeds-Shepp 路径类型，
  // 每个成功的公式都会给出一组段长度（t/u/v/…）；
  // 最终用“total_length_（各段弧长绝对值之和）”选择最短路径。
  auto tryPath = [&](int path_idx, double t_val, double u_val, double v_val, 
                     double w_val = 0.0, double x_val = 0.0) {
    ReedsSheppPath path;
    // path_idx 对应 path_types_ 中的段序列（LEFT/RIGHT/STRAIGHT/NOP）。
    // setSegments 会把段序列与段长度写入 path，并更新 path.length()。
    path.setSegments(path_types_[path_idx], t_val, u_val, v_val, w_val, x_val);
    // best_path 的 total_length_ 初始为最大值，因此首次成功分支会直接更新。
    if (path.length() < best_path.length()) {
      best_path = path;
    }
  };
  
  // CSC paths (公式 8.1, 8.2)
  // 这类路径包含 3 段（C-S-C），通过对 x/y/phi 做镜像翻转可以生成另一侧的对称解。
  if (LpSpLp(x, y, phi, t, u, v)) tryPath(14, t, u, v);
  if (LpSpLp(-x, y, -phi, t, u, v)) tryPath(14, -t, -u, -v);
  if (LpSpLp(x, -y, -phi, t, u, v)) tryPath(15, t, u, v);
  if (LpSpLp(-x, -y, phi, t, u, v)) tryPath(15, -t, -u, -v);
  
  if (LpSpRp(x, y, phi, t, u, v)) tryPath(12, t, u, v);
  if (LpSpRp(-x, y, -phi, t, u, v)) tryPath(12, -t, -u, -v);
  if (LpSpRp(x, -y, -phi, t, u, v)) tryPath(13, t, u, v);
  if (LpSpRp(-x, -y, phi, t, u, v)) tryPath(13, -t, -u, -v);
  
  // CCC paths (公式 8.3, 8.4)
  // 这类路径包含 3 段（C-C-C，全部为转弯弧段），同样利用对称性枚举不同象限解。
  if (LpRmL(x, y, phi, t, u, v)) tryPath(0, t, u, v);
  if (LpRmL(-x, y, -phi, t, u, v)) tryPath(0, -t, -u, -v);
  if (LpRmL(x, -y, -phi, t, u, v)) tryPath(1, t, u, v);
  if (LpRmL(-x, -y, phi, t, u, v)) tryPath(1, -t, -u, -v);
  
  // CCCC paths (公式 8.7, 8.8)
  // 这类路径包含 4 段（C-C-C-C 中间包含不同方向转弯组合）。
  // tryPath 时的符号翻转（如 u -> -u）用于匹配不同段在几何上的方向约束。
  if (LpRupLumRm(x, y, phi, t, u, v)) tryPath(2, t, u, -u, v);
  if (LpRupLumRm(-x, y, -phi, t, u, v)) tryPath(2, -t, -u, u, -v);
  if (LpRupLumRm(x, -y, -phi, t, u, v)) tryPath(3, t, u, -u, v);
  if (LpRupLumRm(-x, -y, phi, t, u, v)) tryPath(3, -t, -u, u, -v);
  
  if (LpRumLumRp(x, y, phi, t, u, v)) tryPath(2, t, u, u, v);
  if (LpRumLumRp(-x, y, -phi, t, u, v)) tryPath(2, -t, -u, -u, -v);
  if (LpRumLumRp(x, -y, -phi, t, u, v)) tryPath(3, t, u, u, v);
  if (LpRumLumRp(-x, -y, phi, t, u, v)) tryPath(3, -t, -u, -u, -v);
  
  // CCSC paths (公式 8.9, 8.10)
  // CCSC 表示 C-C-S-C-S 的特定组合（在实现中对应 4 段路径类型）。
  if (LpRmSmLm(x, y, phi, t, u, v)) tryPath(4, t, -0.5 * PI, u, v);
  if (LpRmSmLm(-x, y, -phi, t, u, v)) tryPath(4, -t, 0.5 * PI, -u, -v);
  if (LpRmSmLm(x, -y, -phi, t, u, v)) tryPath(5, t, -0.5 * PI, u, v);
  if (LpRmSmLm(-x, -y, phi, t, u, v)) tryPath(5, -t, 0.5 * PI, -u, -v);
  
  if (LpRmSmRm(x, y, phi, t, u, v)) tryPath(8, t, -0.5 * PI, u, v);
  if (LpRmSmRm(-x, y, -phi, t, u, v)) tryPath(8, -t, 0.5 * PI, -u, -v);
  if (LpRmSmRm(x, -y, -phi, t, u, v)) tryPath(9, t, -0.5 * PI, u, v);
  if (LpRmSmRm(-x, -y, phi, t, u, v)) tryPath(9, -t, 0.5 * PI, -u, -v);
  
  // CCSCC paths (公式 8.11)
  // 这类路径包含 5 段组合；相应地 tryPath 的 w/x 参数也会被填入固定的 -pi/2 分支值。
  if (LpRmSLmRp(x, y, phi, t, u, v)) tryPath(16, t, -0.5 * PI, u, -0.5 * PI, v);
  if (LpRmSLmRp(-x, y, -phi, t, u, v)) tryPath(16, -t, 0.5 * PI, -u, 0.5 * PI, -v);
  if (LpRmSLmRp(x, -y, -phi, t, u, v)) tryPath(17, t, -0.5 * PI, u, -0.5 * PI, v);
  if (LpRmSLmRp(-x, -y, phi, t, u, v)) tryPath(17, -t, 0.5 * PI, -u, 0.5 * PI, -v);
  
  return best_path;
}

/**
 * @brief 计算两位姿之间的 Reeds-Shepp 距离（最短路径长度）。
 * @param (x0,y0,yaw0) 起点位姿
 * @param (x1,y1,yaw1) 终点位姿
 * @return 最短路径长度（米）。
 */
double ReedsSheppPlanner::distance(double x0, double y0, double yaw0,
                                    double x1, double y1, double yaw1) const
{
  // 转换到以起点为原点的局部坐标系：
  // 1) 平移：dx=x1-x0, dy=y1-y0
  // 2) 旋转：用 yaw0 的 cos/sin 把坐标变到起点航向为 x 轴的系
  double dx = x1 - x0;
  double dy = y1 - y0;
  double c = std::cos(yaw0);
  double s = std::sin(yaw0);
  
  // 归一化：把单位从“米”变成“turning_radius_ 为 1 的尺度”。
  double x = (c * dx + s * dy) / turning_radius_;
  double y = (-s * dx + c * dy) / turning_radius_;
  // phi 是相对航向角，并归一化到 [-pi,pi]。
  double phi = mod2pi(yaw1 - yaw0);
  
  // 在归一化空间计算最短路径的总弧长比例（路径 length()）。
  ReedsSheppPath path = computeOptimalPath(x, y, phi);
  // 再乘以转弯半径恢复到米单位。
  return path.length() * turning_radius_;
}

/**
 * @brief 生成 Reeds-Shepp 最优路径（不插值采样，仅返回段描述）。
 * @return 路径段类型与段长度（t/u/v/w/x；允许逆向，负长度表示后退段）。
 */
ReedsSheppPath ReedsSheppPlanner::planPath(double x0, double y0, double yaw0,
                                            double x1, double y1, double yaw1) const
{
  // 与 distance() 相同的坐标变换与归一化流程；
  // 区别是这里返回的是“最优路径段描述”（段类型 + 段长度）。
  double dx = x1 - x0;
  double dy = y1 - y0;
  double c = std::cos(yaw0);
  double s = std::sin(yaw0);
  
  // 归一化目标相对位姿
  double x = (c * dx + s * dy) / turning_radius_;
  double y = (-s * dx + c * dy) / turning_radius_;
  double phi = mod2pi(yaw1 - yaw0);
  
  return computeOptimalPath(x, y, phi);
}

/**
 * @brief 按路径弧长参数 s 对路径进行插值，得到路径上某一点。
 * @param s 目标弧长距离（米）。
 * @return 对应的路径点（x,y,yaw,是否逆向）。
 */
PathPoint ReedsSheppPlanner::interpolate(double x0, double y0, double yaw0,
                                          const ReedsSheppPath& path, double s) const
{
  double x = x0;
  double y = y0;
  double yaw = yaw0;
  bool is_reverse = false;
  
  // seg 为“归一化弧长比例”，用于判断 s 落在哪一段上。
  // 注意：path.lengths_ 记录的也是归一化空间下的段弧长（角度/弧长比例），
  // 因此需要用 turning_radius_ 做尺度换算。
  double seg = s / turning_radius_;  // 归一化
  
  // 逐段累加，直到找到包含目标点的那一段：
  // - i 段类型由 path.types_ 给出
  // - len 为该段归一化弧长，负号表示该段逆向
  for (int i = 0; i < 5 && path.types_[i] != SegmentType::NOP; ++i) {
    double len = path.lengths_[i];
    double abs_len = std::abs(len);
    
    // 如果目标点落在当前段内部，就对当前段做局部插值并返回。
    if (seg <= abs_len) {
      // 在当前段内
      // d 为当前段在插值位置处的“有符号弧长”：
      // - len>=0 时 d=seg
      // - len<0 时通过取反确保几何计算方向一致
      double d = (len >= 0) ? seg : -seg;
      // 该段是否逆向只由 len 的符号决定（与插值段内位置无关）
      is_reverse = (len < 0);
      
      switch (path.types_[i]) {
        case SegmentType::LEFT:
          x += turning_radius_ * (std::sin(yaw + d) - std::sin(yaw));
          y += turning_radius_ * (-std::cos(yaw + d) + std::cos(yaw));
          yaw = mod2pi(yaw + d);
          break;
        case SegmentType::RIGHT:
          x += turning_radius_ * (-std::sin(yaw - d) + std::sin(yaw));
          y += turning_radius_ * (std::cos(yaw - d) - std::cos(yaw));
          yaw = mod2pi(yaw - d);
          break;
        case SegmentType::STRAIGHT:
          x += turning_radius_ * d * std::cos(yaw);
          y += turning_radius_ * d * std::sin(yaw);
          break;
        default:
          break;
      }
      return PathPoint(x, y, yaw, is_reverse);
    }
    
    // 完成当前段
    double d = len;
    // 当前段完成后，剩余要插值的 seg 需要扣除“该段的实际弧长绝对值”。
    is_reverse = (len < 0);
    
    switch (path.types_[i]) {
      case SegmentType::LEFT:
        x += turning_radius_ * (std::sin(yaw + d) - std::sin(yaw));
        y += turning_radius_ * (-std::cos(yaw + d) + std::cos(yaw));
        yaw = mod2pi(yaw + d);
        break;
      case SegmentType::RIGHT:
        x += turning_radius_ * (-std::sin(yaw - d) + std::sin(yaw));
        y += turning_radius_ * (std::cos(yaw - d) - std::cos(yaw));
        yaw = mod2pi(yaw - d);
        break;
      case SegmentType::STRAIGHT:
        x += turning_radius_ * d * std::cos(yaw);
        y += turning_radius_ * d * std::sin(yaw);
        break;
      default:
        break;
    }
    
    seg -= abs_len;
  }
  
  // 若 s 超过路径总长度（或由于数值误差略超），则返回“末端状态”。
  return PathPoint(x, y, yaw, is_reverse);
}

/**
 * @brief 对给定路径进行离散采样，返回一系列路径点。
 * @param resolution 采样分辨率（米），越小采样点越密集。
 * @return 采样点序列；若路径无效则返回空。
 */
std::vector<PathPoint> ReedsSheppPlanner::samplePath(const ReedsSheppPath& path,
                                                      double x0, double y0, double yaw0,
                                                      double resolution) const
{
  std::vector<PathPoint> points;
  
  if (!path.valid()) {
    return points;
  }
  
  // total_length 使用米单位，便于与 resolution 的米单位匹配。
  double total_length = path.length() * turning_radius_;
  // num_points 向上取整保证最后一个点覆盖到末端，+1 用于包含起点（s=0）。
  int num_points = static_cast<int>(std::ceil(total_length / resolution)) + 1;
  
  for (int i = 0; i <= num_points; ++i) {
    // s 取 min 以避免由于向上取整导致 s 略大于 total_length。
    double s = std::min(static_cast<double>(i) * resolution, total_length);
    points.push_back(interpolate(x0, y0, yaw0, path, s));
  }
  
  return points;
}

}  // namespace reverse_parking_planner
