#ifndef RIGTFORM_H
#define RIGTFORM_H

#include <iostream>
#include <cassert>

#include "matrix4.h"
#include "quat.h"

class RigTForm {
  Cvec3 t_; // translation component
  Quat r_;  // rotation component represented as a quaternion

public:
  RigTForm() : t_(0) {
    assert(norm2(Quat(1,0,0,0) - r_) < CS175_EPS2);
  }

  RigTForm(const Cvec3& t, const Quat& r) {
    // TODO
      t_ = t;
      r_ = r;
  }

  explicit RigTForm(const Cvec3& t) {
    // TODO
      t_ = t;
  }

  explicit RigTForm(const Quat& r) {
    // TODO
      r_ = r;
  }

  Cvec3 getTranslation() const {
    return t_;
  }

  Quat getRotation() const {
    return r_;
  }

  RigTForm& setTranslation(const Cvec3& t) {
    t_ = t;
    return *this;
  }

  RigTForm& setRotation(const Quat& r) {
    r_ = r;
    return *this;
  }

  Cvec4 operator * (const Cvec4& a) const {
    // TODO
      return this->r_ * a + Cvec4(this->t_, 0);
  }

  RigTForm operator * (const RigTForm& a) const {
    // TODO
      Quat new_r = this->r_ * a.r_;
      Cvec4 temp = this->r_ * Cvec4(a.t_[0], a.t_[1], a.t_[2], 1);
      Cvec3 new_t = this->t_ + Cvec3(temp[0], temp[1], temp[2]);
      return RigTForm(new_t, new_r);
  }
};

inline RigTForm inv(const RigTForm& tform) {
  // TODO
    Quat new_r = inv(tform.getRotation());
    Cvec3 lin = tform.getTranslation();
    Cvec4 temp = new_r * Cvec4(lin[0], lin[1], lin[2], 1);
    Cvec3 new_t = - Cvec3(temp[0], temp[1], temp[2]);
    return RigTForm(new_t, new_r);
}

inline RigTForm transFact(const RigTForm& tform) {
  return RigTForm(tform.getTranslation());
}

inline RigTForm linFact(const RigTForm& tform) {
  return RigTForm(tform.getRotation());
}

inline Matrix4 rigTFormToMatrix(const RigTForm& tform) {
  // TODO
    Matrix4 T = Matrix4::makeTranslation(tform.getTranslation());
    Matrix4 R = quatToMatrix(tform.getRotation());
    return T * R;
}
inline Cvec3 lerp(Cvec3 v0, Cvec3 v1, double alpha) {
    return v0 * (1 - alpha) + v1 * alpha;
}

inline Quat slerp(Quat q0, Quat q1, double alpha) {
    if (q0 == q1) return q0;
    return (cn(q1 * inv(q0)) ^ alpha) * q0;
}
inline Cvec3 crlerp(Cvec3 v0, Cvec3 v1, Cvec3 v2, Cvec3 v3, double alpha) {
    const Cvec3 d = (v2 - v0)/6 + v1;
    const Cvec3 e = -(v3 - v1)/6 + v2;
    return v1 * pow(1 - alpha, 3) + d * 3*alpha * pow(1 - alpha, 2) + e *3* (1 - alpha) * pow(alpha, 2) + v2*pow(alpha, 3);
}

inline Quat crslerp(Quat q0, Quat q1, Quat q2, Quat q3, double alpha) {
    Quat d, e;
    if (q0 == q2) d = q1;
    else d = (cn(q2 * inv(q0)) ^ (1 / 6)) * q1;
    if (q1 == q3) e = q2;
    else e = (cn(q3 * inv(q1)) ^ (-1 / 6)) * q2;
    const Quat p01 = slerp(q1, d, alpha);
    const Quat p12 = slerp(d, e, alpha);
    const Quat p23 = slerp(e, q2, alpha);
    const Quat p012 = slerp(p01, p12, alpha);
    const Quat p123 = slerp(p12, p23, alpha);
    const Quat p = slerp(p012, p123, alpha);
    return p;
}

#endif
