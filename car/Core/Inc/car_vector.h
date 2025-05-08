#ifndef CAR_VECTOR_H
#define CAR_VECTOR_H

#include <stdint.h>
#include <math.h>

#define DEFINE_VEC2_OPS(TYPE, SUFFIX)                                              \
    typedef struct {                                                               \
        TYPE x;                                                                    \
        TYPE y;                                                                    \
    } Vec2##SUFFIX;                                                                \
                                                                                   \
    static inline Vec2##SUFFIX vec2##SUFFIX##_add(Vec2##SUFFIX a, Vec2##SUFFIX b) {\
        return (Vec2##SUFFIX){a.x + b.x, a.y + b.y};                               \
    }                                                                              \
                                                                                   \
    static inline Vec2##SUFFIX vec2##SUFFIX##_sub(Vec2##SUFFIX a, Vec2##SUFFIX b) {\
        return (Vec2##SUFFIX){a.x - b.x, a.y - b.y};                               \
    }                                                                              \
                                                                                   \
    static inline Vec2##SUFFIX vec2##SUFFIX##_scale(Vec2##SUFFIX v, TYPE s) {      \
        return (Vec2##SUFFIX){v.x * s, v.y * s};                                   \
    }                                                                              \
                                                                                   \
    static inline Vec2##SUFFIX vec2##SUFFIX##_mul(Vec2##SUFFIX a, Vec2##SUFFIX b) {\
        return (Vec2##SUFFIX){a.x * b.x, a.y * b.y};                               \
    }                                                                              \
                                                                                   \
    static inline uint8_t vec2##SUFFIX##_equal(Vec2##SUFFIX a, Vec2##SUFFIX b) {   \
        return (a.x == b.x) && (a.y == b.y);                                       \
    }                                                                              

#define DEFINE_VEC3_OPS(TYPE, SUFFIX)                                              \
    typedef struct {                                                               \
        TYPE x;                                                                    \
        TYPE y;                                                                    \
        TYPE z;                                                                    \
    } Vec3##SUFFIX;                                                                \
                                                                                   \
    static inline Vec3##SUFFIX vec3##SUFFIX##_add(Vec3##SUFFIX a, Vec3##SUFFIX b) {\
        return (Vec3##SUFFIX){a.x + b.x, a.y + b.y, a.z + b.z};                    \
    }                                                                              \
                                                                                   \
    static inline Vec3##SUFFIX vec3##SUFFIX##_sub(Vec3##SUFFIX a, Vec3##SUFFIX b) {\
        return (Vec3##SUFFIX){a.x - b.x, a.y - b.y, a.z - b.z};                    \
    }                                                                              \
                                                                                   \
    static inline Vec3##SUFFIX vec3##SUFFIX##_scale(Vec3##SUFFIX v, TYPE s) {      \
        return (Vec3##SUFFIX){v.x * s, v.y * s, v.z * s};                          \
    }                                                                              \
    static inline Vec3f vec3##SUFFIX##_scale_f(Vec3##SUFFIX v, float s) {          \
        return (Vec3f){v.x * s, v.y * s, v.z * s};                                 \
    }                                                                              \
    static inline Vec3##SUFFIX vec3##SUFFIX##_mul(Vec3##SUFFIX a, Vec3##SUFFIX b) {\
        return (Vec3##SUFFIX){a.x * b.x, a.y * b.y, a.z * b.z};                    \
    }                                                                              \
    static inline uint8_t vec3##SUFFIX##_equal(Vec3##SUFFIX a, Vec3##SUFFIX b) {   \
        return (a.x == b.x) && (a.y == b.y) && (a.z == b.z);                       \
    }                                                                              \
    static inline TYPE vec3##SUFFIX##_mag(Vec3##SUFFIX v) {                        \
        return (TYPE)sqrt(v.x * v.x + v.y * v.y + v.z * v.z);                      \
    }                                                                              \
    static inline Vec3f vec3##SUFFIX##_conj(Vec3##SUFFIX v) {                      \
        return (Vec3f) {1.0f/v.x, 1.0f/v.y, 1.0f/v.z};                             \
    }

#define DEFINE_TYPE_CONVERT(FROM_TYPE, TO_TYPE, FROM_SUFFIX, TO_SUFFIC)                           \
    static inline Vec2##TO_SUFFIC vec2##TO_SUFFIC##_from_vec2##FROM_SUFFIX(Vec2##FROM_SUFFIX v) { \
        return (Vec2##TO_SUFFIC){(TO_TYPE)v.x, (TO_TYPE)v.y};                                     \
    }                                                                                             \
                                                                                                  \
    static inline Vec3##TO_SUFFIC vec3##TO_SUFFIC##_from_vec3##FROM_SUFFIX(Vec3##FROM_SUFFIX v) { \
        return (Vec3##TO_SUFFIC){(TO_TYPE)v.x, (TO_TYPE)v.y, (TO_TYPE)v.z};                       \
    }                

DEFINE_VEC2_OPS(float, f)
DEFINE_VEC2_OPS(int8_t, i8)
DEFINE_VEC2_OPS(uint8_t, ui8)
DEFINE_VEC2_OPS(int16_t, i16)
DEFINE_VEC2_OPS(int64_t, i64)

DEFINE_VEC3_OPS(float, f)
DEFINE_VEC3_OPS(int16_t, i16)
DEFINE_VEC3_OPS(int64_t, i64)


DEFINE_TYPE_CONVERT(int16_t, float, i16, f)
DEFINE_TYPE_CONVERT(float, int16_t, f, i16)
DEFINE_TYPE_CONVERT(int16_t, int64_t, i16, i64)
DEFINE_TYPE_CONVERT(int64_t, int16_t, i64, i16)

#endif // CAR_VECTOR_H
