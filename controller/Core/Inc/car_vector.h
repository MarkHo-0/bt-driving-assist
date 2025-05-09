#ifndef CAR_VECTOR_H
#define CAR_VECTOR_H

#include <stdint.h> 

// 2D Vector (8-bit signed integer)
typedef struct {
    int8_t x;
    int8_t y;
} Vec2i8;

// 2D Vector (8-bit unsigned integer)
typedef struct {
    uint8_t x;
    uint8_t y;
} Vec2ui8;

// 2D Vector (16-bit signed integer)
typedef struct {
    uint16_t x;
    uint16_t y;
} Vec2i16;


// 2D Vector (32-bit signed integer)
typedef struct {
    int32_t x;
    int32_t y;
} Vec2i32;

// 2D Vector (float)
typedef struct {
    float x;
    float y;
} Vec2f;

// 3D Vector (float)
typedef struct {
    float x;
    float y;
    float z;
} Vec3f;

// --- Function Declarations ---

// Vec2i8 Functions
Vec2i8 vec2i8_add(Vec2i8 a, Vec2i8 b);
Vec2i8 vec2i8_subtract(Vec2i8 a, Vec2i8 b);
Vec2i8 vec2i8_scale(Vec2i8 v, int8_t scalar);
uint8_t vec2i8_equal(Vec2i8 a, Vec2i8 b);

// Vec2ui8 Functions
Vec2ui8 vec2ui8_add(Vec2ui8 a, Vec2ui8 b);
Vec2ui8 vec2ui8_subtract(Vec2ui8 a, Vec2ui8 b);
Vec2ui8 vec2ui8_scale(Vec2ui8 v, uint8_t scalar);

// Vec2ui16 Functions
Vec2i16 vec2ui16_add(Vec2i16 a, Vec2i16 b);
Vec2i16 vec2ui16_subtract(Vec2i16 a, Vec2i16 b);
Vec2i16 vec2ui16_scale(Vec2i16 v, uint16_t scalar);

// Vec2i32 Functions
Vec2i32 vec2i32_add(Vec2i32 a, Vec2i32 b);
Vec2i32 vec2i32_subtract(Vec2i32 a, Vec2i32 b);
Vec2i32 vec2i32_scale(Vec2i32 v, int32_t scalar);

// Vec2f Functions
Vec2f vec2f_add(Vec2f a, Vec2f b);
Vec2f vec2f_subtract(Vec2f a, Vec2f b);
Vec2f vec2f_scale(Vec2f v, float scalar);

// Vec3f Functions
Vec3f vec3f_add(Vec3f a, Vec3f b);
Vec3f vec3f_subtract(Vec3f a, Vec3f b);
Vec3f vec3f_scale(Vec3f v, float scalar);

#endif // VECTOR_H