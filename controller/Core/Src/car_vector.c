#include "car_vector.h"

// --- Vec2i8 Function Implementations ---

Vec2i8 vec2i8_add(Vec2i8 a, Vec2i8 b) {
    Vec2i8 result = {a.x + b.x, a.y + b.y};
    return result;
}

Vec2i8 vec2i8_subtract(Vec2i8 a, Vec2i8 b) {
    Vec2i8 result = {a.x - b.x, a.y - b.y};
    return result;
}

Vec2i8 vec2i8_scale(Vec2i8 v, int8_t scalar) {
    Vec2i8 result = {v.x * scalar, v.y * scalar};
    return result;
}

uint8_t vec2i8_equal(Vec2i8 a, Vec2i8 b) {
    return (a.x == b.x && a.y == b.y);
}   

// --- Vec2ui8 Function Implementations ---

Vec2ui8 vec2ui8_add(Vec2ui8 a, Vec2ui8 b) {
    Vec2ui8 result = {a.x + b.x, a.y + b.y};
    return result;
}

Vec2ui8 vec2ui8_subtract(Vec2ui8 a, Vec2ui8 b) {
    Vec2ui8 result = {a.x - b.x, a.y - b.y};
    return result;
}

Vec2ui8 vec2ui8_scale(Vec2ui8 v, uint8_t scalar) {
    Vec2ui8 result = {v.x * scalar, v.y * scalar};
    return result;
}

// --- Vec2ui16 Function Implementations ---

Vec2i16 vec2ui16_add(Vec2i16 a, Vec2i16 b) {
    Vec2i16 result = {a.x + b.x, a.y + b.y};
    return result;
}

Vec2i16 vec2ui16_subtract(Vec2i16 a, Vec2i16 b) {
    Vec2i16 result = {a.x - b.x, a.y - b.y};
    return result;
}

Vec2i16 vec2ui16_scale(Vec2i16 v, uint16_t scalar) {
    Vec2i16 result = {v.x * scalar, v.y * scalar};
    return result;
}

// --- Vec2i32 Function Implementations ---

Vec2i32 vec2i32_add(Vec2i32 a, Vec2i32 b) {
    Vec2i32 result = {a.x + b.x, a.y + b.y};
    return result;
}

Vec2i32 vec2i32_subtract(Vec2i32 a, Vec2i32 b) {
    Vec2i32 result = {a.x - b.x, a.y - b.y};
    return result;
}

Vec2i32 vec2i32_scale(Vec2i32 v, int32_t scalar) {
    Vec2i32 result = {v.x * scalar, v.y * scalar};
    return result;
}

// --- Vec2f Function Implementations ---

Vec2f vec2f_add(Vec2f a, Vec2f b) {
    Vec2f result = {a.x + b.x, a.y + b.y};
    return result;
}

Vec2f vec2f_subtract(Vec2f a, Vec2f b) {
    Vec2f result = {a.x - b.x, a.y - b.y};
    return result;
}

Vec2f vec2f_scale(Vec2f v, float scalar) {
    Vec2f result = {v.x * scalar, v.y * scalar};
    return result;
}

// --- Vec3f Function Implementations ---

Vec3f vec3f_add(Vec3f a, Vec3f b) {
    Vec3f result = {a.x + b.x, a.y + b.y, a.z + b.z};
    return result;
}

Vec3f vec3f_subtract(Vec3f a, Vec3f b) {
    Vec3f result = {a.x - b.x, a.y - b.y, a.z - b.z};
    return result;
}

Vec3f vec3f_scale(Vec3f v, float scalar) {
    Vec3f result = {v.x * scalar, v.y * scalar, v.z * scalar};
    return result;
}