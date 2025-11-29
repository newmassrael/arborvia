#pragma once

#include <cstdint>
#include <cmath>
#include <vector>
#include <string>

namespace arborvia {

using NodeId = uint32_t;
using EdgeId = uint32_t;

constexpr NodeId INVALID_NODE = UINT32_MAX;
constexpr EdgeId INVALID_EDGE = UINT32_MAX;

struct Point {
    float x = 0.0f;
    float y = 0.0f;

    constexpr Point() = default;
    constexpr Point(float x_, float y_) : x(x_), y(y_) {}

    constexpr Point operator+(const Point& o) const { return {x + o.x, y + o.y}; }
    constexpr Point operator-(const Point& o) const { return {x - o.x, y - o.y}; }
    constexpr Point operator*(float s) const { return {x * s, y * s}; }
    constexpr Point operator/(float s) const { return {x / s, y / s}; }

    float length() const { return std::sqrt(x * x + y * y); }
    float distanceTo(const Point& o) const { return (*this - o).length(); }

    Point normalized() const {
        float len = length();
        return len > 0.0f ? *this / len : Point{0.0f, 0.0f};
    }

    constexpr bool operator==(const Point& o) const { return x == o.x && y == o.y; }
    constexpr bool operator!=(const Point& o) const { return !(*this == o); }
};

struct Size {
    float width = 0.0f;
    float height = 0.0f;

    constexpr Size() = default;
    constexpr Size(float w, float h) : width(w), height(h) {}

    constexpr bool operator==(const Size& o) const {
        return width == o.width && height == o.height;
    }
    constexpr bool operator!=(const Size& o) const { return !(*this == o); }
};

struct Rect {
    float x = 0.0f;
    float y = 0.0f;
    float width = 0.0f;
    float height = 0.0f;

    constexpr Rect() = default;
    constexpr Rect(float x_, float y_, float w, float h)
        : x(x_), y(y_), width(w), height(h) {}
    constexpr Rect(Point pos, Size size)
        : x(pos.x), y(pos.y), width(size.width), height(size.height) {}

    constexpr Point position() const { return {x, y}; }
    constexpr Size size() const { return {width, height}; }
    constexpr Point center() const { return {x + width / 2, y + height / 2}; }

    constexpr float left() const { return x; }
    constexpr float top() const { return y; }
    constexpr float right() const { return x + width; }
    constexpr float bottom() const { return y + height; }

    constexpr bool contains(const Point& p) const {
        return p.x >= x && p.x <= right() && p.y >= y && p.y <= bottom();
    }

    constexpr bool contains(const Rect& r) const {
        return r.x >= x && r.right() <= right() &&
               r.y >= y && r.bottom() <= bottom();
    }

    Rect united(const Rect& other) const {
        if (width <= 0 || height <= 0) return other;
        if (other.width <= 0 || other.height <= 0) return *this;

        float minX = std::min(x, other.x);
        float minY = std::min(y, other.y);
        float maxX = std::max(right(), other.right());
        float maxY = std::max(bottom(), other.bottom());

        return {minX, minY, maxX - minX, maxY - minY};
    }

    Rect expanded(float padding) const {
        return {x - padding, y - padding, width + 2 * padding, height + 2 * padding};
    }

    constexpr bool operator==(const Rect& o) const {
        return x == o.x && y == o.y && width == o.width && height == o.height;
    }
    constexpr bool operator!=(const Rect& o) const { return !(*this == o); }
};

}  // namespace arborvia
