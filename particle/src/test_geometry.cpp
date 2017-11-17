#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "geometry3d.h"
#define _USE_MATH_DEFINES
#include <math.h>


// Test vector rotation around Y axis
TEST(Vector, rotateY)
{
    btVector3 v(1, 0, 0);
    btVector3 res;

    rotateY(v, btScalar(M_PI_2), res);

    EXPECT_NEAR(res.x(), 0, 1e-7);
    EXPECT_EQ(res.y(), 0);
    EXPECT_NEAR(res.z(), -1, 1e-7);
}


// Test box-ray intersection
TEST(Boxrayxing, outside)
{
    btVector3 a(0, 0, 0);
    btVector3 b(10, 10, 10);
    box_t aabb = std::make_pair(a, b);
    btVector3 ro(20, 5, 5);
    btVector3 rd(-1, 0, 0);
    btVector3 xpoint;

    bool res = boxrayintersect(aabb, ro, rd, xpoint);

    EXPECT_TRUE(res);
    EXPECT_NEAR(xpoint.x(), 10, 1e-7);
    EXPECT_NEAR(xpoint.y(), 5, 1e-7);
    EXPECT_NEAR(xpoint.z(), 5, 1e-7);
}

TEST(Boxrayxing, outside2)
{
	btVector3 a(2500, 0, 0);
	btVector3 b(5000, 1010, 3500);
	box_t aabb = std::make_pair(a, b);
	btVector3 ro(1000, 300, 1000);
	btVector3 rd(1, 0, 0);
	btVector3 xpoint;

	bool res = boxrayintersect(aabb, ro, rd, xpoint);

	EXPECT_TRUE(res);
	EXPECT_NEAR(xpoint.x(), a.x(), 1e-7);
	EXPECT_NEAR(xpoint.y(), ro.y(), 1e-7);
	EXPECT_NEAR(xpoint.z(), ro.z(), 1e-7);
}

TEST(Boxrayxing, outside3)
{
	btVector3 a(2500, 0, 3500);
	btVector3 b(5000, 1010, 7000);
	box_t aabb = std::make_pair(a, b);
	btVector3 ro(1000, 300, 1000);
	btVector3 rd(1, 0, 0);
	btVector3 xpoint;

	bool res = boxrayintersect(aabb, ro, rd, xpoint);

	EXPECT_FALSE(res);
}

TEST(Boxrayxing, inside_toleft)
{
    btVector3 a(0, 0, 0);
    btVector3 b(10, 10, 10);
    box_t aabb = std::make_pair(a, b);
    btVector3 ro(5, 5, 5);
    btVector3 rd(-1, 0, 0);
    btVector3 xpoint;

    bool res = boxrayintersect(aabb, ro, rd, xpoint);

    EXPECT_TRUE(res);
    EXPECT_NEAR(xpoint.x(), 10, 1e-7);
    EXPECT_NEAR(xpoint.y(), 5, 1e-7);
    EXPECT_NEAR(xpoint.z(), 5, 1e-7);
}

TEST(Boxrayxing, inside_toright)
{
    btVector3 a(0, 0, 0);
    btVector3 b(10, 10, 10);
    box_t aabb = std::make_pair(a, b);
    btVector3 ro(5, 5, 5);
    btVector3 rd(1, 0, 0);
    btVector3 xpoint;

    bool res = boxrayintersect(aabb, ro, rd, xpoint);

    EXPECT_TRUE(res);
    EXPECT_NEAR(xpoint.x(), 0, 1e-7);
    EXPECT_NEAR(xpoint.y(), 5, 1e-7);
    EXPECT_NEAR(xpoint.z(), 5, 1e-7);
}

TEST(Trirayxing, left_to_right)
{
	// Vertices should be in counter-clock-wise (CCW) sequence
	btVector3 v0(100, 0, 200);
	btVector3 v1(100, 100, 0);
	btVector3 v2(100, 0, 0);

	btVector3 edge1 = v1 - v0;
    btVector3 edge2 = v2 - v0;

	btVector3 origin(5, 5, 5);
    btVector3 direction(1, 0, 0);
    btVector3 xpoint;

	bool res = intersect_triangle(origin, direction,
                                  v0, edge1, edge2, 
                                  xpoint, true);

    EXPECT_TRUE(res);
    EXPECT_NEAR(xpoint.x(), v0.x() - origin.x(), 1e-1);
}

TEST(Trirayxing, right_to_left)
{
	// Vertices should be in counter-clock-wise (CCW) sequence
	btVector3 v0(5, 0, 0);
	btVector3 v1(5, 100, 0);
	btVector3 v2(5, 0, 200);
	
	btVector3 edge1 = v1 - v0;
	btVector3 edge2 = v2 - v0;

	btVector3 origin(100, 5, 5);
	btVector3 direction(-1, 0, 0);
	btVector3 xpoint;

	bool res = intersect_triangle(origin, direction,
		v0, edge1, edge2,
		xpoint, true);

	EXPECT_TRUE(res);
	EXPECT_NEAR(xpoint.x(), origin.x() - v0.x(), 1e-1);
}

TEST(Trirayxing, left_to_right_down_right)
{
	// Vertices should be in counter-clock-wise (CCW) sequence
	btVector3 v0(100, 0, 200);
	btVector3 v1(100, 100, 0);
	btVector3 v2(100, 0, 0);

	btVector3 edge1 = v1 - v0;
	btVector3 edge2 = v2 - v0;

	btVector3 origin(5, 5, 5);
	btVector3 direction0(1, 0, 0);
	btVector3 axis(0, 0, 1);
	btScalar angle = btScalar(M_PI) / 8;
	btVector3 direction = direction0.rotate(axis, angle);

	btVector3 xpoint;

	bool res = intersect_triangle(origin, direction,
		v0, edge1, edge2,
		xpoint, true);

	EXPECT_TRUE(res);
	EXPECT_NEAR(xpoint.x(), (v0.x() - origin.x()) / cos(angle), 1e-1);
}
