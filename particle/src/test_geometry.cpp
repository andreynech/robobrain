#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "geometry3d.h"
#include <math.h>


// Test vector rotation around Y axis
TEST(Vector, rotateY)
{
    btVector3 v(1, 0, 0);
    btVector3 res;

    rotateY(v, M_PI_2, res);

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

