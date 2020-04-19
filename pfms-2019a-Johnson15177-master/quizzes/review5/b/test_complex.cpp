#include "gtest/gtest.h"
#include <iostream>
#include "complex.h"

////The 'TEST' keyword signals to the gtest framework that this is a unit test
TEST (ComplexNumTest, Magnitude) {
  //Two different methods of initialising a struct
  TComplex a = { 4.0, 2.0 };
  double expected = { 4.472 };
  double answer = Complex::magnitude(a);

  //Check that the expected is equal to the actual answer
  EXPECT_NEAR(expected, answer,0.0002);
}

TEST (ComplexNumTest, Conjugate) {
  //Two different methods of initialising a struct
  TComplex a = { 4.0, -4.0 };
  TComplex expected = { 4.0, 4.0};
  TComplex answer = Complex::conjugate(a);

  //Check that the expected is equal to the actual answer
  EXPECT_DOUBLE_EQ(expected.re, answer.re);
  EXPECT_DOUBLE_EQ(expected.im, answer.im);
}

TEST (ComplexNumTest, Add) {
  //Two different methods of initialising a struct
  TComplex a = { 4.0, -4.0 };
  TComplex b = {2.0 , 6.0};
  TComplex expected = { 6.0, 2.0};
  TComplex answer = Complex::add(a, b);

  //Check that the expected is equal to the actual answer
  EXPECT_DOUBLE_EQ(expected.re, answer.re);
  EXPECT_DOUBLE_EQ(expected.im, answer.im);
}

TEST (ComplexNumTest, Subtract) {
  //Two different methods of initialising a struct
  TComplex a = { 4.0, 7.0 };
  TComplex b = {2.0 , -6.0};
  TComplex expected = { 2.0, 13.0};
  TComplex answer = Complex::subtract(a, b);

  //Check that the expected is equal to the actual answer
  EXPECT_DOUBLE_EQ(expected.re, answer.re);
  EXPECT_DOUBLE_EQ(expected.im, answer.im);

}

TEST (ComplexNumTest, Multiply) {
  //Two different methods of initialising a struct
  TComplex a = { 4.0, 7.0 };
  TComplex b = {2.0 , -6.0};
  TComplex expected = { -34.0, -10.0};
  TComplex answer = Complex::multiply(a, b);

  //Check that the expected is equal to the actual answer
  EXPECT_DOUBLE_EQ(expected.re, answer.re);
  EXPECT_DOUBLE_EQ(expected.im, answer.im);

}

TEST (ComplexNumTest, Divide) {
  //Two different methods of initialising a struct
  TComplex a = { .re = 5.0, .im = 0.5 };
  TComplex b = { 2.0, 1.0 };
  TComplex expected = {2.5, 0.50 };
  TComplex answer = Complex::divide(a, b);

  //Check that the expected is equal to the actual answer
  EXPECT_DOUBLE_EQ(expected.re, answer.re);
  EXPECT_DOUBLE_EQ(expected.im, answer.im);
}

TEST (ComplexNumTest, Format) {
  //Two different methods of initialising a struct
  TComplex a = { 4.0, 7.0 };
  std::string expected = {"4 + 7i"};
  std::string answer = Complex::format(a);

  //Check that the expected is equal to the actual answer
  EXPECT_EQ(expected, answer);

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

