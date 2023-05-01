#include <gtest/gtest.h>
#include "fastcat/enum_conversions.h"

TEST(EnumConversions, ConditionalOperatorType){
  MSG("The following error is expected");
  EXPECT_EQ(fastcat::BAD_CONDITIONAL_OPERATOR_TYPE, 
    fastcat::ConditionalOperatorTypeFromString("unknown"));
  EXPECT_EQ(fastcat::LT, fastcat::ConditionalOperatorTypeFromString("<"));
  EXPECT_EQ(fastcat::EQ, fastcat::ConditionalOperatorTypeFromString("=="));
}

TEST(EnumConversions, FilterType){
  MSG("The following error is expected");
  EXPECT_EQ(fastcat::BAD_FILTER_TYPE, fastcat::FilterTypeFromString("unknown"));
  EXPECT_EQ(fastcat::DIGITAL_AB, fastcat::FilterTypeFromString("DIGITAL_AB"));
}

TEST(EnumConversions, FunctionType){
  MSG("The following error is expected");
  EXPECT_EQ(fastcat::BAD_FUNCTION_TYPE, fastcat::FunctionTypeFromString("unknown"));
  EXPECT_EQ(fastcat::SIGMOID, fastcat::FunctionTypeFromString("SIGMOID"));
}
