#include <gtest/gtest.h>
#include "fastcat/enum_conversions.h"

TEST(EnumConversions, ConditionalOperatorType){
  
  EXPECT_EQ(fastcat::BAD_CONDITIONAL_OPERATOR_TYPE, 
    fastcat::ConditionalOperatorTypeFromString("unknown"));
  EXPECT_EQ(fastcat::LT, fastcat::ConditionalOperatorTypeFromString("<"));
  EXPECT_EQ(fastcat::EQ, fastcat::ConditionalOperatorTypeFromString("=="));
}
