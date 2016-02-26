#include <cstring>
#include "gtest/gtest.h"

#define private public

#include "../BufferedReader.h"

TEST(BufferedReaderBasic, GetsNumberAndHello) {
  using namespace std;
  using namespace server::core;

  BufferedReader reader;
  char buf[9];
  int num = 5;
  const char* str = "hello";

  ASSERT_EQ(sizeof(num), 4);

  memcpy(buf, &num, 4);
  memcpy(buf+4, str, 5);

  reader.Write(buf, 9);
  EXPECT_EQ(reader.AvailableReadSpace(), 9);
  bool res;
  int resn;
  res = reader.ReadInt(resn);
  EXPECT_TRUE(res);
  EXPECT_EQ(resn, 5);
  EXPECT_EQ(reader.AvailableReadSpace(), 5);
  char ress[6];
  ress[5] = '\0';
  int bytes_read = reader.ReadAvailableBytes(ress);
  EXPECT_EQ(reader.AvailableReadSpace(), 0);
  EXPECT_EQ(bytes_read, 5);
  EXPECT_STREQ(ress, "hello");
}
