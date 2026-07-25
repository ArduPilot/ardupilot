#include "gtest/gtest.h"
#include "Accumulator.h"
#include <string>
#include <vector>
#include <fstream>

using namespace std;

class AccumulatorTest : public testing::Test
{
 protected:
    virtual void SetUp();
    virtual void TearDown();

    Accumulator * m_accumulator;

};

void AccumulatorTest::SetUp()
{
  m_accumulator = new Accumulator;
}

void AccumulatorTest::TearDown()
{
  delete m_accumulator;
}

static void readlines(const char * filename, vector<string> & lines)
{
  string datafile("input");
  datafile += "/";
  datafile += filename;
  ifstream infile;
  infile.open(datafile.c_str());
  if (infile.is_open())
  {
    char buffer[BUFSIZ];
    while (!infile.eof())
    {
      infile.getline(buffer, BUFSIZ);
      lines.push_back(buffer);
    }
  }
}

TEST_F(AccumulatorTest, test0)
{
  vector<string> lines;
  readlines("test0.txt", lines);
  size_t expected(2);
  ASSERT_EQ(expected, lines.size());
  m_accumulator->accumulate(lines[0].c_str());
  ASSERT_EQ(10, m_accumulator->total());

}

TEST_F(AccumulatorTest, test1)
{
  vector<string> lines;
  readlines("test1.txt", lines);
  size_t expected(6);
  ASSERT_EQ(expected, lines.size());
  for (vector<string>::const_iterator it(lines.begin());
      it != lines.end(); ++it)
  {
    const string & line(*it);
    m_accumulator->accumulate(line.c_str());
  }
  ASSERT_EQ(1+2+3+4+5, m_accumulator->total());

}
