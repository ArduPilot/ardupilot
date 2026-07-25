#include <cppunit/extensions/HelperMacros.h>
#include "Accumulator.h"
#include <string>
#include <vector>
#include <fstream>

using namespace std;

class AccumulatorTest : public CPPUNIT_NS::TestFixture
{
  private:
    CPPUNIT_TEST_SUITE( AccumulatorTest );
    CPPUNIT_TEST( test0 );
    CPPUNIT_TEST( test1 );
    CPPUNIT_TEST_SUITE_END();

  public:
    void test0();
    void test1();

    void setUp();
    void tearDown();

  private:
    Accumulator * m_accumulator;

};

// Registers the fixture into the 'registry'
CPPUNIT_TEST_SUITE_REGISTRATION( AccumulatorTest );

void AccumulatorTest::setUp()
{
  m_accumulator = new Accumulator;
}

void AccumulatorTest::tearDown()
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

void AccumulatorTest::test0()
{
  vector<string> lines;
  readlines("test0.txt", lines);
  size_t expected(2);
  CPPUNIT_ASSERT_EQUAL(expected, lines.size());
  m_accumulator->accumulate(lines[0].c_str());
  CPPUNIT_ASSERT_EQUAL(10, m_accumulator->total());

}

void AccumulatorTest::test1()
{
  vector<string> lines;
  readlines("test1.txt", lines);
  size_t expected(6);
  CPPUNIT_ASSERT_EQUAL(expected, lines.size());
  for (vector<string>::const_iterator it(lines.begin());
      it != lines.end(); ++it)
  {
    const string & line(*it);
    m_accumulator->accumulate(line.c_str());
  }
  CPPUNIT_ASSERT_EQUAL(1+2+3+4+5, m_accumulator->total());

}
