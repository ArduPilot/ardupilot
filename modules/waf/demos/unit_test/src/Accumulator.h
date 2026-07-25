#ifndef Accumulator_h_seen
#define Accumulator_h_seen

/**
 * A not-very-useful class that accumulates int values from input data.
 */
class Accumulator
{
  public:
    Accumulator();

    /**
     * Given some input data, read to end of line and convert to an int
     */
    void accumulate(const char * data);

    int total() const;

  private:
    int m_total;

};
#endif
