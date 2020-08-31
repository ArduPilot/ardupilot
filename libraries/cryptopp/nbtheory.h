// nbtheory.h - originally written and placed in the public domain by Wei Dai

/// \file nbtheory.h
/// \brief Classes and functions  for number theoretic operations

#ifndef CRYPTOPP_NBTHEORY_H
#define CRYPTOPP_NBTHEORY_H

#include "cryptlib.h"
#include "integer.h"
#include "algparam.h"

NAMESPACE_BEGIN(CryptoPP)

/// \brief The Small Prime table
/// \details GetPrimeTable obtains pointer to small prime table and provides the size of the table.
CRYPTOPP_DLL const word16 * CRYPTOPP_API GetPrimeTable(unsigned int &size);

// ************ primality testing ****************

/// \brief Generates a provable prime
/// \param rng a RandomNumberGenerator to produce random material
/// \param bits the number of bits in the prime number
/// \returns Integer() meeting Maurer's tests for primality
CRYPTOPP_DLL Integer CRYPTOPP_API MaurerProvablePrime(RandomNumberGenerator &rng, unsigned int bits);

/// \brief Generates a provable prime
/// \param rng a RandomNumberGenerator to produce random material
/// \param bits the number of bits in the prime number
/// \returns Integer() meeting Mihailescu's tests for primality
/// \details Mihailescu's methods performs a search using algorithmic progressions.
CRYPTOPP_DLL Integer CRYPTOPP_API MihailescuProvablePrime(RandomNumberGenerator &rng, unsigned int bits);

/// \brief Tests whether a number is a small prime
/// \param p a candidate prime to test
/// \returns true if p is a small prime, false otherwise
/// \details Internally, the library maintains a table of the first 32719 prime numbers
///   in sorted order. IsSmallPrime searches the table and returns true if p is
///   in the table.
CRYPTOPP_DLL bool CRYPTOPP_API IsSmallPrime(const Integer &p);

/// \brief Tests whether a number is divisible by a small prime
/// \returns true if p is divisible by some prime less than bound.
/// \details TrialDivision() returns <tt>true</tt> if <tt>p</tt> is divisible by some prime less
///   than <tt>bound</tt>. <tt>bound</tt> should not be greater than the largest entry in the
///   prime table, which is 32719.
CRYPTOPP_DLL bool CRYPTOPP_API TrialDivision(const Integer &p, unsigned bound);

/// \brief Tests whether a number is divisible by a small prime
/// \returns true if p is NOT divisible by small primes.
/// \details SmallDivisorsTest() returns <tt>true</tt> if <tt>p</tt> is NOT divisible by some
///   prime less than 32719.
CRYPTOPP_DLL bool CRYPTOPP_API SmallDivisorsTest(const Integer &p);

/// \brief Determine if a number is probably prime
/// \param n the number to test
/// \param b the base to exponentiate
/// \returns true if the number n is probably prime, false otherwise.
/// \details IsFermatProbablePrime raises <tt>b</tt> to the <tt>n-1</tt> power and checks if
///   the result is congruent to 1 modulo <tt>n</tt>.
/// \details These is no reason to use IsFermatProbablePrime, use IsStrongProbablePrime or
///   IsStrongLucasProbablePrime instead.
/// \sa IsStrongProbablePrime, IsStrongLucasProbablePrime
CRYPTOPP_DLL bool CRYPTOPP_API IsFermatProbablePrime(const Integer &n, const Integer &b);

/// \brief Determine if a number is probably prime
/// \param n the number to test
/// \returns true if the number n is probably prime, false otherwise.
/// \details These is no reason to use IsLucasProbablePrime, use IsStrongProbablePrime or
///   IsStrongLucasProbablePrime instead.
/// \sa IsStrongProbablePrime, IsStrongLucasProbablePrime
CRYPTOPP_DLL bool CRYPTOPP_API IsLucasProbablePrime(const Integer &n);

/// \brief Determine if a number is probably prime
/// \param n the number to test
/// \param b the base to exponentiate
/// \returns true if the number n is probably prime, false otherwise.
CRYPTOPP_DLL bool CRYPTOPP_API IsStrongProbablePrime(const Integer &n, const Integer &b);

/// \brief Determine if a number is probably prime
/// \param n the number to test
/// \returns true if the number n is probably prime, false otherwise.
CRYPTOPP_DLL bool CRYPTOPP_API IsStrongLucasProbablePrime(const Integer &n);

/// \brief Determine if a number is probably prime
/// \param rng a RandomNumberGenerator to produce random material
/// \param n the number to test
/// \param rounds the number of tests to perform
/// \details This is the Rabin-Miller primality test, i.e. repeating the strong probable prime
///   test for several rounds with random bases
/// \sa <A HREF="https://crypto.stackexchange.com/q/17707/10496">Trial divisions before
///   Miller-Rabin checks?</A> on Crypto Stack Exchange
CRYPTOPP_DLL bool CRYPTOPP_API RabinMillerTest(RandomNumberGenerator &rng, const Integer &n, unsigned int rounds);

/// \brief Verifies a number is probably prime
/// \param p a candidate prime to test
/// \returns true if p is a probable prime, false otherwise
/// \details IsPrime() is suitable for testing candidate primes when creating them. Internally,
///   IsPrime() utilizes SmallDivisorsTest(), IsStrongProbablePrime() and IsStrongLucasProbablePrime().
CRYPTOPP_DLL bool CRYPTOPP_API IsPrime(const Integer &p);

/// \brief Verifies a number is probably prime
/// \param rng a RandomNumberGenerator for randomized testing
/// \param p a candidate prime to test
/// \param level the level of thoroughness of testing
/// \returns true if p is a strong probable prime, false otherwise
/// \details VerifyPrime() is suitable for testing candidate primes created by others. Internally,
///   VerifyPrime() utilizes IsPrime() and one-round RabinMillerTest(). If the candiate passes and
///   level is greater than 1, then 10 round RabinMillerTest() primality testing is performed.
CRYPTOPP_DLL bool CRYPTOPP_API VerifyPrime(RandomNumberGenerator &rng, const Integer &p, unsigned int level = 1);

/// \brief Application callback to signal suitability of a cabdidate prime
class CRYPTOPP_DLL PrimeSelector
{
public:
	virtual ~PrimeSelector() {}
	const PrimeSelector *GetSelectorPointer() const {return this;}
	virtual bool IsAcceptable(const Integer &candidate) const =0;
};

/// \brief Finds a random prime of special form
/// \param p an Integer reference to receive the prime
/// \param max the maximum value
/// \param equiv the equivalence class based on the parameter mod
/// \param mod the modulus used to reduce the equivalence class
/// \param pSelector pointer to a PrimeSelector function for the application to signal suitability
/// \returns true if and only if FirstPrime() finds a prime and returns the prime through p. If FirstPrime()
///   returns false, then no such prime exists and the value of p is undefined
/// \details FirstPrime() uses a fast sieve to find the first probable prime
///   in <tt>{x | p<=x<=max and x%mod==equiv}</tt>
CRYPTOPP_DLL bool CRYPTOPP_API FirstPrime(Integer &p, const Integer &max, const Integer &equiv, const Integer &mod, const PrimeSelector *pSelector);

CRYPTOPP_DLL unsigned int CRYPTOPP_API PrimeSearchInterval(const Integer &max);

CRYPTOPP_DLL AlgorithmParameters CRYPTOPP_API MakeParametersForTwoPrimesOfEqualSize(unsigned int productBitLength);

// ********** other number theoretic functions ************

/// \brief Calculate the greatest common divisor
/// \param a the first term
/// \param b the second term
/// \returns the greatest common divisor if one exists, 0 otherwise.
inline Integer GCD(const Integer &a, const Integer &b)
	{return Integer::Gcd(a,b);}

/// \brief Determine relative primality
/// \param a the first term
/// \param b the second term
/// \returns true if <tt>a</tt> and <tt>b</tt> are relatively prime, false otherwise.
inline bool RelativelyPrime(const Integer &a, const Integer &b)
	{return Integer::Gcd(a,b) == Integer::One();}

/// \brief Calculate the least common multiple
/// \param a the first term
/// \param b the second term
/// \returns the least common multiple of <tt>a</tt> and <tt>b</tt>.
inline Integer LCM(const Integer &a, const Integer &b)
	{return a/Integer::Gcd(a,b)*b;}

/// \brief Calculate multiplicative inverse
/// \param a the number to test
/// \param b the modulus
/// \returns an Integer <tt>(a ^ -1) % n</tt> or 0 if none exists.
/// \details EuclideanMultiplicativeInverse returns the multiplicative inverse of the Integer
///   <tt>*a</tt> modulo the Integer <tt>b</tt>. If no Integer exists then Integer 0 is returned.
inline Integer EuclideanMultiplicativeInverse(const Integer &a, const Integer &b)
	{return a.InverseMod(b);}


/// \brief Chinese Remainder Theorem
/// \param xp the first number, mod p
/// \param p the first prime modulus
/// \param xq the second number, mod q
/// \param q the second prime modulus
/// \param u inverse of p mod q
/// \returns the CRT value of the parameters
/// \details CRT uses the Chinese Remainder Theorem to calculate <tt>x</tt> given
///   <tt>x mod p</tt> and <tt>x mod q</tt>, and <tt>u</tt> the inverse of <tt>p mod q</tt>.
CRYPTOPP_DLL Integer CRYPTOPP_API CRT(const Integer &xp, const Integer &p, const Integer &xq, const Integer &q, const Integer &u);

/// \brief Calculate the Jacobi symbol
/// \param a the first term
/// \param b the second term
/// \returns the the Jacobi symbol.
/// \details Jacobi symbols are calculated using the following rules:
///  -# if <tt>b</tt> is prime, then <tt>Jacobi(a, b)</tt>, then return 0
///  -# if <tt>a%b</tt>==0 AND <tt>a</tt> is quadratic residue <tt>mod b</tt>, then return 1
///  -# return -1 otherwise
/// \details Refer to a number theory book for what Jacobi symbol means when <tt>b</tt> is not prime.
CRYPTOPP_DLL int CRYPTOPP_API Jacobi(const Integer &a, const Integer &b);

/// \brief Calculate the Lucas value
/// \returns the Lucas value
/// \details Lucas() calculates the Lucas function <tt>V_e(p, 1) mod n</tt>.
CRYPTOPP_DLL Integer CRYPTOPP_API Lucas(const Integer &e, const Integer &p, const Integer &n);

/// \brief Calculate the inverse Lucas value
/// \returns the inverse Lucas value
/// \details InverseLucas() calculates <tt>x</tt> such that <tt>m==Lucas(e, x, p*q)</tt>,
///   <tt>p q</tt> primes, <tt>u</tt> is inverse of <tt>p mod q</tt>.
CRYPTOPP_DLL Integer CRYPTOPP_API InverseLucas(const Integer &e, const Integer &m, const Integer &p, const Integer &q, const Integer &u);

/// \brief Modular multiplication
/// \param x the first term
/// \param y the second term
/// \param m the modulus
/// \returns an Integer <tt>(x * y) % m</tt>.
inline Integer ModularMultiplication(const Integer &x, const Integer &y, const Integer &m)
	{return a_times_b_mod_c(x, y, m);}

/// \brief Modular exponentiation
/// \param x the base
/// \param e the exponent
/// \param m the modulus
/// \returns an Integer <tt>(a ^ b) % m</tt>.
inline Integer ModularExponentiation(const Integer &x, const Integer &e, const Integer &m)
	{return a_exp_b_mod_c(x, e, m);}

/// \brief Extract a modular square root
/// \param a the number to extract square root
/// \param p the prime modulus
/// \returns the modular square root if it exists
/// \details ModularSquareRoot returns <tt>x</tt> such that <tt>x*x%p == a</tt>, <tt>p</tt> prime
CRYPTOPP_DLL Integer CRYPTOPP_API ModularSquareRoot(const Integer &a, const Integer &p);

/// \brief Extract a modular root
/// \returns a modular root if it exists
/// \details ModularRoot returns <tt>x</tt> such that <tt>a==ModularExponentiation(x, e, p*q)</tt>,
///   <tt>p</tt> <tt>q</tt> primes, and <tt>e</tt> relatively prime to <tt>(p-1)*(q-1)</tt>,
///   <tt>dp=d%(p-1)</tt>, <tt>dq=d%(q-1)</tt>, (d is inverse of <tt>e mod (p-1)*(q-1)</tt>)
///   and <tt>u=inverse of p mod q</tt>.
CRYPTOPP_DLL Integer CRYPTOPP_API ModularRoot(const Integer &a, const Integer &dp, const Integer &dq, const Integer &p, const Integer &q, const Integer &u);

/// \brief Solve a Modular Quadratic Equation
/// \param r1 the first residue
/// \param r2 the second residue
/// \param a the first coefficient
/// \param b the second coefficient
/// \param c the third constant
/// \param p the prime modulus
/// \returns true if solutions exist
/// \details SolveModularQuadraticEquation() finds <tt>r1</tt> and <tt>r2</tt> such that <tt>ax^2 +
///   bx + c == 0 (mod p)</tt> for x in <tt>{r1, r2}</tt>, <tt>p</tt> prime.
CRYPTOPP_DLL bool CRYPTOPP_API SolveModularQuadraticEquation(Integer &r1, Integer &r2, const Integer &a, const Integer &b, const Integer &c, const Integer &p);

/// \brief Estimate work factor
/// \param bitlength the size of the number, in bits
/// \returns the estimated work factor, in operations
/// \details DiscreteLogWorkFactor returns log base 2 of estimated number of operations to
///   calculate discrete log or factor a number.
CRYPTOPP_DLL unsigned int CRYPTOPP_API DiscreteLogWorkFactor(unsigned int bitlength);

/// \brief Estimate work factor
/// \param bitlength the size of the number, in bits
/// \returns the estimated work factor, in operations
/// \details FactoringWorkFactor returns log base 2 of estimated number of operations to
///   calculate discrete log or factor a number.
CRYPTOPP_DLL unsigned int CRYPTOPP_API FactoringWorkFactor(unsigned int bitlength);

// ********************************************************

/// \brief Generator of prime numbers of special forms
class CRYPTOPP_DLL PrimeAndGenerator
{
public:
	/// \brief Construct a PrimeAndGenerator
	PrimeAndGenerator() {}

	/// \brief Construct a PrimeAndGenerator
	/// \param delta +1 or -1
	/// \param rng a RandomNumberGenerator derived class
	/// \param pbits the number of bits in the prime p
	/// \details PrimeAndGenerator() generates a random prime p of the form <tt>2*q+delta</tt>, where delta is 1 or -1 and q is
	///   also prime. Internally the constructor calls <tt>Generate(delta, rng, pbits, pbits-1)</tt>.
	/// \pre <tt>pbits > 5</tt>
	/// \warning This PrimeAndGenerator() is slow because primes of this form are harder to find.
	PrimeAndGenerator(signed int delta, RandomNumberGenerator &rng, unsigned int pbits)
		{Generate(delta, rng, pbits, pbits-1);}

	/// \brief Construct a PrimeAndGenerator
	/// \param delta +1 or -1
	/// \param rng a RandomNumberGenerator derived class
	/// \param pbits the number of bits in the prime p
	/// \param qbits the number of bits in the prime q
	/// \details PrimeAndGenerator() generates a random prime p of the form <tt>2*r*q+delta</tt>, where q is also prime.
	///    Internally the constructor calls <tt>Generate(delta, rng, pbits, qbits)</tt>.
	/// \pre <tt>qbits > 4 && pbits > qbits</tt>
	PrimeAndGenerator(signed int delta, RandomNumberGenerator &rng, unsigned int pbits, unsigned qbits)
		{Generate(delta, rng, pbits, qbits);}

	/// \brief Generate a Prime and Generator
	/// \param delta +1 or -1
	/// \param rng a RandomNumberGenerator derived class
	/// \param pbits the number of bits in the prime p
	/// \param qbits the number of bits in the prime q
	/// \details Generate() generates a random prime p of the form <tt>2*r*q+delta</tt>, where q is also prime.
	void Generate(signed int delta, RandomNumberGenerator &rng, unsigned int pbits, unsigned qbits);

	/// \brief Retrieve first prime
	/// \returns Prime() returns the prime p.
	const Integer& Prime() const {return p;}

	/// \brief Retrieve second prime
	/// \returns SubPrime() returns the prime q.
	const Integer& SubPrime() const {return q;}

	/// \brief Retrieve the generator
	/// \returns Generator() returns the the generator g.
	const Integer& Generator() const {return g;}

private:
	Integer p, q, g;
};

NAMESPACE_END

#endif
