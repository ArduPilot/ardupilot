// eprecomp.h - originally written and placed in the public domain by Wei Dai

/// \file eprecomp.h
/// \brief Classes for precomputation in a group

#ifndef CRYPTOPP_EPRECOMP_H
#define CRYPTOPP_EPRECOMP_H

#include "cryptlib.h"
#include "integer.h"
#include "algebra.h"
#include "stdcpp.h"

NAMESPACE_BEGIN(CryptoPP)

/// \brief DL_GroupPrecomputation interface
/// \tparam T Field element
template <class T>
class DL_GroupPrecomputation
{
public:
	typedef T Element;

	virtual ~DL_GroupPrecomputation() {}

	/// \brief Determines if elements needs conversion
	/// \returns true if the element needs conversion, false otherwise
	/// \details NeedConversions determines if an element must convert between representations.
	virtual bool NeedConversions() const {return false;}

	/// \brief Converts an element between representations
	/// \param v element to convert
	/// \returns an element converted to an alternate representation for internal use
	/// \details ConvertIn is used when an element must convert between representations.
	virtual Element ConvertIn(const Element &v) const {return v;}

	/// \brief Converts an element between representations
	/// \param v element to convert
	/// \returns an element converted from an alternate representation
	virtual Element ConvertOut(const Element &v) const {return v;}

	/// \brief Retrieves AbstractGroup interface
	/// \returns GetGroup() returns the AbstractGroup interface
	virtual const AbstractGroup<Element> & GetGroup() const =0;

	/// \brief Decodes element in DER format
	/// \param bt BufferedTransformation object
	/// \returns element in the group
	virtual Element BERDecodeElement(BufferedTransformation &bt) const =0;

	/// \brief Encodes element in DER format
	/// \param bt BufferedTransformation object
	/// \param P Element to encode
	virtual void DEREncodeElement(BufferedTransformation &bt, const Element &P) const =0;
};

/// \brief DL_FixedBasePrecomputation interface
/// \tparam T Field element
template <class T>
class DL_FixedBasePrecomputation
{
public:
	typedef T Element;

	virtual ~DL_FixedBasePrecomputation() {}

	/// \brief Determines whether this object is initialized
	/// \returns true if this object is initialized, false otherwise
	virtual bool IsInitialized() const =0;

	/// \brief Set the base element
	/// \param group the group
	/// \param base element in the group
	virtual void SetBase(const DL_GroupPrecomputation<Element> &group, const Element &base) =0;

	/// \brief Get the base element
	/// \param group the group
	/// \returns base element in the group
	virtual const Element & GetBase(const DL_GroupPrecomputation<Element> &group) const =0;

	/// \brief Perform precomputation
	/// \param group the group
	/// \param maxExpBits used to calculate the exponent base
	/// \param storage the suggested number of objects for the precompute table
	/// \details The exact semantics of Precompute() varies, but it typically means calculate
	///   a table of n objects that can be used later to speed up computation.
	/// \details If a derived class does not override Precompute(), then the base class throws
	///   NotImplemented.
	/// \sa SupportsPrecomputation(), LoadPrecomputation(), SavePrecomputation()
	virtual void Precompute(const DL_GroupPrecomputation<Element> &group, unsigned int maxExpBits, unsigned int storage) =0;

	/// \brief Retrieve previously saved precomputation
	/// \param group the the group
	/// \param storedPrecomputation BufferedTransformation with the saved precomputation
	/// \throws NotImplemented
	/// \sa SupportsPrecomputation(), Precompute()
	virtual void Load(const DL_GroupPrecomputation<Element> &group, BufferedTransformation &storedPrecomputation) =0;

	/// \brief Save precomputation for later use
	/// \param group the the group
	/// \param storedPrecomputation BufferedTransformation to write the precomputation
	/// \throws NotImplemented
	/// \sa SupportsPrecomputation(), Precompute()
	virtual void Save(const DL_GroupPrecomputation<Element> &group, BufferedTransformation &storedPrecomputation) const =0;

	/// \brief Exponentiates an element
	/// \param group the group
	/// \param exponent the exponent
	/// \return the result of the exponentiation
	virtual Element Exponentiate(const DL_GroupPrecomputation<Element> &group, const Integer &exponent) const =0;

	/// \brief Exponentiates an element
	/// \param pc1 the first the group precomputation
	/// \param exponent1 the first exponent
	/// \param pc2 the second the group precomputation
	/// \param exponent2 the first exponent2
	/// \returns the public element raised to the exponent
	/// \details CascadeExponentiateBaseAndPublicElement raises the public element to
	///   the base element and precomputation.
	virtual Element CascadeExponentiate(const DL_GroupPrecomputation<Element> &pc1, const Integer &exponent1, const DL_FixedBasePrecomputation<Element> &pc2, const Integer &exponent2) const =0;
};

/// \brief DL_FixedBasePrecomputation adapter class
/// \tparam T Field element
template <class T>
class DL_FixedBasePrecomputationImpl : public DL_FixedBasePrecomputation<T>
{
public:
	typedef T Element;

	virtual ~DL_FixedBasePrecomputationImpl() {}

	DL_FixedBasePrecomputationImpl() : m_windowSize(0) {}

	// DL_FixedBasePrecomputation
	bool IsInitialized() const
		{return !m_bases.empty();}
	void SetBase(const DL_GroupPrecomputation<Element> &group, const Element &base);
	const Element & GetBase(const DL_GroupPrecomputation<Element> &group) const
		{return group.NeedConversions() ? m_base : m_bases[0];}
	void Precompute(const DL_GroupPrecomputation<Element> &group, unsigned int maxExpBits, unsigned int storage);
	void Load(const DL_GroupPrecomputation<Element> &group, BufferedTransformation &storedPrecomputation);
	void Save(const DL_GroupPrecomputation<Element> &group, BufferedTransformation &storedPrecomputation) const;
	Element Exponentiate(const DL_GroupPrecomputation<Element> &group, const Integer &exponent) const;
	Element CascadeExponentiate(const DL_GroupPrecomputation<Element> &pc1, const Integer &exponent1, const DL_FixedBasePrecomputation<Element> &pc2, const Integer &exponent2) const;

private:
	void PrepareCascade(const DL_GroupPrecomputation<Element> &group, std::vector<BaseAndExponent<Element> > &eb, const Integer &exponent) const;

	Element m_base;
	unsigned int m_windowSize;
	Integer m_exponentBase;			// what base to represent the exponent in
	std::vector<Element> m_bases;	// precalculated bases
};

NAMESPACE_END

#ifdef CRYPTOPP_MANUALLY_INSTANTIATE_TEMPLATES
#include "eprecomp.cpp"
#endif

#endif
