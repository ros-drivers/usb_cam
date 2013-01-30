/*
 * This file is part of ALVAR, A Library for Virtual and Augmented Reality.
 *
 * Copyright 2007-2012 VTT Technical Research Centre of Finland
 *
 * Contact: VTT Augmented Reality Team <alvar.info@vtt.fi>
 *          <http://www.vtt.fi/multimedia/alvar.html>
 *
 * ALVAR is free software; you can redistribute it and/or modify it under the
 * terms of the GNU Lesser General Public License as published by the Free
 * Software Foundation; either version 2.1 of the License, or (at your option)
 * any later version.
 *
 * This library is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License
 * for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with ALVAR; if not, see
 * <http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html>.
 */

#ifndef BITSET_H
#define BITSET_H

/**
 * \file Bitset.h
 *
 * \brief This file implements bit set handling.
 */

#include "Alvar.h"
#include <iostream>
#include <deque>
#include <string>
#include <sstream>
#include <iomanip>

namespace alvar {

/**
 * \brief \e Bitset is a basic class for handling bit sequences
 *
 * The bits are stored internally using deque<bool> (See http://osdir.com/ml/gis.geos.devel/2006-04/msg00142.html).
 * The bitset is assumed to have most significant bits left i.e. the push_back() methods add to the least 
 * significant end of the bit sequence. The usage is clarified by the following example.
 *
 * \section Usage
 * \code
 * Bitset b;
 * b.push_back(true);
 * b.push_back(false);
 * b.push_back(false);
 * b.push_back(false);
 * b.push_back(8, 4);
 * b.push_back(0x88, 8);
 * b.fill_zeros_left(32);
 * b.Output(std::cout); // b contains now: 00000000 00000000 10001000 10001000
 * \endcode
 */
class ALVAR_EXPORT Bitset {
protected:
	std::deque<bool> bits;
	
public:
	/** \brief The length of the \e Bitset */
	int Length();
	/** \brief Output the bits to selected ostream 
	 *  \param os The output stream to be used for outputting e.g. std::cout
	 */
	std::ostream &Output(std::ostream &os) const;
	/** \brief Clear the bits */
	void clear();
	/** \brief Push back one bit
	 *  \param bit Boolean (true/false) to be pushed to the end of bit sequence.
	 */
	void push_back(const bool bit);
	/** \brief Push back \e bit_count bits from 'byte' \e b
	 *  \param b Unsigned character (8-bits) to be pushed to the end of bit sequence.
	 *  \param bit_count Number of bits to be pushed (default/max is 8 bits)
	 */
	void push_back(const unsigned char b, const int bit_count=8);
	/** \brief Push back \e bit_count bits from 'short' \e s
	 *  \param s Unsigned short (16-bits) to be pushed to the end of bit sequence.
	 *  \param bit_count Number of bits to be pushed (default/max is 16 bits)
	 */
	void push_back(const unsigned short s, const int bit_count=16);
	/** \brief Push back \e bit_count bits from 'long' \e l
	 *  \param l Unsigned long (32-bits) to be pushed to the end of bit sequence.
	 *  \param bit_count Number of bits to be pushed (default/max is 32 bits)
	 */
	void push_back(const unsigned long l, const int bit_count=32);
	/** \brief Push back meaningful bits from 'long' \e l
	 *  \param l The meaningful bits of the given unsigned long (32-bits) are pushed to the end of bit sequence.
	 */
	void push_back_meaningful(const unsigned long l);
	/** \brief Fill the \e Bitset with non-meaningful zeros 
	 *  \param bit_count Non-meaningful zeros are added until this given \e bit_count is reached.
	 */
	void fill_zeros_left(const size_t bit_count);
	/** \brief Push back a string of characters 
	 *  \param s String of characters to be pushed to the end of bit sequence.
	 */
	void push_back(std::string s);
	/** \brief Pop the front bit */
	bool pop_front();
	/** \brief Pop the back bit */
	bool pop_back();
	/** \brief Flip the selected bit 
	 *  \param pos The bit in this given position is flipped.
	 */
	void flip(size_t pos);
	/** \brief The \e Bitset as a hex string */
	std::string hex();
	/** \brief The \e Bitset as 'unsigned long' */
	unsigned long ulong();
	/** \brief The \e Bitset as 'unsigned char' */
	unsigned char uchar();
	/** \brief The \e Bitset as 'deque<bool>' */
	inline std::deque<bool>& GetBits()
	{
		return bits;
	}
};

/**
 * \brief An extended \e Bitset ( \e BitsetExt ) for handling e.g. Hamming encoding/decoding
 *
 * This class is based on the basic \e Bitset. It provides additional features for Hamming coding
 * (See http://en.wikipedia.org/wiki/Hamming_code).
 *
 * The \e BitsetExt is used e.g by \e MarkerData
 */
class ALVAR_EXPORT BitsetExt : public Bitset {
protected:
	bool verbose;
	void hamming_enc_block(unsigned long block_len, std::deque<bool>::iterator &iter);
	int hamming_dec_block(unsigned long block_len, std::deque<bool>::iterator &iter);
public:
	/** \brief Constructor */
	BitsetExt();
	/** \brief Constructor */
	BitsetExt(bool _verbose);
	/** \brief Set the verbose/silent mode */
	void SetVerbose(bool _verbose);
	/** \brief Count how many bits will be in the Bitset after hamming encoding */
	static int count_hamming_enc_len(int block_len, int dec_len);
	/** \brief Count how many bits will be in the Bitset after hamming decoding */
	static int count_hamming_dec_len(int block_len, int enc_len);
	/** \brief Hamming encoding 'in-place' using the defined block length */
	void hamming_enc(int block_len);
	/** \brief Hamming decoding 'in-place' using the defined block length */
	int hamming_dec(int block_len);
};

} // namespace alvar

#endif
