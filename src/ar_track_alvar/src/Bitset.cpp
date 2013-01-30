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

#include "Bitset.h"

using namespace std;

namespace alvar {
using namespace std;

int Bitset::Length() {
	return bits.size();
}
ostream &Bitset::Output(ostream &os) const {
	deque<bool>::const_iterator iter=bits.begin();
	while (iter != bits.end()) {
		if (*iter) os<<"1";
		else os<<"0";
		iter++;
	}
	return os;
}
void Bitset::clear() { bits.clear(); }
void Bitset::push_back(const bool bit) { bits.push_back(bit); }
void Bitset::push_back(const unsigned char b, int bit_count /*=8*/) {
	push_back((const unsigned long)b, bit_count);
}
void Bitset::push_back(const unsigned short s, int bit_count /*=16*/) {
	push_back((const unsigned long)s, bit_count);
}
void Bitset::push_back(const unsigned long l, int bit_count /*=32*/) {
	unsigned long mask;
	if ((bit_count > 32) || (bit_count == 0)) bit_count=32;
	mask = 0x01<<(bit_count-1);
	for (int i=0; i<bit_count; i++) {
		if (l & mask) push_back(true);
		else push_back(false);
		mask>>=1;
	}
}
void Bitset::push_back_meaningful(const unsigned long l) {
	int bit_count = 1;
	for (int i=0; i<32; i++) {
		unsigned long mask = 0x01<<i;
		if (l & mask) bit_count = i+1;
	}
	push_back(l, bit_count);
}
void Bitset::fill_zeros_left(size_t bit_count) {
	while (bits.size() < bit_count) {
		bits.push_front(false);
	}
}

void Bitset::push_back(string s) {
	string::const_iterator iter = s.begin();
	while (iter != s.end()) {
		unsigned char c = *iter;
		push_back(c);
		iter++;
	}
}
bool Bitset::pop_front()
{
	bool ret = bits.front();
	bits.pop_front();	
	return ret;
}
bool Bitset::pop_back()
{
	bool ret = bits.back();
	bits.pop_back();	
	return ret;
}

void Bitset::flip(size_t pos) {
	bits[pos] = !bits[pos];
}

string Bitset::hex() 
{
	stringstream ss;
	ss.unsetf(std::ios_base::dec);
	ss.setf(std::ios_base::hex);
	unsigned long b=0;
	int bitpos = (0x08 << (bits.size() % 4));
	if (bitpos > 0x08) bitpos >>= 4;
	for (size_t i=0; i < bits.size(); i++) {
		if (bits[i]) b = b | bitpos;
		else b = b & (0x0f ^ bitpos);
		bitpos >>= 1;
		if (bitpos == 0x00) {
			bitpos = 0x08;
			ss << b;
		}
	}
	return ss.str();
}

unsigned long Bitset::ulong()
{
	//if(bits.size() > (sizeof(unsigned long)*8))
	//	throw "code too big for unsigned long\n";
	stringstream ss;
	ss << setbase(16) << hex();
	unsigned long v;
	ss >> v;
	return v;
}

unsigned char Bitset::uchar()
{
	//if(bits.size() > (sizeof(unsigned char)*8))
	//	throw "code too big for unsigned char\n";
	stringstream ss;
	ss << setbase(16) << hex();
	unsigned long v; //ttehop: Note, that this cannot be char
	ss >> v;
	return (unsigned char)v;
}

void BitsetExt::hamming_enc_block(unsigned long block_len, deque<bool>::iterator &iter) {
	if (verbose) cout<<"hamming_enc_block: ";
	unsigned long next_parity=1;
	for (unsigned long i=1; i<=block_len; i++) {
		// Add a parity bit if this a place for such
		if (i == next_parity) {
			if (verbose) cout<<"p";
			next_parity <<= 1;
			iter = bits.insert(iter, false);
		} 
		// Otherwise if this bit is 1 change all related parity bits
		else {
			if (iter == bits.end()) {
				block_len = i-1;
				break;
			}
			if (verbose) cout<<(*iter?1:0);
			if (*iter) {
				unsigned long parity = next_parity>>1;
				while (parity) {
					if (i & parity) {
						deque<bool>::iterator parity_iter=(iter - (i - parity));
						*parity_iter = !*parity_iter;
					}
					parity >>= 1;
				}
			}
		}
		iter++;
	}
	// Update the last parity bit if we have one
	// Note, that the last parity bit can safely be removed from the code if it is not desired...
	if (block_len == (next_parity >> 1)) {
		// If the last bit is parity bit - make parity over the previous data
		for (unsigned long ii=1; ii<block_len; ii++) {
			if (*(iter-ii-1)) *(iter-1) = !*(iter-1);
		}
	}
	if (verbose) {
		cout<<" -> ";
		for (unsigned long ii=block_len; ii>=1; ii--) {
			cout<<(*(iter-ii)?1:0);
		}
		cout<<" block_len: "<<block_len<<endl;
	}
}
int BitsetExt::hamming_dec_block(unsigned long block_len, deque<bool>::iterator &iter) {
	if (verbose) cout<<"hamming_dec_block: ";
	bool potentially_double_error = false;
	unsigned long total_parity=0;
	unsigned long parity=0;
	unsigned long next_parity=1;
	for (unsigned long i=1; i<=block_len; i++) {
		if (iter == bits.end()) {
			// ttehop: 
			// At 3.12.2009 I changed the following line because
			// it crashed with 7x7 markers. However, I didn't fully
			// understand the reason why it should be so. Lets
			// give more thought to it when we have more time.
			// old version: block_len = i-1;
			block_len = i;
			break;
		}
		if (*iter) {
			parity = parity ^ i;
			total_parity = total_parity ^ 1;
		}
		if (i == next_parity) {
			if (verbose) cout<<"("<<*iter<<")";
			next_parity <<= 1;
			iter = bits.erase(iter);
		} else {
			if (verbose) cout<<*iter;
			iter++;
		}
	}
	if (block_len < 3)  {
		if (verbose) cout<<" too short"<<endl;
		return 0;
	}
	if (block_len == (next_parity >> 1)) {
		parity = parity & ~(next_parity >> 1); // The last parity bit shouldn't be included in the other parity tests (TODO: Better solution)
		if (total_parity == 0) {
			potentially_double_error = true;
		}
	}
	int steps=0;
	if (verbose) cout<<" parity: "<<parity;
	if (parity) {
		if (potentially_double_error) {
			if (verbose) cout<<" double error"<<endl;
			return -1;
		}
		next_parity = 1;
		for (unsigned long i=1; i<=block_len; i++) {
			if (i == next_parity) {
				next_parity <<= 1;
				if (i == parity) {
					if (verbose) cout<<" parity bit error"<<endl;
					return 1; // Only parity bit was erroneous
				}
			} else if (i >= parity) {
				steps++;
			}
		}
		iter[-steps] = !iter[-steps];
		if (verbose) cout<<" corrected"<<endl;
		return 1;
	}
	if (verbose) cout<<" ok"<<endl;
	return 0;
}
BitsetExt::BitsetExt() {
	SetVerbose(false);
}
BitsetExt::BitsetExt(bool _verbose) {
	SetVerbose(_verbose);
}
/*
void BitsetExt::str_8to6bit() {
	// TODO: Assume that the bitset contains 8-bit ASCII chars and change them into 6-bit chars
}
void BitsetExt::str_6to8bit() {
	// TODO: Assume that the bitset contains 6-bit chars and change them into 8-bit ASCII chars
}
void BitsetExt::crc_enc(int crc_len) {
	// TODO: Add crc_len-bit checksum for the bitset
}
bool BitsetExt::crc_dec(int crc_len) {
	// TODO: Check the CRC
	return false;
}
*/
void BitsetExt::SetVerbose(bool _verbose) {
	verbose = _verbose;
}
int BitsetExt::count_hamming_enc_len(int block_len, int dec_len) {
	int parity_len=0;
	int dec_len_count = dec_len;
	while (dec_len_count > 0) {
		unsigned long next_parity = 1;
		for (unsigned long i=1; i<=(unsigned long)block_len; i++) {
			if (i == next_parity) {
				parity_len++;
				next_parity <<= 1;
			} else {
				dec_len_count--;
			}
			if (dec_len_count == 0) break;
		}
	}
	return dec_len + parity_len;
}
int BitsetExt::count_hamming_dec_len(int block_len, int enc_len) {
	int parity_len=0;
	int enc_len_count = enc_len;
	while (enc_len_count > 0) {
		unsigned long next_parity = 1;
		unsigned long i;
		for (i=1; i<=(unsigned long)block_len; i++) {
			if (i == next_parity) {
				parity_len++;
				next_parity <<= 1;
			}
			enc_len_count--;
			if (enc_len_count == 0) break;
		}
	}
	return enc_len - parity_len;
}
void BitsetExt::hamming_enc(int block_len) {
	deque<bool>::iterator iter=bits.begin();
	while (iter != bits.end()) {
		hamming_enc_block(block_len, iter);
	}
}
// Returns number of corrected errors (or -1 if there were unrecoverable error)
int BitsetExt::hamming_dec(int block_len) {
	int error_count=0;
	deque<bool>::iterator iter=bits.begin();
	while (iter != bits.end()) {
		int error=hamming_dec_block(block_len, iter);
		if ((error == -1) || (error_count == -1)) error_count=-1;
		else error_count += error;
	}
	return error_count;
}

} // namespace alvar
