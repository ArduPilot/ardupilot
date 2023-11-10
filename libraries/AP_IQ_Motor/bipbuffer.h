/*
   Copyright (c) 2003 Simon Cooke, All Rights Reserved

   Licensed royalty-free for commercial and non-commercial
   use, without warranty or guarantee of suitability for any purpose.
   All that I ask is that you send me an email
   telling me that you're using my code. It'll make me
   feel warm and fuzzy inside. spectecjr@gmail.com

*/

/*
  Name: bibuffer.h
  Last update: 3/7/2019 by Raphael Van Hoffelen
  Author: Simon Cooke
  Contributors: Matthew Piccoli, Raphael Van Hoffelen
*/

/*
  Changed header gaurd from pragma once to ifndef BipBuffer_H.
  Chaning types to fixed width integers.
  Removed destructor, AllocateBuffer, FreeBuffer.
  Constructor expects array and size to be passed in.
  Including stddef for NULL
  Matthew Piccoli, 4/1/2016
*/

#ifndef BipBuffer_H
#define BipBuffer_H

#include <stddef.h>

class Bip_Buffer
{
private:
    uint8_t* _pBuffer; // Pointer to the data buffer
    uint16_t _ixa; // Starting index of region A
    uint16_t _sza; // Size of region A
    uint16_t _ixb; // Starting index of region B
    uint16_t _szb; // Size of region B
    uint16_t _buflen; // Length of full buffer
    uint16_t _ixResrv; // Starting index of reserved region
    uint16_t _szResrv; // Size of the reserved region

public:
    Bip_Buffer() : _pBuffer(NULL), _ixa(0), _sza(0), _ixb(0), _szb(0), _buflen(0), _ixResrv(0), _szResrv(0)
    {

    }
    Bip_Buffer(uint8_t* buffer_in, uint16_t buffer_length_in) : _ixa(0), _sza(0), _ixb(0), _szb(0), _ixResrv(0), _szResrv(0)
    {
        _pBuffer = buffer_in;
        _buflen = buffer_length_in;
    }

    ///
    /// \brief Clears the buffer of any allocations.
    ///
    /// Clears the buffer of any allocations or reservations. Note; it
    /// does not wipe the buffer memory; it merely resets all pointers,
    /// returning the buffer to a completely empty state ready for new
    /// allocations.
    ///
    void Clear()
    {
        _ixa = _sza = _ixb = _szb = _ixResrv = _szResrv = 0;
    }

    // Reserve
    //
    // Reserves space in the buffer for a memory write operation
    //
    // Parameters:
    //   int size                amount of space to reserve
    //   uint16_t& reserved        size of space actually reserved
    //
    // Returns:
    //   uint8_t*                    pointer to the reserved block
    //
    // Notes:
    //   Will return NULL for the pointer if no space can be allocated.
    //   Can return any value from 1 to size in reserved.
    //   Will return NULL if a previous reservation has not been committed.
    uint8_t* Reserve(uint16_t size, uint16_t& reserved)
    {
        // We always allocate on B if B exists; this means we have two blocks and our buffer is filling.
        if (_szb) {
            uint16_t freespace = GetBFreeSpace();

            if (size < freespace) {
                freespace = size;
            }

            if (freespace == 0) {
                return NULL;
            }

            _szResrv = freespace;
            reserved = freespace;
            _ixResrv = _ixb + _szb;
            return _pBuffer + _ixResrv;
        } else {
            // Block b does not exist, so we can check if the space AFTER a is bigger than the space
            // before A, and allocate the bigger one.
            uint16_t freespace = GetSpaceAfterA();
            if (freespace >= _ixa) { // If space after A > space before
                if (freespace == 0) {
                    return NULL;
                }
                if (size < freespace) {
                    freespace = size;
                }

                _szResrv = freespace;
                reserved = freespace;
                _ixResrv = _ixa + _sza;
                return _pBuffer + _ixResrv;
            } else { // space before A > space after A
                if (_ixa == 0) {
                    return NULL;
                }
                if (_ixa < size) {
                    size = _ixa;
                }
                _szResrv = size;
                reserved = size;
                _ixResrv = 0;
                return _pBuffer;
            }
        }
    }

    // Commit
    //
    // Commits space that has been written to in the buffer
    //
    // Parameters:
    //   uint16_t size                number of bytes to commit
    //
    // Notes:
    //   Committing a size > than the reserved size will cause an assert in a debug build;
    //   in a release build, the actual reserved size will be used.
    //   Committing a size < than the reserved size will commit that amount of data, and release
    //   the rest of the space.
    //   Committing a size of 0 will release the reservation.
    void Commit(uint16_t size)
    {
        if (size == 0) {
            // decommit any reservation
            _szResrv = _ixResrv = 0;
            return;
        }

        CommitPartial(size);

        // Decommit rest of reservation
        _ixResrv = 0;
        _szResrv = 0;
    }

    // CommitPartial
    //
    // Commits space that has been written to in the buffer, but does not cancel the rest of the reservation
    //
    // Parameters:
    //   uint16_t size                number of bytes to commit
    //
    // Returns:
    //   uint16_t                     number of bytes actually committed
    //
    // Notes:
    //   Committing a size > than the reserved size will cause an assert in a debug build;
    //   in a release build, the actual reserved size will be used.
    //   Committing a size < than the reserved size will commit that amount of data, but will not realese the remaining reservation
    //   Committing a size of 0 does nothing (space is still reserved)
    uint16_t CommitPartial(uint16_t size)
    {
        // If we try to commit more space than we asked for, clip to the size we asked for.
        if (size > _szResrv) {
            size = _szResrv;
        }

        // If we have no blocks being used currently, we create one in A.
        if (_sza == 0 && _szb == 0) {
            _ixa = _ixResrv;
        }

        // If the reserve index is at the end of block A
        if (_ixResrv == _sza + _ixa) {
            _sza += size; // Grow A by committed size
        } else {
            _szb += size; // Otherwise grow B by committed size
        }

        // Advance the reserved index
        _ixResrv += size;
        // Update reserved size
        _szResrv -= size;

        return size;
    }

    // GetContiguousBlock
    //
    // Gets a pointer to the first contiguous block in the buffer, and returns the size of that block.
    //
    // Parameters:
    //   uint16_t & size            returns the size of the first contiguous block
    //
    // Returns:
    //   uint8_t*                    pointer to the first contiguous block, or NULL if empty.
    uint8_t* GetContiguousBlock(uint16_t& size)
    {
        if (_sza == 0) {
            size = 0;
            return NULL;
        }

        size = _sza;
        return _pBuffer + _ixa;

    }

    // DecommitBlock
    //
    // Decommits space from the first contiguous block
    //
    // Parameters:
    //   int size                amount of memory to decommit
    //
    // Returns:
    //   nothing
    void DecommitBlock(uint16_t size)
    {
        if (size >= _sza) {
            _ixa = _ixb;
            _sza = _szb;
            _ixb = 0;
            _szb = 0;
        } else {
            _sza -= size;
            _ixa += size;
        }
    }

    // GetCommittedSize
    //
    // Queries how much data (in total) has been committed in the buffer
    //
    // Parameters:
    //   none
    //
    // Returns:
    //   uint16_t         total amount of committed data in the buffer
    uint16_t GetCommittedSize() const
    {
        return _sza + _szb;
    }

    // GetReservationSize
    //
    // Queries how much space has been reserved in the buffer.
    //
    // Parameters:
    //   none
    //
    // Returns:
    //   uint16_t                    number of bytes that have been reserved
    //
    // Notes:
    //   A return value of 0 indicates that no space has been reserved
    uint16_t GetReservationSize() const
    {
        return _szResrv;
    }

    // GetBufferSize
    //
    // Queries the maximum total size of the buffer
    //
    // Parameters:
    //   none
    //
    // Returns:
    //   uint16_t                    total size of buffer
    uint16_t GetBufferSize() const
    {
        return _buflen;
    }

    // IsInitialized
    //
    // Queries whether or not the buffer has been allocated
    //
    // Parameters:
    //   none
    //
    // Returns:
    //   uint8_t                    true if the buffer has been allocated
    uint8_t IsInitialized() const
    {
        return _pBuffer != NULL;
    }

private:
    uint16_t GetSpaceAfterA() const
    {
        return _buflen - _ixa - _sza;
    }

    uint16_t GetBFreeSpace() const
    {
        return _ixa - _ixb - _szb;
    }
};

#endif // BipBuffer_H