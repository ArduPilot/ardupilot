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

class BipBuffer
{
private:
    uint8_t* pBuffer; // Pointer to the data buffer
    uint16_t ixa; // Starting index of region A
    uint16_t sza; // Size of region A
    uint16_t ixb; // Starting index of region B
    uint16_t szb; // Size of region B
    uint16_t buflen; // Length of full buffer
    uint16_t ixResrv; // Starting index of reserved region
    uint16_t szResrv; // Size of the reserved region

public:
    BipBuffer() : pBuffer(NULL), ixa(0), sza(0), ixb(0), szb(0), buflen(0), ixResrv(0), szResrv(0)
    {
      
    }
    BipBuffer(uint8_t* buffer_in, uint16_t buffer_length_in) : ixa(0), sza(0), ixb(0), szb(0), ixResrv(0), szResrv(0)
    {
      pBuffer = buffer_in;
      buflen = buffer_length_in;
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
        ixa = sza = ixb = szb = ixResrv = szResrv = 0;
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
        if (szb)
        {
            uint16_t freespace = GetBFreeSpace();

            if (size < freespace) freespace = size;

            if (freespace == 0) return NULL;

            szResrv = freespace;
            reserved = freespace;
            ixResrv = ixb + szb;
            return pBuffer + ixResrv;
        }
        else
        {
            // Block b does not exist, so we can check if the space AFTER a is bigger than the space
            // before A, and allocate the bigger one.
            uint16_t freespace = GetSpaceAfterA();
            if (freespace >= ixa) // If space after A > space before
            {
                if (freespace == 0) return NULL;
                if (size < freespace) freespace = size;

                szResrv = freespace;
                reserved = freespace;
                ixResrv = ixa + sza;
                return pBuffer + ixResrv;
            }
            else // space before A > space after A
            {
                if (ixa == 0) return NULL;
                if (ixa < size) size = ixa;
                szResrv = size;
                reserved = size;
                ixResrv = 0;
                return pBuffer;
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
      if (size == 0)
      {
        // decommit any reservation
        szResrv = ixResrv = 0;
        return;
      }

      CommitPartial(size);
      
      // Decommit rest of reservation
      ixResrv = 0;
      szResrv = 0;
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
      if (size > szResrv)
      {
        size = szResrv;
      }

      // If we have no blocks being used currently, we create one in A.
      if (sza == 0 && szb == 0)
      {
        ixa = ixResrv;
      }

      // If the reserve index is at the end of block A
      if (ixResrv == sza + ixa)
      {
        sza += size; // Grow A by committed size
      }
      else
      {
        szb += size; // Otherwise grow B by committed size
      }
      
      // Advance the reserved index
      ixResrv += size;
      // Update reserved size
      szResrv -= size;
      
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
        if (sza == 0)
        {
            size = 0;
            return NULL;
        }

        size = sza;
        return pBuffer + ixa;

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
        if (size >= sza)
        {
            ixa = ixb;
            sza = szb;
            ixb = 0;
            szb = 0;
        }
        else
        {
            sza -= size;
            ixa += size;
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
        return sza + szb;
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
        return szResrv;
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
        return buflen;
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
        return pBuffer != NULL;
    }

private:
    uint16_t GetSpaceAfterA() const
    {
        return buflen - ixa - sza;
    }

    uint16_t GetBFreeSpace() const
    {
        return ixa - ixb - szb;
    }
};

#endif // BipBuffer_H