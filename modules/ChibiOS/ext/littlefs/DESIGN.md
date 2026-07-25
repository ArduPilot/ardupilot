## The design of littlefs

A little fail-safe filesystem designed for microcontrollers.

```
   | | |     .---._____
  .-----.   |          |
--|o    |---| littlefs |
--|     |---|          |
  '-----'   '----------'
   | | |
```

littlefs was originally built as an experiment to learn about filesystem design
in the context of microcontrollers. The question was: How would you build a
filesystem that is resilient to power-loss and flash wear without using
unbounded memory?

This document covers the high-level design of littlefs, how it is different
than other filesystems, and the design decisions that got us here. For the
low-level details covering every bit on disk, check out [SPEC.md](SPEC.md).

## The problem

The embedded systems littlefs targets are usually 32-bit microcontrollers with
around 32 KiB of RAM and 512 KiB of ROM. These are often paired with SPI NOR
flash chips with about 4 MiB of flash storage. These devices are too small for
Linux and most existing filesystems, requiring code written specifically with
size in mind.

Flash itself is an interesting piece of technology with its own quirks and
nuance. Unlike other forms of storage, writing to flash requires two
operations: erasing and programming. Programming (setting bits to 0) is
relatively cheap and can be very granular. Erasing however (setting bits to 1),
requires an expensive and destructive operation which gives flash its name.
[Wikipedia][wikipedia-flash] has more information on how exactly flash works.

To make the situation more annoying, it's very common for these embedded
systems to lose power at any time. Usually, microcontroller code is simple and
reactive, with no concept of a shutdown routine. This presents a big challenge
for persistent storage, where an unlucky power loss can corrupt the storage and
leave a device unrecoverable.

This leaves us with three major requirements for an embedded filesystem.

1. **Power-loss resilience** - On these systems, power can be lost at any time.
   If a power loss corrupts any persistent data structures, this can cause the
   device to become unrecoverable. An embedded filesystem must be designed to
   recover from a power loss during any write operation.

1. **Wear leveling** - Writing to flash is destructive. If a filesystem
   repeatedly writes to the same block, eventually that block will wear out.
   Filesystems that don't take wear into account can easily burn through blocks
   used to store frequently updated metadata and cause a device's early death.

1. **Bounded RAM/ROM** - If the above requirements weren't enough, these
   systems also have very limited amounts of memory. This prevents many
   existing filesystem designs, which can lean on relatively large amounts of
   RAM to temporarily store filesystem metadata.

   For ROM, this means we need to keep our design simple and reuse code paths
   were possible. For RAM we have a stronger requirement, all RAM usage is
   bounded. This means RAM usage does not grow as the filesystem changes in
   size or number of files. This creates a unique challenge as even presumably
   simple operations, such as traversing the filesystem, become surprisingly
   difficult.

## Existing designs?

So, what's already out there? There are, of course, many different filesystems,
however they often share and borrow feature from each other. If we look at
power-loss resilience and wear leveling, we can narrow these down to a handful
of designs.

1. First we have the non-resilient, block based filesystems, such as [FAT] and
   [ext2]. These are the earliest filesystem designs and often the most simple.
   Here storage is divided into blocks, with each file being stored in a
   collection of blocks. Without modifications, these filesystems are not
   power-loss resilient, so updating a file is a simple as rewriting the blocks
   in place.

   ```
               .--------.
               |  root  |
               |        |
               |        |
               '--------'
               .-'    '-.
              v          v
         .--------.  .--------.
         |   A    |  |   B    |
         |        |  |        |
         |        |  |        |
         '--------'  '--------'
         .-'         .-'    '-.
        v           v          v
   .--------.  .--------.  .--------.
   |   C    |  |   D    |  |   E    |
   |        |  |        |  |        |
   |        |  |        |  |        |
   '--------'  '--------'  '--------'
   ```

   Because of their simplicity, these filesystems are usually both the fastest
   and smallest. However the lack of power resilience is not great, and the
   binding relationship of storage location and data removes the filesystem's
   ability to manage wear.

2. In a completely different direction, we have logging filesystems, such as
   [JFFS], [YAFFS], and [SPIFFS], storage location is not bound to a piece of
   data, instead the entire storage is used for a circular log which is
   appended with every change made to the filesystem. Writing appends new
   changes, while reading requires traversing the log to reconstruct a file.
   Some logging filesystems cache files to avoid the read cost, but this comes
   at a tradeoff of RAM.

   ```
                                                             v
   .--------.--------.--------.--------.--------.--------.--------.--------.
   |        C        | new B  | new A  |                 |   A    |   B    |
   |                 |        |        |->               |        |        |
   |                 |        |        |                 |        |        |
   '--------'--------'--------'--------'--------'--------'--------'--------'
   ```

   Logging filesystem are beautifully elegant. With a checksum, we can easily
   detect power-loss and fall back to the previous state by ignoring failed
   appends. And if that wasn't good enough, their cyclic nature means that
   logging filesystems distribute wear across storage perfectly.

   The main downside is performance. If we look at garbage collection, the
   process of cleaning up outdated data from the end of the log, I've yet to
   see a pure logging filesystem that does not have one of these two costs:

   1. _O(n&sup2;)_ runtime
   2. _O(n)_ RAM

   SPIFFS is a very interesting case here, as it uses the fact that repeated
   programs to NOR flash is both atomic and masking. This is a very neat
   solution, however it limits the type of storage you can support.

3. Perhaps the most common type of filesystem, a journaling filesystem is the
   offspring that happens when you mate a block based filesystem with a logging
   filesystem. [ext4] and [NTFS] are good examples. Here, we take a normal
   block based filesystem and add a bounded log where we note every change
   before it occurs.

   ```
                             journal
                            .--------.--------.
               .--------.   | C'| D'|     | E'|
               |  root  |-->|   |   |->   |   |
               |        |   |   |   |     |   |
               |        |   '--------'--------'
               '--------'
               .-'    '-.
              v          v
         .--------.  .--------.
         |   A    |  |   B    |
         |        |  |        |
         |        |  |        |
         '--------'  '--------'
         .-'         .-'    '-.
        v           v          v
   .--------.  .--------.  .--------.
   |   C    |  |   D    |  |   E    |
   |        |  |        |  |        |
   |        |  |        |  |        |
   '--------'  '--------'  '--------'
   ```


   This sort of filesystem takes the best from both worlds. Performance can be
   as fast as a block based filesystem (though updating the journal does have
   a small cost), and atomic updates to the journal allow the filesystem to
   recover in the event of a power loss.

   Unfortunately, journaling filesystems have a couple of problems. They are
   fairly complex, since there are effectively two filesystems running in
   parallel, which comes with a code size cost. They also offer no protection
   against wear because of the strong relationship between storage location
   and data.

4. Last but not least we have copy-on-write (COW) filesystems, such as
   [btrfs] and [ZFS]. These are very similar to other block based filesystems,
   but instead of updating block inplace, all updates are performed by creating
   a copy with the changes and replacing any references to the old block with
   our new block. This recursively pushes all of our problems upwards until we
   reach the root of our filesystem, which is often stored in a very small log.

   ```
               .--------.                  .--------.
               |  root  |       write      |new root|
               |        |        ==>       |        |
               |        |                  |        |
               '--------'                  '--------'
               .-'    '-.                    |    '-.
              |  .-------|------------------'        v
              v v        v                       .--------.
         .--------.  .--------.                  | new B  |
         |   A    |  |   B    |                  |        |
         |        |  |        |                  |        |
         |        |  |        |                  '--------'
         '--------'  '--------'                  .-'    |
         .-'         .-'    '-.    .------------|------'
        |           |          |  |             v
        v           v          v  v        .--------.
   .--------.  .--------.  .--------.      | new D  |
   |   C    |  |   D    |  |   E    |      |        |
   |        |  |        |  |        |      |        |
   |        |  |        |  |        |      '--------'
   '--------'  '--------'  '--------'
   ```

   COW filesystems are interesting. They offer very similar performance to
   block based filesystems while managing to pull off atomic updates without
   storing data changes directly in a log. They even disassociate the storage
   location of data, which creates an opportunity for wear leveling.

   Well, almost. The unbounded upwards movement of updates causes some
   problems. Because updates to a COW filesystem don't stop until they've
   reached the root, an update can cascade into a larger set of writes than
   would be needed for the original data. On top of this, the upward motion
   focuses these writes into the block, which can wear out much earlier than
   the rest of the filesystem.

## littlefs

So what does littlefs do?

If we look at existing filesystems, there are two interesting design patterns
that stand out, but each have their own set of problems. Logging, which
provides independent atomicity, has poor runtime performance. And COW data
structures, which perform well, push the atomicity problem upwards.

Can we work around these limitations?

Consider logging. It has either a _O(n&sup2;)_ runtime or _O(n)_ RAM cost. We
can't avoid these costs, _but_ if we put an upper bound on the size we can at
least prevent the theoretical cost from becoming problem. This relies on the
super secret computer science hack where you can pretend any algorithmic
complexity is _O(1)_ by bounding the input.

In the case of COW data structures, we can try twisting the definition a bit.
Let's say that our COW structure doesn't copy after a single write, but instead
copies after _n_ writes. This doesn't change most COW properties (assuming you
can write atomically!), but what it does do is prevent the upward motion of
wear. This sort of copy-on-bounded-writes (CObW) still focuses wear, but at
each level we divide the propagation of wear by _n_. With a sufficiently
large _n_ (&gt; branching factor) wear propagation is no longer a problem.

See where this is going? Separate, logging and COW are imperfect solutions and
have weaknesses that limit their usefulness. But if we merge the two they can
mutually solve each other's limitations.

This is the idea behind littlefs. At the sub-block level, littlefs is built
out of small, two block logs that provide atomic updates to metadata anywhere
on the filesystem. At the super-block level, littlefs is a CObW tree of blocks
that can be evicted on demand.

```
                    root
                   .--------.--------.
                   | A'| B'|         |
                   |   |   |->       |
                   |   |   |         |
                   '--------'--------'
                .----'   '--------------.
       A       v                 B       v
      .--------.--------.       .--------.--------.
      | C'| D'|         |       | E'|new|         |
      |   |   |->       |       |   | E'|->       |
      |   |   |         |       |   |   |         |
      '--------'--------'       '--------'--------'
      .-'   '--.                  |   '------------------.
     v          v              .-'                        v
.--------.  .--------.        v                       .--------.
|   C    |  |   D    |   .--------.       write       | new E  |
|        |  |        |   |   E    |        ==>        |        |
|        |  |        |   |        |                   |        |
'--------'  '--------'   |        |                   '--------'
                         '--------'                   .-'    |
                         .-'    '-.    .-------------|------'
                        v          v  v              v
                   .--------.  .--------.       .--------.
                   |   F    |  |   G    |       | new F  |
                   |        |  |        |       |        |
                   |        |  |        |       |        |
                   '--------'  '--------'       '--------'
```

There are still some minor issues. Small logs can be expensive in terms of
storage, in the worst case a small log costs 4x the size of the original data.
CObW structures require an efficient block allocator since allocation occurs
every _n_ writes. And there is still the challenge of keeping the RAM usage
constant.

## Metadata pairs

Metadata pairs are the backbone of littlefs. These are small, two block logs
that allow atomic updates anywhere in the filesystem.

Why two blocks? Well, logs work by appending entries to a circular buffer
stored on disk. But remember that flash has limited write granularity. We can
incrementally program new data onto erased blocks, but we need to erase a full
block at a time. This means that in order for our circular buffer to work, we
need more than one block.

We could make our logs larger than two blocks, but the next challenge is how
do we store references to these logs? Because the blocks themselves are erased
during writes, using a data structure to track these blocks is complicated.
The simple solution here is to store a two block addresses for every metadata
pair. This has the added advantage that we can change out blocks in the
metadata pair independently, and we don't reduce our block granularity for
other operations.

In order to determine which metadata block is the most recent, we store a
revision count that we compare using [sequence arithmetic][wikipedia-sna]
(very handy for avoiding problems with integer overflow). Conveniently, this
revision count also gives us a rough idea of how many erases have occurred on
the block.

```
metadata pair pointer: {block 0, block 1}
                           |        '--------------------.
                            '-.                           |
disk                           v                          v
.--------.--------.--------.--------.--------.--------.--------.--------.
|                 |        |metadata|                 |metadata|        |
|                 |        |block 0 |                 |block 1 |        |
|                 |        |        |                 |        |        |
'--------'--------'--------'--------'--------'--------'--------'--------'
                               '--.                  .----'
                                   v                v
             metadata pair .----------------.----------------.
                           |   revision 11  |   revision 12  |
             block 1 is    |----------------|----------------|
             most recent   |       A        |       A''      |
                           |----------------|----------------|
                           |    checksum    |    checksum    |
                           |----------------|----------------|
                           |       B        |       A'''     | <- most recent A
                           |----------------|----------------|
                           |       A''      |    checksum    |
                           |----------------|----------------|
                           |    checksum    |       |        |
                           |----------------|       v        |
                           '----------------'----------------'
```

So how do we atomically update our metadata pairs? Atomicity (a type of
power-loss resilience) requires two parts: redundancy and error detection.
Error detection can be provided with a checksum, and in littlefs's case we
use a 32-bit [CRC][wikipedia-crc]. Maintaining redundancy, on the other hand,
requires multiple stages.

1. If our block is not full and the program size is small enough to let us
   append more entries, we can simply append the entries to the log. Because
   we don't overwrite the original entries (remember rewriting flash requires
   an erase), we still have the original entries if we lose power during the
   append.

   ```
                                    commit A
   .----------------.----------------.    .----------------.----------------.
   |   revision 1   |   revision 0   | => |   revision 1   |   revision 0   |
   |----------------|----------------|    |----------------|----------------|
   |       |        |                |    |       A        |                |
   |       v        |                |    |----------------|                |
   |                |                |    |    checksum    |                |
   |                |                |    |----------------|                |
   |                |                |    |       |        |                |
   |                |                |    |       v        |                |
   |                |                |    |                |                |
   |                |                |    |                |                |
   |                |                |    |                |                |
   |                |                |    |                |                |
   '----------------'----------------'    '----------------'----------------'
   ```

   Note that littlefs doesn't maintain a checksum for each entry. Many logging
   filesystems do this, but it limits what you can update in a single atomic
   operation. What we can do instead is group multiple entries into a commit
   that shares a single checksum. This lets us update multiple unrelated pieces
   of metadata as long as they reside on the same metadata pair.

   ```
                                 commit B and A'
   .----------------.----------------.    .----------------.----------------.
   |   revision 1   |   revision 0   | => |   revision 1   |   revision 0   |
   |----------------|----------------|    |----------------|----------------|
   |       A        |                |    |       A        |                |
   |----------------|                |    |----------------|                |
   |    checksum    |                |    |    checksum    |                |
   |----------------|                |    |----------------|                |
   |       |        |                |    |       B        |                |
   |       v        |                |    |----------------|                |
   |                |                |    |       A'       |                |
   |                |                |    |----------------|                |
   |                |                |    |    checksum    |                |
   |                |                |    |----------------|                |
   '----------------'----------------'    '----------------'----------------'
   ```

2. If our block _is_ full of entries, we need to somehow remove outdated
   entries to make space for new ones. This process is called garbage
   collection, but because littlefs has multiple garbage collectors, we
   also call this specific case compaction.

   Compared to other filesystems, littlefs's garbage collector is relatively
   simple. We want to avoid RAM consumption, so we use a sort of brute force
   solution where for each entry we check to see if a newer entry has been
   written. If the entry is the most recent we append it to our new block. This
   is where having two blocks becomes important, if we lose power we still have
   everything in our original block.

   During this compaction step we also erase the metadata block and increment
   the revision count. Because we can commit multiple entries at once, we can
   write all of these changes to the second block without worrying about power
   loss. It's only when the commit's checksum is written that the compacted
   entries and revision count become committed and readable.

   ```
                           commit B', need to compact
   .----------------.----------------.    .----------------.----------------.
   |   revision 1   |   revision 0   | => |   revision 1   |   revision 2   |
   |----------------|----------------|    |----------------|----------------|
   |       A        |                |    |       A        |       A'       |
   |----------------|                |    |----------------|----------------|
   |    checksum    |                |    |    checksum    |       B'       |
   |----------------|                |    |----------------|----------------|
   |       B        |                |    |       B        |    checksum    |
   |----------------|                |    |----------------|----------------|
   |       A'       |                |    |       A'       |       |        |
   |----------------|                |    |----------------|       v        |
   |    checksum    |                |    |    checksum    |                |
   |----------------|                |    |----------------|                |
   '----------------'----------------'    '----------------'----------------'
   ```

3. If our block is full of entries _and_ we can't find any garbage, then what?
   At this point, most logging filesystems would return an error indicating no
   more space is available, but because we have small logs, overflowing a log
   isn't really an error condition.

   Instead, we split our original metadata pair into two metadata pairs, each
   containing half of the entries, connected by a tail pointer. Instead of
   increasing the size of the log and dealing with the scalability issues
   associated with larger logs, we form a linked list of small bounded logs.
   This is a tradeoff as this approach does use more storage space, but at the
   benefit of improved scalability.

   Despite writing to two metadata pairs, we can still maintain power
   resilience during this split step by first preparing the new metadata pair,
   and then inserting the tail pointer during the commit to the original
   metadata pair.

   ```
                         commit C and D, need to split
   .----------------.----------------.    .----------------.----------------.
   |   revision 1   |   revision 2   | => |   revision 3   |   revision 2   |
   |----------------|----------------|    |----------------|----------------|
   |       A        |       A'       |    |       A'       |       A'       |
   |----------------|----------------|    |----------------|----------------|
   |    checksum    |       B'       |    |       B'       |       B'       |
   |----------------|----------------|    |----------------|----------------|
   |       B        |    checksum    |    |      tail    ---------------------.
   |----------------|----------------|    |----------------|----------------| |
   |       A'       |       |        |    |    checksum    |                | |
   |----------------|       v        |    |----------------|                | |
   |    checksum    |                |    |       |        |                | |
   |----------------|                |    |       v        |                | |
   '----------------'----------------'    '----------------'----------------' |
                                                   .----------------.---------'
                                                  v                v
                                          .----------------.----------------.
                                          |   revision 1   |   revision 0   |
                                          |----------------|----------------|
                                          |       C        |                |
                                          |----------------|                |
                                          |       D        |                |
                                          |----------------|                |
                                          |    checksum    |                |
                                          |----------------|                |
                                          |       |        |                |
                                          |       v        |                |
                                          |                |                |
                                          |                |                |
                                          '----------------'----------------'
   ```

There is another complexity the crops up when dealing with small logs. The
amortized runtime cost of garbage collection is not only dependent on its
one time cost (_O(n&sup2;)_ for littlefs), but also depends on how often
garbage collection occurs.

Consider two extremes:

1. Log is empty, garbage collection occurs once every _n_ updates
2. Log is full, garbage collection occurs **every** update

Clearly we need to be more aggressive than waiting for our metadata pair to
be full. As the metadata pair approaches fullness the frequency of compactions
grows very rapidly.

Looking at the problem generically, consider a log with ![n] bytes for each
entry, ![d] dynamic entries (entries that are outdated during garbage
collection), and ![s] static entries (entries that need to be copied during
garbage collection). If we look at the amortized runtime complexity of updating
this log we get this formula:

![cost = n + n (s / d+1)][metadata-formula1]

If we let ![r] be the ratio of static space to the size of our log in bytes, we
find an alternative representation of the number of static and dynamic entries:

![s = r (size/n)][metadata-formula2]

![d = (1 - r) (size/n)][metadata-formula3]

Substituting these in for ![d] and ![s] gives us a nice formula for the cost of
updating an entry given how full the log is:

![cost = n + n (r (size/n) / ((1-r) (size/n) + 1))][metadata-formula4]

Assuming 100 byte entries in a 4 KiB log, we can graph this using the entry
size to find a multiplicative cost:

![Metadata pair update cost graph][metadata-cost-graph]

So at 50% usage, we're seeing an average of 2x cost per update, and at 75%
usage, we're already at an average of 4x cost per update.

To avoid this exponential growth, instead of waiting for our metadata pair
to be full, we split the metadata pair once we exceed 50% capacity. We do this
lazily, waiting until we need to compact before checking if we fit in our 50%
limit. This limits the overhead of garbage collection to 2x the runtime cost,
giving us an amortized runtime complexity of _O(1)_.

---

If we look at metadata pairs and linked-lists of metadata pairs at a high
level, they have fairly nice runtime costs. Assuming _n_ metadata pairs,
each containing _m_ metadata entries, the _lookup_ cost for a specific
entry has a worst case runtime complexity of _O(nm)_. For _updating_ a specific
entry, the worst case complexity is _O(nm&sup2;)_, with an amortized complexity
of only _O(nm)_.

However, splitting at 50% capacity does mean that in the best case our
metadata pairs will only be 1/2 full. If we include the overhead of the second
block in our metadata pair, each metadata entry has an effective storage cost
of 4x the original size. I imagine users would not be happy if they found
that they can only use a quarter of their original storage. Metadata pairs
provide a mechanism for performing atomic updates, but we need a separate
mechanism for storing the bulk of our data.

## CTZ skip-lists

Metadata pairs provide efficient atomic updates but unfortunately have a large
storage cost. But we can work around this storage cost by only using the
metadata pairs to store references to more dense, copy-on-write (COW) data
structures.

[Copy-on-write data structures][wikipedia-cow], also called purely functional
data structures, are a category of data structures where the underlying
elements are immutable.  Making changes to the data requires creating new
elements containing a copy of the updated data and replacing any references
with references to the new elements. Generally, the performance of a COW data
structure depends on how many old elements can be reused after replacing parts
of the data.

littlefs has several requirements of its COW structures. They need to be
efficient to read and write, but most frustrating, they need to be traversable
with a constant amount of RAM. Notably this rules out
[B-trees][wikipedia-B-tree], which can not be traversed with constant RAM, and
[B+-trees][wikipedia-B+-tree], which are not possible to update with COW
operations.

---

So, what can we do? First let's consider storing files in a simple COW
linked-list. Appending a block, which is the basis for writing files, means we
have to update the last block to point to our new block. This requires a COW
operation, which means we need to update the second-to-last block, and then the
third-to-last, and so on until we've copied out the entire file.

```
A linked-list
.--------.  .--------.  .--------.  .--------.  .--------.  .--------.
| data 0 |->| data 1 |->| data 2 |->| data 4 |->| data 5 |->| data 6 |
|        |  |        |  |        |  |        |  |        |  |        |
|        |  |        |  |        |  |        |  |        |  |        |
'--------'  '--------'  '--------'  '--------'  '--------'  '--------'
```

To avoid a full copy during appends, we can store the data backwards. Appending
blocks just requires adding the new block and no other blocks need to be
updated. If we update a block in the middle, we still need to copy the
following blocks, but can reuse any blocks before it. Since most file writes
are linear, this design gambles that appends are the most common type of data
update.

```
A backwards linked-list
.--------.  .--------.  .--------.  .--------.  .--------.  .--------.
| data 0 |<-| data 1 |<-| data 2 |<-| data 4 |<-| data 5 |<-| data 6 |
|        |  |        |  |        |  |        |  |        |  |        |
|        |  |        |  |        |  |        |  |        |  |        |
'--------'  '--------'  '--------'  '--------'  '--------'  '--------'
```

However, a backwards linked-list does have a rather glaring problem. Iterating
over a file _in order_ has a runtime cost of _O(n&sup2;)_. A quadratic runtime
just to read a file! That's awful.

Fortunately we can do better. Instead of a singly linked list, littlefs
uses a multilayered linked-list often called a
[skip-list][wikipedia-skip-list]. However, unlike the most common type of
skip-list, littlefs's skip-lists are strictly deterministic built around some
interesting properties of the count-trailing-zeros (CTZ) instruction.

The rules CTZ skip-lists follow are that for every _n_&zwj;th block where _n_
is divisible by 2&zwj;_&#739;_, that block contains a pointer to block
_n_-2&zwj;_&#739;_. This means that each block contains anywhere from 1 to
log&#8322;_n_ pointers that skip to different preceding elements of the
skip-list.

The name comes from heavy use of the [CTZ instruction][wikipedia-ctz], which
lets us calculate the power-of-two factors efficiently. For a give block _n_,
that block contains ctz(_n_)+1 pointers.

```
A backwards CTZ skip-list
.--------.  .--------.  .--------.  .--------.  .--------.  .--------.
| data 0 |<-| data 1 |<-| data 2 |<-| data 3 |<-| data 4 |<-| data 5 |
|        |<-|        |--|        |<-|        |--|        |  |        |
|        |<-|        |--|        |--|        |--|        |  |        |
'--------'  '--------'  '--------'  '--------'  '--------'  '--------'
```

The additional pointers let us navigate the data-structure on disk much more
efficiently than in a singly linked list.

Consider a path from data block 5 to data block 1. You can see how data block 3
was completely skipped:
```
.--------.  .--------.  .--------.  .--------.  .--------.  .--------.
| data 0 |  | data 1 |<-| data 2 |  | data 3 |  | data 4 |<-| data 5 |
|        |  |        |  |        |<-|        |--|        |  |        |
|        |  |        |  |        |  |        |  |        |  |        |
'--------'  '--------'  '--------'  '--------'  '--------'  '--------'
```

The path to data block 0 is even faster, requiring only two jumps:
```
.--------.  .--------.  .--------.  .--------.  .--------.  .--------.
| data 0 |  | data 1 |  | data 2 |  | data 3 |  | data 4 |<-| data 5 |
|        |  |        |  |        |  |        |  |        |  |        |
|        |<-|        |--|        |--|        |--|        |  |        |
'--------'  '--------'  '--------'  '--------'  '--------'  '--------'
```

We can find the runtime complexity by looking at the path to any block from
the block containing the most pointers. Every step along the path divides
the search space for the block in half, giving us a runtime of _O(log n)_.
To get _to_ the block with the most pointers, we can perform the same steps
backwards, which puts the runtime at _O(2 log n)_ = _O(log n)_. An interesting
note is that this optimal path occurs naturally if we greedily choose the
pointer that covers the most distance without passing our target.

So now we have a [COW] data structure that is cheap to append with a runtime
of _O(1)_, and can be read with a worst case runtime of _O(n log n)_. Given
that this runtime is also divided by the amount of data we can store in a
block, this cost is fairly reasonable.

---

This is a new data structure, so we still have several questions. What is the
storage overhead? Can the number of pointers exceed the size of a block? How do
we store a CTZ skip-list in our metadata pairs?

To find the storage overhead, we can look at the data structure as multiple
linked-lists. Each linked-list skips twice as many blocks as the previous,
or from another perspective, each linked-list uses half as much storage as
the previous. As we approach infinity, the storage overhead forms a geometric
series. Solving this tells us that on average our storage overhead is only
2 pointers per block.

![lim,n->inf((1/n)sum,i,0->n(ctz(i)+1)) = sum,i,0->inf(1/2^i) = 2][ctz-formula1]

Because our file size is limited the word width we use to store sizes, we can
also solve for the maximum number of pointers we would ever need to store in a
block. If we set the overhead of pointers equal to the block size, we get the
following equation. Note that both a smaller block size (![B][bigB]) and larger
word width (![w]) result in more storage overhead.

![B = (w/8)ceil(log2(2^w / (B-2w/8)))][ctz-formula2]

Solving the equation for ![B][bigB] gives us the minimum block size for some
common word widths:

1. 32-bit CTZ skip-list => minimum block size of 104 bytes
2. 64-bit CTZ skip-list => minimum block size of 448 bytes

littlefs uses a 32-bit word width, so our blocks can only overflow with
pointers if they are smaller than 104 bytes. This is an easy requirement, as
in practice, most block sizes start at 512 bytes. As long as our block size
is larger than 104 bytes, we can avoid the extra logic needed to handle
pointer overflow.

This last question is how do we store CTZ skip-lists? We need a pointer to the
head block, the size of the skip-list, the index of the head block, and our
offset in the head block. But it's worth noting that each size maps to a unique
index + offset pair. So in theory we can store only a single pointer and size.

However, calculating the index + offset pair from the size is a bit
complicated. We can start with a summation that loops through all of the blocks
up until our given size. Let ![B][bigB] be the block size in bytes, ![w] be the
word width in bits, ![n] be the index of the block in the skip-list, and
![N][bigN] be the file size in bytes:

![N = sum,i,0->n(B-(w/8)(ctz(i)+1))][ctz-formula3]

This works quite well, but requires _O(n)_ to compute, which brings the full
runtime of reading a file up to _O(n&sup2; log n)_. Fortunately, that summation
doesn't need to touch the disk, so the practical impact is minimal.

However, despite the integration of a bitwise operation, we can actually reduce
this equation to a _O(1)_ form.  While browsing the amazing resource that is
the [On-Line Encyclopedia of Integer Sequences (OEIS)][oeis], I managed to find
[A001511], which matches the iteration of the CTZ instruction,
and [A005187], which matches its partial summation. Much to my
surprise, these both result from simple equations, leading us to a rather
unintuitive property that ties together two seemingly unrelated bitwise
instructions:

![sum,i,0->n(ctz(i)+1) = 2n-popcount(n)][ctz-formula4]

where:

1. ctz(![x]) = the number of trailing bits that are 0 in ![x]
2. popcount(![x]) = the number of bits that are 1 in ![x]

Initial tests of this surprising property seem to hold. As ![n] approaches
infinity, we end up with an average overhead of 2 pointers, which matches our
assumption from earlier. During iteration, the popcount function seems to
handle deviations from this average. Of course, just to make sure I wrote a
quick script that verified this property for all 32-bit integers.

Now we can substitute into our original equation to find a more efficient
equation for file size:

![N = Bn - (w/8)(2n-popcount(n))][ctz-formula5]

Unfortunately, the popcount function is non-injective, so we can't solve this
equation for our index. But what we can do is solve for an ![n'] index that
is greater than ![n] with error bounded by the range of the popcount function.
We can repeatedly substitute ![n'] into the original equation until the error
is smaller than our integer resolution. As it turns out, we only need to
perform this substitution once, which gives us this formula for our index:

![n = floor((N-(w/8)popcount(N/(B-2w/8))) / (B-2w/8))][ctz-formula6]

Now that we have our index ![n], we can just plug it back into the above
equation to find the offset. We run into a bit of a problem with integer
overflow, but we can avoid this by rearranging the equation a bit:

![off = N - (B-2w/8)n - (w/8)popcount(n)][ctz-formula7]

Our solution requires quite a bit of math, but computers are very good at math.
Now we can find both our block index and offset from a size in _O(1)_, letting
us store CTZ skip-lists with only a pointer and size.

CTZ skip-lists give us a COW data structure that is easily traversable in
_O(n)_, can be appended in _O(1)_, and can be read in _O(n log n)_. All of
these operations work in a bounded amount of RAM and require only two words of
storage overhead per block. In combination with metadata pairs, CTZ skip-lists
provide power resilience and compact storage of data.

```
                                    .--------.
                                   .|metadata|
                                   ||        |
                                   ||        |
                                   |'--------'
                                   '----|---'
                                        v
.--------.  .--------.  .--------.  .--------.
| data 0 |<-| data 1 |<-| data 2 |<-| data 3 |
|        |<-|        |--|        |  |        |
|        |  |        |  |        |  |        |
'--------'  '--------'  '--------'  '--------'

write data to disk, create copies
=>
                                    .--------.
                                   .|metadata|
                                   ||        |
                                   ||        |
                                   |'--------'
                                   '----|---'
                                        v
.--------.  .--------.  .--------.  .--------.
| data 0 |<-| data 1 |<-| data 2 |<-| data 3 |
|        |<-|        |--|        |  |        |
|        |  |        |  |        |  |        |
'--------'  '--------'  '--------'  '--------'
     ^ ^           ^
     | |           |    .--------.  .--------.  .--------.  .--------.
     | |           '----| new    |<-| new    |<-| new    |<-| new    |
     | '----------------| data 2 |<-| data 3 |--| data 4 |  | data 5 |
     '------------------|        |--|        |--|        |  |        |
                        '--------'  '--------'  '--------'  '--------'

commit to metadata pair
=>
                                                            .--------.
                                                           .|new     |
                                                           ||metadata|
                                                           ||        |
                                                           |'--------'
                                                           '----|---'
                                                                |
.--------.  .--------.  .--------.  .--------.                  |
| data 0 |<-| data 1 |<-| data 2 |<-| data 3 |                  |
|        |<-|        |--|        |  |        |                  |
|        |  |        |  |        |  |        |                  |
'--------'  '--------'  '--------'  '--------'                  |
     ^ ^           ^                                            v
     | |           |    .--------.  .--------.  .--------.  .--------.
     | |           '----| new    |<-| new    |<-| new    |<-| new    |
     | '----------------| data 2 |<-| data 3 |--| data 4 |  | data 5 |
     '------------------|        |--|        |--|        |  |        |
                        '--------'  '--------'  '--------'  '--------'
```

## The block allocator

So we now have the framework for an atomic, wear leveling filesystem. Small two
block metadata pairs provide atomic updates, while CTZ skip-lists provide
compact storage of data in COW blocks.

But now we need to look at the [elephant] in the room. Where do all these
blocks come from?

Deciding which block to use next is the responsibility of the block allocator.
In filesystem design, block allocation is often a second-class citizen, but in
a COW filesystem its role becomes much more important as it is needed for
nearly every write to the filesystem.

Normally, block allocation involves some sort of free list or bitmap stored on
the filesystem that is updated with free blocks. However, with power
resilience, keeping these structures consistent becomes difficult. It doesn't
help that any mistake in updating these structures can result in lost blocks
that are impossible to recover.

littlefs takes a cautious approach. Instead of trusting a free list on disk,
littlefs relies on the fact that the filesystem on disk is a mirror image of
the free blocks on the disk. The block allocator operates much like a garbage
collector in a scripting language, scanning for unused blocks on demand.

```
          .----.
          |root|
          |    |
          '----'
   v-------'  '-------v
.----.    .    .    .----.
| A  |    .    .    | B  |
|    |    .    .    |    |
'----'    .    .    '----'
.    .    .    .  v--'  '------------v---------v
.    .    .    .----.    .         .----.    .----.
.    .    .    | C  |    .         | D  |    | E  |
.    .    .    |    |    .         |    |    |    |
.    .    .    '----'    .         '----'    '----'
.    .    .    .    .    .         .    .    .    .
.----.----.----.----.----.----.----.----.----.----.----.----.
| A  |    |root| C  | B  |         | D  |    | E  |         |
|    |    |    |    |    |         |    |    |    |         |
'----'----'----'----'----'----'----'----'----'----'----'----'
        ^                   ^    ^                   ^    ^
         '-------------------'----'-------------------'----'-- free blocks
```

While this approach may sound complicated, the decision to not maintain a free
list greatly simplifies the overall design of littlefs. Unlike programming
languages, there are only a handful of data structures we need to traverse.
And block deallocation, which occurs nearly as often as block allocation,
is simply a noop. This "drop it on the floor" strategy greatly reduces the
complexity of managing on disk data structures, especially when handling
high-risk error conditions.

---

Our block allocator needs to find free blocks efficiently. You could traverse
through every block on storage and check each one against our filesystem tree;
however, the runtime would be abhorrent. We need to somehow collect multiple
blocks per traversal.

Looking at existing designs, some larger filesystems that use a similar "drop
it on the floor" strategy store a bitmap of the entire storage in [RAM]. This
works well because bitmaps are surprisingly compact. We can't use the same
strategy here, as it violates our constant RAM requirement, but we may be able
to modify the idea into a workable solution.

```
.----.----.----.----.----.----.----.----.----.----.----.----.
| A  |    |root| C  | B  |         | D  |    | E  |         |
|    |    |    |    |    |         |    |    |    |         |
'----'----'----'----'----'----'----'----'----'----'----'----'
  1    0    1    1    1    0    0    1    0    1    0    0
 \---------------------------+----------------------------/
                             v
               bitmap: 0xb94 (0b101110010100)
```

The block allocator in littlefs is a compromise between a disk-sized bitmap and
a brute force traversal. Instead of a bitmap the size of storage, we keep track
of a small, fixed-size bitmap called the lookahead buffer. During block
allocation, we take blocks from the lookahead buffer. If the lookahead buffer
is empty, we scan the filesystem for more free blocks, populating our lookahead
buffer. In each scan we use an increasing offset, circling the storage as
blocks are allocated.

Here's what it might look like to allocate 4 blocks on a decently busy
filesystem with a 32 bit lookahead and a total of 128 blocks (512 KiB
of storage if blocks are 4 KiB):
```
boot...         lookahead:
                fs blocks: fffff9fffffffffeffffffffffff0000
scanning...     lookahead: fffff9ff
                fs blocks: fffff9fffffffffeffffffffffff0000
alloc = 21      lookahead: fffffdff
                fs blocks: fffffdfffffffffeffffffffffff0000
alloc = 22      lookahead: ffffffff
                fs blocks: fffffffffffffffeffffffffffff0000
scanning...     lookahead:         fffffffe
                fs blocks: fffffffffffffffeffffffffffff0000
alloc = 63      lookahead:         ffffffff
                fs blocks: ffffffffffffffffffffffffffff0000
scanning...     lookahead:         ffffffff
                fs blocks: ffffffffffffffffffffffffffff0000
scanning...     lookahead:                 ffffffff
                fs blocks: ffffffffffffffffffffffffffff0000
scanning...     lookahead:                         ffff0000
                fs blocks: ffffffffffffffffffffffffffff0000
alloc = 112     lookahead:                         ffff8000
                fs blocks: ffffffffffffffffffffffffffff8000
```

This lookahead approach has a runtime complexity of _O(n&sup2;)_ to completely
scan storage; however, bitmaps are surprisingly compact, and in practice only
one or two passes are usually needed to find free blocks. Additionally, the
performance of the allocator can be optimized by adjusting the block size or
size of the lookahead buffer, trading either write granularity or RAM for
allocator performance.

## Wear leveling

The block allocator has a secondary role: wear leveling.

Wear leveling is the process of distributing wear across all blocks in the
storage to prevent the filesystem from experiencing an early death due to
wear on a single block in the storage.

littlefs has two methods of protecting against wear:
1. Detection and recovery from bad blocks
2. Evenly distributing wear across dynamic blocks

---

Recovery from bad blocks doesn't actually have anything to do with the block
allocator itself. Instead, it relies on the ability of the filesystem to detect
and evict bad blocks when they occur.

In littlefs, it is fairly straightforward to detect bad blocks at write time.
All writes must be sourced by some form of data in RAM, so immediately after we
write to a block, we can read the data back and verify that it was written
correctly. If we find that the data on disk does not match the copy we have in
RAM, a write error has occurred and we most likely have a bad block.

Once we detect a bad block, we need to recover from it. In the case of write
errors, we have a copy of the corrupted data in RAM, so all we need to do is
evict the bad block, allocate a new, hopefully good block, and repeat the write
that previously failed.

The actual act of evicting the bad block and replacing it with a new block is
left up to the filesystem's copy-on-bounded-writes (CObW) data structures. One
property of CObW data structures is that any block can be replaced during a
COW operation. The bounded-writes part is normally triggered by a counter, but
nothing prevents us from triggering a COW operation as soon as we find a bad
block.

```
     .----.
     |root|
     |    |
     '----'
   v--'  '----------------------v
.----.                        .----.
| A  |                        | B  |
|    |                        |    |
'----'                        '----'
.    .                      v---'  .
.    .                   .----.    .
.    .                   | C  |    .
.    .                   |    |    .
.    .                   '----'    .
.    .                   .    .    .
.----.----.----.----.----.----.----.----.----.----.
| A  |root|              | C  | B  |              |
|    |    |              |    |    |              |
'----'----'----'----'----'----'----'----'----'----'

update C
=>
     .----.
     |root|
     |    |
     '----'
   v--'  '----------------------v
.----.                        .----.
| A  |                        | B  |
|    |                        |    |
'----'                        '----'
.    .                      v---'  .
.    .                   .----.    .
.    .                   |bad |    .
.    .                   |blck|    .
.    .                   '----'    .
.    .                   .    .    .
.----.----.----.----.----.----.----.----.----.----.
| A  |root|              |bad | B  |              |
|    |    |              |blck|    |              |
'----'----'----'----'----'----'----'----'----'----'

oh no! bad block! relocate C
=>
     .----.
     |root|
     |    |
     '----'
   v--'  '----------------------v
.----.                        .----.
| A  |                        | B  |
|    |                        |    |
'----'                        '----'
.    .                      v---'  .
.    .                   .----.    .
.    .                   |bad |    .
.    .                   |blck|    .
.    .                   '----'    .
.    .                   .    .    .
.----.----.----.----.----.----.----.----.----.----.
| A  |root|              |bad | B  |bad |         |
|    |    |              |blck|    |blck|         |
'----'----'----'----'----'----'----'----'----'----'
                            --------->
oh no! bad block! relocate C
=>
     .----.
     |root|
     |    |
     '----'
   v--'  '----------------------v
.----.                        .----.
| A  |                        | B  |
|    |                        |    |
'----'                        '----'
.    .                      v---'  .
.    .                   .----.    .    .----.
.    .                   |bad |    .    | C' |
.    .                   |blck|    .    |    |
.    .                   '----'    .    '----'
.    .                   .    .    .    .    .
.----.----.----.----.----.----.----.----.----.----.
| A  |root|              |bad | B  |bad | C' |    |
|    |    |              |blck|    |blck|    |    |
'----'----'----'----'----'----'----'----'----'----'
                            -------------->
successfully relocated C, update B
=>
     .----.
     |root|
     |    |
     '----'
   v--'  '----------------------v
.----.                        .----.
| A  |                        |bad |
|    |                        |blck|
'----'                        '----'
.    .                      v---'  .
.    .                   .----.    .    .----.
.    .                   |bad |    .    | C' |
.    .                   |blck|    .    |    |
.    .                   '----'    .    '----'
.    .                   .    .    .    .    .
.----.----.----.----.----.----.----.----.----.----.
| A  |root|              |bad |bad |bad | C' |    |
|    |    |              |blck|blck|blck|    |    |
'----'----'----'----'----'----'----'----'----'----'

oh no! bad block! relocate B
=>
     .----.
     |root|
     |    |
     '----'
   v--'  '----------------------v
.----.                        .----.         .----.
| A  |                        |bad |         |bad |
|    |                        |blck|         |blck|
'----'                        '----'         '----'
.    .                      v---'  .         .    .
.    .                   .----.    .    .----.    .
.    .                   |bad |    .    | C' |    .
.    .                   |blck|    .    |    |    .
.    .                   '----'    .    '----'    .
.    .                   .    .    .    .    .    .
.----.----.----.----.----.----.----.----.----.----.
| A  |root|              |bad |bad |bad | C' |bad |
|    |    |              |blck|blck|blck|    |blck|
'----'----'----'----'----'----'----'----'----'----'
                                 -------------->
oh no! bad block! relocate B
=>
     .----.
     |root|
     |    |
     '----'
   v--'  '----------------------v
.----.    .----.              .----.
| A  |    | B' |              |bad |
|    |    |    |              |blck|
'----'    '----'              '----'
.    .    .  | .            .---'  .
.    .    .  '--------------v-------------v
.    .    .    .         .----.    .    .----.
.    .    .    .         |bad |    .    | C' |
.    .    .    .         |blck|    .    |    |
.    .    .    .         '----'    .    '----'
.    .    .    .         .    .    .    .    .
.----.----.----.----.----.----.----.----.----.----.
| A  |root| B' |         |bad |bad |bad | C' |bad |
|    |    |    |         |blck|blck|blck|    |blck|
'----'----'----'----'----'----'----'----'----'----'
------------>                    ------------------
successfully relocated B, update root
=>
     .----.
     |root|
     |    |
     '----'
   v--'  '--v
.----.    .----.
| A  |    | B' |
|    |    |    |
'----'    '----'
.    .    .   '---------------------------v
.    .    .    .                        .----.
.    .    .    .                        | C' |
.    .    .    .                        |    |
.    .    .    .                        '----'
.    .    .    .                        .    .
.----.----.----.----.----.----.----.----.----.----.
| A  |root| B' |         |bad |bad |bad | C' |bad |
|    |    |    |         |blck|blck|blck|    |blck|
'----'----'----'----'----'----'----'----'----'----'
```

We may find that the new block is also bad, but hopefully after repeating this
cycle we'll eventually find a new block where a write succeeds. If we don't,
that means that all blocks in our storage are bad, and we've reached the end of
our device's usable life. At this point, littlefs will return an "out of space"
error. This is technically true, as there are no more good blocks, but as an
added benefit it also matches the error condition expected by users of
dynamically sized data.

---

Read errors, on the other hand, are quite a bit more complicated. We don't have
a copy of the data lingering around in RAM, so we need a way to reconstruct the
original data even after it has been corrupted. One such mechanism for this is
[error-correction-codes (ECC)][wikipedia-ecc].

ECC is an extension to the idea of a checksum. Where a checksum such as CRC can
detect that an error has occurred in the data, ECC can detect and actually
correct some amount of errors. However, there is a limit to how many errors ECC
can detect: the [Hamming bound][wikipedia-hamming-bound]. As the number of
errors approaches the Hamming bound, we may still be able to detect errors, but
can no longer fix the data. If we've reached this point the block is
unrecoverable.

littlefs by itself does **not** provide ECC. The block nature and relatively
large footprint of ECC does not work well with the dynamically sized data of
filesystems, correcting errors without RAM is complicated, and ECC fits better
with the geometry of block devices. In fact, several NOR flash chips have extra
storage intended for ECC, and many NAND chips can even calculate ECC on the
chip itself.

In littlefs, ECC is entirely optional. Read errors can instead be prevented
proactively by wear leveling. But it's important to note that ECC can be used
at the block device level to modestly extend the life of a device. littlefs
respects any errors reported by the block device, allowing a block device to
provide additional aggressive error detection.

---

To avoid read errors, we need to be proactive, as opposed to reactive as we
were with write errors.

One way to do this is to detect when the number of errors in a block exceeds
some threshold, but is still recoverable. With ECC we can do this at write
time, and treat the error as a write error, evicting the block before fatal
read errors have a chance to develop.

A different, more generic strategy, is to proactively distribute wear across
all blocks in the storage, with the hope that no single block fails before the
rest of storage is approaching the end of its usable life. This is called
wear leveling.

Generally, wear leveling algorithms fall into one of two categories:

1. [Dynamic wear leveling][wikipedia-dynamic-wear-leveling], where we
   distribute wear over "dynamic" blocks. The can be accomplished by
   only considering unused blocks.

2. [Static wear leveling][wikipedia-static-wear-leveling], where we
   distribute wear over both "dynamic" and "static" blocks. To make this work,
   we need to consider all blocks, including blocks that already contain data.

As a tradeoff for code size and complexity, littlefs (currently) only provides
dynamic wear leveling. This is a best effort solution. Wear is not distributed
perfectly, but it is distributed among the free blocks and greatly extends the
life of a device.

On top of this, littlefs uses a statistical wear leveling algorithm. What this
means is that we don’t actively track wear, instead we rely on a uniform
distribution of wear across storage to approximate a dynamic wear leveling
algorithm. Despite the long name, this is actually a simplification of dynamic
wear leveling.

The uniform distribution of wear is left up to the block allocator, which
creates a uniform distribution in two parts. The easy part is when the device
is powered, in which case we allocate the blocks linearly, circling the device.
The harder part is what to do when the device loses power. We can't just
restart the allocator at the beginning of storage, as this would bias the wear.
Instead, we start the allocator as a random offset every time we mount the
filesystem. As long as this random offset is uniform, the combined allocation
pattern is also a uniform distribution.

![Cumulative wear distribution graph][wear-distribution-graph]

Initially, this approach to wear leveling looks like it creates a difficult
dependency on a power-independent random number generator, which must return
different random numbers on each boot. However, the filesystem is in a
relatively unique situation in that it is sitting on top of a large of amount
of entropy that persists across power loss.

We can actually use the data on disk to directly drive our random number
generator. In practice, this is implemented by xoring the checksums of each
metadata pair, which is already calculated to fetch and mount the filesystem.

```
            .--------. \                         probably random
           .|metadata| |                                ^
           ||        | +-> crc ----------------------> xor
           ||        | |                                ^
           |'--------' /                                |
           '---|--|-'                                   |
            .-'    '-------------------------.          |
           |                                  |         |
           |        .--------------> xor ------------> xor
           |        |                 ^       |         ^
           v       crc               crc      v        crc
      .--------. \  ^   .--------. \  ^   .--------. \  ^
     .|metadata|-|--|-->|metadata| |  |  .|metadata| |  |
     ||        | +--'  ||        | +--'  ||        | +--'
     ||        | |     ||        | |     ||        | |
     |'--------' /     |'--------' /     |'--------' /
     '---|--|-'        '----|---'        '---|--|-'
      .-'    '-.            |             .-'    '-.
     v          v           v            v          v
.--------.  .--------.  .--------.  .--------.  .--------.
|  data  |  |  data  |  |  data  |  |  data  |  |  data  |
|        |  |        |  |        |  |        |  |        |
|        |  |        |  |        |  |        |  |        |
'--------'  '--------'  '--------'  '--------'  '--------'
```

Note that this random number generator is not perfect. It only returns unique
random numbers when the filesystem is modified. This is exactly what we want
for distributing wear in the allocator, but means this random number generator
is not useful for general use.

---

Together, bad block detection and dynamic wear leveling provide a best effort
solution for avoiding the early death of a filesystem due to wear. Importantly,
littlefs's wear leveling algorithm provides a key feature: You can increase the
life of a device simply by increasing the size of storage. And if more
aggressive wear leveling is desired, you can always combine littlefs with a
[flash translation layer (FTL)][wikipedia-ftl] to get a small power resilient
filesystem with static wear leveling.

## Files

Now that we have our building blocks out of the way, we can start looking at
our filesystem as a whole.

The first step: How do we actually store our files?

We've determined that CTZ skip-lists are pretty good at storing data compactly,
so following the precedent found in other filesystems we could give each file
a skip-list stored in a metadata pair that acts as an inode for the file.


```
                                    .--------.
                                   .|metadata|
                                   ||        |
                                   ||        |
                                   |'--------'
                                   '----|---'
                                        v
.--------.  .--------.  .--------.  .--------.
| data 0 |<-| data 1 |<-| data 2 |<-| data 3 |
|        |<-|        |--|        |  |        |
|        |  |        |  |        |  |        |
'--------'  '--------'  '--------'  '--------'
```

However, this doesn't work well when files are small, which is common for
embedded systems. Compared to PCs, _all_ data in an embedded system is small.

Consider a small 4-byte file. With a two block metadata-pair and one block for
the CTZ skip-list, we find ourselves using a full 3 blocks. On most NOR flash
with 4 KiB blocks, this is 12 KiB of overhead. A ridiculous 3072x increase.

```
file stored as inode, 4 bytes costs ~12 KiB

 .----------------.                  \
.|    revision    |                  |
||----------------|    \             |
||    skiplist   ---.  +- metadata   |
||----------------| |  /  4x8 bytes  |
||    checksum    | |     32 bytes   |
||----------------| |                |
||       |        | |                +- metadata pair
||       v        | |                |  2x4 KiB
||                | |                |  8 KiB
||                | |                |
||                | |                |
||                | |                |
|'----------------' |                |
'----------------'  |                /
          .--------'
         v
 .----------------.    \             \
 |      data      |    +- data       |
 |----------------|    /  4 bytes    |
 |                |                  |
 |                |                  |
 |                |                  |
 |                |                  +- data block
 |                |                  |  4 KiB
 |                |                  |
 |                |                  |
 |                |                  |
 |                |                  |
 |                |                  |
 '----------------'                  /
```

We can make several improvements. First, instead of giving each file its own
metadata pair, we can store multiple files in a single metadata pair. One way
to do this is to directly associate a directory with a metadata pair (or a
linked list of metadata pairs). This makes it easy for multiple files to share
the directory's metadata pair for logging and reduces the collective storage
overhead.

The strict binding of metadata pairs and directories also gives users
direct control over storage utilization depending on how they organize their
directories.

```
multiple files stored in metadata pair, 4 bytes costs ~4 KiB

       .----------------.
      .|    revision    |
      ||----------------|
      ||    A name      |
      ||   A skiplist  -----.
      ||----------------|   |  \
      ||    B name      |   |  +- metadata
      ||   B skiplist  ---. |  |  4x8 bytes
      ||----------------| | |  /  32 bytes
      ||    checksum    | | |
      ||----------------| | |
      ||       |        | | |
      ||       v        | | |
      |'----------------' | |
      '----------------'  | |
         .----------------' |
        v                   v
.----------------.  .----------------.  \           \
|     A data     |  |     B data     |  +- data     |
|                |  |----------------|  /  4 bytes  |
|                |  |                |              |
|                |  |                |              |
|                |  |                |              |
|                |  |                |              + data block
|                |  |                |              | 4 KiB
|                |  |                |              |
|----------------|  |                |              |
|                |  |                |              |
|                |  |                |              |
|                |  |                |              |
'----------------'  '----------------'              /
```

The second improvement we can make is noticing that for very small files, our
attempts to use CTZ skip-lists for compact storage backfires. Metadata pairs
have a ~4x storage cost, so if our file is smaller than 1/4 the block size,
there's actually no benefit in storing our file outside of our metadata pair.

In this case, we can store the file directly in our directory's metadata pair.
We call this an inline file, and it allows a directory to store many small
files quite efficiently. Our previous 4 byte file now only takes up a
theoretical 16 bytes on disk.

```
inline files stored in metadata pair, 4 bytes costs ~16 bytes

 .----------------.
.|    revision    |
||----------------|
||    A name      |
||   A skiplist  ---.
||----------------| |  \
||    B name      | |  +- data
||    B data      | |  |  4x4 bytes
||----------------| |  /  16 bytes
||    checksum    | |
||----------------| |
||       |        | |
||       v        | |
|'----------------' |
'----------------'  |
          .---------'
         v
 .----------------.
 |     A data     |
 |                |
 |                |
 |                |
 |                |
 |                |
 |                |
 |                |
 |----------------|
 |                |
 |                |
 |                |
 '----------------'
```

Once the file exceeds 1/4 the block size, we switch to a CTZ skip-list. This
means that our files never use more than 4x storage overhead, decreasing as
the file grows in size.

![File storage cost graph][file-cost-graph]

## Directories

Now we just need directories to store our files. As mentioned above we want
a strict binding of directories and metadata pairs, but there are a few
complications we need to sort out.

On their own, each directory is a linked-list of metadata pairs. This lets us
store an unlimited number of files in each directory, and we don't need to
worry about the runtime complexity of unbounded logs. We can store other
directory pointers in our metadata pairs, which gives us a directory tree, much
like what you find on other filesystems.

```
            .--------.
           .| root   |
           ||        |
           ||        |
           |'--------'
           '---|--|-'
            .-'    '-------------------------.
           v                                  v
      .--------.        .--------.        .--------.
     .| dir A  |------->| dir A  |       .| dir B  |
     ||        |       ||        |       ||        |
     ||        |       ||        |       ||        |
     |'--------'       |'--------'       |'--------'
     '---|--|-'        '----|---'        '---|--|-'
      .-'    '-.            |             .-'    '-.
     v          v           v            v          v
.--------.  .--------.  .--------.  .--------.  .--------.
| file C |  | file D |  | file E |  | file F |  | file G |
|        |  |        |  |        |  |        |  |        |
|        |  |        |  |        |  |        |  |        |
'--------'  '--------'  '--------'  '--------'  '--------'
```

The main complication is, once again, traversal with a constant amount of
[RAM]. The directory tree is a tree, and the unfortunate fact is you can't
traverse a tree with constant RAM.

Fortunately, the elements of our tree are metadata pairs, so unlike CTZ
skip-lists, we're not limited to strict COW operations. One thing we can do is
thread a linked-list through our tree, explicitly enabling cheap traversal
over the entire filesystem.

```
            .--------.
           .| root   |-.
           ||        | |
   .-------||        |-'
   |       |'--------'
   |       '---|--|-'
   |        .-'    '-------------------------.
   |       v                                  v
   |  .--------.        .--------.        .--------.
   '->| dir A  |------->| dir A  |------->| dir B  |
     ||        |       ||        |       ||        |
     ||        |       ||        |       ||        |
     |'--------'       |'--------'       |'--------'
     '---|--|-'        '----|---'        '---|--|-'
      .-'    '-.            |             .-'    '-.
     v          v           v            v          v
.--------.  .--------.  .--------.  .--------.  .--------.
| file C |  | file D |  | file E |  | file F |  | file G |
|        |  |        |  |        |  |        |  |        |
|        |  |        |  |        |  |        |  |        |
'--------'  '--------'  '--------'  '--------'  '--------'
```

Unfortunately, not sticking to pure COW operations creates some problems. Now,
whenever we want to manipulate the directory tree, multiple pointers need to be
updated. If you're familiar with designing atomic data structures this should
set off a bunch of red flags.

To work around this, our threaded linked-list has a bit of leeway. Instead of
only containing metadata pairs found in our filesystem, it is allowed to
contain metadata pairs that have no parent because of a power loss. These are
called orphaned metadata pairs.

With the possibility of orphans, we can build power loss resilient operations
that maintain a filesystem tree threaded with a linked-list for traversal.

Adding a directory to our tree:

```
         .--------.
        .| root   |-.
        ||        | |
.-------||        |-'
|       |'--------'
|       '---|--|-'
|        .-'    '-.
|       v          v
|  .--------.  .--------.
'->| dir A  |->| dir C  |
  ||        | ||        |
  ||        | ||        |
  |'--------' |'--------'
  '--------'  '--------'

allocate dir B
=>
         .--------.
        .| root   |-.
        ||        | |
.-------||        |-'
|       |'--------'
|       '---|--|-'
|        .-'    '-.
|       v          v
|  .--------.    .--------.
'->| dir A  |--->| dir C  |
  ||        | .->|        |
  ||        | | ||        |
  |'--------' | |'--------'
  '--------'  | '--------'
              |
   .--------. |
  .| dir B  |-'
  ||        |
  ||        |
  |'--------'
  '--------'

insert dir B into threaded linked-list, creating an orphan
=>
         .--------.
        .| root   |-.
        ||        | |
.-------||        |-'
|       |'--------'
|       '---|--|-'
|        .-'    '-------------.
|       v                      v
|  .--------.  .--------.  .--------.
'->| dir A  |->| dir B  |->| dir C  |
  ||        | || orphan!| ||        |
  ||        | ||        | ||        |
  |'--------' |'--------' |'--------'
  '--------'  '--------'  '--------'

add dir B to parent directory
=>
               .--------.
              .| root   |-.
              ||        | |
.-------------||        |-'
|             |'--------'
|             '--|-|-|-'
|        .------'  |  '-------.
|       v          v           v
|  .--------.  .--------.  .--------.
'->| dir A  |->| dir B  |->| dir C  |
  ||        | ||        | ||        |
  ||        | ||        | ||        |
  |'--------' |'--------' |'--------'
  '--------'  '--------'  '--------'
```

Removing a directory:

```
               .--------.
              .| root   |-.
              ||        | |
.-------------||        |-'
|             |'--------'
|             '--|-|-|-'
|        .------'  |  '-------.
|       v          v           v
|  .--------.  .--------.  .--------.
'->| dir A  |->| dir B  |->| dir C  |
  ||        | ||        | ||        |
  ||        | ||        | ||        |
  |'--------' |'--------' |'--------'
  '--------'  '--------'  '--------'

remove dir B from parent directory, creating an orphan
=>
         .--------.
        .| root   |-.
        ||        | |
.-------||        |-'
|       |'--------'
|       '---|--|-'
|        .-'    '-------------.
|       v                      v
|  .--------.  .--------.  .--------.
'->| dir A  |->| dir B  |->| dir C  |
  ||        | || orphan!| ||        |
  ||        | ||        | ||        |
  |'--------' |'--------' |'--------'
  '--------'  '--------'  '--------'

remove dir B from threaded linked-list, returning dir B to free blocks
=>
         .--------.
        .| root   |-.
        ||        | |
.-------||        |-'
|       |'--------'
|       '---|--|-'
|        .-'    '-.
|       v          v
|  .--------.  .--------.
'->| dir A  |->| dir C  |
  ||        | ||        |
  ||        | ||        |
  |'--------' |'--------'
  '--------'  '--------'
```

In addition to normal directory tree operations, we can use orphans to evict
blocks in a metadata pair when the block goes bad or exceeds its allocated
erases. If we lose power while evicting a metadata block we may end up with
a situation where the filesystem references the replacement block while the
threaded linked-list still contains the evicted block. We call this a
half-orphan.

```
               .--------.
              .| root   |-.
              ||        | |
.-------------||        |-'
|             |'--------'
|             '--|-|-|-'
|        .------'  |  '-------.
|       v          v           v
|  .--------.  .--------.  .--------.
'->| dir A  |->| dir B  |->| dir C  |
  ||        | ||        | ||        |
  ||        | ||        | ||        |
  |'--------' |'--------' |'--------'
  '--------'  '--------'  '--------'

try to write to dir B
=>
                  .--------.
                 .| root   |-.
                 ||        | |
.----------------||        |-'
|                |'--------'
|                '-|-||-|-'
|        .--------'  ||  '-----.
|       v            |v         v
|  .--------.     .--------.  .--------.
'->| dir A  |---->| dir B  |->| dir C  |
  ||        |-.   |        | ||        |
  ||        | |   |        | ||        |
  |'--------' |   '--------' |'--------'
  '--------'  |      v       '--------'
              |  .--------.
              '->| dir B  |
                 | bad    |
                 | block! |
                 '--------'

oh no! bad block detected, allocate replacement
=>
                  .--------.
                 .| root   |-.
                 ||        | |
.----------------||        |-'
|                |'--------'
|                '-|-||-|-'
|        .--------'  ||  '-------.
|       v            |v           v
|  .--------.     .--------.    .--------.
'->| dir A  |---->| dir B  |--->| dir C  |
  ||        |-.   |        | .->|        |
  ||        | |   |        | | ||        |
  |'--------' |   '--------' | |'--------'
  '--------'  |      v       | '--------'
              |  .--------.  |
              '->| dir B  |  |
                 | bad    |  |
                 | block! |  |
                 '--------'  |
                             |
                 .--------.  |
                 | dir B  |--'
                 |        |
                 |        |
                 '--------'

insert replacement in threaded linked-list, creating a half-orphan
=>
                  .--------.
                 .| root   |-.
                 ||        | |
.----------------||        |-'
|                |'--------'
|                '-|-||-|-'
|        .--------'  ||  '-------.
|       v            |v           v
|  .--------.     .--------.    .--------.
'->| dir A  |---->| dir B  |--->| dir C  |
  ||        |-.   |        | .->|        |
  ||        | |   |        | | ||        |
  |'--------' |   '--------' | |'--------'
  '--------'  |      v       | '--------'
              |  .--------.  |
              |  | dir B  |  |
              |  | bad    |  |
              |  | block! |  |
              |  '--------'  |
              |              |
              |  .--------.  |
              '->| dir B  |--'
                 | half   |
                 | orphan!|
                 '--------'

fix reference in parent directory
=>
               .--------.
              .| root   |-.
              ||        | |
.-------------||        |-'
|             |'--------'
|             '--|-|-|-'
|        .------'  |  '-------.
|       v          v           v
|  .--------.  .--------.  .--------.
'->| dir A  |->| dir B  |->| dir C  |
  ||        | ||        | ||        |
  ||        | ||        | ||        |
  |'--------' |'--------' |'--------'
  '--------'  '--------'  '--------'
```

Finding orphans and half-orphans is expensive, requiring a _O(n&sup2;)_
comparison of every metadata pair with every directory entry. But the tradeoff
is a power resilient filesystem that works with only a bounded amount of RAM.
Fortunately, we only need to check for orphans on the first allocation after
boot, and a read-only littlefs can ignore the threaded linked-list entirely.

If we only had some sort of global state, then we could also store a flag and
avoid searching for orphans unless we knew we were specifically interrupted
while manipulating the directory tree (foreshadowing!).

## The move problem

We have one last challenge: the move problem. Phrasing the problem is simple:

How do you atomically move a file between two directories?

In littlefs we can atomically commit to directories, but we can't create
an atomic commit that spans multiple directories. The filesystem must go
through a minimum of two distinct states to complete a move.

To make matters worse, file moves are a common form of synchronization for
filesystems. As a filesystem designed for power-loss, it's important we get
atomic moves right.

So what can we do?

- We definitely can't just let power-loss result in duplicated or lost files.
  This could easily break users' code and would only reveal itself in extreme
  cases. We were only able to be lazy about the threaded linked-list because
  it isn't user facing and we can handle the corner cases internally.

- Some filesystems propagate COW operations up the tree until a common parent
  is found. Unfortunately this interacts poorly with our threaded tree and
  brings back the issue of upward propagation of wear.

- In a previous version of littlefs we tried to solve this problem by going
  back and forth between the source and destination, marking and unmarking the
  file as moving in order to make the move atomic from the user perspective.
  This worked, but not well. Finding failed moves was expensive and required
  a unique identifier for each file.

In the end, solving the move problem required creating a new mechanism for
sharing knowledge between multiple metadata pairs. In littlefs this led to the
introduction of a mechanism called "global state".

---

Global state is a small set of state that can be updated from _any_ metadata
pair. Combining global state with metadata pairs' ability to update multiple
entries in one commit gives us a powerful tool for crafting complex atomic
operations.

How does global state work?

Global state exists as a set of deltas that are distributed across the metadata
pairs in the filesystem. The actual global state can be built out of these
deltas by xoring together all of the deltas in the filesystem.

```
 .--------.  .--------.  .--------.  .--------.  .--------.
.|        |->| gdelta |->|        |->| gdelta |->| gdelta |
||        | || 0x23   | ||        | || 0xff   | || 0xce   |
||        | ||        | ||        | ||        | ||        |
|'--------' |'--------' |'--------' |'--------' |'--------'
'--------'  '----|---'  '--------'  '----|---'  '----|---'
                 v                       v           v
       0x00 --> xor ------------------> xor ------> xor --> gstate 0x12
```

To update the global state from a metadata pair, we take the global state we
know and xor it with both our changes and any existing delta in the metadata
pair. Committing this new delta to the metadata pair commits the changes to
the filesystem's global state.

```
 .--------.  .--------.  .--------.  .--------.  .--------.
.|        |->| gdelta |->|        |->| gdelta |->| gdelta |
||        | || 0x23   | ||        | || 0xff   | || 0xce   |
||        | ||        | ||        | ||        | ||        |
|'--------' |'--------' |'--------' |'--------' |'--------'
'--------'  '----|---'  '--------'  '--|---|-'  '----|---'
                 v                     v   |         v
       0x00 --> xor ----------------> xor -|------> xor --> gstate = 0x12
                                           |                          |
                                           |                          |
change gstate to 0xab --> xor <------------|--------------------------'
=>                         |               v
                           '------------> xor
                                           |
                                           v
 .--------.  .--------.  .--------.  .--------.  .--------.
.|        |->| gdelta |->|        |->| gdelta |->| gdelta |
||        | || 0x23   | ||        | || 0x46   | || 0xce   |
||        | ||        | ||        | ||        | ||        |
|'--------' |'--------' |'--------' |'--------' |'--------'
'--------'  '----|---'  '--------'  '----|---'  '----|---'
                 v                       v           v
       0x00 --> xor ------------------> xor ------> xor --> gstate = 0xab
```

To make this efficient, we always keep a copy of the global state in RAM. We
only need to iterate over our metadata pairs and build the global state when
the filesystem is mounted.

You may have noticed that global state is very expensive. We keep a copy in
RAM and a delta in an unbounded number of metadata pairs. Even if we reset
the global state to its initial value, we can't easily clean up the deltas on
disk. For this reason, it's very important that we keep the size of global
state bounded and extremely small. But, even with a strict budget, global
state is incredibly valuable.

---

Now we can solve the move problem. We can create global state describing our
move atomically with the creation of the new file, and we can clear this move
state atomically with the removal of the old file.

```
               .--------.    gstate = no move
              .| root   |-.
              ||        | |
.-------------||        |-'
|             |'--------'
|             '--|-|-|-'
|        .------'  |  '-------.
|       v          v           v
|  .--------.  .--------.  .--------.
'->| dir A  |->| dir B  |->| dir C  |
  ||        | ||        | ||        |
  ||        | ||        | ||        |
  |'--------' |'--------' |'--------'
  '----|---'  '--------'  '--------'
       v
   .--------.
   | file D |
   |        |
   |        |
   '--------'

begin move, add reference in dir C, change gstate to have move
=>
               .--------.    gstate = moving file D in dir A (m1)
              .| root   |-.
              ||        | |
.-------------||        |-'
|             |'--------'
|             '--|-|-|-'
|        .------'  |  '-------.
|       v          v           v
|  .--------.  .--------.  .--------.
'->| dir A  |->| dir B  |->| dir C  |
  ||        | ||        | || gdelta |
  ||        | ||        | || =m1    |
  |'--------' |'--------' |'--------'
  '----|---'  '--------'  '----|---'
       |     .----------------'
       v    v
     .--------.
     | file D |
     |        |
     |        |
     '--------'

complete move, remove reference in dir A, change gstate to no move
=>
               .--------.    gstate = no move (m1^~m1)
              .| root   |-.
              ||        | |
.-------------||        |-'
|             |'--------'
|             '--|-|-|-'
|        .------'  |  '-------.
|       v          v           v
|  .--------.  .--------.  .--------.
'->| dir A  |->| dir B  |->| dir C  |
  || gdelta | ||        | || gdelta |
  || =~m1   | ||        | || =m1    |
  |'--------' |'--------' |'--------'
  '--------'  '--------'  '----|---'
                               v
                           .--------.
                           | file D |
                           |        |
                           |        |
                           '--------'
```


If, after building our global state during mount, we find information
describing an ongoing move, we know we lost power during a move and the file
is duplicated in both the source and destination directories. If this happens,
we can resolve the move using the information in the global state to remove
one of the files.

```
                 .--------.    gstate = moving file D in dir A (m1)
                .| root   |-.             ^
                ||        |------------> xor
.---------------||        |-'             ^
|               |'--------'               |
|               '--|-|-|-'                |
|        .--------'  |  '---------.       |
|       |            |             |      |
|       |     .----------> xor --------> xor
|       v     |      v      ^      v      ^
|  .--------. |  .--------. |  .--------. |
'->| dir A  |-|->| dir B  |-|->| dir C  | |
  ||        |-' ||        |-' || gdelta |-'
  ||        |   ||        |   || =m1    |
  |'--------'   |'--------'   |'--------'
  '----|---'    '--------'    '----|---'
       |     .---------------------'
       v    v
     .--------.
     | file D |
     |        |
     |        |
     '--------'
```

We can also move directories the same way we move files. There is the threaded
linked-list to consider, but leaving the threaded linked-list unchanged works
fine as the order doesn't really matter.

```
               .--------.    gstate = no move (m1^~m1)
              .| root   |-.
              ||        | |
.-------------||        |-'
|             |'--------'
|             '--|-|-|-'
|        .------'  |  '-------.
|       v          v           v
|  .--------.  .--------.  .--------.
'->| dir A  |->| dir B  |->| dir C  |
  || gdelta | ||        | || gdelta |
  || =~m1   | ||        | || =m1    |
  |'--------' |'--------' |'--------'
  '--------'  '--------'  '----|---'
                               v
                           .--------.
                           | file D |
                           |        |
                           |        |
                           '--------'

begin move, add reference in dir C, change gstate to have move
=>
                .--------.    gstate = moving dir B in root (m1^~m1^m2)
               .| root   |-.
               ||        | |
.--------------||        |-'
|              |'--------'
|              '--|-|-|-'
|        .-------'  |  '----------.
|       v           |              v
|  .--------.       |          .--------.
'->| dir A  |-.     |       .->| dir C  |
  || gdelta | |     |       | || gdelta |
  || =~m1   | |     |       | || =m1^m2 |
  |'--------' |     |       | |'--------'
  '--------'  |     |       | '---|--|-'
              |     |    .-------'   |
              |     v   v   |        v
              |  .--------. |    .--------.
              '->| dir B  |-'    | file D |
                ||        |      |        |
                ||        |      |        |
                |'--------'      '--------'
                '--------'

complete move, remove reference in root, change gstate to no move
=>
             .--------.    gstate = no move (m1^~m1^m2^~m2)
            .| root   |-.
            || gdelta | |
.-----------|| =~m2   |-'
|           |'--------'
|           '---|--|-'
|        .-----'    '-----.
|       v                  v
|  .--------.          .--------.
'->| dir A  |-.     .->| dir C  |
  || gdelta | |     | || gdelta |
  || =~m1   | |     '-|| =m1^m2 |-------.
  |'--------' |       |'--------'       |
  '--------'  |       '---|--|-'        |
              |        .-'    '-.       |
              |       v          v      |
              |  .--------.  .--------. |
              '->| dir B  |--| file D |-'
                ||        |  |        |
                ||        |  |        |
                |'--------'  '--------'
                '--------'
```

Global state gives us a powerful tool we can use to solve the move problem.
And the result is surprisingly performant, only needing the minimum number
of states and using the same number of commits as a naive move. Additionally,
global state gives us a bit of persistent state we can use for some other
small improvements.

## Conclusion

And that's littlefs, thanks for reading!


[wikipedia-flash]: https://en.wikipedia.org/wiki/Flash_memory
[wikipedia-sna]: https://en.wikipedia.org/wiki/Serial_number_arithmetic
[wikipedia-crc]: https://en.wikipedia.org/wiki/Cyclic_redundancy_check
[wikipedia-cow]: https://en.wikipedia.org/wiki/Copy-on-write
[wikipedia-B-tree]: https://en.wikipedia.org/wiki/B-tree
[wikipedia-B+-tree]: https://en.wikipedia.org/wiki/B%2B_tree
[wikipedia-skip-list]: https://en.wikipedia.org/wiki/Skip_list
[wikipedia-ctz]: https://en.wikipedia.org/wiki/Count_trailing_zeros
[wikipedia-ecc]: https://en.wikipedia.org/wiki/Error_correction_code
[wikipedia-hamming-bound]: https://en.wikipedia.org/wiki/Hamming_bound
[wikipedia-dynamic-wear-leveling]: https://en.wikipedia.org/wiki/Wear_leveling#Dynamic_wear_leveling
[wikipedia-static-wear-leveling]: https://en.wikipedia.org/wiki/Wear_leveling#Static_wear_leveling
[wikipedia-ftl]: https://en.wikipedia.org/wiki/Flash_translation_layer

[oeis]: https://oeis.org
[A001511]: https://oeis.org/A001511
[A005187]: https://oeis.org/A005187

[fat]: https://en.wikipedia.org/wiki/Design_of_the_FAT_file_system
[ext2]: http://e2fsprogs.sourceforge.net/ext2intro.html
[jffs]: https://www.sourceware.org/jffs2/jffs2-html
[yaffs]: https://yaffs.net/documents/how-yaffs-works
[spiffs]: https://github.com/pellepl/spiffs/blob/master/docs/TECH_SPEC
[ext4]: https://ext4.wiki.kernel.org/index.php/Ext4_Design
[ntfs]: https://en.wikipedia.org/wiki/NTFS
[btrfs]: https://btrfs.wiki.kernel.org/index.php/Btrfs_design
[zfs]: https://en.wikipedia.org/wiki/ZFS

[cow]: https://upload.wikimedia.org/wikipedia/commons/0/0c/Cow_female_black_white.jpg
[elephant]: https://upload.wikimedia.org/wikipedia/commons/3/37/African_Bush_Elephant.jpg
[ram]: https://upload.wikimedia.org/wikipedia/commons/9/97/New_Mexico_Bighorn_Sheep.JPG

[metadata-formula1]: https://latex.codecogs.com/svg.latex?cost%20%3D%20n%20&plus;%20n%20%5Cfrac%7Bs%7D%7Bd&plus;1%7D
[metadata-formula2]: https://latex.codecogs.com/svg.latex?s%20%3D%20r%20%5Cfrac%7Bsize%7D%7Bn%7D
[metadata-formula3]: https://latex.codecogs.com/svg.latex?d%20%3D%20%281-r%29%20%5Cfrac%7Bsize%7D%7Bn%7D
[metadata-formula4]: https://latex.codecogs.com/svg.latex?cost%20%3D%20n%20&plus;%20n%20%5Cfrac%7Br%5Cfrac%7Bsize%7D%7Bn%7D%7D%7B%281-r%29%5Cfrac%7Bsize%7D%7Bn%7D&plus;1%7D

[ctz-formula1]: https://latex.codecogs.com/svg.latex?%5Clim_%7Bn%5Cto%5Cinfty%7D%5Cfrac%7B1%7D%7Bn%7D%5Csum_%7Bi%3D0%7D%5E%7Bn%7D%5Cleft%28%5Ctext%7Bctz%7D%28i%29&plus;1%5Cright%29%20%3D%20%5Csum_%7Bi%3D0%7D%5Cfrac%7B1%7D%7B2%5Ei%7D%20%3D%202
[ctz-formula2]: https://latex.codecogs.com/svg.latex?B%20%3D%20%5Cfrac%7Bw%7D%7B8%7D%5Cleft%5Clceil%5Clog_2%5Cleft%28%5Cfrac%7B2%5Ew%7D%7BB-2%5Cfrac%7Bw%7D%7B8%7D%7D%5Cright%29%5Cright%5Crceil
[ctz-formula3]: https://latex.codecogs.com/svg.latex?N%20%3D%20%5Csum_i%5En%5Cleft%5BB-%5Cfrac%7Bw%7D%7B8%7D%5Cleft%28%5Ctext%7Bctz%7D%28i%29&plus;1%5Cright%29%5Cright%5D
[ctz-formula4]: https://latex.codecogs.com/svg.latex?%5Csum_i%5En%5Cleft%28%5Ctext%7Bctz%7D%28i%29&plus;1%5Cright%29%20%3D%202n-%5Ctext%7Bpopcount%7D%28n%29
[ctz-formula5]: https://latex.codecogs.com/svg.latex?N%20%3D%20Bn%20-%20%5Cfrac%7Bw%7D%7B8%7D%5Cleft%282n-%5Ctext%7Bpopcount%7D%28n%29%5Cright%29
[ctz-formula6]: https://latex.codecogs.com/svg.latex?n%20%3D%20%5Cleft%5Clfloor%5Cfrac%7BN-%5Cfrac%7Bw%7D%7B8%7D%5Cleft%28%5Ctext%7Bpopcount%7D%5Cleft%28%5Cfrac%7BN%7D%7BB-2%5Cfrac%7Bw%7D%7B8%7D%7D-1%5Cright%29&plus;2%5Cright%29%7D%7BB-2%5Cfrac%7Bw%7D%7B8%7D%7D%5Cright%5Crfloor
[ctz-formula7]: https://latex.codecogs.com/svg.latex?%5Cmathit%7Boff%7D%20%3D%20N%20-%20%5Cleft%28B-2%5Cfrac%7Bw%7D%7B8%7D%5Cright%29n%20-%20%5Cfrac%7Bw%7D%7B8%7D%5Ctext%7Bpopcount%7D%28n%29

[bigB]: https://latex.codecogs.com/svg.latex?B
[d]: https://latex.codecogs.com/svg.latex?d
[m]: https://latex.codecogs.com/svg.latex?m
[bigN]: https://latex.codecogs.com/svg.latex?N
[n]: https://latex.codecogs.com/svg.latex?n
[n']: https://latex.codecogs.com/svg.latex?n%27
[r]: https://latex.codecogs.com/svg.latex?r
[s]: https://latex.codecogs.com/svg.latex?s
[w]: https://latex.codecogs.com/svg.latex?w
[x]: https://latex.codecogs.com/svg.latex?x

[metadata-cost-graph]: https://raw.githubusercontent.com/geky/littlefs/gh-images/metadata-cost.svg?sanitize=true
[wear-distribution-graph]: https://raw.githubusercontent.com/geky/littlefs/gh-images/wear-distribution.svg?sanitize=true
[file-cost-graph]: https://raw.githubusercontent.com/geky/littlefs/gh-images/file-cost.svg?sanitize=true
