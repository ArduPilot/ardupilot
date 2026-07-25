#ifndef D_MemoryLeakDetector_h
#define D_MemoryLeakDetector_h

#define MEM_LEAK_NONE "No memory leaks were detected."
#define MEM_LEAK_HEADER "Memory leak(s) found.\n"
#define MEM_LEAK_LEAK "Alloc num (%u) Leak size: %d Allocated at: %s and line: %d. Type: \"%s\"\n\t Memory: <%p> Content: \"%.15s\"\n"
#define MEM_LEAK_TOO_MUCH "\netc etc etc etc. !!!! Too much memory leaks to report. Bailing out\n"
#define MEM_LEAK_FOOTER "Total number of leaks: "
#define MEM_LEAK_ADDITION_MALLOC_WARNING "NOTE:\n" \
										 "\tMemory leak reports about malloc and free can be caused by allocating using the cpputest version of malloc,\n" \
										 "\tbut deallocate using the standard free.\n" \
										 "\tIf this is the case, check whether your malloc/free replacements are working (#define malloc cpputest_malloc etc).\n"

#define MEM_LEAK_NORMAL_FOOTER_SIZE (sizeof(MEM_LEAK_FOOTER) + 10 + sizeof(MEM_LEAK_TOO_MUCH)) /* the number of leaks */
#define MEM_LEAK_NORMAL_MALLOC_FOOTER_SIZE (MEM_LEAK_NORMAL_FOOTER_SIZE + sizeof(MEM_LEAK_ADDITION_MALLOC_WARNING))


#define MEM_LEAK_ALLOC_DEALLOC_MISMATCH "Allocation/deallocation type mismatch\n"
#define MEM_LEAK_MEMORY_CORRUPTION "Memory corruption (written out of bounds?)\n"
#define MEM_LEAK_ALLOC_LOCATION "   allocated at file: %s line: %d size: %d type: %s\n"
#define MEM_LEAK_DEALLOC_LOCATION "   deallocated at file: %s line: %d type: %s\n"
#define MEM_LEAK_DEALLOC_NON_ALLOCATED "Deallocating non-allocated memory\n"

enum MemLeakPeriod
{
	mem_leak_period_all,
	mem_leak_period_disabled,
	mem_leak_period_enabled,
	mem_leak_period_checking
};

class TestMemoryAllocator;

#include "StandardCLibrary.h"

class MemoryLeakFailure
{
public:
	virtual ~MemoryLeakFailure()
	{
	}
	;
	virtual void fail(char* fail_string)=0;
};

struct SimpleStringBuffer
{
	enum
	{
		SIMPLE_STRING_BUFFER_LEN = 4096
	};

	SimpleStringBuffer();
	void clear();
	void add(const char* format, ...);
	char* toString();

	void setWriteLimit(int write_limit);
	void resetWriteLimit();
	bool reachedItsCapacity();
private:
	char buffer_[SIMPLE_STRING_BUFFER_LEN];
	int positions_filled_;
	int write_limit_;
};

struct MemoryLeakDetectorNode
{
	MemoryLeakDetectorNode() :
		size_(0), next_(0)
	{
	}

	void init(char* memory, unsigned number, size_t size, TestMemoryAllocator* allocator, MemLeakPeriod period, const char* file, int line);

	size_t size_;
	unsigned number_;
	char* memory_;
	const char* file_;
	int line_;
	TestMemoryAllocator* allocator_;
	MemLeakPeriod period_;

private:
	friend struct MemoryLeakDetectorList;
	MemoryLeakDetectorNode* next_;
};

struct MemoryLeakDetectorList
{
	MemoryLeakDetectorList() :
		head_(0)
	{}

	void addNewNode(MemoryLeakDetectorNode* node);
	MemoryLeakDetectorNode* retrieveNode(char* memory);
	MemoryLeakDetectorNode* removeNode(char* memory);

	MemoryLeakDetectorNode* getFirstLeak(MemLeakPeriod period);
	MemoryLeakDetectorNode* getNextLeak(MemoryLeakDetectorNode* node,
			MemLeakPeriod period);
	MemoryLeakDetectorNode* getLeakFrom(MemoryLeakDetectorNode* node,
			MemLeakPeriod period);

	int getTotalLeaks(MemLeakPeriod period);
	bool hasLeaks(MemLeakPeriod period);
	void clearAllAccounting(MemLeakPeriod period);

	bool isInPeriod(MemoryLeakDetectorNode* node, MemLeakPeriod period);

private:
	MemoryLeakDetectorNode* head_;
};

struct MemoryLeakDetectorTable
{
	void clearAllAccounting(MemLeakPeriod period);

	void addNewNode(MemoryLeakDetectorNode* node);
	MemoryLeakDetectorNode* retrieveNode(char* memory);
	MemoryLeakDetectorNode* removeNode(char* memory);

	bool hasLeaks(MemLeakPeriod period);
	int getTotalLeaks(MemLeakPeriod period);

	MemoryLeakDetectorNode* getFirstLeak(MemLeakPeriod period);
	MemoryLeakDetectorNode* getNextLeak(MemoryLeakDetectorNode* leak,
			MemLeakPeriod period);

private:
	unsigned long hash(char* memory);

	enum
	{
		hash_prime = MEMORY_LEAK_HASH_TABLE_SIZE
	};
	MemoryLeakDetectorList table_[hash_prime];
};

class MemoryLeakDetector
{
public:
	MemoryLeakDetector(MemoryLeakFailure* reporter);
	virtual ~MemoryLeakDetector()
	{
	}

	void enable();
	void disable();

	void disableAllocationTypeChecking();
	void enableAllocationTypeChecking();

	void startChecking();
	void stopChecking();

	const char* report(MemLeakPeriod period);
	void markCheckingPeriodLeaksAsNonCheckingPeriod();
	int totalMemoryLeaks(MemLeakPeriod period);
	void clearAllAccounting(MemLeakPeriod period);

	char* allocMemory(TestMemoryAllocator* allocator, size_t size, bool allocatNodesSeperately = false);
	char* allocMemory(TestMemoryAllocator* allocator, size_t size,
			const char* file, int line, bool allocatNodesSeperately = false);
	void deallocMemory(TestMemoryAllocator* allocator, void* memory, bool allocatNodesSeperately = false);
	void deallocMemory(TestMemoryAllocator* allocator, void* memory, const char* file, int line, bool allocatNodesSeperately = false);
	char* reallocMemory(TestMemoryAllocator* allocator, char* memory, size_t size, const char* file, int line, bool allocatNodesSeperately = false);

	void invalidateMemory(char* memory);
	void removeMemoryLeakInformationWithoutCheckingOrDeallocating(void* memory);
	enum
	{
		memory_corruption_buffer_size = 3
	};

	unsigned getCurrentAllocationNumber();
private:
	MemoryLeakFailure* reporter_;
	MemLeakPeriod current_period_;
	SimpleStringBuffer output_buffer_;
	MemoryLeakDetectorTable memoryTable_;
	bool doAllocationTypeChecking_;
	unsigned allocationSequenceNumber_;

	bool validMemoryCorruptionInformation(char* memory);
    bool matchingAllocation(TestMemoryAllocator *alloc_allocator, TestMemoryAllocator *free_allocator);

	void storeLeakInformation(MemoryLeakDetectorNode * node, char *new_memory, size_t size, TestMemoryAllocator *allocator, const char *file, int line);
    void ConstructMemoryLeakReport(MemLeakPeriod period);
	void reportFailure(const char* message, const char* allocFile,
			int allocLine, size_t allocSize,
			TestMemoryAllocator* allocAllocator, const char* freeFile,
			int freeLine, TestMemoryAllocator* freeAllocator);

	size_t sizeOfMemoryWithCorruptionInfo(size_t size);
	MemoryLeakDetectorNode* getNodeFromMemoryPointer(char* memory, size_t size);

	char* reallocateMemoryAndLeakInformation(TestMemoryAllocator* allocator, char* memory, size_t size, const char* file, int line);

	void addMemoryCorruptionInformation(char* memory);
	void checkForCorruption(MemoryLeakDetectorNode* node, const char* file, int line, TestMemoryAllocator* allocator, bool allocateNodesSeperately);
};

#endif
