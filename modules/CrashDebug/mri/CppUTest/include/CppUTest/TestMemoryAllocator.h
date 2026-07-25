#ifndef D_TestMemoryAllocator_h
#define D_TestMemoryAllocator_h

struct MemoryLeakNode;
class TestMemoryAllocator;

extern void setCurrentNewAllocator(TestMemoryAllocator* allocator);
extern TestMemoryAllocator* getCurrentNewAllocator();
extern void setCurrentNewAllocatorToDefault();
extern TestMemoryAllocator* defaultNewAllocator();

extern void setCurrentNewArrayAllocator(TestMemoryAllocator* allocator);
extern TestMemoryAllocator* getCurrentNewArrayAllocator();
extern void setCurrentNewArrayAllocatorToDefault();
extern TestMemoryAllocator* defaultNewArrayAllocator();

extern void setCurrentMallocAllocator(TestMemoryAllocator* allocator);
extern TestMemoryAllocator* getCurrentMallocAllocator();
extern void setCurrentMallocAllocatorToDefault();
extern TestMemoryAllocator* defaultMallocAllocator();

class TestMemoryAllocator
{
public:
	TestMemoryAllocator(const char* name_str = "generic", const char* alloc_name_str = "alloc", const char* free_name_str = "free");
	virtual ~TestMemoryAllocator();
	bool hasBeenDestroyed();

	virtual char* alloc_memory(size_t size, const char* file, int line);
	virtual void free_memory(char* memory, const char* file, int line);

	virtual const char* name();
	virtual const char* alloc_name();
	virtual const char* free_name();

	virtual bool isOfEqualType(TestMemoryAllocator* allocator);

	virtual char* allocMemoryLeakNode(size_t size);
	virtual void freeMemoryLeakNode(char* memory);

protected:

	const char* name_;
	const char* alloc_name_;
	const char* free_name_;

	bool hasBeenDestroyed_;
};

class CrashOnAllocationAllocator : public TestMemoryAllocator
{
	unsigned allocationToCrashOn_;
public:
	CrashOnAllocationAllocator();

	virtual void setNumberToCrashOn(unsigned allocationToCrashOn);

	virtual char* alloc_memory(size_t size, const char* file, int line);
};


class NullUnknownAllocator: public TestMemoryAllocator
{
public:
	NullUnknownAllocator();
	virtual char* alloc_memory(size_t size, const char* file, int line);
	virtual void free_memory(char* memory, const char* file, int line);

	static TestMemoryAllocator* defaultAllocator();
};

#endif

