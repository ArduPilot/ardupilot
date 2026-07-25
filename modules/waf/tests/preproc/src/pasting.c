#define STRINGIFY2( x) #x
#define STRINGIFY(x) STRINGIFY2(x)

#define PASTE2( x, y) x ## y
#define PASTE(x, y) PASTE2(x, y)

#include STRINGIFY(PASTE(PREFIX_VAL, PASTE(a, SUFFIX_VAL)).h)

int main() {return 0;}
