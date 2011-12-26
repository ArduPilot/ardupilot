
#define ft2m(x) ((x) * 0.3048)
#define kt2mps(x) ((x) * 0.514444444)
#define sqr(x) ((x)*(x))

void set_nonblocking(int fd);
double normalise(double v, double min, double max);
double normalise180(double v);
void runInterrupt(uint8_t inum);
