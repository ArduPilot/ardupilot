
#define ft2m(x) ((x) * 0.3048)
#define kt2mps(x) ((x) * 0.514444444)
#define sqr(x) ((x)*(x))

double swap_double(double d);
void swap_doubles(double *d, unsigned count);
float swap_float(float f);
void swap_floats(float *f, unsigned count);
void set_nonblocking(int fd);
double normalise(double v, double min, double max);
