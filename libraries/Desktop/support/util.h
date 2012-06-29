
#define ft2m(x) ((x) * 0.3048)
#define kt2mps(x) ((x) * 0.514444444)
#define sqr(x) ((x)*(x))
#define ToRad(x) (x*0.01745329252)  // *pi/180

void set_nonblocking(int fd);
double normalise(double v, double min, double max);
double normalise180(double v);
void runInterrupt(uint8_t inum);

// generate a random float between -1 and 1
#define rand_float() (((((unsigned)random()) % 2000000) - 1.0e6) / 1.0e6)

#ifdef VECTOR3_H
Vector3f rand_vec3f(void);
#endif
