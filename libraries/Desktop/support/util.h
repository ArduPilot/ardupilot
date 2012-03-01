
#define ft2m(x) ((x) * 0.3048)
#define kt2mps(x) ((x) * 0.514444444)
#define sqr(x) ((x)*(x))
#define ToRad(x) (x*0.01745329252)  // *pi/180

void set_nonblocking(int fd);
double normalise(double v, double min, double max);
double normalise180(double v);
void runInterrupt(uint8_t inum);

void convert_body_frame(double rollDeg, double pitchDeg,
			double rollRate, double pitchRate, double yawRate,
			double *p, double *q, double *r);
