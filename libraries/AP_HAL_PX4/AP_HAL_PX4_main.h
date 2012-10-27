// this is included at the end of every sketch
extern "C" __EXPORT int SKETCH_MAIN(int argc, char *argv[]);
int SKETCH_MAIN(int argc, char *argv[]) 
{
    printf("%s starting\n", SKETCHNAME);
    hal.init(NULL);
    setup();
    for(;;) loop();
    return OK;
}
