
struct desktop_state {
	bool slider; // slider switch state, True means CLI mode
	struct timeval sketch_start_time;
};

extern struct desktop_state desktop_state;

void desktop_serial_select_setup(fd_set *fds, int *fd_high);

