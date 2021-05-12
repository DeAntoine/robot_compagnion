typedef struct {
    char* nomDuPort;
    int fd;
} serial_com;

void serial_open(serial_com *sp, char *name);
void serial_read(serial_com *sp);
void serial_close(serial_com *sp);
void serial_write(char txt, serial_com *sp);