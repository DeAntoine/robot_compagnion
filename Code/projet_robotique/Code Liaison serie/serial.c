#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include "serial.h"

void serial_open(serial_com *sp, char *name){
    sp->fd = open(name, O_RDWR);
    sp->nomDuPort = malloc(20*sizeof(char));
    strcpy(sp->nomDuPort, name);
}

void serial_close(serial_com *sp){
    close(sp->fd);
}

void serial_read(serial_com *sp){
    char buf;
    int n;
    while (1) 
    {
        read(sp->fd, &buf, 1);
        printf("%c\n ",buf);
    }
}

void serial_write(char txt, serial_com *sp)
{
        write(sp->fd, &txt, 1);
}

int main()
{
    char c;
    serial_com *sp = malloc(sizeof(serial_com));
    printf( "waiting for connexion\n");
    serial_open(sp, "/dev/cu.usbmodem142401");
    printf( "connexion established\n");
  
    while(1)
    {
        //printf("Donnez un caractère à envoyer :\n");
        c = getchar();
        serial_write(c, sp);
    }
    serial_close(sp);
    return 0;
}