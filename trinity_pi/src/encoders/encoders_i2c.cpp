/*
A user-space program to get data from an I2C device.
                Gustavo Zamboni
*/
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
 
char buf[10];
extern int com_serial;
extern int failcount;
 
//////////
// Init I2Cdevice
//////////
int i2c_init(char filename[40], int addr)
    {
        int file;
 
        if ((file = open(filename,O_RDWR)) < 0)
        {
            printf("Failed to open the bus.");
            /* ERROR HANDLING; you can check errno to see what went wrong */
        com_serial=0;
            exit(1);
            }
 
    if (ioctl(file,I2C_SLAVE,addr) < 0)
        {
            printf("Failed to acquire bus access and/or talk to slave.\n");
            /* ERROR HANDLING; you can check errno to see what went wrong */
        com_serial=0;
            exit(1);
            }
    return file;
    }
 
 
//////////
// Set pointer address
//////////
void i2c_set_pointer(int address,int value,int file)
    {
    
    //printf("end: 0x%x%x\n", address,value);
    if (i2c_smbus_write_byte_data(file, address, value)<0)
    {
        fprintf(stderr, "Warning - write failed\n");
    } 
    
    /*char buf[10];
    buf[0]=address;
    buf[1]=value;
 
    if (write(file, buf, 2) != 2)
        {
        fprintf(stderr, "Error setting pointer\n");
        com_serial=0;
        failcount++;
        } 
    else
        {
        //printf("w_0x%0*x\n", 2, buf[0]);
        //printf("w_0x%0*x\n", 2, buf[1]);
        com_serial=1;
        failcount=0;
        }*/
 
    }
 
//////////
// Read n bytes
//////////
 
char * i2c_read(int add1, int add2, int nbytes,int file)
    {
    int n;
 
    i2c_set_pointer(add1,add2,file);
 
    char* data = i2c_smbus_read_word_data(file, add2);
        /*{
        fprintf(stderr, "Error reading %i bytes\n",nbytes);
        com_serial=0;
        failcount++;
        } 
    else
        {
        for (n=0;n<nbytes;n++)
            {
            //printf("r_0x%0*x\n", 2, buf[n]);
            }
        com_serial=1;
        failcount=0;
        return buf;
        }*/
	return data;
    }
 
//////////
// Write n bytes
//////////
void i2c_write(int add1,int add2,int nbytes,char value[10],int file)
    {
    int n;
    unsigned char buf[10];
    buf[0] = add1;
    buf[1] = add2;
    if (nbytes>=1) buf[2] = value[0];
    if (nbytes>=2) buf[3] = value[1];
    if (nbytes>=3) buf[4] = value[2];
    if (nbytes>=4) buf[5] = value[3];
 
    if (write(file, buf, nbytes) != nbytes)
        {
        fprintf(stderr, "Error writing %i bytes\n",nbytes);
        com_serial=0;
        failcount++;
        } 
    else
        {
        for (n=0;n<(nbytes+2);n++)
            {
            //printf("w_0x%0*x\n", 2, buf[n]);
            }
        com_serial=1;
        failcount=0;
        }
    }

int main(int argc, char** argv){
	char *buffer
	char buf[12];

	file = i2c_init("/dev/i2c-1", 0xAA);

	buffer = (char*) i2c_read(0x00, 0x0E, 12, file); //read at address 0xE60E

	printf("%s\n", buffer);

	close(file);
}
