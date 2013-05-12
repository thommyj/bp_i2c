/*
 *  bp_i2c.c
 *
 *  Created by Thommy Jakobsson 2013
 *  Copyright 2013 Thommy Jakobsson. All rights reserved.
 *
 */

/*
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <time.h>
#include <stdlib.h>


//how many reads to send to BP each time
//data will be stored in serial buffer
#define CONS_READ 20

//max for BP at the moment
//need to be a multiple of CHIP_CONS_WRITE
#define CONS_WRITE 16
#define CHIP_CONS_WRITE 128

#define BP_RESET 0x00
#define BP_I2C   0x02
#define BP_EXIT  0x0F
#define BP_POWER 0x48
#define BP_PULLU 0x44
#define BP_400k  0x63
#define BP_P33V  0x51

#define BP_RET_BIN "BBIO1"
#define BP_RET_I2C "I2C1"


#define I2C_START 0x02
#define I2C_STOP  0x03
#define I2C_WRITE 0x10
#define I2C_ACK   0x06
#define I2C_NACK  0x07
#define I2C_READ  0x04

#define I2C_START_RET 0x01
#define I2C_STOP_RET  0x01
#define I2C_WRITE_RET 0x01
#define I2C_TRANSF_OK 0x00

#define CHIP_ADDR 0xA0

int init_port(int fd) {
	struct termios options;
	
	printf("setting up serial...");
	// Get the current options for the port...
	tcgetattr(fd, &options);
	// Set the baud rates to 19200...
	cfsetispeed(&options, B115200);
	cfsetospeed(&options, B115200);
	//ignore modem controllines + enable receiver 
	options.c_cflag |= (CLOCAL | CREAD);
        //ingen paritet
	options.c_cflag &= ~PARENB;
	//no stop
        options.c_cflag &= ~CSTOPB;
	//8bits character
        options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;
        //no hw flowcontrol
        options.c_cflag &= ~CRTSCTS;

        //raw mode
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        //no sw flowcontrol
        options.c_lflag &= ~(IXON | IXOFF | IXANY);

	//raw mode output
        options.c_oflag &= ~OPOST;
	//non blocking
	options.c_cc[VMIN]=0;
	options.c_cc[VTIME]=0;
	// Set the new options for the port...
	tcsetattr(fd, TCSANOW|TCSAFLUSH, &options);
	
	printf("done\r\n");	

	return 1;
}

/*
 * read from serial port with timeout (without select)
 *
 */
#define TO_CHAR 100
#define TO_POLL (TO_CHAR/10)
int read_ser(const int fd,unsigned char *buf,const int bytes){
	int i,rb;
	clock_t t1,t2;

	t1=clock();
	t2=clock();
	for(i=0;(t2-t1)/(CLOCKS_PER_SEC/1000)<(TO_CHAR*bytes);){
		rb = read(fd,buf+i,bytes);
		i += rb;
		if(i==bytes)
			break;
		usleep(TO_POLL);
		t2=clock();
	}
	return i;
}

/*
 * Write to serial port
 */
int write_ser(const int fd,const unsigned char *buf,const int bytes){
        return write(fd,buf,bytes);
}

void flush_ser(const int fd)
{
        usleep(10000);
	tcflush(fd,TCIFLUSH);
	usleep(10000);
}

int init_bp(int fd)
{
	char rb[10];
	unsigned char wb[10];
	int i,ret;
	

	/*
	 * reset device according to wiki
	*/
	printf("if in some configuration, try to get BP out...");
	wb[0] = '\r';
	wb[1] = '\n';
	for(i=0;i<10;i++){
		write_ser(fd,wb,2);
		usleep(1000);
	}
	wb[0] = '#';
	write_ser(fd,wb,1);
	printf("done\r\n");        
	/*
	 *send binary mode request
	*/
	printf("set BP in binary mode...");
	flush_ser(fd);
	for(i=0;i<20;i++){
		wb[0] = BP_RESET;
		write_ser(fd,&wb[0],1);
		usleep(1000);
	}

	/*
	 *check if in binary mode
	 */
	memset(rb,0,5);
	ret = read_ser(fd,(unsigned char*)rb,5);
	if(ret!=5 || strncmp(BP_RET_BIN,rb,strlen(BP_RET_BIN))){
		rb[ret] = '\0';
		printf("nok (read %s)\r\n",rb);
		return 0;
	}
	printf("ok\r\n");
	
	/*
	 *setting up I2C*
	 */
	printf("trying to set I2C..."),
        flush_ser(fd);
        wb[0] = BP_I2C;
        write_ser(fd,wb,1);
        ret = read_ser(fd,(unsigned char*)rb,4);
        if(ret==4 && !strncmp(BP_RET_I2C,rb,strlen(BP_RET_I2C))){
                printf("ok\r\n");
        }
        else{
                rb[ret]='\0';
                printf("nok (read \"%s\")\r\n",rb);
                return 0;
        }

	/*
	 *setting i2c speed
	 */
	printf("setting i2c speed...");
	wb[0] = BP_400k;
	write_ser(fd,wb,1);
        usleep(1000);
        read_ser(fd,(unsigned char*)rb,1);
        if(ret!=1 && rb[0]!=1){
                printf("nok (read %d)\r\n",rb[0]);
                return 0;
        }
        else{
                printf("ok\r\n");
                return 1;
        }
}

int init_pwr(int fd)
{
	unsigned char tmp[10];
	int ret;
	/*
	 *config pullups 3.3V
	 */
	printf("setting pullup to 3.3V...");
	tmp[0] = BP_P33V;
        write_ser(fd,tmp,1);
        usleep(1000);
        read_ser(fd,tmp,1);
        if(ret!=1 && tmp[0]!=1){
                printf("nok (read %d)\r\n",tmp[0]);
                return 0;
        }
        else{
                printf("ok\r\n");
        }	

	/*
	 * pwrup memory
	 */
	printf("enabling power and pullup...");
	tmp[0] = BP_POWER | BP_PULLU;
	write_ser(fd,tmp,1);
	usleep(1000);
	read_ser(fd,tmp,1);
	if(ret!=1 && tmp[0]!=1){
		printf("nok (read %d)\r\n",tmp[0]);
		return 0;
	}
	else{
		printf("ok\r\n");
		return 1;
	}

}

void dis_pwr(int fd)
{
	unsigned char tmp;
	printf("disabling power...");
	tmp = 0x0;
	write_ser(fd,&tmp,1);
	read_ser(fd,&tmp,1);
	printf("done\r\n");
}

int read_to_file(int fdser,unsigned char* buf,int bytes)
{
	char unsigned data[CONS_READ*2];
	char unsigned retdata[CONS_READ*2];
	int ret;
	int i,j;
	int bytes_to_read;
	
	/*
	 * send address
	 */
	printf("writing address to chip..");
	data[0] = I2C_START;
	data[1] = I2C_WRITE | 0x2;
	data[2] = CHIP_ADDR;
	//read from the beginning
	data[3] = 0x0;
	data[4] = 0x0;
	data[5] = I2C_START;
	data[6] = I2C_WRITE | 0x0;
	data[7] = CHIP_ADDR | 0x1; 
	write_ser(fdser,data,8);

	//setup expected return value
	//and check if everything is ok
	retdata[0] = I2C_START_RET;
	retdata[1] = I2C_WRITE_RET;
	retdata[2] = I2C_TRANSF_OK;
	retdata[3] = I2C_TRANSF_OK;
	retdata[4] = I2C_TRANSF_OK;
	retdata[5] = I2C_START_RET;
	retdata[6] = I2C_WRITE_RET;
	retdata[7] = I2C_TRANSF_OK;
	memset(data,0,8);
	ret = read_ser(fdser,data,8);
	if(ret!=8 || memcmp(data,retdata,8)){
		printf("nok\r\n");
		return 0;
	}
	printf("ok\r\n");

	/*
	 * read out the data
	 */
	printf("reading from flash...");
        fflush(stdout); //make sure that text comes out

	//send and receive Xbytes at the time
	//doesn't matter if we read to much
	for(i=0;i<CONS_READ*2;i+=2){
		data[i] = I2C_READ;
        	data[i+1] = I2C_ACK;
	}

	for(i=0;i<bytes;){
		if(bytes-i<=CONS_READ){
			//each data byte is followed by an ACK
			//except last data, it should be a NACK (so -1)
			bytes_to_read = (bytes-i)*2-1;
		}
		else
			//send an ACK for each byte read
			bytes_to_read = CONS_READ*2;
		write_ser(fdser,data,bytes_to_read);
		ret = read_ser(fdser,retdata,bytes_to_read);
		if(ret!=bytes_to_read){
			break;
		}
		//copy every second byte to buffer
		//the rest are just BP returns for our ACKs
		//+1 beacuse of last byte can be uneven number
		//due to no ACK
		for(j=0;j<(bytes_to_read+1)/2;j++){
			buf[i++] = retdata[2*j];
		}
	}
	//close transfer with NACK followed by a STOP
	//according to datasheet
	data[0] = I2C_NACK;
	data[1] = I2C_STOP;
	write_ser(fdser,data,2);
	read_ser(fdser,data,2);

        if(i!=bytes){
                printf("..nok\r\n");
        }else{
                printf("..ok\r\n");
        }
	
	return 1;
} 

int write_to_file(int fdser,unsigned char* buf,int bytes)
{
	char unsigned setup_data[6];
	char unsigned retsetup_data[6];
	char unsigned data[CONS_WRITE*2];
	char unsigned retdata[CONS_WRITE*2];
	int ret;
	int ri,j=0;
	int bytes_to_write;
	
	/*
	 * setup data as much as possible
	 */
	setup_data[0] = I2C_START;
	setup_data[1] = I2C_WRITE | 0x2;
	setup_data[2] = CHIP_ADDR;

	retsetup_data[0] = I2C_START_RET;
        retsetup_data[1] = I2C_WRITE_RET;
        retsetup_data[2] = I2C_TRANSF_OK;
        retsetup_data[3] = I2C_TRANSF_OK;
        retsetup_data[4] = I2C_TRANSF_OK;

	//setup expected return data
	retdata[0] = I2C_WRITE_RET;
	for(j=1;j<CONS_WRITE+1;j++)
		retdata[j] = I2C_TRANSF_OK;

	/*
         * write the data
         */
        printf("writing to flash...");
        fflush(stdout); //make sure that text comes out	

	for(ri=0;ri<bytes;){
		//setup page write
		if(ri%CHIP_CONS_WRITE==0){
			//not first byte, send stop for previous
			if(ri!=0){
				data[0] = I2C_STOP;
				write_ser(fdser,data,1);
				read_ser(fdser,data,1);
			}
	
			setup_data[3] = (ri>>8)&0xFF;
			setup_data[4] = ri&0xFF;
			write_ser(fdser,setup_data,5);

			//and check if everything is ok
        		ret = read_ser(fdser,data,5);
        		if(ret!=5 || memcmp(data,retsetup_data,5)){
				printf("unable to setup pagewrite...");
				break;
			}
		}
	
		//16bytes left in buffer? if not write rest of buffer
		if(bytes-ri<CONS_WRITE)
			bytes_to_write = bytes-ri;
		else
			bytes_to_write = CONS_WRITE;

		//setup data to send (up to 16bytes at the time)
		data[0] = I2C_WRITE | (bytes_to_write-1);
                for(j=1;j<(bytes_to_write+1);j++){
                	data[j] = buf[ri++];
        	}	

		//write data, and read back answer
		write_ser(fdser,data,bytes_to_write+1);
		ret = read_ser(fdser,data,bytes_to_write+1);
		if(ret!=bytes_to_write+1){
			printf("expect %d bytes, wrote only %d...",CONS_WRITE+1,ret);
			break;
		}

		//check for ACKs
		if(memcmp(data,retdata,bytes_to_write+1)){
			printf("got nack...");
			break;
		}

	}
	//close transfer with NACK followed by a STOP
	//according to datasheet
	data[0] = I2C_STOP;
	write_ser(fdser,data,1);
	read_ser(fdser,data,1);

        if(ri!=bytes){
                printf("nok\r\n");
        }else{
                printf("ok\r\n");
        }
	
	return 1;
} 


void printusage(char *name)
{
        printf("usage: %s device <r/w> file bytes\r\nif writing, bytes can be left out\r\n",name);
}

int main(int argc, char **argv) {
	int fdser = 0;
	FILE *file = NULL;
	int bytes,filesize;
	int pwron = 0;
	unsigned char *buf = NULL;
	char rw;

	//to few qrguments?
	if(argc<4){
		printf("to few arguments!\r\n");
		printusage(argv[0]);
		goto error;
	}

	//read or write?
	rw = argv[2][0];
	//to few arguments for read
	if(rw == 'r' && argc < 5){
		printf("to few arguments for read\r\n");
                printusage(argv[0]);
                goto error;
	}
	
	//neither r or w?
	if(rw != 'r' && rw != 'w'){
                printf("read or write?\r\n");
                printusage(argv[0]);
                goto error;
        }
	
	if(argc>4)
		sscanf(argv[4],"%d",&bytes);
	else
		bytes = -1;

	if(rw == 'r'){
		printf("using %s, dumping %d bytes to %s\r\n",argv[1],bytes,argv[3]);
	}else if(rw == 'w'){
		if(bytes == -1)
	                printf("using %s, writing entire %s\r\n",argv[1],argv[3]);
		else
			 printf("using %s, writing %d bytes from %s\r\n",argv[1],bytes,argv[3]);
	}

	fdser = open(argv[1], O_RDWR | O_NOCTTY | O_NDELAY);
	if (fdser == -1) {
		perror("Unable to open port");
		goto error;
	}else{
		//fcntl(fdser, F_SETFL, 0);
	}

	if(!init_port(fdser))
		goto error;
	if(!init_bp(fdser))
		goto error;
	if(!init_pwr(fdser))
		goto error;
	else
		pwron = 1;

	/*
	 * read
	 */
	if(rw == 'r'){
	        buf = malloc(bytes);
        	if(!buf){
                	printf("unable to allocate buffer\r\n");
                	goto error;
        	}

		if(!read_to_file(fdser,buf,bytes)){
			printf("unable to read from memory!\r\n");
			goto error;
		}
	        file = fopen(argv[3],"wb");

        	if(!file){
                	perror("unable to open file:");
                	goto error;
        	}

		if(1!=fwrite(buf,bytes,1,file)){
			perror("unable to write to file:");
			goto error;
		}
	}else{
		file = fopen(argv[3],"rb");
		if(!file){
			perror("unable to open file:");
			goto error;
		}
		//checking file size	
		fseek(file,0L,SEEK_END);
		filesize = ftell(file);
		if(bytes > filesize){
			printf("warning: file is smaller than requested #bytes, will saturate\r\n");
			bytes = filesize;
		}else if(bytes==-1)
			bytes = filesize;

		//create buffer
		buf = malloc(bytes);
		if(!buf){
                        printf("unable to allocate buffer\r\n");
                        goto error;
                }
		
		//read in file into buffer
		rewind(file);
		if(1!=fread(buf,bytes,1,file)){
			perror("unable to write to file:");
			goto error;
		}

		if(!write_to_file(fdser,buf,bytes)){
			printf("unable to write to memory!\r\n");
			goto error;
		}
	}

	
error:
	if(buf)
		free(buf);
	if(pwron)
		dis_pwr(fdser);
	if(fdser)
		close(fdser);
	if(file)
		fclose(file);
	return 0;
}



