/*************************************************************************
    > File Name: uart_operation.c
    > Author: AnSwEr
    > Mail: 1045837697@qq.com
    > Created Time: 2015年09月02日 星期三 12时45分48秒
 ************************************************************************/

#include<stdio.h>
#include<stdlib.h>
#include<fcntl.h>
#include<unistd.h>
#include<assert.h>
#include<termios.h>
#include<string.h>
#include<sys/time.h>
#include<sys/types.h>
#include<errno.h>

static int ret;
static int fd;

/*
 * 安全读写函数
 */

ssize_t safe_write(int fd, const void *vptr, size_t n)
{
    size_t  nleft;
    ssize_t nwritten;
    const char *ptr;

    ptr = vptr;
    nleft = n;

    while(nleft > 0)
    {
    if((nwritten = write(fd, ptr, nleft)) <= 0)
        {
            if(nwritten < 0&&errno == EINTR)
                nwritten = 0;
            else
                return -1;
        }
        nleft -= nwritten;
        ptr   += nwritten;
    }
    return(n);
}

ssize_t safe_read(int fd,void *vptr,size_t n)
{
    size_t nleft;
    ssize_t nread;
    char *ptr;

    ptr=vptr;
    nleft=n;

    while(nleft > 0)
    {
        if((nread = read(fd,ptr,nleft)) < 0)
        {
            if(errno == EINTR)//被信号中断
                nread = 0;
            else
                return -1;
        }
        else
        if(nread == 0)
            break;
        nleft -= nread;
        ptr += nread;
    }
    return (n-nleft);
}

int uart_open(int fd,const char *pathname)
{
    assert(pathname);

    /*打开串口*/
    fd = open(pathname,O_RDWR|O_NOCTTY|O_NDELAY);
    if(fd == -1)
    {
        perror("Open UART failed!");
        return -1;
    }

    /*清除串口非阻塞标志*/
    if(fcntl(fd,F_SETFL,0) < 0)
    {
        fprintf(stderr,"fcntl failed!\n");
        return -1;
    }
    printf("open file ok\n");
    return fd;
}

int speed_arr[] = {B115200, B57600, B38400, B19200, B9600, B4800, B2400, B1200, B300};
int name_arr[] 	= {115200,  57600,  38400,  19200,  9600,  4800,  2400,  1200,  300};
 
 #define TRUE 1
#define FALSE 0
 
/*
**********************************************************************************************************
*说    明:设置波特率，数据位，停止位，奇偶校验位
*入口参数:
*出口参数:
**********************************************************************************************************
*/
int wzt_SetSerialParameter(int fd, int baudrate, int databits, int stopbits, int parity)
{
	struct	termios	scmTermios	;
 
	if(tcgetattr(fd,&scmTermios) != 0){
		perror("Get serial parameter error");
		return(FALSE);
	}
	/*	设置波特率	*/
	int i = 0;
	for(i = 0; i < sizeof(speed_arr)/sizeof(int); i++){
		if(baudrate == name_arr[i]){
			tcflush(fd, TCIOFLUSH);										/*	清除输入输出缓冲区				*/
			if(0 != cfsetispeed(&scmTermios,speed_arr[i])){				/*	设置输入波特率	成功返回0 否则-1	*/
				printf("ERROR:Set input Serial baudrate fail\n");
			}
			if(0 != cfsetospeed(&scmTermios,speed_arr[i])){				/*	设置输出波特率	成功返回0 否则-1	*/
				printf("ERROR:Set output Serial baudrate fail\n");
			}
			break;
		}
	}
	/*	设置数据位	*/
	scmTermios.c_cflag &= ~CSIZE;							/*	清零							*/
	switch (databits){
		case 7:	scmTermios.c_cflag |= CS7;	break;
		case 8:	scmTermios.c_cflag |= CS8;	break;
		default:fprintf(stderr,"Unsupported data size\n");return (FALSE);
	}
	/*	设置停止位	*/
	switch(stopbits){
		case 1:	scmTermios.c_cflag &= ~CSTOPB;	break;
		case 2:	scmTermios.c_cflag |= CSTOPB;	break;
		default:fprintf(stderr,"Unsupported stop bits\n");return (FALSE);
	}
	/*	奇偶校验位	*/
	switch (parity){
		case 'n':
		case 'N':
			scmTermios.c_cflag &= ~PARENB;
			scmTermios.c_iflag &= ~INPCK;
			break;
		case 'o':
		case 'O':
			scmTermios.c_cflag |= (PARODD | PARENB);
			scmTermios.c_iflag |= INPCK;
			break;
		case 'e':
		case 'E':
			scmTermios.c_cflag |= PARENB;
			scmTermios.c_cflag &= ~PARODD;
			scmTermios.c_iflag |= INPCK;
			break;
		case 'S':
		case 's':
			scmTermios.c_cflag &= ~PARENB;
			scmTermios.c_cflag &= ~CSTOPB;
			break;
		default:
			fprintf(stderr,"Unsupported parity\n");
			return (FALSE);
	}
	if(parity != 'n') scmTermios.c_iflag |= INPCK;
 
	scmTermios.c_cc[VTIME] 	= 150;						/*	非规范模式读取时的超时时间			*/
	scmTermios.c_cc[VMIN] 	= 0;						/*	非规范模式读取时的最小字符数			*/
 
	tcflush(fd, TCIOFLUSH);						/*	清除输入输出缓冲区				*/
	if (tcsetattr(fd,TCSANOW,&scmTermios) != 0){		/*	不等数据传输完毕就立即改变属性		<span style="white-space:pre">	</span>*/
		perror("Set serial parameter error");
		return (FALSE);
	}
	else{
		printf("MSG  :Set serial parameter successfully!\n");
	}
 
	return (TRUE);
}
 
/*
**********************************************************************************************************
*说    明:标准输入模式，显示输入字符
*入口参数:
*出口参数:
**********************************************************************************************************
*/
void wzt_DisableTerminalReturn(int fd)
{
    struct termios scmTermios;
 
    tcgetattr(fd, &scmTermios);
    scmTermios.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(fd, TCSANOW, &scmTermios);
}
 

int uart_set(int fd,int baude,int c_flow,int bits,char parity,int stop)
{
    struct termios options;

    /*获取终端属性*/
    if(tcgetattr(fd,&options) < 0)
    {
        perror("tcgetattr error");
        return -1;
    }


    /*设置输入输出波特率，两者保持一致*/
    switch(baude)
    {
        case 4800:
            cfsetispeed(&options,B4800);
            cfsetospeed(&options,B4800);
            break;
        case 9600:
            cfsetispeed(&options,B9600);
            cfsetospeed(&options,B9600);
            break;
        case 19200:
            cfsetispeed(&options,B19200);
            cfsetospeed(&options,B19200);
            break;
        case 38400:
            cfsetispeed(&options,B38400);
            cfsetospeed(&options,B38400);
            break;
        case 115200: 
            printf("baud set is 115200 \n");
			tcflush(fd, TCIOFLUSH);										/*	清除输入输出缓冲区				*/
			if(0 != cfsetispeed(&options,B115200)){				/*	设置输入波特率	成功返回0 否则-1	*/
				printf("ERROR:Set input Serial baudrate fail\n");
			}
			if(0 != cfsetospeed(&options,B115200)){				/*	设置输出波特率	成功返回0 否则-1	*/
				printf("ERROR:Set output Serial baudrate fail\n");
			}
            break;

        default:
            fprintf(stderr,"Unkown baude!\n");
            return -1;
    }

    /*设置控制模式*/
    //options.c_cflag |= CLOCAL;//保证程序不占用串口
    //options.c_cflag |= CREAD;//保证程序可以从串口中读取数据
    #if 0
    switch(c_flow)
    {
        case 0://不进行流控制
            options.c_cflag &= ~CRTSCTS;
            break;
        case 1://进行硬件流控制
            options.c_cflag |= CRTSCTS;
            break;
        case 2://进行软件流控制
            options.c_cflag |= IXON|IXOFF|IXANY;
            break;
        default:
            fprintf(stderr,"Unkown c_flow!\n");
            return -1;
    }
    #endif

    /*设置数据位*/
    switch(bits)
    {
        case 5:
            options.c_cflag &= ~CSIZE;//屏蔽其它标志位
            options.c_cflag |= CS5;
            break;
        case 6:
            options.c_cflag &= ~CSIZE;//屏蔽其它标志位
            options.c_cflag |= CS6;
            break;
        case 7:
            options.c_cflag &= ~CSIZE;//屏蔽其它标志位
            options.c_cflag |= CS7;
            break;
        case 8:
            options.c_cflag &= ~CSIZE;//屏蔽其它标志位
            printf("set dat bit is 8\n");
            options.c_cflag |= CS8;
            break;
        default:
            fprintf(stderr,"Unkown bits!\n");
            return -1;
    }

    /*设置校验位*/
    switch(parity)
    {
        /*无奇偶校验位*/
        case 'n':
        case 'N':
            options.c_cflag &= ~PARENB;//PARENB：产生奇偶位，执行奇偶校验
            options.c_cflag &= ~INPCK;//INPCK：使奇偶校验起作用
            break;
        /*设为空格,即停止位为2位*/
        case 's':
        case 'S':
            options.c_cflag &= ~PARENB;//PARENB：产生奇偶位，执行奇偶校验
            options.c_cflag &= ~CSTOPB;//CSTOPB：使用两位停止位
            break;
        /*设置奇校验*/
        case 'o':
        case 'O':
            options.c_cflag |= PARENB;//PARENB：产生奇偶位，执行奇偶校验
            options.c_cflag |= PARODD;//PARODD：若设置则为奇校验,否则为偶校验
            options.c_cflag |= INPCK;//INPCK：使奇偶校验起作用
            options.c_cflag |= ISTRIP;//ISTRIP：若设置则有效输入数字被剥离7个字节，否则保留全部8位
            break;
        /*设置偶校验*/
        case 'e':
        case 'E':
            options.c_cflag |= PARENB;//PARENB：产生奇偶位，执行奇偶校验
            options.c_cflag &= ~PARODD;//PARODD：若设置则为奇校验,否则为偶校验
            options.c_cflag |= INPCK;//INPCK：使奇偶校验起作用
            options.c_cflag |= ISTRIP;//ISTRIP：若设置则有效输入数字被剥离7个字节，否则保留全部8位
            break;
        default:
            fprintf(stderr,"Unkown parity!\n");
            return -1;
    }

    /*设置停止位*/
    switch(stop)
    {
        case 1:
            options.c_cflag &= ~CSTOPB;//CSTOPB：使用两位停止位
            break;
        case 2:
            options.c_cflag |= CSTOPB;//CSTOPB：使用两位停止位
            break;
        default:
            fprintf(stderr,"Unkown stop!\n");
            return -1;
    }

    /*设置输出模式为原始输出*/
    //options.c_oflag &= ~OPOST;//OPOST：若设置则按定义的输出处理，否则所有c_oflag失效

    /*设置本地模式为原始模式*/
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    /*
     *ICANON：允许规范模式进行输入处理
     *ECHO：允许输入字符的本地回显
     *ECHOE：在接收EPASE时执行Backspace,Space,Backspace组合
     *ISIG：允许信号
     */

    /*设置等待时间和最小接受字符*/
    options.c_cc[VTIME] = 0;//可以在select中设置
    options.c_cc[VMIN] = 1;//最少读取一个字符


    /*如果发生数据溢出，只接受数据，但是不进行读操作*/
    tcflush(fd,TCIOFLUSH);

    /*激活配置*/
    if(tcsetattr(fd,TCSANOW,&options) < 0)
    {
        perror("tcsetattr failed");
        return -1;
    }

    return 0;
}

int uart_read(int fd,char *r_buf,size_t len)
{
    ssize_t cnt = 0;
    fd_set rfds;
    struct timeval time;

    /*将文件描述符加入读描述符集合*/
    FD_ZERO(&rfds);
    FD_SET(fd,&rfds);

    /*设置超时为15s*/
    time.tv_sec = 15;
    time.tv_usec = 0;

    /*实现串口的多路I/O*/
    ret = select(fd+1,&rfds,NULL,NULL,&time);
    switch(ret)
    {
        case -1:
            fprintf(stderr,"select error!\n");
            return -1;
        case 0:
            fprintf(stderr,"time over!\n");
            return -1;
        default:
            cnt = safe_read(fd,r_buf,len);
            if(cnt == -1)
            {
                fprintf(stderr,"read error!\n");
                return -1;
            }
            return cnt;
    }
}

int uart_write(int fd,const char *w_buf,size_t len)
{
    ssize_t cnt = 0;

    cnt = safe_write(fd,w_buf,len);
    if(cnt == -1)
    {
        fprintf(stderr,"write error!\n");
        return -1;
    }

    return cnt;
}

int uart_close(int fd)
{
    assert(fd);
    close(fd);

    /*可以在这里做些清理工作*/

    return 0;
}

int main(int argc,char** argv)
{
    if(argc < 2){
        printf("please input the tty devices \n");
        return 0;
    }

    const char *w_buf = "123456789\r\n";
    size_t w_len = strlen(w_buf);

    char r_buf[1024];
    bzero(r_buf,1024);

    fd = uart_open(fd,argv[1]);/*串口号/dev/ttySn,USB口号/dev/ttyUSBn*/
    if(fd == -1)
    {
        fprintf(stderr,"uart_open error\n");
        exit(EXIT_FAILURE);
    }

     #if 0
    if(uart_set(fd,115200,0,8,'N',1) == -1)
    {
        fprintf(stderr,"uart set failed!\n");
        exit(EXIT_FAILURE);
   }
   #else
   		wzt_SetSerialParameter(fd, 115200, 8, 1, 'N');
		wzt_DisableTerminalReturn(fd);
    #endif

    printf("serial set ok\n");


       int n=0;
		char cmd[12];
		for(n=0;n<10;n++){cmd[n]='0'+n;}cmd[10] = '\r';cmd[11] = '\n';
 
		while(1){
			cmd[0]='0'+(n++)%8;
			ret = write(fd,cmd,12);
			sleep(2);
		} 
   while(1) { 
    ret = uart_write(fd,w_buf,w_len);
    //ret = write(fd,w_buf,w_len);
    if(ret == -1)
    {
        fprintf(stderr,"uart write failed!\n");
        exit(EXIT_FAILURE);
    }
    printf("writtng finished totali2222 %d\n", w_len);
    sleep(2);
}

    while(1)
    {
        ret = uart_read(fd,r_buf,1024);
        if(ret == -1)
        {
            fprintf(stderr,"uart read failed!\n");
            exit(EXIT_FAILURE);
        }
    }

    ret = uart_close(fd);
    if(ret == -1)
    {
        fprintf(stderr,"uart_close error\n");
        exit(EXIT_FAILURE);
    }

    exit(EXIT_SUCCESS);
}