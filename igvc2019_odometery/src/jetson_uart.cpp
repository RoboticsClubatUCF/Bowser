#include <stdio.h>
#include <string.h>
#include <cstdlib>
#include <termios.h>
#include <unistd.h>
#include <sys/signal.h>
#include <unistd.h>
#include <fcntl.h>

static struct termios oldt, newt;
int serial_descriptor;
void clean_and_exit(int code);
extern "C" void quit_signal_handler(int signum);
extern "C" void uart_signal_handler(int signum);

unsigned char translateChar(char a)
{
    if(a % 2 == 1)
    {
        //a = ((unsigned char)a << 1);
        a = ~a;
    }
    else
    {
        a = ((unsigned char)a << 1 | (unsigned char)a >> 7);
        a = ~a;
    }
    return (a & 127);
}

int main (int argc, char **argv)
{
    // help variables
    int c;

    // tty settings
    tcgetattr( STDIN_FILENO, &oldt);
    newt = oldt;

    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr( STDIN_FILENO, TCSANOW, &newt);

    // Open serial port
    serial_descriptor = open("/dev/ttyUSB0", O_RDWR | O_NDELAY | O_NONBLOCK);
    if (serial_descriptor == -1)
    {
        printf("Could not open serial port on /dev/ttyUSB0!\n");
        clean_and_exit(-1);
    }
    else fcntl(serial_descriptor, F_SETFL, 0);

    // Configure serial port
    struct sigaction saio;
    saio.sa_handler = uart_signal_handler;
    saio.sa_flags = 0;
    saio.sa_restorer = NULL;
    sigaction(SIGIO,&saio,NULL);

    fcntl(serial_descriptor, F_SETFL, FNDELAY);         // Make fd wait
    fcntl(serial_descriptor, F_SETOWN, getpid());       // Allow to receive SIGIO
    fcntl(serial_descriptor, F_SETFL,  O_ASYNC );       // Make fd asynchronous

    // UART settings
    struct termios termAttr;
    tcgetattr(serial_descriptor,&termAttr);
    cfsetispeed(&termAttr,B115200);                       // Input speed
    cfsetospeed(&termAttr,B115200);                       // Output speed
    termAttr.c_iflag = 0;                               // Turn off input processing
    termAttr.c_oflag = 0;                               // Turn of output processing
    termAttr.c_lflag = 0;                               // Turn off line procesinng
    termAttr.c_cflag = 0;                               // Turn off character processing
    termAttr.c_cflag |= (CS8|CREAD|CLOCAL);             // Read 8 bit
    termAttr.c_cc[VMIN] = 0;                            // No minimal chars
    termAttr.c_cc[VTIME] = 1;                           // Wait 0.1 s for input
    tcsetattr(serial_descriptor,TCSANOW,&termAttr);   // Save settings

    /* Flush anything already in the serial buffer */
     tcflush(serial_descriptor, TCIFLUSH);
     tcflush(STDIN_FILENO, TCIFLUSH);

    //Kill signal handler
    signal(SIGINT,quit_signal_handler);

    // loop
    while ((c=getchar()) != 'q')
    {
        if (c=='0')
        {
            printf("Writing command 0...\n\r");
            write(serial_descriptor, "0", sizeof(char));
        } else if (c=='1')
        {
            printf("Writing command 1...\n\r");
            write(serial_descriptor, "1", sizeof(char));
        } else if (c=='q')
        {
            printf("Writing command q...\n\r");
            write(serial_descriptor, "q", sizeof(char));
        } else if (c>0)
            printf("Press 1 or 0 or q.\n\r");
    }

    // cleanup
    printf("Will end now!\n");
    clean_and_exit(0);
}

void clean_and_exit(int code)
{
    // close serial
    close(serial_descriptor);

    // tty reset settings
    tcsetattr( STDIN_FILENO, TCSANOW, &oldt);

    // exit
    std::exit(code);
}

void quit_signal_handler(int signum)
{
    printf("Will end now!\n");
    clean_and_exit(0);
}

void uart_signal_handler(int signum)
{
    char buffer[1024];
    memset(buffer, '\0', 1024);
    if ((read(serial_descriptor, &buffer, sizeof(buffer)))>0)
    {
        if (buffer[0] == 'q')
        {
            printf("Will end now!\n");
            clean_and_exit(0);
        } else {
            printf("Normal = %u\n", buffer[0]);
            printf("Translated = %u\n", translateChar(buffer[0]));
        }
    }
}