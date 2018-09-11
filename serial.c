#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <pthread.h>
#include <stdlib.h>
#include <signal.h>
#include <poll.h>

#define POLL_TIMEOUT 2000
//--> Implement queue
typedef struct LatLong
{
  double lat6;
  double lon6;
  double lat7;
  double lon7;
}llh;

typedef struct QNote
{
  llh data;
  struct QNote *next;
}QNote;

typedef struct Queue
{
  QNote *front, *rear;
}Queue;

QNote *newNote(llh data)
{
  QNote *tmp = (QNote*)malloc(sizeof(QNote));
  tmp->data = data;
  tmp->next = NULL;
  return tmp;
}

Queue *createQueue()
{
  Queue *q = (Queue*)malloc(sizeof(Queue));
  q->front=q->rear= NULL;
  return q;
}

void enQueue(Queue *q, llh data)
{
  QNote *tmp = newNote(data);
  if (q->rear == NULL)
  {
    q->front = q->rear = tmp;
    return;
  }
  q->rear->next = tmp;
  q->rear = tmp;
}

QNote *deQueue(Queue *q)
{
  if (q->front == NULL)
    return NULL;
  QNote *tmp = q->front;
  q->front = q->front->next;
  if (q->front == NULL)
    q->rear = NULL;
  return tmp;
}

Queue *q = NULL;
//<-- Complete implement Q

//

typedef struct serial_t
{
  int fd;
  int state;
  int running;
  pthread_t rx_thread;
}serial;

static int SerialStop(serial *tty)
{
  tty->running = 0;
  return 0;
}

int SerialClose(serial *tty)
{
  SerialStop(tty);
  return 0;
}

int SerialRead(serial *tty,char *buff, int size)
{
  return read(tty->fd,buff,size);
}

// Create a new thread to handle UART event
static void UART_IRQ(char *buff)
{
  llh tmp;
//  printf ("%d   ", strlen(buff));
//  printf ("\n%s\n",buff);
  sscanf(buff,"%lf,%lf,%lf,%lf",&tmp.lat6, &tmp.lon6, &tmp.lat7, &tmp.lon7);
  enQueue(q, tmp);
}


static void *SerialThreadHandle(void *param)
{
  int ret = 0;
  int err = -1;
  int count = 0;
  char rx_buff[64];
  int blen = sizeof(rx_buff);

  serial *tty = (serial*)param;
  int fd = tty->fd;
  struct pollfd ufds;

  ufds.fd = fd;
  ufds.events = POLLIN;

  while(tty->running != 0)
  {
    ret = poll(&ufds,1,POLL_TIMEOUT);
    if (ret > 0)
    {
      count = SerialRead(tty,rx_buff,38);
      if (count > 0)
      {
        //printf("\n%d\n",count);
        rx_buff[count] = '\0';
        UART_IRQ(rx_buff);
      }
      else if (count < 0)
      {
        printf("\nError! Connection fail");
        err = 1;
        break;
      }
    }
    else if (ret < 0)
    {
      printf("\nError! Polling error in serial thread");
      err = 1;
      break;
    } 
  }
  if (err == 1)
  {
    SerialClose(tty);
  }
  ret = close(tty->fd);
  return NULL;
}

int configSerialPort(int fd, int speed, int parity)
{
  struct termios tty;
  memset(&tty, 0, sizeof(tty));
  if (tcgetattr(fd, &tty) != 0)
  {  
    printf("\n Error! in getting attributes\n");
    printf("%s\n",strerror(errno));
    return -1;
  }
  else
  {
    printf("\n Get attr success\n");
  }
  cfsetospeed(&tty,speed);
  cfsetispeed(&tty,speed);
  tty.c_iflag &= ~IGNBRK;
  tty.c_lflag = 0;
  tty.c_oflag = 0;
  tty.c_cc[VMIN] = 38;
  tty.c_cc[VTIME] = 5;
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
  tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,enable reading
  tty.c_cflag &= ~(PARENB | PARODD);      // shut off pariy
  //No Parity, use mode 8N1
  tty.c_cflag |= parity;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;

  tty.c_cflag &= ~CRTSCTS;
  tcflush(fd, TCIFLUSH);
  if (tcsetattr(fd, TCSANOW, &tty) != 0)
  {
    printf("\nError! in setting attributes\n");
    return -1;
  }
  return 0;
}

void set_blocking (int fd, int should_block)
{
  struct termios tty;
  memset (&tty, 0, sizeof tty);
  if (tcgetattr (fd, &tty) != 0)
  {
    printf("error %d from tggetattr", errno);
    return;
  }

  tty.c_cc[VMIN]  = should_block ? 1 : 0;
  tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

  tcflush(fd, TCIFLUSH);
  if (tcsetattr (fd, TCSANOW, &tty) != 0)
    printf("error %d setting term attributes", errno);
}

static int SerialStart(serial *tty)
{
  if (tty->running != 1)
  {
    tty->running = 1;
    int res = pthread_create(&tty->rx_thread, NULL, SerialThreadHandle, (void*)tty);
    if (res != 0)
    {
      return -2;
    }
    return 0;
  }
  else 
  {
    return -1;
  }
}

int main(void)
{
  int fd;
  char *tty="/dev/ttyACM0";
  fd = open(tty,O_RDWR| O_NOCTTY);
  if (fd == 1)
  {
    printf("\n Error! in opening Arduino Serial Port\n");
  }
  else
  {
    printf("\n Arduino Serial port is opening\n");
  }
  if(configSerialPort(fd,B115200,0) != 0)
    return 0;
//  set_blocking(fd, 1);
//  char *buff = "Nguyen Manh Toan";
//  //write (fd, buff, strlen(buff));
//  while(1)
//  {
//    char rd_buff[1024];
//    memset(rd_buff,0x00,sizeof(rd_buff));
//    int ret = read (fd, rd_buff, 38);
//    printf ("%d\n",ret);
//    printf ("%s\n",rd_buff);
//  }
//  close(fd);
  q = createQueue();
  pthread_t tThread; 
  serial my_tty = {
    .fd = fd,
    .state = -1,
    .running = -1,
    .rx_thread = tThread

  };
  /*
TODO: Need a handshake protocol before start reading thread
  */
  int res = SerialStart(&my_tty);
  while(1)
  {
    QNote *tmp = NULL;
    if ((tmp = deQueue(q)) != NULL)
    {
      printf("%f,%f,%f,%f\n",tmp->data.lat6,tmp->data.lon6,tmp->data.lat7,tmp->data.lon7);
    }
  }
  close(fd);
  return 0;
}
