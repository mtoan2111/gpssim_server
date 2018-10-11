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

#include "serial.h"

Queue *q = NULL;

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
  sscanf(buff,"%lf,%lf,%lf,%lf,%lf,%lf",&tmp.lat6, &tmp.lon6, &tmp.alt6, &tmp.lat7, &tmp.lon7, &tmp.alt7);
  tmp.alt6 = 15;
  tmp.alt7 = 15;
  enQueue(q, tmp);
}


static void *SerialThreadHandle(void *param)
{
  int ret = 0;
  int err = -1;
  int count = 0;
  char rx_buff[1 << 6];
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
      //clear buffer
      memset(rx_buff, 0x00, blen);
      //Reading the number of bytes will be recv
      count = SerialRead(tty,rx_buff,2);
      if (count > 0)
      {
        //printf("\n%d\n",count);
        rx_buff[count] = '\0';
        int cRecv = atoi(rx_buff);
#ifdef DEBUG
        printf ("Size of buff   -->%d\n", cRecv);
#endif
        //clear buffer
        memset(rx_buff,0x00, blen);
        //handshake
        //HandShake(fd);
        ByteRate(fd,cRecv);
        ret = poll(&ufds,1,POLL_TIMEOUT);
        if (ret > 0)
        {
          count = SerialRead(tty,rx_buff,cRecv);
#ifdef DEBUG
          printf ("First buff     -->%d   : %s\n", count, rx_buff);
#endif
          int clen = strlen(rx_buff);
          if (count > 0)
          {
            if (clen < cRecv)
            {
              //transfer doesn't complete
              char tmp_buff[1 << 6] = {0,};
              ByteRate(fd,cRecv - clen);
              ret = poll(&ufds,1,POLL_TIMEOUT);
              if (ret > 0)
              {
                count = SerialRead(tty,tmp_buff,cRecv - clen);
                if (count > 0)
                {
                  tmp_buff[count] = '\0';
                  //concatenate string
                  strncat(rx_buff,tmp_buff,strlen(tmp_buff));
#ifdef   DEBUG
                  printf ("Next buff      -->%d   : %s\n", count, tmp_buff);
                  printf ("Complete buff  -->%d   : %s\n", count, rx_buff);
#endif
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
          }
          else if (count < 0)
          {
            printf("\nError! Connection fail");
            err = 1;
            break;

          }
          UART_IRQ(rx_buff);
        }
        else if (ret < 0)
        {
          printf("\nError! Polling error in serial thread");
          err = 1;
          break;
        } 
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

int ConfigSerialPort(int fd, int speed, int parity)
{
  struct termios tty;
  memset(&tty, 0, sizeof(tty));
  if (tcgetattr(fd, &tty) != 0)
  {  
    printf("Error! in getting attributes\n");
    printf("%s\n",strerror(errno));
    return -1;
  }
  else
  {
    printf("Get attr success\n");
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
    printf("Error! in setting attributes\n");
    return -1;
  }
  return 0;
}

void ByteRate (int fd, int min)
{
  struct termios tty;
  memset (&tty, 0, sizeof tty);
  if (tcgetattr (fd, &tty) != 0)
  {
    printf("Error %d from tggetattr", errno);
    return;
  }

  tty.c_cc[VMIN]  = min;
  tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

  tcflush(fd, TCIFLUSH);
  if (tcsetattr (fd, TCSANOW, &tty) != 0)
    printf("error %d setting term attributes", errno);
}

int SerialStart(serial *tty)
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

void HandShake(int fd)
{
  unsigned char ACK='U';
  unsigned char rd[1<<3];
  while(1)
  {
    memset(rd,0x00, sizeof(rd));
    int ret = write(fd,&ACK,1);
    ByteRate(fd,1);
    ret = read (fd, rd,1);
    if (rd[0] == ACK)
      return;
  }
}
