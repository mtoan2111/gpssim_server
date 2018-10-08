#ifndef SERIAL_H_
#define SERIAL_H_

#define POLL_TIMEOUT 1000

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

QNote *newNote(llh data);
Queue *createQueue();
void enQueue(Queue *q, llh data);
QNote *deQueue(Queue *q);
extern Queue *q;

typedef struct serial_t
{
  int fd;
  int state;
  int running;
  pthread_t rx_thread;
}serial;

static int SerialStop(serial *tty);
int SerialRead(serial *tty, char *buff, int size);
static void UART_IRQ(char *buff);
static void *SerialThreadHandle(void *param);
int ConfigSerialPort(int fd, int speed, int parity);
void ByteRate(int fd, int min);
int SerialStart(serial *tty);
void HandShake(int fd);

#endif
