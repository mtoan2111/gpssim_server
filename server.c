#ifdef _WIN32_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <conio.h>
#include <WinSock2.h>

#else

#include <fcntl.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <sys/stat.h>

#endif

#include "serial.h"
#include <termios.h>

int USER_MOTION = 3;

#define MAX_LEN 100
#define WGS84_RADIUS 6378137.0
#define WGS84_ECCENTRICITY 0.0818191908426
#define V_MAX 27.7
#define R2D 57.2957795131


double xyz6[3][3];
double xyz7[3][3];
double llh6[3][3];
double llh7[3][3];
double neu6[3][3];
double neu7[3][3];


void error(char *msg)
{
  perror(msg);
  exit(1);
}


void subVect(double *A1, double *A2, double *result)
{
  result[0] = A1[0] - A2[0];
  result[1] = A1[1] - A2[1];
  result[2] = A1[2] - A2[2];
}

bool isSimillar(double *A, double *B)
{
  if (A[0] == B[0])
    return 0;
  if (A[1] == B[1])
    return 0;
  if (A[2] == B[2])
    return 0;
  return 1;
}

double normVect(double *A)
{
  return sqrt(pow(A[0], 2) + pow(A[1], 2) + pow(A[2], 2));
}

double detectDirection(double *A, double *B, double *C)
{
  if (!(isSimillar(A, B) && isSimillar(B, C)))
    return 0;
  double AB[3];
  subVect(B, A, AB);
  double cABC, cosABC, norm;
  cABC = AB[0] * C[0] + AB[1] * C[1] + AB[2] * C[2];
  norm = (normVect(AB)*normVect(C));
  cosABC = cABC / norm;
  if (cosABC > 1)
  {
    cosABC = 1;
  }
  if (cosABC < -1)
  {
    cosABC = -1;
  }
#ifdef DEBUG
{
	printf("%lf \n", acos(cosABC) * R2D);
}
#endif
  return (acos(cosABC)*R2D);
}

double calculateAngle(double *A, double *B, double *C)
{
  if (!(isSimillar(A, B) && isSimillar(B, C)))
    return 0;
  double AB[3], BC[3];
  subVect(B, A, AB);
  subVect(B, C, BC);
  double cABC, cosABC, norm;
  cABC = AB[0] * BC[0] + AB[1] * BC[1] + AB[2] * BC[2];
  norm = (normVect(AB)*normVect(BC));
  cosABC = cABC / norm;
  if (cosABC > 1)
  {
    cosABC = 1;
  }
  if (cosABC < -1)
  {
    cosABC = -1;
  }
  return acos(cosABC)*R2D;
}

void readUserMotion(double **xyz, FILE *fp)
{
  int num;
  char str[MAX_LEN];
  double t, x, y, z;
  for (num = 0; num < USER_MOTION; num++)
  {
    if (fgets(str, MAX_LEN, fp) == NULL)
      break;

    if (EOF == sscanf(str, "%lf,%lf,%lf,%lf", &t, &x, &y, &z))
      break;
    xyz[num][0] = x;
    xyz[num][1] = y;
    xyz[num][2] = z;
  }
  USER_MOTION = num;
  return;
}

double normVector(double *xyz)
{
  return sqrt(pow(xyz[0], 2) + pow(xyz[1], 2) + pow(xyz[2], 2));
}

void xyz2llh(double **xyz, double **llh)
{
  int num;
  double a, eps, e, e2;
  double x, y, z;
  double rho2, dz, zdz, nh, slat, n, dz_new;
    
  a = WGS84_RADIUS;
  e = WGS84_ECCENTRICITY;

  eps = 1.0e-3;
  e2 = pow(e, 2);
  for (num = 0; num < USER_MOTION; num++)
  {
    if (normVector(xyz[num]) < eps)
    {
      llh[num][0] = 0.0;
      llh[num][1] = 0.0;
      llh[num][2] = -a;
      continue;
    }
    x = xyz[num][0];
    y = xyz[num][1];
    z = xyz[num][2];

    rho2 = pow(x, 2) + pow(y, 2);
    dz = e2 * z;
    while (true)
    {
      zdz = z + dz;
      nh = sqrt(rho2 + pow(zdz, 2));
      slat = zdz / nh;
      n = a / sqrt(1.0 - e2*pow(slat, 2));
      dz_new = n*e2*slat;

      if (fabs(dz - dz_new) < eps)
      {
        break;
      }
      dz = dz_new;
    }
    llh[num][0] = atan2(zdz, sqrt(rho2))*R2D;
    llh[num][1] = atan2(y, x)*R2D;
    llh[num][2] = (nh - n)*R2D;
  }
  return;
}

void llh2xyz(const double *llh, double *xyz)
{
  double n;
  double a;
  double e;
  double e2;
  double clat;
  double slat;
  double clon;
  double slon;
  double d,nph;
  double tmp;

  a = WGS84_RADIUS;
  e = WGS84_ECCENTRICITY;
  e2 = e*e;

  clat = cos(llh[0]);
  slat = sin(llh[0]);
  clon = cos(llh[1]);
  slon = sin(llh[1]);
  d = e*slat;

  n = a/sqrt(1.0-d*d);
  nph = n + llh[2];

  tmp = nph*clat;
  xyz[0] = tmp*clon;
  xyz[1] = tmp*slon;
  xyz[2] = ((1.0-e2)*n + llh[2])*slat;

  return;
}

void tmat(double *llh, double t[3][3])
{
  double slat, clat;
  double slon, clon;

  slat = sin(llh[0]);
  clat = cos(llh[0]);
  slon = sin(llh[1]);
  clon = cos(llh[1]);

  t[0][0] = -slat * clon;
  t[0][1] = -slat * slon;
  t[0][2] = clat;
  t[1][0] = -slon;
  t[1][1] = clon;
  t[1][2] = 0.0;
  t[2][0] = clat * clon;
  t[2][1] = clat * slon;
  t[2][2] = slat;
}

double calculateSpeed(double *neu1, double *neu2)
{
  double distance, t;
  t = 0.1;

  distance = sqrt(pow((neu2[0] - neu1[0]), 2) + pow((neu2[1] - neu1[1]), 2));
  return distance / t;
}

void xyz2neu(double **xyz, double **llh, double **neu)
{
  int num;
  double t[3][3];
  for (num = 0; num < USER_MOTION; num++)
  {
    tmat(llh[num], t);
    neu[num][0] = t[0][0] * xyz[num][0] + t[0][1] * xyz[num][1] + t[0][2] * xyz[num][2];
    neu[num][1] = t[1][0] * xyz[num][0] + t[1][1] * xyz[num][1] + t[1][2] * xyz[num][2];
    neu[num][2] = t[2][0] * xyz[num][0] + t[2][1] * xyz[num][1] + t[2][2] * xyz[num][2];
  }
}

void rmReduantNeu(double **neu, double **neu_new)
{
  int num;
  for (num = 0; num < USER_MOTION / 2; num++)
  {
    neu_new[num][0] = neu[num][0];
    neu_new[num][1] = neu[num][1];
    neu_new[num][2] = neu[num][2];
  }
}

void calWrongNeu( double **xyz_wrong, double **xyz, double **llh)
{
  int num;
  double velocity, tm;
  double t[3][3];
  double neu[3];
  velocity = V_MAX;
  tm = 0.1;
  tmat(llh[0], t);
  for (num = 0; num < USER_MOTION; num++)
  {
    neu[0] = 0;
    neu[1] = 0;
    neu[2] = 0;
    if (num > 2)
    {
      double angle = calculateAngle(xyz[num - 2], xyz[num - 1], xyz[num]);
      if (angle < 170)
      {
        //drones redirected
        double subN = xyz[num][0] - xyz[num - 1][0];
        double subE = xyz[num][1] - xyz[num - 1][1];
        /*	N
        IV  |  I
        --------- E
        III |  II
        */
        // I
        if (subN > 0 && subE > 0)
        {
          neu[0] = (velocity * tm)*cos((180 - angle)*R2D) *2;
          neu[1] = (velocity * tm)*sin((180 - angle)*R2D) *2;
        }
        // II
        if (subN < 0 && subE > 0)
        {
          neu[0] = -(velocity * tm)*cos((180 - angle)*R2D) *2;
          neu[1] = (velocity * tm)*sin((180 - angle)*R2D) *2;
        }
        // III
        if (subN < 0 && subE < 0)
        {
          neu[0] = -(velocity * tm)*cos((180 - angle)*R2D) *2;
          neu[1] = -(velocity * tm)*sin((180 - angle)*R2D) *2;
        }
        // IV
        if (subN > 0 && subE < 0)
        {
          neu[0] = (velocity * tm)*cos((180 - angle)*R2D) *2;
          neu[1] = -(velocity * tm)*sin((180 - angle)*R2D) *2;
        }
      }
      else
      {
        double xVect[3];
        xVect[0] = 0.0;
        xVect[1] = 1.0;
        xVect[2] = 0.0;
        double angle = detectDirection(xyz[num - 2], xyz[num - 1], xVect);
        if (angle < 45 || angle > 135)
        {
          neu[1] = -(velocity * tm);
        }
        else
        {
          neu[0] = -(velocity * tm);
        }
      }
    }
    xyz_wrong[num][0] = xyz[num][0] + t[0][0] * neu[0] + t[0][1] * neu[1] + t[0][2] * neu[2];
    xyz_wrong[num][1] = xyz[num][1] + t[1][0] * neu[0] + t[1][1] * neu[1] + t[1][2] * neu[2];
    xyz_wrong[num][2] = xyz[num][2] + t[2][0] * neu[0] + t[2][1] * neu[1] + t[2][2] * neu[2];
  }
}

double **createBuff(int i, int j)
{
  double **array = (double **)malloc(i * sizeof(double **));
  for (int m = 0; m < i; m++)
  {
    array[m] = (double *)malloc(j * sizeof(double));
  }
  return array;
}

void ReplaceData(double A[3], double B[3])
{
  A[0] = B[0];
  A[1] = B[1];
  A[2] = B[2];
}

void PushBuff(QNote *tmp)
{
  ReplaceData(llh6[0], llh6[1]);
  ReplaceData(llh6[1], llh6[2]);
  ReplaceData(llh7[0], llh7[1]);
  ReplaceData(llh7[1], llh7[2]);
  ReplaceData(xyz6[0], xyz6[1]);
  ReplaceData(xyz6[1], xyz6[2]);
  ReplaceData(xyz7[0], xyz7[1]);
  ReplaceData(xyz7[1], xyz7[2]);
  llh6[2][0] = tmp->data.lat6;
  llh6[2][1] = tmp->data.lon6;
  llh6[2][2] = tmp->data.alt6;
  llh7[2][0] = tmp->data.lat7;
  llh7[2][1] = tmp->data.lon7;
  llh7[2][2] = tmp->data.alt7;
  llh2xyz(llh6[2],xyz6[2]);
  llh2xyz(llh7[2],xyz7[2]);
}

int main(int argc, const char **argv)
{
#ifdef _WIN32_
  int ret;
  WSADATA data;
  ret = WSAStartup(MAKEWORD(2, 2), &data);
  if (ret == -1)
  {
    printf("Error 1!");
    return 0;
  }
  SOCKET s, c;
  sockaddr_in Saddr, Caddr;
  Saddr.sin_family = AF_INET;
  Saddr.sin_port = htons(8080);
  Saddr.sin_addr.s_addr = htonl(INADDR_ANY);
  //--> Create socket
  s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  //--> Binding socket
  ret = bind(s, (sockaddr*)&Saddr, sizeof(Saddr));
  if (ret == -1)
  {
    printf("Error 2!");
    return 0;
  }
  //--> Open queue
  listen(s, 10);
#endif
  int fd;
  if (argc < 2)
  {
    printf ("Error! Missing Serial Module File\n");
    return 0;
  }
  char tty[1<<4];
  strcpy(tty, argv[1]);
  fd = open (tty, O_RDWR | O_NOCTTY);
#ifdef DEBUG  
  printf ("%d",fd);
#endif
  if (fd < 0)
  {
    printf("Error! in opening Arduino Serial Port\n");
    return 0;
  }
  else
  {
    printf("Arduino Serial port is opening\n");
  }
  if(ConfigSerialPort(fd,B115200,0) != 0)
    return 0;
  q = createQueue();
  pthread_t tThread; 
  serial my_tty = {
    .fd = fd,
    .state = -1,
    .running = -1,
    .rx_thread = tThread
  };
/*
TODO: Need a handshake protocol
*/
  ByteRate(fd,1);
  printf ("Handshaking....\n");
  HandShake(fd); 
  printf ("Handshake succeed\n");
  //......
  ByteRate(fd,61);
  int res = SerialStart(&my_tty);
  if (res < 0)
    return 0;
  int count = 0;
#ifdef DEBUG
  while(1)
  {
  }
#else
  //Read 3 packages first
  while(count++ < 3)
  {
    QNote *tmp = NULL;
    if ((tmp = deQueue(q)) != NULL)
    {
      PushBuff(tmp);
    }
  }
  while(1)
  {
    QNote *tmp = NULL;
    if ((tmp = deQueue(q)) != NULL)
    {
     /*
      Capture data and calculating...
     */
      
     //Step1: Convert lat, lon, hgt to xyz and push to buffer
      PushBuff(tmp);
     //Step2: Convert xyz to neu
     // xyz2neu(
      printf ("%lf,%lf,%lf,%lf,%lf,%lf\n",xyz6[2][0], xyz6[2][1], xyz6[2][2], xyz7[2][0], xyz7[2][1], xyz7[2][2]);
    }
  }
#endif
/**
  FILE *in;
  FILE *out;
  FILE *out_llh;
  FILE *out_neu;
  FILE *out_llh_wrong;
  double **xyz;
  double **xyz_wrong;
  double **llh;
  double **neu;
  double **neu_new;
  double **llh_wrong;
  float tp;
  char *input_source = "/home/pi/simulator.csv";
  char *output_source = "/home/pi/spoof.csv";
  char *output_llh_source = "/home/pi/mout_llh.csv";
  char *output_neu_source = "/home/pi/out_neu.csv";
  char *output_llh_wrong = "/home/pi/spoof_llh.csv";
  in = fopen(input_source, "r");
  out = fopen(output_source, "wb");
  out_llh = fopen(output_llh_source, "wb");
  out_neu = fopen(output_neu_source, "wb");
  out_llh_wrong = fopen(output_llh_wrong, "wb");
  if (in == NULL)
  {
    printf("Can't open motion file. Exit!\n");
    return 0;
  }
  //-->Read all line of user motion file and store into xyz
  xyz = createBuff(USER_MOTION, 3);
  readUserMotion(xyz, in);
  //-->Convert all xyz coordinates to llh coordinates
  llh = createBuff(USER_MOTION, 3);
  xyz2llh(xyz, llh);
  tp = 0.0;
  for (int i = 0; i < USER_MOTION; i++)
  {
    char buff1[100];
    sprintf(buff1, "  %.1f, %lf, %lf, %lf\n", tp, llh[i][0], llh[i][1], llh[i][2]);
    fwrite(buff1, strlen(buff1), 1, out_llh);
    tp = tp + (0.1 * 10) / 10.0;
  }
  //-->Convert all xyz coordinates to neu coordinates
  neu = createBuff(USER_MOTION, 3);
  xyz2neu(xyz, llh, neu);
  tp = 0.0;
  for (int i = 0; i < USER_MOTION; i++)
  {
    char buff1[100];
    sprintf(buff1, "  %.1f, %lf, %lf, %lf\n", tp, neu[i][0], neu[i][1], neu[i][2]);
    fwrite(buff1, strlen(buff1), 1, out_neu);
    tp = tp + (0.1 * 10) / 10.0;
  }
  //-->Remove some reduant coordinates
  //neu_new = createBuff(USER_MOTION, 3);
  //rmReduantNeu(neu, neu_new);
  //-->Calculate wrong coordinates from neu coordinates
  xyz_wrong =createBuff(USER_MOTION, 3);
  calWrongNeu(xyz_wrong, xyz, llh);
  tp = 0.0;
  for (int i = 0; i < USER_MOTION; i++)
  {
    char buff[100];
    sprintf(buff, "  %.1f,%.3lf, %.3lf, %.3lf\n", tp, xyz_wrong[i][0], xyz_wrong[i][1], xyz_wrong[i][2]);
    fwrite(buff, strlen(buff), 1, out);
    tp = tp + (0.1*10)/10.0;    
  }
  llh_wrong = createBuff(USER_MOTION, 3);
  xyz2llh(xyz_wrong, llh_wrong);
  tp = 0.0;
  for (int i = 0; i < USER_MOTION; i++)
  {
    char buff1[100];
    sprintf(buff1, "  %.1f, %lf, %lf, %lf\n", tp, llh_wrong[i][0], llh_wrong[i][1], llh_wrong[i][2]);
    fwrite(buff1, strlen(buff1), 1, out_llh_wrong);
    tp = tp + (0.1 * 10) / 10.0;
  }
  //waiting client connect
#ifdef _WIN32_
  int len = sizeof(Caddr);
  printf("Waiting client connect\n");
#else
//  struct sockaddr_in Caddr;
//  int clen = sizeof(Caddr);
#endif
//  c = accept(s, (struct sockaddr *)&Caddr, &clen);
  //-->send first xyz to calculate tmat, neu
  printf("Success!\n");
  char fist_mess[100];
  char ok_mess[8];
  sprintf(fist_mess, "%.3lf, %.3lf, %.3lf\n", llh_wrong[0][0], llh_wrong[0][1], llh_wrong[0][2]);
//  ret = recv(c, ok_mess, sizeof(ok_mess), 0);
//  if (ret == -1)
//  {
//    printf("Error 3!\n");
//    return 0;
//  }
//  else
//  {
//    if (strcmp(ok_mess, "ok") == 0)
//    {
//      send(c, fist_mess, sizeof(fist_mess), 0);
//    }
//  }
  //-->send data
  FILE *fd;
  char *myfifo = "/tmp/test";
  //mkfifo (myfifo,0666);
  fd = fopen(myfifo, "w");
  for (int i = 0; i < USER_MOTION; i++)
  {
    char buff[100];
    char ok[8];
    sprintf(buff, "%.3lf, %.3lf, %.3lf\n", xyz_wrong[i][0], xyz_wrong[i][1], xyz_wrong[i][2]);
    //printf("%s",buff);
    fwrite(buff,strlen(buff),1,fd);
//    ret = recv(c, ok, sizeof(ok), 0);
//    if (ret == -1)
//    {
//      printf("Error 3!\n");
//      return 0;
//    }
//    else
//    {
//      if (strncmp(ok, "OK",2) == 0)
//      {
//        //printf("%s\n", buff);
//        send(c, buff, sizeof(buff), 0);
//      }
//    }
  }
  fclose(fd);
  //unlink(myfifo);
  //close(c);
  fclose(in);
  fclose(out);
  fclose(out_llh);
  fclose(out_neu);
  fclose(out_llh_wrong);
  //-->save coordinates
  // _getch();
/**/
  close(fd);
  return 0;
}
