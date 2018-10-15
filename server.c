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
#include <time.h>

#endif

#include "serial.h"
#include <termios.h>
#include "server.h"

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

void write2file(FILE *fd, double A[3])
{
  char buff[1 << 6] = {0,};
  sprintf (buff,"%lf,%lf,%lf\n",A[0],A[1],A[2]);
  fwrite(buff, strlen(buff), 1 , fd);
  return;
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

void genImLocation(FILE *fd, double A[3], double B[3], int num)
{
  double C[3] = {0.0,};
  C[0] = B[0] - A[0];
  C[1] = B[1] - A[1];
  C[2] = B[2] - A[2];
  printf ("A interpolcating --> %lf,%lf,%lf\n",A[0],A[1],A[2]);
  printf ("B interpolcating --> %lf,%lf,%lf\n",B[0],B[1],B[2]);
 double D[3] = {0.0,};
  for (int i = 1; i < num; i++)
  {
    D[0] = A[0] + C[0] * i / num;
    D[1] = A[1] + C[1] * i / num;
    D[2] = A[2] + C[2] * i / num;
    double llh7_t[3] = {0.0,};
    xyz2llh(D,llh7_t);
    Pushllh7(llh7_t, 1, D);
    printf ("llh interpolcating --> %lf,%lf,%lf\n",llh7_t[0],llh7_t[1],llh7_t[2]);
    frame++;
    fprintf(stderr,"\nProcessing frame (Interpolating Data Mode)   : %d", frame);
    write2file(fd,llh7_t);
  }
  return;
}

void velVector(double *A, double *B, double *C)
{
  C[0] = B[0] - A[0];
  C[1] = B[1] - A[1];
  C[2] = B[2] - A[2];
  double n = normVect(C);
  double sV = sqrt(V_MAX);
  C[0] *= sV/n;
  C[1] *= sV/n;
  C[2] *= sV/n;
}

void nextCoordinate(double *A, double *B, double *D, int step)
{
  double tmin = 0.1;
  double C[3];
  velVector(A, B, C);
  D[0] = B[0] + C[0] * tmin * step;
  D[1] = B[1] + C[1] * tmin * step;
  D[2] = B[2] + C[2] * tmin * step;
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
#ifdef DEBUG_SERVER
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

//double normVector(double *xyz)
//{
//  return sqrt(pow(xyz[0], 2) + pow(xyz[1], 2) + pow(xyz[2], 2));
//}

void xyz2llh(double *xyz, double *llh)
{
  double a, eps, e, e2;
  double x, y, z;
  double rho2, dz, zdz, nh, slat, n, dz_new;
    
  a = WGS84_RADIUS;
  e = WGS84_ECCENTRICITY;

  eps = 1.0e-3;
  e2 = pow(e, 2);
  if (normVect(xyz) < eps)
  {
    llh[0] = 0.0;
    llh[1] = 0.0;
    llh[2] = -a;
    return;
  }
  x = xyz[0];
  y = xyz[1];
  z = xyz[2];

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
  llh[0] = atan2(zdz, sqrt(rho2))*R2D;
  llh[1] = atan2(y, x)*R2D;
  llh[2] = (nh - n);

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

  clat = cos(llh[0]/R2D);
  slat = sin(llh[0]/R2D);
  clon = cos(llh[1]/R2D);
  slon = sin(llh[1]/R2D);
  d = e*slat;

  n = a/sqrt(1.0-d*d);
  nph = n + llh[2];

  tmp = nph*clat;
  xyz[0] = tmp*clon;
  xyz[1] = tmp*slon;
  xyz[2] = ((1.0-e2)*n + llh[2])*slat;
#ifdef DEBUG_SERVER
  printf ("llh input        ---> %lf,%lf,%lf\n",llh[0],llh[1],llh[2]);
  printf ("xyz ouput        ---> %lf,%lf,%lf\n",xyz[0],xyz[1],xyz[2]);
#endif

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

double calDistance(double A[3], double B[3])
{
  return sqrt(pow((B[0] - A[0]), 2) + pow((B[1] - A[1]), 2) + pow((B[2] - A[2]),2));
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

void calWrongNeu( double xyz_wrong[3][3], double xyz[3][3], double llh[3][3])
{
  int num;
  double velocity, tm;
  double t[3][3];
  double neu[3];
  velocity = 200;
  tm = 0.1;
  tmat(llh[0], t);
  for (num = 0; num < 3; num++)
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

void InsertData(double A[3][3])
{
  ReplaceData(A[0],A[1]);
  ReplaceData(A[1],A[2]);
}

void Pushllh6(double llh[3], int isXyzExist, double xyz[3])
{
  InsertData(llh6);
  InsertData(xyz6);
  llh6[2][0] = llh[0];
  llh6[2][1] = llh[1];
  llh6[2][2] = llh[2];
  if (isXyzExist < 0)
    llh2xyz(llh6[2],xyz6[2]);
  else
  {
    xyz6[2][0] = xyz[0];
    xyz6[2][1] = xyz[1];
    xyz6[2][2] = xyz[2];
  }
}

void Pushllh7(double tmp[3], int isXyzExist, double xyz[3])
{
  InsertData(llh7);
  InsertData(xyz7);
  llh7[2][0] = tmp[0];
  llh7[2][1] = tmp[1];
  llh7[2][2] = tmp[2];
  if (isXyzExist < 0)
    llh2xyz(llh7[2],xyz7[2]);
  else
  {
    xyz7[2][0] = xyz[0];
    xyz7[2][1] = xyz[1];
    xyz7[2][2] = xyz[2];
  }

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
#ifdef DEBUG_SERVER  
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
  q1 = createQueue();
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
//  HandShake(fd); 
  printf ("Handshake succeed\n");
  //......
  ByteRate(fd,61);
  int res = SerialStart(&my_tty);
  if (res < 0)
    return 0;
  int count = 0;
  //Read 3 packages first
  while(count < 3)
  {
    QNote *tmp = NULL;
    if ((tmp = deQueue(q)) != NULL)
    {
      double llh6_t[3] = {0.0,};
      double xyz6_t[3] = {0.0,};
      llh6_t[0] = tmp->data.lat6;
      llh6_t[1] = tmp->data.lon6;
      llh6_t[2] = tmp->data.alt6;
      Pushllh6(llh6_t, -1, xyz6_t);

      double llh7_t[3] = {0.0,};
      double xyz7_t[3] = {0.0,};
      llh7_t[0] = tmp->data.lat7;
      llh7_t[1] = tmp->data.lon7;
      llh7_t[2] = tmp->data.alt7;
      Pushllh7(llh7_t, -1, xyz7_t);
      count++;
    }
  }
  char *out = "/home/toannm/Desktop/GPSTEST/gpssim_server/out.csv";
  FILE *ou; 
  ou = fopen(out, "wb");
  clock_t start = 0;
  clock_t end = 0;
  double time_used;
  int step = 0;
  while(1)
  {
    QNote *tmp = NULL;
    if ((tmp = deQueue(q)) != NULL)
    {
      /*
       Capture data and calculating...
      */
      //Step1: Convert lat, lon, hgt to xyz and push to buffer
#ifdef DEBUG_SERVER
      printf("Start\n");
#endif
      start = clock();
      double llh6_t[3] = {0.0,};
      double xyz6_t[3] = {0.0,};
      llh6_t[0] = tmp->data.lat6;
      llh6_t[1] = tmp->data.lon6;
      llh6_t[2] = tmp->data.alt6;

      double llh7_t[3] = {0.0,};
      double xyz7_t[3] = {0.0,};
      llh7_t[0] = tmp->data.lat7;
      llh7_t[1] = tmp->data.lon7;
      llh7_t[2] = tmp->data.alt7;
      llh2xyz(llh7_t,xyz7_t);
           //Step2: Calculate
#ifdef DEBUG_SERVER
      printf ("calculatating wrong location\n");
#endif
      if (step > 5)
      {
        //Need to generate some immediate location
        double range;
        range = calDistance(xyz7_t,lxyz7);
        printf (" %lf -->", range);
        if (range < 1800)
        {
          int nPoint = range / (V_MAX * 0.1);
          printf (" %d\n", nPoint);
          //if the range is far enough --> ignore
          genImLocation(ou,lxyz7,llh7_t,nPoint);
          printf ("Done\n");
        }
      }
      Pushllh6(llh6_t, -1, xyz6_t);
      Pushllh7(llh7_t, -1, xyz7_t);
      double next[3] = {0.0,};
      double llh[3] = {0.0,};
      calWrongNeu(wxyz7,xyz7,llh7);
      step = 1;
#ifdef DEBUG_SERVER
      printf ("calculating next location\n");
#endif
      nextCoordinate(wxyz7[1],wxyz7[2],next,step);
      xyz2llh(next,llh);
#ifdef DEBUG_SERVER
      printf ("Raw xyz input:   --> %lf,%lf,%lf,%lf,%lf,%lf\n",xyz6[2][0], xyz6[2][1], xyz6[2][2], xyz7[2][0], xyz7[2][1], xyz7[2][2]);
      printf ("Raw llh input:   --> %lf,%lf,%lf,%lf,%lf,%lf\n",llh6[2][0], llh6[2][1], llh6[2][2], llh7[2][0], llh7[2][1], llh7[2][2]);
      printf ("first location:  --> %lf,%lf,%lf\n",xyz7[1][0],xyz7[1][1],xyz7[1][2]);
      printf ("second location: --> %lf,%lf,%lf\n",xyz7[2][0],xyz7[2][1],xyz7[2][2]);
      printf ("next location:   --> %lf,%lf,%lf\n",next[0],next[1],next[2]);
      printf ("next location    --> %lf,%lf,%lf\n",llh[0],llh[1],llh[2]);
#endif
      frame++;
      fprintf(stderr,"\rProcessing frame (Normal Data Mode)          : %d", frame);
      write2file(ou,llh);
    }
    if (start != 0)
    {
      end = clock();
      time_used = (double)(end - start)/(CLOCKS_PER_SEC/1000);
      
      if (time_used > 70)
      {
        /*
        Interpolate next location if data doesn't come on time.
        */
        step++;
        double next[3] = {0.0,};
        double llh[3] = {0.0,};
        nextCoordinate(wxyz7[1],wxyz7[2], next, step);
        start = clock();
        xyz2llh(next,llh);
        Pushllh7(llh, 1, next);
        lxyz7[0] = next[0];
        lxyz7[1] = next[1];
        lxyz7[2] = next[2];
        frame++;
        fprintf(stderr,"\rProcessing frame (Missing Data Mode)         : %d", frame);
        write2file(ou,llh);
      }
    }
  }
  close(fd);

  return 0;
}
