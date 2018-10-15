#ifndef SERVER_H_
#define SERVER_H_

int USER_MOTION = 3;

#define MAX_LEN 100
#define WGS84_RADIUS 6378137.0
#define WGS84_ECCENTRICITY 0.0818191908426
#define V_MAX 4.17 //m/s ~ 15 km/h
#define R2D 57.2957795131


double xyz6[3][3];
double xyz7[3][3];
double llh6[3][3];
double llh7[3][3];
double neu6[3][3];
double neu7[3][3];
double wxyz7[3][3];
double lxyz7[3];
int frame;
Queue *q1 = NULL;

void error(char *msg);
void subVect(double *A1, double *A2, double *result);
void write2file(FILE *fd, double A[3]);
bool isSimilar(double *A, double *B);
double normVect(double *A);
void genImLocation(FILE *fd, double A[3], double B[3], int num);
void velVector(double *A, double *B, double *C);
void nextCoordinate(double *A, double *B, double *D, int step);
double detectDirection(double *A, double *B, double *C);
double calculateAngle(double *A, double *B, double *C);
void readUserMotion(double **xyz, FILE *fp);
void xyz2llh(double *xyz, double *llh);
void llh2xyz(const double *llh, double *xyz);
void tmat(double *llh, double t[3][3]);
double calDistance(double A[3], double B[3]);
double calculateSpeed(double *neu1, double *neu2);
void xyz2neu(double **xyz, double **llh, double **neu);
void rmReduantNeu(double **neu, double **neu_new);
void calWrongNeu(double xyz_wrong[3][3], double xyz[3][3], double llh[3][3]);
double **createBuff(int i, int j);
void ReplaceData(double A[3], double B[3]);
void InsertData(double A[3][3]);
void Pushllh6(double llh[3], int isXyzExist, double xyz[3]);
void Pushllh7(double tmp[3], int isXyzExist, double xyz[3]);





#endif
