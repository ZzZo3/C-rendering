#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

/*-----------------> NOTICE <-----------------*/
/*
  A MATRIX SHALL BE DEFINED AND REFERENCED WITH "M[column][row]". -> float M[3][3] = {{1,4,7},{2,5,8},{3,6,9}};
  A POINT 2 SHALL BE DEFINED AS A MATRIX OF 1 ROW AND 2 COLUMNS.  -> float P2[1][2] = {{x,y}};
  A POINT 3 SHALL BE DEFINED AS A MATRIX OF 1 ROW AND 3 COLUMNS.  -> float P3[1][3] = {{x,y,z}};
      |1 2 3|        |x|        |x|
  M = |4 5 6| ; P2 = |y| ; P3 = |y|
      |7 8 9|                   |z|
*/
/*-----------------> CONSTANTS <-----------------*/

#define yDim /*65*/  117
#define xDim /*211*/ 375
#define gradL 14
#define MAXINPUT 64
#define triL 9
#define camHeight 5

/*-----------------> UTIL <-----------------*/

typedef int Pt2[2];
typedef float Pt3[3];
typedef float Mtrx33[9];
typedef float ObjShade[2];
/* ObjShade Table
0|Mono        |Value
1|Distance    |Multiplier
2|AxisGradiant|Axis: 1:x -1:-x 2:y -2:-y 3:z -3:-z
*/

struct Tri3 {
  Pt3 a,b,c;
  ObjShade shade;
};

int pol(float v) {
  if (v>0) { return 1;};
  if (v<0) { return -1;};
  return 0;
};

int polBin(float v) {
  if (v<0) { return -1;};
  return 1;
};

void termLine() {
  for (int i=0; i<xDim+2; i++) {
    printf("-");
  };
  printf("\n");
};

float distA(Pt3 A) {
  return sqrt(A[0]*A[0]+A[1]*A[1]+A[2]*A[2]);
};

float distAB(Pt3 A, Pt3 B) {
  return sqrt((A[0]-B[0])*(A[0]-B[0])+(A[1]-B[1])*(A[1]-B[1])+(A[2]-B[2])*(A[2]-B[2]));
};

void applyMtrxPt3(Pt3 M31, Mtrx33 M33) {
  Pt3 result;
  for (int colR=0; colR<3; colR++) {
    result[colR] = 0;
    for (int i=0; i<3; i++) { result[colR] += M31[i]*M33[3*i+colR];};
  };
  for (int i=0; i<3; i++) { M31[i] = result[i];};
};

float dot(Pt3 A, Pt3 B) {
  return (A[0]*B[0]+A[1]*B[1]+A[2]*B[2]);
};

float dotNorm(Pt3 A, Pt3 B) {
  return (A[0]*B[0]+A[1]*B[1]+A[2]*B[2]) / (distA(A)*distA(B));
}

void cross(Pt3 A, Pt3 B, Pt3 C) {
  C[0] = A[1]*B[2]-A[2]*B[1];
  C[1] = A[2]*B[0]-A[0]*B[2];
  C[2] = A[0]*B[1]-A[1]*B[0];
};

/*-----------------> PREDEFINED VARIABLES <-----------------*/

char* tmlScnSpcA; // screenspace
char* tmlScnSpcB; // UI layer
Pt3 CAM = {0,camHeight,0};
Pt3 POS = {0,0,0};
float pitch = 0; // Radians
float yaw = 0;
float radius = 100; // render distance
float pseuFOV = 4; // inital z-val for screenspace points | as "pseuFOV" -> 0, FOV -> 180
float yScale = 0.10; // [0..1] coef for dimensions of screenspace
float xScale = 0.065;

int scnDims[2] = {xDim,yDim};

char grad[gradL] = " .,:~>+=so$W%@";

Mtrx33 xySwpMtrx = {0,1,0,1,0,0,0,0,1};
Mtrx33 xzSwpMtrx = {0,0,1,0,1,0,1,0,0};
Mtrx33 pitchMtrx = {1,0,0,0,1,0,0,0,1};
Mtrx33 yawMtrx = {1,0,0,0,1,0,0,0,1};

struct Tri3 TRI[triL];
Pt3 TRINORM[triL];

/*-----------------> FILE HANDLING <-----------------*/

void saveData() {
  FILE* fPtr;
  fPtr = fopen("data.txt","w");
  if (fPtr=NULL) { printf("ERROR: \"./data.txt\" not created\n");return;};
  float b = 0.79832;
  fprintf(fPtr,"%f\n",b);
  printf("data saved.\n");
};

/*-----------------> MATRIX <-----------------*/

void drawPoint(char* ARRAY, Pt2 p, char fill) { ARRAY[p[1]*xDim+p[0]] = fill;};

void drawLine(char* ARRAY, Pt2 a, Pt2 b, char fill) {
  bool sortX = b[0] > a[0], sortY = b[1] > a[1];
  bool vert = a[0] == b[0], horz = a[1] == b[1];
  bool shallow = ( fabs(((float)b[1] - a[1])/(b[0] - a[0])) <= 1.0 || horz ) && !vert;
  /*if ( !((sortX & sortY) || (sortX & (shallow || horz)) || (sortY & (!shallow || vert))) ) { };*/
  if ( !(sortX & sortY) & !(sortX & (shallow||horz)) & !(sortY & (!shallow||vert)) ) { Pt2 temp={b[0],b[1]};b=a;a=temp;};
  if (shallow) {
    for (int xi=a[0]; xi<=b[0]; xi++) {
      float m = ((float)b[1]-a[1])/(b[0]-a[0]);
      float y = m * (xi - a[0]) + a[1];
      Pt2 pTemp = {xi,y+0.5};
      drawPoint(ARRAY,pTemp,fill);
    };
  } else {
    for (int yi=a[1]; yi<=b[1]; yi++) {
      float m = ((float)b[0]-a[0])/(b[1]-a[1]);
      float x = m * (yi-a[1]) + a[0];
      Pt2 pTemp = {x+0.5,yi};
      drawPoint(ARRAY,pTemp,fill);
    };
  };
};

void printTml() {
  termLine();
  for(int yi=0; yi<yDim; yi++) {
    printf("|");
    for (int xi=0; xi<xDim; xi++) {
      if (tmlScnSpcB[yi*xDim+xi]==' ') {
        printf("%c",tmlScnSpcA[yi*xDim+xi]);
      } else {
        printf("%c",tmlScnSpcB[yi*xDim+xi]);
      };
    };
    printf("|\n");
  }
  termLine();
};

void emptyTmlScnSpc() {
  for(int y=0; y<yDim; y++) { for(int x=0; x<xDim; x++) { tmlScnSpcA[y*xDim+x]=' '; tmlScnSpcB[y*xDim+x]=' ';};};
};

void writeTxtLine(Pt2 corner, char txt[], int length, int size) {
  for (int cursor=0; cursor<length; cursor++) {
    /*...*/
    printf("hello");
    Pt2 a = {corner[0]+(size*cursor),corner[1]};
    Pt2 b = {corner[0]+(size*(cursor+1))-1,corner[1]+size-1};
    drawLine(tmlScnSpcB,a,b,'X');
  };
};

void buildUI() {
  Pt2 pUIa = {0,0}, pUIb = {floor(xDim/2),0}, pUIc = {xDim-1,0};
  Pt2 pUId = {0,floor(yDim/2)}, pUIe = {xDim-1,floor(yDim/2)};
  Pt2 pUIf = {0,yDim-1}, pUIg = {floor(xDim/2),yDim-1}, pUIh = {xDim-1,yDim-1};
  drawPoint(tmlScnSpcB, pUIa,'\\');drawPoint(tmlScnSpcB, pUIb,'|');drawPoint(tmlScnSpcB, pUIc,'/');
  drawPoint(tmlScnSpcB, pUId,'-');drawPoint(tmlScnSpcB, pUIe,'-');
  drawPoint(tmlScnSpcB, pUIf,'/');drawPoint(tmlScnSpcB, pUIg,'|');drawPoint(tmlScnSpcB, pUIh,'\\');
  Pt2 _corner = {10,10};
  char _text[5] = "hello";
  writeTxtLine(_corner,_text,5,6);
};

/*-----------------> SCENE <-----------------*/

void castRay(Pt3 RAY, Pt2 px) {
  bool DP = false;
  float value = 0.0;
  Pt3 a,b,c,n,p,q,T; // TRI points, normal, temps pq, v: target of RAY on Plane
  Pt3 Tfnl = {radius,radius,radius}; // intersection of RAY and TRI plane
  bool foundTRI = false;
  ObjShade shade;
  for(int i=0; i<triL; i++) {
    for (int j=0; j<3; j++) {
      a[j] = TRI[i].a[j]-CAM[j];
      b[j] = TRI[i].b[j]-CAM[j];
      c[j] = TRI[i].c[j]-CAM[j];
      n[j] = TRINORM[i][j];
    };
    float t = dot(a,n)/dot(n,RAY);
    for (int j=0; j<3; j++) { T[j]=RAY[j]*t;};
    for (int j=0; j<3; j++) {
      a[j]-=T[j];
      b[j]-=T[j];
      c[j]-=T[j];
    };
    float inOutTheta = acos(dotNorm(a,b))+acos(dotNorm(a,c))+acos(dotNorm(b,c));
    if (inOutTheta>=2*M_PI&t>=0) {
      foundTRI=true;
      for (int j=0; j<2; j++) { shade[j]=TRI[i].shade[j];};
      if (distA(T)<distA(Tfnl)) { for(int j=0; j<3; j++) { Tfnl[j]=T[j];};};
    } else {
      // outside of triangle (no render)
    };
  };
  if (foundTRI) {
    value = (radius-distA(Tfnl))/radius;
    if (value<1.0/(gradL-1)) { value=1.0/(gradL-1);};
  } else {
     //value = fabs((RAY[0]+RAY[1])/(sqrt(3)*radius));
     //if (fmodf(fabs(RAY[0]),10)<1.33 || fmodf(fabs(RAY[1]),10)<1.33 || fmodf(fabs(RAY[2]),10)<1.33) { value+=0.15;value*=2;}; 
    shade[0]=0.0;shade[1]=0.0;
  };
//SHADING
 /*mono*/ if (shade[0]==0.0) { value = shade[1];} else
 /*dist*/ if (shade[0]==1.0) { value = (radius-distA(Tfnl))/radius;} else
 /*axis*/ if (shade[0]==2.0) {
    if (shade[1]==1.0) { value=Tfnl[0]/radius;} else
    if (shade[1]==-1.0) { value=(radius-Tfnl[0])/radius;} else
    if (shade[1]==2.0) { value=Tfnl[1]/radius;} else
    if (shade[1]==-2.0) { value=(radius-Tfnl[1])/radius;} else
    if (shade[1]==3.0) { value=Tfnl[2]/radius;} else
    if (shade[1]==-3.0) { value=(radius-Tfnl[2])/radius;} else { value=0.0;};
  } else { value=0.0;};
  if (value>1.0) { value=1.0;} else if (value<0.0) { value=0.0;};
  int normValue = round(value*(gradL-1));
  tmlScnSpcA[px[1]*xDim+px[0]] = grad[normValue];
};

void  castNet(){
  for(int yi=0; yi<yDim; yi++) {
    for(int xi=0; xi<xDim; xi++) {
//0. Generate Pt3 for pixel
      Pt3 p3Dummy = {xScale*(xi-floor(xDim/2)), yScale*(yi-floor(yDim/2)), pseuFOV};
//1. Apply Screenspace Transform
      for (int i=0; i<2; i++) { if (p3Dummy[i]!=0) {
        p3Dummy[i] += (2*pow(p3Dummy[i],3))/(fabs(p3Dummy[i])*scnDims[i]); p3Dummy[i]/=3;
      };};
      p3Dummy[1] *= -1;
//2. Get Slopes (X-Swaps?)
      float tempF, dydx, dzdx;
      int slopeCase = 0;
      if (p3Dummy[0]==0) {
        if (p3Dummy[1]!=0) { slopeCase=1; applyMtrxPt3(p3Dummy,xySwpMtrx);} else
        if (p3Dummy[2]!=0) { slopeCase=2; applyMtrxPt3(p3Dummy,xzSwpMtrx);}
        else { printf("ERROR: castNet() failed due to pxTarget at {0,0,0}\n");return;};
      };
      dydx = p3Dummy[1]/p3Dummy[0];
      dzdx = p3Dummy[2]/p3Dummy[0];
//3. Extrude Ray to Sphere
      float xOptA = sqrt(radius*radius/(dydx*dydx+dzdx*dzdx+1));
      float xOptB = -xOptA;
      if (pol(p3Dummy[0])==pol(xOptA)) { p3Dummy[0]=xOptA;} else { p3Dummy[0]=xOptB;};
      p3Dummy[1] = dydx*(p3Dummy[0]);
      p3Dummy[2] = dzdx*(p3Dummy[0]);
//4. Undo X-Swaps
      if (slopeCase==1) { applyMtrxPt3(p3Dummy,xySwpMtrx);} else
      if (slopeCase==2) { applyMtrxPt3(p3Dummy,xzSwpMtrx);};
//5. MATRIX ROTATION: 
      applyMtrxPt3(p3Dummy,pitchMtrx);
      applyMtrxPt3(p3Dummy,yawMtrx);
//6. Cast Ray
      Pt2 px = {xi,yi};
      castRay(p3Dummy, px);
    };
  };
};

/*-----------------> CONTENT <-----------------*/

void buildScene() {
  struct Tri3 FloorA = {{70,0,70},{70,0,-70},{-70,0,-70},{1,0.15}};
  struct Tri3 FloorB = {{70,0,70},{-70,0,70},{-70,0,-70},{1,0.15}};
  struct Tri3 HouseA = {{-20,0,25},{-10,10,25},{-10,0,25},{0,1}};
  struct Tri3 HouseB = {{-20,0,25},{-10,10,25},{-20,10,25},{0,0.5}};
  struct Tri3 HouseC = {{-10,0,25},{-10,10,70},{-10,0,70},{0,1}};
  struct Tri3 HouseD = {{-10,0,25},{-10,10,70},{-10,10,25},{0,0.5}};
  struct Tri3 RoofA = {{-20,10,25},{-10,10,25},{-15,15,25},{0,1}};
  struct Tri3 RoofB = {{-15,15,25},{-7,7,70},{-7,7,25},{0,1}};
  struct Tri3 RoofC = {{-15,15,25},{-7,7,70},{-15,15,70},{0,0.5}};
  TRI[0] = FloorA;
  TRI[1] = FloorB;
  TRI[2] = HouseA;
  TRI[3] = HouseB;
  TRI[4] = HouseC;
  TRI[5] = HouseD;
  TRI[6] = RoofA;
  TRI[7] = RoofB;
  TRI[8] = RoofC;
};

/*-----------------> PROG <-----------------*/

void startup() {
  tmlScnSpcA=(char*)malloc(xDim*yDim);
  tmlScnSpcB=(char*)malloc(xDim*yDim);
  buildScene();
  Pt3 a,b,c,p,q,n;
  for (int i=0; i<triL; i++) {
    for (int j=0; j<3; j++) {
      a[j] = TRI[i].a[j]-CAM[j];
      b[j] = TRI[i].b[j]-CAM[j];
      c[j] = TRI[i].c[j]-CAM[j];
    };
    for (int j=0; j<3; j++) {
      p[j] = b[j]-a[j];
      q[j] = c[j]-a[j];
    };
    cross(p,q,n);
    for (int j=0; j<3; j++) { TRINORM[i][j]=n[j];};
    TRINORM[i][3] = dot(n,a);
  };
};

void render() {
  emptyTmlScnSpc();
  pitchMtrx[4]=cos(pitch);pitchMtrx[5]=-sin(pitch);pitchMtrx[7]=sin(pitch);pitchMtrx[8]=cos(pitch);
  yawMtrx[0]=cos(yaw);yawMtrx[2]=sin(yaw);yawMtrx[6]=-sin(yaw);yawMtrx[8]=cos(yaw);
  castNet();
  buildUI();
  printTml();
};

void pitchUp(float by) { pitch+=by;render();};
void yawLeft(float by) { yaw+=by;render();};
void walkForward(float by) {
  Pt3 unitZ = {0,0,by};
  applyMtrxPt3(unitZ,yawMtrx);
  POS[0]+=unitZ[0];POS[2]+=unitZ[2];
  CAM[0]+=unitZ[0];CAM[2]+=unitZ[2];
  render();
};
void walkRight(float by) {
  Pt3 unitZ = {by,0,0};
  applyMtrxPt3(unitZ,yawMtrx);
  POS[0]+=unitZ[0];POS[2]+=unitZ[2];
  CAM[0]+=unitZ[0];CAM[2]+=unitZ[2];
  render();
};

int parse(char cmd[]) {
  if (cmd[0]=='x') { return 1;} else
  if (cmd[0]=='w') { return 2;} else
  if (cmd[0]=='s') { return 3;} else
  if (cmd[0]=='a') { return 4;} else
  if (cmd[0]=='d') { return 5;} else
  if (cmd[0]=='i') { return 6;} else
  if (cmd[0]=='k') { return 7;} else
  if (cmd[0]=='j') { return 8;} else
  if (cmd[0]=='l') { return 9;} else
  if (cmd[0]=='S') { return 10;};
  return 0;
};

void run(int code) {
  if (code==2) {pitchUp(-3.1415/32);} else
  if (code==3) {pitchUp(3.1415/32);} else
  if (code==4) {yawLeft(3.1415/32);} else
  if (code==5) {yawLeft(-3.1415/32);} else
  if (code==6) {walkForward(2.5);} else
  if (code==7) {walkForward(-2.5);} else
  if (code==8) {walkRight(-2.5);} else
  if (code==9) {walkRight(2.5);} else
  if (code=10) {saveData();};
  return;
};

/*-----------------> MAIN <-----------------*/

int main() {
  startup();
  render();
  bool running = true;
  while (running) {
    char input[MAXINPUT];
    printf("> ");
    scanf(" %[^\n]",input);
    int commandSeparateIndexes[MAXINPUT];
    int commandCount=0;
    for (int i=0; i<strlen(input); i++){ if (input[i]==' ') {
      commandSeparateIndexes[commandCount]=i;
      commandCount++;
    } else { commandSeparateIndexes[i]=0;};};
    int parseCursorA=0, parseCursorB=0;
    int runs=1;
// parse commands in line
    for (int i=0; i<commandCount; i++) {
      while (parseCursorB<=commandSeparateIndexes[i]) { parseCursorB++;};
      int length = parseCursorB - parseCursorA - 1;
      char command[MAXINPUT];
      strncpy(command, input+parseCursorA, length);
      command[length] = '\0';
      char *endPtr;
      long int num;
      num = strtol(command, &endPtr, 10);
      if (endPtr!=command || *endPtr=='\0') { runs=num;};
      int code = parse(command);
      if (code==1) { running=false;}
      else if (code!=0) { 
        for (int runi=0; runi<runs; runi++) { run(code);};
        runs = 1;
      };
      parseCursorA = parseCursorB;
    };
  };
  return 0;
};
