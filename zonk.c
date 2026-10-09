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

/*-----------------> UTIL <-----------------*/

struct scnSpcPt {
  int x,y;
};

typedef float Pt3[1][3];
typedef float Pt3Trns[3][3];

struct Point3 {
  float x,y,z;
};

struct Tri3 {
  Pt3 a,b,c;
};

char* tmlScnSpcA; // screenspace
char* tmlScnSpcB; // UI layer
struct Point3 CAM = {0,0,0};
float pitch = 0; // Radians
float yaw = 0;
float radius = 100; // render distance
float pseuFOV = 4; // inital z-val for screenspace points | as "pseuFOV" -> 0, FOV -> 180
float yScale = 0.10; // [0..1] coef for dimensions of screenspace
float xScale = 0.061;

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

char grad[gradL] = " .,:~>+=so$W%@";
//char grad1[8] = "⠂⠢⠪⡪⡺⣫⣻⣿";

//typedef int bool; bool false = 0, true = 1;

void multplyM33xM31(Pt3 M31, Pt3Trns M33) {
  float result[1][3];
  for (int colR=0; colR<3; colR++) {
    result [0][colR] = 0;
    for (int i=0; i<3; i++) { result [0][colR] += M31[0][i]*M33[i][colR];};
  };

  for (int i=0; i<3; i++) {
    M31[0][i] = result [0][i];
  };
};

/*-----------------> MATRIX <-----------------*/

void drawPoint(char* ARRAY, struct scnSpcPt *p, char fill) { ARRAY[p->y*xDim+p->x] = fill;};


void drawLine(char* ARRAY, struct scnSpcPt *aTemp, struct scnSpcPt *bTemp, char fill) {
  //printf(" drawing Line...\n");
  // conditions
  bool sortX = bTemp->x > aTemp->x, sortY = bTemp->y > aTemp->y;
  bool vert = aTemp->x == bTemp->x, horz = aTemp->y == bTemp->y;
  bool shallow = ( fabs(((float)bTemp->y - aTemp->y)/(bTemp->x - aTemp->x)) <= 1.0 || horz ) && !vert;
  // a<->b
  struct scnSpcPt a, b;
  if ( (sortX & sortY) || (sortX & (shallow || horz)) || (sortY & (!shallow || vert)) ) { a = *aTemp; b = *bTemp;} else { a = *bTemp; b = *aTemp;};
  // draw
  if (shallow) {
    for (int xi=a.x; xi<=b.x; xi++) {
      float m = ((float)b.y-a.y)/(b.x-a.x);
      float y = m * (xi - a.x) + a.y;
      struct scnSpcPt pTemp = {xi,y+0.5};
      drawPoint(ARRAY,&pTemp,fill);
    };
  } else {
    for (int yi=a.y; yi<=b.y; yi++) {
      float m = ((float)b.x-a.x)/(b.y-a.y);
      float x = m * (yi-a.y) + a.x;
      struct scnSpcPt pTemp = {x+0.5,yi};
      drawPoint(ARRAY,&pTemp,fill);
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

/*-----------------> SCENE <-----------------*/

struct Tri3 TRIANGLES[10];

float scnSpcTrns(float p, char axis) {
  if (p==0) { return 0;}
  if (axis=='x') {
    p += (2*pow(p,3))/(fabs(p)*xDim);
    p /= 3;
  } else if (axis=='y') {
    p += (2*pow(p,3))/(fabs(p)*xDim);
    p /= -3;
  };
  return p;
};

void castRay(struct Point3 *A, struct Point3 *B, struct scnSpcPt *px) {
  float value = 0.0;
  
  struct Point3 d; d.x = B->x - A->x; d.y = B->y - A->y; d.z = B->z - A->z;
  
  //check list of TRIANGLES; for each, set Plane, check for Ray direction(toward,away), check if point within or outside of triangle.
  
  float dAbs = sqrt(pow(d.x,2)+pow(d.y,2)+pow(d.z,2));
  value = fabs((d.x+d.y)/(sqrt(3)*radius));
  if (fmodf(fabs(B->x),10)<1.33 || fmodf(fabs(B->y),10)<1.33 || fmodf(fabs(B->z),10)<1.33) { value+=0.15;value*=2;};

  if (value>1.0) { value=1.0;} else if (value<0.0) { value=0.0;};
  int normValue = round(value*(gradL-2)+1);
  if (B->z==0) { printf("  B->z: %f  value: %f  normValue: %d\n",B->z,value,normValue);};
  tmlScnSpcA[px->y*xDim+px->x] = grad[normValue];
};

void  castNet(){
  for(int yi=0; yi<yDim; yi++) {
    for(int xi=0; xi<xDim; xi++) {
// 0. Define px for later
      struct scnSpcPt px = {xi,yi};
// 1. Normalize pixel as pxNorm
      struct Point3 pxTarget = {xScale*(xi-floor(xDim/2)),yScale*(yi-floor(yDim/2)),pseuFOV};
// 2. Apply Matrix Transform
      pxTarget.x = scnSpcTrns(pxTarget.x,'x');
      pxTarget.y = scnSpcTrns(pxTarget.y,'y');
// 5. Get slopes from Line {0,0,0} -> pxTarget
      float tempF, dydx, dzdx;
      Pt3 p3Dummy = {{pxTarget.x,pxTarget.y,pxTarget.z}};
      Pt3Trns xySwpMtrx = {{0,1,0},{1,0,0},{0,0,1}};
      Pt3Trns xzSwpMtrx = {{0,0,1},{0,1,0},{1,0,0}};
      int caseX = 0;
      if (pxTarget.x==0) {
        if (pxTarget.y!=0) { // swap x->y
          caseX = 1;
          multplyM33xM31(p3Dummy,xySwpMtrx);
        } else if (pxTarget.z!=0) { // swap x->z 
          caseX = 2;
          multplyM33xM31(p3Dummy,xzSwpMtrx);
        } else {
          printf("ERROR: castNet() failed due to pxTarget at {0,0,0}\n");
          return;
        };
      };
      pxTarget.x = p3Dummy[0][0];
      pxTarget.y = p3Dummy[0][1];
      pxTarget.z = p3Dummy[0][2];
//
      dydx = pxTarget.y/pxTarget.x;
      dzdx = pxTarget.z/pxTarget.x;
// 6. Extrude Ray to Sphere
      struct Point3 spherePoint;
      float xSphereARR[2];
      xSphereARR[0] = sqrt(radius*radius/(pow(dydx,2)+pow(dzdx,2)+1));
      xSphereARR[1] = 0-xSphereARR[0];
      if (pol(pxTarget.x)==pol(xSphereARR[0])) { spherePoint.x = xSphereARR[0];}
      else { spherePoint.x = xSphereARR[1];};
      spherePoint.y = dydx*(spherePoint.x);
      spherePoint.z = dzdx*(spherePoint.x);
// 7. Translate backwards {x,y,z}
      p3Dummy[0][0] = spherePoint.x;
      p3Dummy[0][1] = spherePoint.y;
      p3Dummy[0][2] = spherePoint.z;
      if (caseX==1) { // swap x->y
        multplyM33xM31(p3Dummy,xySwpMtrx);
      } else if (caseX==2) { // swap x->z
        multplyM33xM31(p3Dummy,xzSwpMtrx); 
      }
// 8. MATRIX ROTATION:
 // 0. Setup
      Pt3Trns pitchMtrx = {{1,0,0},{0,cos(pitch),-sin(pitch)},{0,sin(pitch),cos(pitch)}};
      Pt3Trns yawMtrx = {{cos(yaw),0,sin(yaw)},{0,1,0},{-sin(yaw),0,cos(yaw)}};
 // 1. Pitch, Yaw 
      multplyM33xM31(p3Dummy,pitchMtrx);
      multplyM33xM31(p3Dummy,yawMtrx);
 // 2. Return values to pxTarget
      spherePoint.x = p3Dummy[0][0]+CAM.x;
      spherePoint.y = p3Dummy[0][1]+CAM.y;
      spherePoint.z = p3Dummy[0][2]+CAM.z;
// 9. Cast Ray
      castRay(&CAM, &spherePoint, &px);
    };
  };
};

/*-----------------> CONTENT <-----------------*/

void buildScene() {
  /*p3 p3A = {{65,-30,-60}};
  p3 p3B = {{65,-30,60}};
  p3 p3C = {{80,60,0}};
  struct Tri3 TriangleA = {p3A,p3B,p3C};
  TRIANGLES[0] = TriangleA;*/
};

void buildUI() {
  struct scnSpcPt pUIa = {0,0}, pUIb = {floor(xDim/2),0}, pUIc = {xDim-1,0};
  struct scnSpcPt pUId = {0,floor(yDim/2)}, pUIe = {xDim-1,floor(yDim/2)};
  struct scnSpcPt pUIf = {0,yDim-1}, pUIg = {floor(xDim/2),yDim-1}, pUIh = {xDim-1,yDim-1};
  drawPoint(tmlScnSpcB, &pUIa,'\\');drawPoint(tmlScnSpcB, &pUIb,'|');drawPoint(tmlScnSpcB, &pUIc,'/');
  drawPoint(tmlScnSpcB, &pUId,'-');drawPoint(tmlScnSpcB, &pUIe,'-');
  drawPoint(tmlScnSpcB, &pUIf,'/');drawPoint(tmlScnSpcB, &pUIg,'|');drawPoint(tmlScnSpcB, &pUIh,'\\');
};

/*-----------------> PROG <-----------------*/

void startup() {
  emptyTmlScnSpc();
};

void render() {
  buildScene();
  castNet();
  buildUI();
  printTml();
};

void pitchUp(float by) { pitch+=by;startup();render();};
void yawLeft(float by) { yaw+=by;startup();render();};

int parse(char cmd[]) {
  if (cmd[0]=='x') { return 1;};
  if (cmd[0]=='w') { return 2;};
  if (cmd[0]=='s') { return 3;};
  if (cmd[0]=='a') { return 4;};
  if (cmd[0]=='d') { return 5;};
  return 0;
};

void run(int code) {
  if (code==2) {pitchUp(-3.1415/32);}
  else if (code==3) {pitchUp(3.1415/32);}
  else if (code==4) {yawLeft(3.1415/32);}
  else if (code==5) {yawLeft(-3.1415/32);};
  return;
};

/*-----------------> MAIN <-----------------*/

int main() {
  tmlScnSpcA=(char*)malloc(xDim*yDim);
  tmlScnSpcB=(char*)malloc(xDim*yDim);
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
