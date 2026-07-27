#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/*-----------------> UTIL <-----------------*/

int pol(float v) {
  if (v>0) { return 1;}
  if (v<0) { return -1;}
  return 0;
}

void termLine() {
  for (int i=0; i<210; i++) {
    printf("-");
  }
  printf("\n");
}

/*-----------------> DRAW <-----------------*/

int yDim = 64, xDim = 210;
char MATRIX[64][210];
//         [ Y][  X]

struct Point {
  int x;
  int y;
};

void drawPoint(struct Point *p, char fill) { MATRIX[p->y][p->x] = fill;}

void drawLine(struct Point *aTemp, struct Point *bTemp, char fill) {
  printf(" drawing Line...\n");
  // conditions
  bool sortX = bTemp->x > aTemp->x, sortY = bTemp->y > aTemp->y;
  bool vert = aTemp->x == bTemp->x, horz = aTemp->y == bTemp->y;
  bool shallow = ( abs(((float)bTemp->y - aTemp->y)/(bTemp->x - aTemp->x)) <= 1.0 || horz ) && !vert;
  // a<->b
  struct Point a, b;
  if ( (sortX & sortY) || (sortX & (shallow || horz)) || (sortY & (!shallow || vert)) )
  { a = *aTemp; b = *bTemp;} else { a = *bTemp; b = *aTemp;}
  // draw
  if (shallow) {
    for (int xi=a.x; xi<=b.x; xi++) {
      float m = ((float)b.y-a.y)/(b.x-a.x);
      float y = m * (xi - a.x) + a.y;
      struct Point pTemp = {xi,y+0.5};
      drawPoint(&pTemp,fill);
    }
  } else {
    for (int yi=a.y; yi<=b.y; yi++) {
      float m = ((float)b.x-a.x)/(b.y-a.y);
      float x = m * (yi-a.y) + a.x;
      struct Point pTemp = {x+0.5,yi};
      drawPoint(&pTemp,fill);
    }
  }
}

void drawFuncX(float (*f)(int), char fill) {
  printf(" drawing f(x)...\n");
  for (int xi=0; xi<xDim; xi++) {
    float y = (*f)(xi);
    struct Point pointTemp = {xi,y};
    drawPoint(&pointTemp,fill);
  }
}

float s_in(int x) { return 16*sin(x*M_PI/105) + 32;}

void printMATRIX() {
  printf(" printing MATRIX...\n");
  termLine();
  for(int yi=0; yi<yDim; yi++) {
    for (int xi=0; xi<xDim; xi++) { printf("%c",MATRIX[yi][xi]);}
    printf("\n");
  }
  termLine();
}

void drawUI() {
  printf(" drawing UI...\n");
  struct Point p1 = {0,0};
  struct Point p2 = {105,0};
  struct Point p3 = {209,0};
  struct Point p4 = {209,32};
  struct Point p5 = {209,63};
  struct Point p6 = {105,63};
  struct Point p7 = {0,63};
  struct Point p8 = {0,32};
  drawPoint(&p1,'\\');
  drawPoint(&p2,'|');
  drawPoint(&p3,'/');
  drawPoint(&p4,'-');
  drawPoint(&p5,'\\');
  drawPoint(&p6,'|');
  drawPoint(&p7,'/');
  drawPoint(&p8,'-');
}

void clearMATRIX() {
  printf(" clearing MATRIX...\n");
  for(int y=0; y<yDim; y++) { for(int x=0; x<xDim; x++) { MATRIX[y][x] = ' ';} }
}

/*-----------------> SPACIAL <-----------------*/

int zDim3 = 10, yDim3 = 64, xDim3 = 210;
bool SPACIAL[10][64][210];
//          [ Z][ Y][ X]

struct Point3 {
  int x;
  int y;
  int z;
};

void draw3Point(struct Point3 *p) { SPACIAL[p->z][p->y][p->x] = 1;}

void draw3Line(struct Point3 *aTemp, struct Point3 *bTemp) {
  printf(" printing 3D Line...\n");
  /*struct Point3 a, b;
  if (aTemp->x > bTemp->x) {
    a.x = bTemp->x;
    a.y = bTemp->y;
    a.z = bTemp->z;
    b.x = aTemp->x;
    b.y = aTemp->y;
    b.z = aTemp->z;
  } else {
    a.x = aTemp->x;
    a.y = aTemp->y;
    a.z = aTemp->z;
    b.x = bTemp->x;
    b.y = bTemp->y;
    b.z = bTemp->z;
  }
  int dx = b.x - a.x;
  int dy = b.y - a.y;
  int dz = b.z - a.z;
  float yM = (float)dy / dx;
  float zM = (float)dz / dx;
  for (int xi = a.x; abs(xi) < abs(b.x); xi+=pol(dx)) {
    float y = yM * (xi - a.x) + a.y;
    float z = zM * (xi - a.x) + a.z;
    struct Point3 point3Temp = {xi,y,z};
    draw3Point(&point3Temp);
  }*/

  // conditions
  bool sortX = bTemp->x > aTemp->x, sortY = bTemp->y > aTemp->y, sortZ = bTemp->z > aTemp->z;
  bool alikeX = aTemp->x == bTemp->x, alikeY = aTemp->y == bTemp->y, alikeZ = aTemp->z == bTemp->z;
  bool shallowXY = ( abs(((float)bTemp->y - aTemp->y)/(bTemp->x - aTemp->x)) <= 1.0 || alikeY ) && !alikeX;
  bool shallowXZ = ( abs(((float)bTemp->z - aTemp->z)/(bTemp->x - aTemp->x)) <= 1.0 || alikeZ ) && !alikeX;
  // a<->b
  struct Point3 a, b;
  if ( (sortX & sortY) || (sortX & (shallowXY || alikeY)) || (sortY & (!shallowXY || alikeX)) )
  { a = *aTemp; b = *bTemp;} else { a = *bTemp; b = *aTemp;}
  // draw
  if (shallowXY & shallowXZ) {
    for (int xi=a.x; xi<=b.x; xi++) {
      float yM = ((float)b.y-a.y)/(b.x-a.x);
      float zM = ((float)b.z-a.z)/(b.x-a.x);
      float y = yM * (xi - a.x) + a.y;
      float z = zM * (xi - a.x) + a.z;
      struct Point3 pTemp = {xi,y+0.5,z+0.5};
      draw3Point(&pTemp);
    }
  }/* else {
    for (int yi=a.y; yi<=b.y; yi++) {
      float m = ((float)b.x-a.x)/(b.y-a.y);
      float x = m * (yi-a.y) + a.x;
      float z = ...
      struct Point3 pTemp = {x+0.5,yi,z+0.5};
      draw3Point(&pTemp);
    }
  }*/
}

char arr[10] = "@%#*+=~-:.";

void SPACIAL_to_MATRIX() {
  printf(" transing SPACIAL to MATRIX...\n");
  for (int zi=zDim3-1; zi>=0; zi-=1) {
    for (int yi=0; yi<yDim3; yi++) {
      for (int xi=0; xi<xDim3; xi++) {
        char shade = arr[zi];
  if (SPACIAL[zi][yi][xi]==0) { }
  else { MATRIX[yi][xi] = shade;}
      }
    }
  }
}

void clearSPACIAL() {
  printf(" clearing SPACIAL...\n");
  for (int zi=0; zi<zDim3; zi++) { for(int yi=0; yi<yDim3; yi++) { for(int xi=0; xi<xDim3; xi++) { SPACIAL[zi][yi][xi] = 0;} } }
}

/*-----------------> PROG <-----------------*/

void startup() {
  termLine();
  printf("startup()\n");
  clearMATRIX();
  clearSPACIAL();
}

void render() {
  printf("render()\n");
  SPACIAL_to_MATRIX();
  drawUI();
  printMATRIX();
}

int main() {
  startup();

// TEST SHAPE 2D

  struct Point pA = {100,30};
  struct Point pB = {100,10};
  struct Point pC = {120,20};
  struct Point pD = {120,30};
  struct Point pE = {120,40};
  struct Point pF = {100,50};
  struct Point pG = {80,40};
  struct Point pH = {80,30};
  struct Point pI = {80,20};
  struct Point pJ = {90,10};
  struct Point pK = {110,10};
  struct Point pL = {110,50};
  struct Point pM = {90,50};
  struct Point pN = {85,15};
  struct Point pO = {115,15};
  struct Point pP = {115,45};
  struct Point pQ = {85,45};

  drawLine(&pA,&pB,'.');
  drawLine(&pA,&pC,'.');
  drawLine(&pA,&pD,'.');
  drawLine(&pA,&pE,'.');
  drawLine(&pA,&pF,'.');
  drawLine(&pA,&pG,'.');
  drawLine(&pA,&pH,'.');
  drawLine(&pA,&pI,'.');
  drawLine(&pA,&pJ,'.');
  drawLine(&pA,&pK,'.');
  drawLine(&pA,&pL,'.');
  drawLine(&pA,&pM,'.');
  drawLine(&pA,&pN,'.');
  drawLine(&pA,&pO,'.');
  drawLine(&pA,&pP,'.');
  drawLine(&pA,&pQ,'.');
  drawFuncX(s_in,'.');
  
// PYRAMID 

  struct Point3 p3At = {40,20,0}; //mid
  struct Point3 p3Bt = {60,30,9}; //right
  struct Point3 p3Ct = {40,10,3}; //top
  struct Point3 p3Dt = {25,25,6}; //left

  draw3Line(&p3At,&p3Bt); //succeeds
  draw3Line(&p3At,&p3Ct); //fails
  draw3Line(&p3At,&p3Dt); //succeeds
  draw3Line(&p3Bt,&p3Ct); //succeeds
  draw3Line(&p3Bt,&p3Dt); //succeeds
  draw3Line(&p3Ct,&p3Dt); //succeeds

// TEST SHAPE 3D a

  struct Point3 p3A = {160,30,9};
  struct Point3 p3B = {100,10,0};
  struct Point3 p3C = {120,20,0};
  struct Point3 p3D = {120,30,0};
  struct Point3 p3E = {120,40,0};
  struct Point3 p3F = {100,50,0};
  struct Point3 p3G = {80,40,0};
  struct Point3 p3H = {80,30,0};
  struct Point3 p3I = {80,20,0};
  struct Point3 p3J = {90,10,0};
  struct Point3 p3K = {110,10,0};
  struct Point3 p3L = {110,50,0};
  struct Point3 p3M = {90,50,0};
  struct Point3 p3N = {85,15,0};
  struct Point3 p3O = {115,15,0};
  struct Point3 p3P = {115,45,0};
  struct Point3 p3Q = {85,45,0};

  draw3Line(&p3A,&p3B);
  draw3Line(&p3A,&p3C);
  draw3Line(&p3A,&p3D);
  draw3Line(&p3A,&p3E);
  draw3Line(&p3A,&p3F);
  draw3Line(&p3A,&p3G);
  draw3Line(&p3A,&p3H);
  draw3Line(&p3A,&p3I);
  draw3Line(&p3A,&p3J);
  draw3Line(&p3A,&p3K);
  draw3Line(&p3A,&p3L);
  draw3Line(&p3A,&p3M);
  draw3Line(&p3A,&p3N);
  draw3Line(&p3A,&p3O);
  draw3Line(&p3A,&p3P);
  draw3Line(&p3A,&p3Q);

  render();
};
