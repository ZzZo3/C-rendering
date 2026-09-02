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

typedef int bool; bool false = 0, true = 1;

/*-----------------> MATRIX [NOT SCREEN] <-----------------*/

int yDim = 256, xDim = 256;
char MATRIX[256][256];
//         [ Y][  X]

struct Point {
  int x;
  int y;
};

void drawPoint(struct Point *p, char fill) { MATRIX[p->y][p->x] = fill;}

//
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

void printMATRIX() {
  printf(" drawing MATRIX...\n");
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

//char grad[13] = "@W%$so=+~:-,.";

/*-----------------> PROG <-----------------*/

void startup() {
  termLine();
  printf("startup()\n");
  clearMATRIX();
}

void render() {
  printf("render()\n");
  drawUI();
  printMATRIX();
}

/*-----------------> CONTENT <-----------------*/

void build() {
// TEST SHAPE 2D
  printf(">Test Shape IID\n");
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
  drawLine(&pA,&pB,'.');drawLine(&pA,&pC,'.');drawLine(&pA,&pD,'.');drawLine(&pA,&pE,'.');
  drawLine(&pA,&pF,'.');drawLine(&pA,&pG,'.');drawLine(&pA,&pH,'.');drawLine(&pA,&pI,'.');
  drawLine(&pA,&pJ,'.');drawLine(&pA,&pK,'.');drawLine(&pA,&pL,'.');drawLine(&pA,&pM,'.');
  drawLine(&pA,&pN,'.');drawLine(&pA,&pO,'.');drawLine(&pA,&pP,'.');drawLine(&pA,&pQ,'.');
}

int main() {
  startup();
  build();
  render();
};