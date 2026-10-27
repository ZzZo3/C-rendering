#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <ctype.h>

/*-----------------> NOTICE <-----------------*/
/*
  A MATRIX SHALL BE DEFINED AND REFERENCED WITH "M[column][row]". -> double M[3][3] = {{1,4,7},{2,5,8},{3,6,9}};
  A POINT 2 SHALL BE DEFINED AS A MATRIX OF 1 ROW AND 2 COLUMNS.  -> double P2[1][2] = {{x,y}};
  A POINT 3 SHALL BE DEFINED AS A MATRIX OF 1 ROW AND 3 COLUMNS.  -> double P3[1][3] = {{x,y,z}};
      |1 2 3|        |x|        |x|
  M = |4 5 6| ; P2 = |y| ; P3 = |y|
      |7 8 9|                   |z|
*/
/*-----------------> CONSTANTS <-----------------*/

#define yDim /*65*/  117
#define xDim /*211*/ 373
#define gradL 14
#define MAXINPUT 64
#define triL 9
#define camHeight 5
#define COMMANDSCOUNT 13
#define MAXPARAML 16
#define MAXPARAMS 16

/*-----------------> UTIL <-----------------*/

typedef int Pt2[2];
typedef double Pt3[3];
typedef double Mtrx33[9];
typedef double ObjShade[2];
/* ObjShade Table
0|Mono        |Value
1|Distance    |Multiplier
2|AxisGradiant|Axis: 1:x -1:-x 2:y -2:-y 3:z -3:-z
*/

struct Triangle3 {
  Pt3 a,b,c;
  ObjShade shade;
} typedef Tri3;

int pol(double v) {
  if (v>0) return 1;
  if (v<0) return -1;
  return 0;
};

int polBin(double v) {
  if (v<0) return -1;
  return 1;
};

double distA(Pt3 A) {
  return sqrt(A[0]*A[0]+A[1]*A[1]+A[2]*A[2]);
};

double distAB(Pt3 A, Pt3 B) {
  return sqrt((A[0]-B[0])*(A[0]-B[0])+(A[1]-B[1])*(A[1]-B[1])+(A[2]-B[2])*(A[2]-B[2]));
};

void applyMtrxPt3(Pt3 M31, Mtrx33 M33) {
  Pt3 result;
  for (int colR=0; colR<3; colR++) {
    result[colR] = 0;
    for (int i=0; i<3; i++) result[colR] += M31[i]*M33[3*i+colR];
  };
  for (int i=0; i<3; i++) M31[i] = result[i];
};

double dot(Pt3 A, Pt3 B) {
  return (A[0]*B[0]+A[1]*B[1]+A[2]*B[2]);
};

double dotNorm(Pt3 A, Pt3 B) {
  return (A[0]*B[0]+A[1]*B[1]+A[2]*B[2]) / (distA(A)*distA(B));
}

void cross(Pt3 A, Pt3 B, Pt3 C) {
  C[0] = A[1]*B[2]-A[2]*B[1];
  C[1] = A[2]*B[0]-A[0]*B[2];
  C[2] = A[0]*B[1]-A[1]*B[0];
};

char* strtokat(const char *_str, const char *delim, int index) {
  if (!_str||!delim||index<0) return NULL;
  char *strc = strdup(_str);
  if (!strc) return NULL;
  int i=0;
  char *target_token = NULL;
  char *token = strtok(strc,delim);
  while (token!=NULL) {
    if (i==index) {
      target_token = strdup(token);
      break;
    };
    i++;
    token = strtok(NULL,delim);
  };
  return target_token;
};

/*-----------------> PREDEFINED VARIABLES <-----------------*/

char* tmlScnSpcA; // screenspace
char* tmlScnSpcB; // UI layer
Pt3 CAM = {0,camHeight,0};
Pt3 POS = {0,0,0};
double yaw = 0, pitch = 0; // Radians
double radius = 100; // render distance
double pseuFOV = 4; // inital z-val for screenspace points | as "pseuFOV" -> 0, FOV -> 180
double xScale = 0.065, yScale = 0.10; // [0..1] coef for dimensions of screenspace
bool changedUI = true; // whether or not the UI has been changed since last render(), so as to avoid calling UItxt extraneously

int scnDims[2] = {xDim,yDim};

char grad[gradL] = " .,:~>+=so$W%@";

Mtrx33 xySwpMtrx = {0,1,0,1,0,0,0,0,1};
Mtrx33 xzSwpMtrx = {0,0,1,0,1,0,1,0,0};
Mtrx33 pitchMtrx = {1,0,0,0,1,0,0,0,1};
Mtrx33 yawMtrx = {1,0,0,0,1,0,0,0,1};

Tri3 TRI[triL];
Pt3 TRINORM[triL];

/*-----------------> FILE HANDLING <-----------------*/

void saveData(char fileName[]) {
  FILE* file;
  char *fPath = "./saveData/";
  if (strcmp(fileName," ")==0) {
    strcat(fPath,"data.txt");
  } else {
    fileName[strcspn(fileName,"\n\r\0. /\\()[]{}<>:;\"\'`~!@#$%^&*")] = 0;
    strcat(fPath,fileName);
    strcat(fPath,".txt");
  };
  file = fopen(fPath,"w");
  if (file==NULL) { printf("ERROR: \"./data.txt\" not created.\n");return;};
  fprintf(file,"POS:%f:%f:%f\n",POS[0],POS[1],POS[2]);
  fprintf(file,"YAW/PITCH:%f:%f\n",yaw,pitch);
  fclose(file);
  printf("data saved.\n");
};

int loadData(char fileName[]) {
  FILE *file;
  char *fPath = "./saveData/";
  if (strcmp(fileName," ")==0) {
    strcat(fPath,"data.txt");
  } else {
    fileName[strcspn(fileName,"\n\r\0. /\\()[]{}<>:;\"\'`~!@#$%^&*")] = 0;
    strcat(fPath,fileName);
    strcat(fPath,".txt");
  };
  file = fopen(fPath,"r");
  char buffer[256];
  char *token;
  char *id;
  double _f;
  bool dataSuccess = true;
  while(fgets(buffer,sizeof(buffer),file) != NULL) {
    printf("raw-%s",buffer);
    id = strtokat(buffer,":",0);
    printf("id: %s\n",id);
    if (strcmp(id,"POS")==0) {
      printf("->POS");
      for (int i=0; dataSuccess&i<3; i++) {
        token=strtokat(buffer,":",i+1);
        _f=strtof(token,NULL);
        if (_f!=_f) dataSuccess=false;
        else {
          POS[i]=_f;
          CAM[i]=_f;
          if (i==1) CAM[i]+=camHeight;
        };
        printf(":%f",_f);
      };
      printf("\n");
    } else if (strcmp(id,"YAW/PITCH")==0) {
      printf("->YAW/PITCH");
      token=strtokat(buffer,":",1);
      _f=strtof(token,NULL);
      if (_f!=_f) dataSuccess=false;
      else yaw=_f;
      printf(":%f",_f);
      token=strtokat(buffer,":",2);
      _f=strtof(token,NULL);
      if (_f!=_f) dataSuccess=false;
      else pitch=_f;
      printf(":%f\n",_f);
    };
  };
  fclose(file);
  free(token);free(id);
  if (!dataSuccess) {
    printf("data failed to load.\n");
    return -1;
  };
  printf("data loaded.\n");
  return 0;
};

/*-----------------> MATRIX <-----------------*/

void drawPoint(char* ARRAY, Pt2 p, char fill) { ARRAY[p[1]*xDim+p[0]] = fill;};

void drawLine(char* ARRAY, Pt2 a, Pt2 b, char fill) {
  bool sortX = b[0] > a[0], sortY = b[1] > a[1];
  bool vert = a[0] == b[0], horz = a[1] == b[1];
  bool shallow = ( fabs(((double)b[1] - a[1])/(b[0] - a[0])) <= 1.0 || horz ) && !vert;
  if ( !(sortX & sortY) & !(sortX & (shallow||horz)) & !(sortY & (!shallow||vert)) ) { Pt2 temp={b[0],b[1]};b=a;a=temp;};
  if (shallow) {
    for (int xi=a[0]; xi<=b[0]; xi++) {
      double m = ((double)b[1]-a[1])/(b[0]-a[0]);
      double y = m * (xi - a[0]) + a[1];
      Pt2 pTemp = {xi,y+0.5};
      drawPoint(ARRAY,pTemp,fill);
    };
  } else {
    for (int yi=a[1]; yi<=b[1]; yi++) {
      double m = ((double)b[0]-a[0])/(b[1]-a[1]);
      double x = m * (yi-a[1]) + a[0];
      Pt2 pTemp = {x+0.5,yi};
      drawPoint(ARRAY,pTemp,fill);
    };
  };
};

void tmlLine() {
  for (int i=0; i<xDim+6; i++) printf("=");
  printf("\n");
};

void printScnPt(int x, int y) {
  if (tmlScnSpcB[y*xDim+x]==' ') {printf("%c",tmlScnSpcA[y*xDim+x]); return;};
  printf("%c",tmlScnSpcB[y*xDim+x]);
  return;
};

void printTml() {
  tmlLine();
  for(int yi=0; yi<yDim; yi++) {
    if (yi<10) printf("|%d  |",yi);
    else if (yi<100) printf("|%d |",yi);
    else printf("|%d|",yi);
    for (int xi=0; xi<xDim; xi++) printScnPt(xi, yi);
    printf("|\n");
  }
  tmlLine();
};

void emptyTmlScnSpc() {
  for(int m=0; m<yDim*xDim; m++) {
    tmlScnSpcA[m]=' ';
    if (changedUI) tmlScnSpcB[m]=' ';
  };
};

char fontCharArr[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ1234567890 -,.!?\"\':";

void UItxt(Pt2 corner, char txt[], int size, int spacing) {
  FILE *font;
  char buffer[512];
  char *end;
  for (int cursor=0; cursor<strlen(txt); cursor++) {
    font = fopen("./font.txt","r");
    int fontLinesCount=0;
    char c;
    for (c=fgetc(font);c!=EOF;c=fgetc(font)) if (c=='\n') fontLinesCount++;
    fclose(font);
    font = fopen("./font.txt","r");
    int fileLine=0;
    while (fileLine<=fontLinesCount & (int)strtof(buffer,&end)!=size) {fgets(buffer,sizeof(buffer),font);fileLine++;};
    if (fileLine==fontLinesCount) {printf("ERROR: UItxt [size] not found."); return;};
    int _l=0;
    for (int _guard=0; _l<sizeof(fontCharArr)/sizeof(char) & toupper(txt[cursor])!=fontCharArr[_l]; _guard++) _l++;
    for (int yi=0; yi<size; yi++) {
      fgets(buffer,sizeof(buffer),font);
      int y = corner[1]+yi;
      for (int xi=0; xi<size; xi++) {
        int x = corner[0]+((size+spacing)*cursor)+xi;
        tmlScnSpcB[y*xDim+x] = buffer[_l*size+xi];
      };
    };
    fclose(font);
  };
  return;
};

void buildUI() {
  Pt2 pUIa = {0,0}, pUIb = {floor(xDim/2),0}, pUIc = {xDim-1,0};
  Pt2 pUId = {0,floor(yDim/2)}, pUIe = {xDim-1,floor(yDim/2)};
  Pt2 pUIf = {0,yDim-1}, pUIg = {floor(xDim/2),yDim-1}, pUIh = {xDim-1,yDim-1};
  drawPoint(tmlScnSpcB, pUIa,'\\');drawPoint(tmlScnSpcB, pUIb,'|');drawPoint(tmlScnSpcB, pUIc,'/');
  drawPoint(tmlScnSpcB, pUId,'-');drawPoint(tmlScnSpcB, pUIe,'-');
  drawPoint(tmlScnSpcB, pUIf,'/');drawPoint(tmlScnSpcB, pUIg,'|');drawPoint(tmlScnSpcB, pUIh,'\\');
  UItxt((Pt2){4,6},"the year: 2000. the news: nothing.",5,1);
  UItxt((Pt2){4,12},"unimportant: lord returns.",7,2);
  UItxt((Pt2){10,20},"scotus says: \"huh?\"",5,1);
};

/*-----------------> SCENE <-----------------*/

void castRay(Pt3 RAY, Pt2 px) {
  bool DP = false;
  double value = 0.0;
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
    double t = dot(a,n)/dot(n,RAY);
    for (int j=0; j<3; j++) T[j]=RAY[j]*t;
    for (int j=0; j<3; j++) {
      a[j]-=T[j];
      b[j]-=T[j];
      c[j]-=T[j];
    };
    double inOutTheta = acos(dotNorm(a,b))+acos(dotNorm(a,c))+acos(dotNorm(b,c));
    if (inOutTheta>=2*M_PI&t>=0) {
      foundTRI=true;
      for (int j=0; j<2; j++) { shade[j]=TRI[i].shade[j];};
      if (distA(T)<distA(Tfnl)) for(int j=0; j<3; j++) Tfnl[j]=T[j];
    }
  };
  if (foundTRI) {
    value = (radius-distA(Tfnl))/radius;
    if (value<1.0/(gradL-1)) { value=1.0/(gradL-1);};
  } else {
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
  int xi=0, yi=0;
  for(int m=0; m<yDim*xDim; m++) {
    if (xi<xDim) {xi++;} else {xi=0;yi++;};
//0. Generate Pt3 for pixel
    Pt3 p3Dummy = {xScale*(xi-floor(xDim/2)), yScale*(yi-floor(yDim/2)), pseuFOV};
//1. Apply Screenspace Transform
    for (int i=0; i<2; i++) if (p3Dummy[i]!=0) {
      p3Dummy[i] += (2*pow(p3Dummy[i],3))/(fabs(p3Dummy[i])*scnDims[i]); p3Dummy[i]/=3;
    };
    p3Dummy[1] *= -1;
//2. Get Slopes (X-Swaps?)
    double tempF, dydx, dzdx;
    int slopeCase = 0;
    if (p3Dummy[0]==0) {
      if (p3Dummy[1]!=0) { slopeCase=1; applyMtrxPt3(p3Dummy,xySwpMtrx);} else
      if (p3Dummy[2]!=0) { slopeCase=2; applyMtrxPt3(p3Dummy,xzSwpMtrx);}
      else { printf("ERROR: castNet() failed due to pxTarget at {0,0,0}\n");return;};
    };
    dydx = p3Dummy[1]/p3Dummy[0];
    dzdx = p3Dummy[2]/p3Dummy[0];
//3. Extrude Ray to Sphere
    double xOptA = sqrt(radius*radius/(dydx*dydx+dzdx*dzdx+1));
    double xOptB = -xOptA;
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

/*-----------------> CONTENT <-----------------*/

void buildScene() {
  TRI[0] = (Tri3){{70,0,70},{70,0,-70},{-70,0,-70},{1,0.15}};
  TRI[1] = (Tri3){{70,0,70},{-70,0,70},{-70,0,-70},{1,0.15}};
  TRI[2] = (Tri3){{-20,0,25},{-10,10,25},{-10,0,25},{0,1}};
  TRI[3] = (Tri3){{-20,0,25},{-10,10,25},{-20,10,25},{0,0.5}};
  TRI[4] = (Tri3){{-10,0,25},{-10,10,70},{-10,0,70},{0,1}};
  TRI[5] = (Tri3){{-10,0,25},{-10,10,70},{-10,10,25},{0,0.5}};
  TRI[6] = (Tri3){{-20,10,25},{-10,10,25},{-15,15,25},{0,1}};
  TRI[7] = (Tri3){{-15,15,25},{-7,7,70},{-7,7,25},{0,1}};
  TRI[8] = (Tri3){{-10,0,25},{0,10,25},{-10,10,25},{1,1}};
};

/*-----------------> COMMAND FUNCTIONS <-----------------*/

void pitchUp(double by) { pitch+=by;return;};

void yawLeft(double by) { yaw+=by;return;};

void walkForward(double by) {
  Pt3 unitZ = {0,0,by};
  applyMtrxPt3(unitZ,yawMtrx);
  POS[0]+=unitZ[0];POS[2]+=unitZ[2];
  CAM[0]+=unitZ[0];CAM[2]+=unitZ[2];
  return;
};

void walkRight(double by) {
  Pt3 unitZ = {by,0,0};
  applyMtrxPt3(unitZ,yawMtrx);
  POS[0]+=unitZ[0];POS[2]+=unitZ[2];
  CAM[0]+=unitZ[0];CAM[2]+=unitZ[2];
  return;
};

int parseL(char line[MAXINPUT]);

void autoCmdFunc(char name[]) {
  printf("automating...\n");
  if (strcmp(name," ")==0) {
    printf("ERROR: auto empty.\n");
    return;
  };
  FILE *autoFile;
  char *token;
  autoFile = fopen("./auto.txt","r");
  int autosCount=0;
  char c;
  for (c=fgetc(autoFile);c!=EOF;c=fgetc(autoFile)) if (c=='\n') autosCount++;
  fclose(autoFile);
  bool foundAuto=false;
  char buffer[MAXINPUT];
  autoFile = fopen("./auto.txt","r");
  for (int i=0; !foundAuto&(i<autosCount); i++) {
    fgets(buffer, sizeof(buffer), autoFile);
    token = strtokat(buffer,"~~~",0);
    if (strcmp(name,token)==0) foundAuto=true;
  };
  fclose(autoFile);
  if (!foundAuto) {
    printf("ERROR: could not find auto.\n");
    free(token);
    return;
  };
  token = strtokat(buffer,"~~~",1);
  token[strcspn(token, "\r\n")] = 0; /* Tim Čas : https://stackoverflow.com/a/28462221 : 2026-10-26, License - CC BY-SA 3.0*/
  parseL(token);
  free(token);
  return;
};

/*-----------------> COMMANDS / RENDER LOOP <-----------------*/

void render() {
  emptyTmlScnSpc();
  pitchMtrx[4]=cos(pitch);pitchMtrx[5]=-sin(pitch);pitchMtrx[7]=sin(pitch);pitchMtrx[8]=cos(pitch);
  yawMtrx[0]=cos(yaw);yawMtrx[2]=sin(yaw);yawMtrx[6]=-sin(yaw);yawMtrx[8]=cos(yaw);
  castNet();
  if (changedUI) {buildUI();changedUI=false;};
  printTml();
  return;
};

char commands[][COMMANDSCOUNT] = {"x","w","s","a","d","W","S","A","D","save","load","test","auto"};
int commandParamCounts[COMMANDSCOUNT] = {0,0,0,0,0,0,0,0,0,1,1,2,1};

int run(int code, char parameters[][16]) {
//printf("running: %d\n",code);
  if (code==1) {printf("exiting.\n");} else
  if (code==2) {pitchUp(-3.1415/32);render();} else
  if (code==3) {pitchUp(3.1415/32);render();} else
  if (code==4) {yawLeft(3.1415/32);render();} else
  if (code==5) {yawLeft(-3.1415/32);render();} else
  if (code==6) {walkForward(2.5);render();} else
  if (code==7) {walkForward(-2.5);render();} else
  if (code==8) {walkRight(-2.5);render();} else
  if (code==9) {walkRight(2.5);render();} else
  if (code==10) {saveData(parameters[0]);} else
  if (code==11) {if (loadData(parameters[0])==0) render();} else
  if (code==12) {printf("test->{%s,%s}\n",parameters[0],parameters[1]);} else
  if (code==13) {autoCmdFunc(parameters[0]);};
  return code;
};

int parse(char _cmd[]) {
// 0. Setup
  char *cmd = strdup(_cmd);
  int segments=1, runs=1, code=0;
  for (int i=0; i<strlen(cmd); i++) if (cmd[i]==':') segments++;
//1. Check For Runs (Int as First Segment)
  int slashes=0;
  for (int i=0; i<strlen(cmd); i++) if (cmd[i]=='/') slashes++;
  if (slashes!=0&slashes!=1) {
    printf("ERROR: too many '/' in command.");
    free(cmd);return 0;
  };
  char *token=strtokat(cmd,"/",0);
  if (!token) {free(token);free(cmd);return 0;};
  char *end;
  long _runs=strtol(token, &end, 10);
  bool changeRuns=true;
  if (end==token) changeRuns=false;
  while (*end!='\0') {
    if (!isspace((unsigned char)*end)) changeRuns=false;
    end++;
  };
  if (_runs<=0) changeRuns=false;
  if (changeRuns) { runs=(int)_runs;};
//2 Remove Leading Int
  if (changeRuns) {
    token=strtokat(cmd,"/",1);
    if (token!=NULL) cmd=token;
  };
//3. Get Command Code
  token=strtokat(cmd,":",0);
  if (!token) {free(token);free(cmd);return 0;};
  for (int i=0; i<COMMANDSCOUNT; i++) { if (strcmp(token,commands[i])==0) { code=i+1;};};
  if (code<1) {
    printf("ERROR: unrecognized command.\n");
    free(token);free(cmd);return 0;
  };
//4. Get Parameters
  int paramCount = commandParamCounts[code-1];
  char parameters[MAXPARAML][MAXPARAMS];
  if (segments-1!=paramCount) {
    printf("ERROR: incorrect parameter count\n");
    free(token);free(cmd);return 0;
  };
  for (int i=1; i<segments; i++) {
    token=strtokat(cmd,":",i);
    if (token!=NULL) strcpy(parameters[i-1],token);
    else strcpy(parameters[i-1]," ");
  };
  for (int i=0; i<runs; i++) code=run(code,parameters);
  free(token);free(cmd);return code;
};

int parseL(char line[MAXINPUT]) {
  int _code, code=0, spaces=0;
  for (int i=0; i<strlen(line); i++) if (line[i]==' ') spaces++;
  char *cmdPtr;
  if (spaces==0) {
    cmdPtr = strdup(line);
    _code = parse(cmdPtr);
    if (_code==1) code=1;
  } else {
    for (int i=0; i<=spaces; i++) {
      cmdPtr=strtokat(line," ",i);
      _code = parse(cmdPtr);
      if (_code==1) code=1;
    };
  };
  free(cmdPtr);
  return code;
};

/*-----------------> BAKING <-----------------*/

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
    int code = parseL(input);
    if (code==1) return 0;
  };
};
