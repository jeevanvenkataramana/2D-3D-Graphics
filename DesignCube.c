#include <cr_section_macros.h>
#include <NXP/crp.h>
#include "LPC17xx.h"                        /* LPC17xx definitions */
#include "ssp.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#define PORT_NUM            1
#define LOCATION_NUM        0
#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
uint8_t src_addr[SSP_BUFSIZE];
uint8_t dest_addr[SSP_BUFSIZE];
int colstart = 0;
int rowstart = 0;
/***************************
** Function name:       LCD_TEST
** Descriptions:        3-D vector graphics - 3D co-ordinates, 3D cube, 3D trandformations, decorate cube surface, trees, square and a alpha character on surface, shade of cube
** parameters:            None
** Returned value:        None
***************************/
//LCD
#define ST7735_TFTWIDTH  127
#define ST7735_TFTHEIGHT 159
#define ST7735_CASET   0x2A
#define ST7735_RASET   0x2B
#define ST7735_RAMWR   0x2C
#define ST7735_SLPOUT 0x11
#define ST7735_DISPON 0x29
#define LCD_D_C				21
#define LCD_RST				22
#define LCD_CS				16
#define swap(x, y) { x = x + y; y = x - y; x = x - y; }
#define swap_int16_t(a, b) { int16_t t = a; a = b; b = t; }
#define LIGHTBLUE 0x00FFE0
#define GREEN 0x00FF00
#define DARKBLUE 0x000033
#define BLACK 0x000000
#define BLUE 0x0007FF
#define RED 0xFF0000
#define MAGENTA 0x00F81F
#define WHITE 0xFFFFFF
#define PURPLE 0xCC33FF
#define BROWN 0xA52A2A
int _height = ST7735_TFTHEIGHT;
int _width = ST7735_TFTWIDTH;
int cursor_x = 0, cursor_y = 0;
int rotation = 0;
int textsize = 1;
int dx = 50;
int dy = 50;
struct coordinates{
int x;
int y;
};
int xCam = 100;
int yCam = 100;
int zCam = 100;
#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#define pi 3.14
struct Vertex {
   int x;
   int y;
};
struct Point {
   float x;
   float y;
};

//for writing data into the SPI
void spiwrite(uint8_t c)
{
    int portnum = 0;
    src_addr[0] = c;
    SSP_SSELToggle( portnum, 0 );
    SSPSend( portnum, (uint8_t *)src_addr, 1 );
    SSP_SSELToggle( portnum, 1 );
}

//writing commands into SPI

//making LCD ready to write data
void writedata(uint8_t c) {
    LPC_GPIO0->FIOSET |= (0x1<<21);
    spiwrite(c);
}
//writing data to the LCD

void writeword(uint16_t c) {
    uint8_t d;
    d = c >> 8;
    writedata(d);
    d = c & 0xFF;
    writedata(d);
}

//write colour
void write888(uint32_t color, uint32_t repeat) {
    uint8_t red, green, blue;
    int i;
    red = (color >> 16);
    green = (color >> 8) & 0xFF;
    blue = color & 0xFF;
    for (i = 0; i< repeat; i++) {
        writedata(red);
        writedata(green);
        writedata(blue);
    }
}

void setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1,
                    uint16_t y1) {
      writecommand(ST7735_CASET);
      writeword(x0);
      writeword(x1);
      writecommand(ST7735_RASET);
      writeword(y0);
      writeword(y1);
}

void drawPixel(int16_t x, int16_t y, uint32_t color) {
    if((x < 0) ||(x >= _width) || (y < 0) || (y >= _height)) return;
    setAddrWindow(x,y,x+1,y+1);
    writecommand(ST7735_RAMWR);
    write888(color, 1);
}


//Initialize LCD
void lcd_init()
{
     uint32_t portnum = 0;
     int i;
     printf("LCD initialized\n");
     LPC_GPIO0->FIODIR |= (0x1<<LCD_CS);
     LPC_GPIO0->FIODIR |= (0x1<<LCD_D_C);
     LPC_GPIO0->FIODIR |= (0x1<<LCD_RST);
     // Hardware Reset Sequence
     LPC_GPIO0->FIOSET |= (0x1<<LCD_RST);
     lcddelay(500);
     LPC_GPIO0->FIOCLR |= (0x1<<LCD_RST);
     lcddelay(500);
     LPC_GPIO0->FIOSET |= (0x1<<LCD_RST);
     lcddelay(500);
     for ( i = 0; i < SSP_BUFSIZE; i++ )    /* Init RD and WR buffer */
        {
            src_addr[i] = 0;
            dest_addr[i] = 0;
        }
     writecommand(ST7735_SLPOUT);
     lcddelay(200);
     writecommand(ST7735_DISPON);
      lcddelay(200);
}

void writecommand(uint8_t c)
{
LPC_GPIO0->FIOCLR |= (0x1<<LCD_D_C);
spiwrite(c);
}

void lcddelay(int ms)
{
int count = 24000;
int i;
for ( i = count*ms; i--; i > 0);
}

//Draw line function
void drawline(int16_t x0, int16_t y0, int16_t x1, int16_t y1,uint32_t color)
{
int16_t slope = abs(y1 - y0) > abs(x1 - x0);
if (slope) {
swap(x0, y0);
swap(x1, y1);
}
if (x0 > x1) {
swap(x0, x1);
swap(y0, y1);
}
int16_t dx, dy;
dx = x1 - x0;
dy = abs(y1 - y0);
int16_t err = dx / 2;
int16_t ystep;
if (y0 < y1) {
ystep = 1;
} else {
ystep = -1;
}
for (; x0<=x1; x0++) {
if (slope) {
drawPixel(y0, x0, color);
} else {
drawPixel(x0, y0, color);
}
err -= dy;
if (err < 0) {
y0 += ystep;
err += dx;
}
}
}

void drawRect(int16_t x, int16_t y,int16_t w, int16_t h,uint32_t color) {
drawFastHLine(x, y, w, color);
drawFastHLine(x, y+h-1, w, color);
drawFastVLine(x, y, h, color);
drawFastVLine(x+w-1, y, h, color);
}

void drawFastHLine(int16_t x, int16_t y,int16_t w, uint32_t color) {
drawline(x, y, x+w-1, y, color);
}

void draw_HorizontalLine(int16_t x, int16_t y, int16_t width, uint32_t color)
{
drawline(x, y, x+width-1, y, color);
}

void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint32_t color)
{
int16_t i;
// Update in subclasses if desired!
for (i=x; i<x+w; i++) {
drawFastVLine(i, y, h, color);
}
}

void drawFastVLine(int16_t x, int16_t y, int16_t h, uint32_t color)
{
// Update in subclasses if desired!
drawline(x, y, x, y+h-1, color);
}

void DrawSquares(){
int baseCoOrdinates[4][2]={{20,20},{20,50},{50,50},{50,20}};
int newCoOrdinates[4][2]={{0,0},{0,0},{0,0},{0,0}};
uint32_t *color[9] ={LIGHTBLUE,BLACK,RED,BLUE,RED,GREEN,MAGENTA,PURPLE,RED};
int numSavers=0;
while(numSavers<10){
int intSquareSize=rand()%(40)+20;
int x=rand()%(60)+0;
int y=rand()%(80)+0;
baseCoOrdinates[0][0]=x;
baseCoOrdinates[0][1]=y;
baseCoOrdinates[1][0]=x;
baseCoOrdinates[1][1]=y+intSquareSize;
baseCoOrdinates[2][0]=x+intSquareSize;
baseCoOrdinates[2][1]=y+intSquareSize;
baseCoOrdinates[3][0]=x+intSquareSize;
baseCoOrdinates[3][1]=y;
int intColor=rand()%(8)+0;
int saverItrations=0;
while(saverItrations<10){
int i=0;
while(i<4){
if(i==3){
drawline(baseCoOrdinates[i][0],baseCoOrdinates[i][1],baseCoOrdinates[0][0],baseCoOrdinates[0][1],color[intColor]);
newCoOrdinates[i][0]=baseCoOrdinates[i][0]+0.2*(baseCoOrdinates[0][0]-baseCoOrdinates[i][0]);
newCoOrdinates[i][1]=baseCoOrdinates[i][1]+0.2*(baseCoOrdinates[0][1]-baseCoOrdinates[i][1]);
}
else{
drawline(baseCoOrdinates[i][0],baseCoOrdinates[i][1],baseCoOrdinates[i+1][0],baseCoOrdinates[i+1][1],color[intColor]);
newCoOrdinates[i][0]=baseCoOrdinates[i][0]+0.2*(baseCoOrdinates[i+1][0]-baseCoOrdinates[i][0]);
newCoOrdinates[i][1]=baseCoOrdinates[i][1]+0.2*(baseCoOrdinates[i+1][1]-baseCoOrdinates[i][1]);
}
i++;
}
memcpy(baseCoOrdinates, newCoOrdinates, sizeof(baseCoOrdinates));
saverItrations++;
}
numSavers++;
}
}

void generateTrees(struct Point start, struct Point end, uint16_t color, int num) {
	if (num <= 0)
		return;
	int angle = 30;
	drawline(start.x, start.y, end.x, end.y, BROWN);
	struct Point temp;
	temp.x = start.x;
	temp.y = start.y;
	start.x = end.x;
	start.y = end.y;
	end.x = end.x+ (start.x - temp.x) * .8;
	end.y = end.y+ (start.y - temp.y) * .8;
	int deviation = 0;
	time_t t;
	rotateEndPoint(start, &end, angle + deviation);
	drawline(start.x, start.y, end.x, end.y, color);
	generateTrees(start, end, BROWN, num - 1);
	rotateEndPoint(start, &end, 360-angle-deviation);
	drawline(start.x, start.y, end.x, end.y, color);
	generateTrees(start, end, GREEN, num - 1);
	rotateEndPoint(start, &end, 360-angle-deviation);
	drawline(start.x, start.y, end.x, end.y, color);
	generateTrees(start, end, GREEN, num - 1);
}

void rotateEndPoint(struct Point start, struct Point *end, int angle) {
	float temp_x, temp_y, rot_x, rot_y;
 	float cos_val, sin_val, rotation_angle;

 	rotation_angle = angle * (3.14159265359 / 180);

 	temp_x = end->x - start.x;
 	temp_y = end->y - start.y;
 	cos_val = cos(rotation_angle); // compute trig. functions only once
 	sin_val = sin(rotation_angle);
 	rot_x = temp_x * cos_val - temp_y * sin_val;
 	rot_y = temp_x * sin_val + temp_y * cos_val;
 	end->x = rot_x + start.x;
 	end->y = rot_y + start.y;
 	return;
}

/**********************************************************
********************
3D transformations
**********************************************************
********************/
struct coordinates project2D(int xWorld, int yWorld, int zWorld)
{
int xScreen, yScreen, eyeScrnDist=100, dx=86, dy=50;
double xPrime, yPrime, zPrime, theta, phi, rho;
struct coordinates screen;
//theta = theta*pi/180;
//phi = phi*pi/180;
theta = acos(xCam/sqrt(pow(xCam,2)+pow(yCam,2)));
phi = acos(zCam/sqrt(pow(xCam,2)+pow(yCam,2)+pow(zCam,2)));
//theta = 0.785;
//phi = 0.785;
rho= sqrt((pow(xCam,2))+(pow(yCam,2))+(pow(zCam,2)));
xPrime = (yWorld*cos(theta))-(xWorld*sin(theta));
yPrime = (zWorld*sin(phi))-(xWorld*cos(theta)*cos(phi))-(yWorld*cos(phi)*sin(theta));
zPrime = rho-(yWorld*sin(phi)*cos(theta))-(xWorld*sin(phi)*cos(theta))-(zWorld*cos(phi));
xScreen = xPrime*eyeScrnDist/zPrime;
yScreen = yPrime*eyeScrnDist/zPrime;
xScreen = dx+xScreen;
yScreen = dy-yScreen;
screen.x = xScreen;
screen.y = yScreen;
return screen;
}

void drawCoordinates ()
{
struct coordinates lcd;
int x1,y1,x2,y2, x3,y3,x4,y4;	//draw world coordinate system
lcd = project2D(0,0,0);
x1=lcd.x;
y1=lcd.y;
lcd = project2D(180,0,0);
x2=lcd.x;
y2=lcd.y;
lcd = project2D(0,180,0);
x3=lcd.x;
y3=lcd.y;
lcd = project2D(0,0,180);
x4=lcd.x;
y4=lcd.y;
drawline(x1-30,y1-10,x2,y2,RED);//x axis red
drawline(x1-30,y1-10,x3,y3,GREEN); //y axis green
drawline(x1-30, y1-10, x4, y4,BLUE); //z axis magneta
}

void drawCube(int startPt, int size)
{
struct coordinates lcd;
int x1,y1,x2,y2,x3,y3,x4,y4,x5,y5,x6,y6,x7,y7,i;
xCam = 200;
yCam = 200;
zCam = 200;
lcd = project2D(startPt,startPt,(size+startPt));
x1=lcd.x;
y1=lcd.y;
lcd = project2D((size+startPt),startPt,(size+startPt));
x2=lcd.x;
y2=lcd.y;
lcd = project2D((size+startPt),(size+startPt),(size+startPt));
x3=lcd.x;
y3=lcd.y;
lcd = project2D(startPt,(size+startPt),(size+startPt));
x4=lcd.x;
y4=lcd.y;
lcd = project2D((size+startPt),startPt,startPt);
x5=lcd.x;
y5=lcd.y;
lcd = project2D((size+startPt),(size+startPt),startPt);
x6=lcd.x;
y6=lcd.y;
lcd = project2D(startPt,(size+startPt),startPt);
x7=lcd.x;
y7=lcd.y;
drawline(x1, y1, x2, y2,BLACK);
drawline(x2, y2, x3, y3,BLACK);
drawline(x3, y3, x4, y4,BLACK);
drawline(x4, y4, x1, y1,BLACK);
drawline(x2, y2, x5, y5,BLACK);
drawline(x5, y5, x6, y6,BLACK);
drawline(x6, y6, x3, y3,BLACK);
drawline(x6, y6, x7, y7,BLACK);
drawline(x7, y7, x4, y4,BLACK);
}


void fillCube(int startpt,int size)
{
struct coordinates s1;
int i,j;
size=size+startpt;
int a[size][size];
for(i=0;i<size;i++)
{
for(j=0;j<size;j++)
{
s1=project2D(j,i,size); //top fill
drawPixel(s1.x,s1.y,0x009933FF);
s1=project2D(i,size,j); // right fill
drawPixel(s1.x,s1.y,0x00000000);
s1=project2D(size,j,i); // left fill
drawPixel(s1.x,s1.y,0x00FF66B2);
}
}
}

void draw_Shadow(double inti_pos, double size, double shadow_Xposition, double shadow_Yposition, double shadow_Zposition)
{
int x_pos[8]={0}, y_pos[8]={0}, z_pos[8]={0};
struct coordinates coord1,coord2,coord3,coord4;
double x[8] = {inti_pos,(inti_pos+size), (inti_pos+size),inti_pos,inti_pos,(inti_pos+size), (inti_pos+size),inti_pos};
double y[8] = {inti_pos, inti_pos, inti_pos+size, inti_pos+size, inti_pos, inti_pos, (inti_pos+size), (inti_pos+size) };
double z[8] = {inti_pos, inti_pos, inti_pos, inti_pos, (inti_pos+size), (inti_pos+size), (inti_pos+size), (inti_pos+size)};
int i;
for(i=0; i<8; i++){
x_pos[i]=x[i]-((z[i]/(shadow_Zposition-z[i]))*(shadow_Xposition-x[i]));
y_pos[i]=y[i]-((z[i]/(shadow_Zposition-z[i]))*(shadow_Yposition-y[i]));
z_pos[i]=z[i]-((z[i]/(shadow_Zposition-z[i]))*(shadow_Zposition-z[i]));
}
coord1 = project2D(x_pos[4],y_pos[4],z_pos[4]);
coord2 = project2D(x_pos[5],y_pos[5],z_pos[5]);
coord3 = project2D(x_pos[6],y_pos[6],z_pos[6]);
coord4 = project2D(x_pos[7],y_pos[7],z_pos[7]);
drawline(coord1.x, coord1.y, coord2.x, coord2.y,BLACK);
drawline(coord2.x, coord2.y, coord3.x, coord3.y,BLACK);
drawline(coord3.x, coord3.y, coord4.x, coord4.y,BLACK);
drawline(coord1.x, coord1.y, coord4.x, coord4.y,BLACK);
fill_Triangle(coord1.x,coord1.y,coord2.x,coord2.y,coord3.x,coord3.y,BLACK);
fill_Triangle(coord1.x,coord1.y,coord3.x,coord3.y,coord4.x,coord4.y,BLACK);
}

void fill_Triangle(int16_t x0, int16_t y0,int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint32_t color) {
int16_t x, y, j, l;
if (y0 > y1) {
swap_int16_t(y0, y1);
swap_int16_t(x0, x1);
}
if (y1 > y2) {
swap_int16_t(y2, y1);
swap_int16_t(x2, x1);
}
if (y0 > y1) {
swap_int16_t(y0, y1);
swap_int16_t(x0, x1);
}
if(y0 == y2) {
x = y = x0;
if(x1 < x) x = x1;
else if(x1 > y) y = x1;
if(x2 < x) x = x2;
else if(x2 > y) y = x2;
draw_HorizontalLine(x, y0, y-x+1, color);
return;
}
int16_t dx01 = x1 - x0, dy01 = y1 - y0, dx02 = x2 - x0, dy02 = y2 - y0, dx12 = x2 - x1, dy12 = y2 - y1;
int32_t sa = 0, sb = 0;
if(y1 == y2) l = y1;
else l = y1-1;
for(j=y0; j<=l; j++) {
x = x0 + sa / dy01;
y = x0 + sb / dy02;
sa += dx01;
sb += dx02;
if(x > y) swap_int16_t(x,y);
draw_HorizontalLine(x, j, y-x+1, color);
}
sa = dx12 * (j - y1);
sb = dx02 * (j - y0);
for(; j<=y2; j++) {
x = x1 + sa / dy12;
y = x0 + sb / dy02;
sa += dx12;
sb += dx02;
if(x > y) swap_int16_t(x,y);
draw_HorizontalLine(x, j, y-x+1, color);
}
}

void squares2Dgraphics(int side)
{
int x0,y0,y1,x1,x2,y2,x3,y3,size,initial_color=0,i=0;
struct coordinates coord;
uint32_t color, colorArray [12]={0x00FF0000,0x0000FFFF, 0x00FF007F,0x00FF8000,0x0000FF80,0x000000FF,0x00FFFF00,0x00330066,0x0000FF80,0x00FF00FF, 0x0000FF00,0x000080FF};
while(i<6)
{
i++;
x0= 1+ rand() % (side-10);
y0=1+ rand() % (side-10);
size=10 + rand() % (side/4);
if(initial_color>12)
{
initial_color =0;
}
color = colorArray[initial_color];
initial_color++;
x1=size+x0;
if(x1>side){
x1=side-1;
}
x2=x1;
x3=x0;
y1=y0;
y2=size+y1;
if(y2>side){
y2=side-1;
}
y3=y2;
coord = project2D(side,x0,y0);
x0=coord.x;
y0=coord.y;
coord = project2D(side,x1,y1);
x1=coord.x;
y1=coord.y;
coord = project2D(side,x2,y2);
x2=coord.x;
y2=coord.y;
coord = project2D(side,x3,y3);
x3=coord.x;
y3=coord.y;
drawline(x0, y0, x1, y1,color);
drawline(x1, y1, x2, y2,color);
drawline(x2, y2, x3, y3,color);
drawline(x3, y3, x0, y0,color);
//rotation
int j;
for(j=0;j<6;j++)
{
x0=(x0+(0.2*(x1-x0)));
y0=(y0+(0.2*(y1-y0)));
x1=(x1+(0.2*(x2-x1)));
y1=(y1+(0.2*(y2-y1)));
x2=(x2+(0.2*(x3-x2)));
y2=(y2+(0.2*(y3-y2)));
x3=(x3+(0.2*(x0-x3)));
y3=(y3+(0.2*(y0-y3)));
drawline(x0, y0, x1, y1,color);
drawline(x1, y1, x2, y2,color);
drawline(x2, y2, x3, y3,color);
drawline(x3, y3, x0, y0,color);
drawRect(0,0,127,159,BLACK);
}
drawRect(0,0,127,159,BLACK);
}
}

void trees2Dgraphis(int initial_pos, int size)
{
int i=0, side;
struct coordinates coord;
int tree[3][3]={{initial_pos+10,initial_pos+30,0.5*size},{initial_pos+20,initial_pos+30,0.3*size},{initial_pos+15,initial_pos+37,0.8*size}};
while(i<3)
{
int x0, y0, y1, x1,dx0,dx1,dy0,dy1;
side = initial_pos+size;
x0=tree[i][0];
x1=tree[i][1];
y0=tree[i][2];
y1=y0;
i++;
coord = project2D(y0,side,x0);
dx0=coord.x;
dy0=coord.y;
coord = project2D(y1,side,x1);
dx1=coord.x;
dy1=coord.y;
drawline(dx0, dy0, dx1, dy1,BROWN);
drawline((dx0+1), (dy0+1), (dx1+1), (dy1+1),BROWN);
drawline((dx0-1), (dy0-1), (dx1-1), (dy1-1),BROWN);
int j=0;
for(j=0;j<5;j++){
int16_t x2=(0.6*(x1-x0))+x1;
int16_t y2=y1;
coord = project2D(y2,side,x2);
int dx2=coord.x;
int dy2=coord.y;
drawline(dx1, dy1, dx2, dy2,GREEN);
drawRect(0,0,127,159,BLACK);
int16_t x_rotated= ((0.134*x1)+(0.866*x2)-(0.5*y2)+(0.5*y1));
int16_t y_rotated=((0.5*x2)-(0.5*x1)+(0.866*y2)-(0.866*y1)+y1);
coord = project2D(y_rotated,side,x_rotated);
int dx_rotated=coord.x;
int dy_rotated=coord.y;
int16_t x_leftrotated=((0.134*x1)+(0.866*x2)+(0.5*y2)-(0.5*y1));
int16_t y_leftrotated=((0.5*x1)-(0.5*x2)+(0.134*y2)+(0.866*y1));
coord = project2D(y_leftrotated,side,x_leftrotated);
int dx_leftrotated=coord.x;
int dy_leftrotated=coord.y;
drawline(dx1, dy1, dx_rotated,dy_rotated,GREEN);
drawline(dx1, dy1, dx_leftrotated,dy_leftrotated,GREEN);
drawRect(0,0,127,159,BLACK);
int16_t xright_branchlen = sqrt(pow((x_rotated-x1),2)+pow((y_rotated-y1),2));
int16_t xright_imaginary= (0.8*xright_branchlen)+x_rotated;
int16_t xright_1 = ((0.134*x_rotated)+(0.866*xright_imaginary)-(0.5*y_rotated)+(0.5*y_rotated));
int16_t yright_1 =((0.5*xright_imaginary)-(0.5*x_rotated)+(0.866*y_rotated)-(0.866*y_rotated)+y_rotated);
coord = project2D(yright_1,side,xright_1);
int xp_1=coord.x;
int yp_1=coord.y;
int16_t xrotated_1,xrotated_left,yrotated_1,yrotated_left;
xrotated_1 = ((0.134*x_rotated)+(0.866*xright_1)-(0.5*yright_1)+(0.5*y_rotated));
yrotated_1 = ((0.5*xright_1)-(0.5*x_rotated)+(0.866*yright_1)-(0.866*y_rotated)+y_rotated);
coord = project2D(yrotated_1,side,xrotated_1);
int xprr=coord.x;
int yprr=coord.y;
xrotated_left = ((0.134*x_rotated)+(0.866*xright_1)+(0.5*yright_1)-(0.5*y_rotated));
yrotated_left = ((0.5*x_rotated)-(0.5*xright_1)+(0.134*y_rotated)+(0.866*yright_1));
coord = project2D(yrotated_left,side,xrotated_left);
int xprl=coord.x;
int yprl=coord.y;
int16_t xlImag= (0.8*xright_branchlen)+x_leftrotated; //imaginary vertical line x coordinate, y= y_rotated
int16_t xl1 = ((0.134*x_leftrotated) +(0.866*xlImag)+(0.5*y_leftrotated)-(0.5*y_leftrotated));
int16_t yl1 = ((0.5*x_leftrotated)- (0.5*xlImag)+(0.134*y_leftrotated)+(0.866*y_leftrotated));
coord = project2D(yl1,side,xl1);
int xpl1=coord.x;
int ypl1=coord.y;
int16_t xlr,xll,ylr,yll;
xlr = ((0.134*x_leftrotated)+(0.866*xl1)-(0.5*yl1)+(0.5*y_leftrotated));
ylr = ((0.5*xl1)-(0.5*x_leftrotated)+(0.866*yl1)-(0.866*y_leftrotated)+y_leftrotated);
coord = project2D(ylr,side,xlr);
int xplr=coord.x;
int yplr=coord.y;
xll = ((0.134*x_leftrotated)+(0.866*xl1)+(0.5*yl1)-(0.5*y_leftrotated));
yll = ((0.5*x_leftrotated)-(0.5*xl1)+(0.134*y_leftrotated)+(0.866*yl1));
coord = project2D(yll,side,xll);
int xpll=coord.x;
int ypll=coord.y;
drawline(dx_rotated, dy_rotated, xp_1,yp_1,GREEN);
drawline(dx_rotated, dy_rotated, xprr,yprr,GREEN);
drawline(dx_rotated, dy_rotated, xprl,yprl,GREEN);
drawline(xplr, yplr, xpl1,ypl1,GREEN);
drawline(xplr, yplr, xplr,yplr,GREEN);
drawline(xplr, yplr, xpll,ypll,GREEN);
x0=x1;
x1=x2;
}
drawRect(0,0,127,159,BLACK);
}
}

void alpha2D(int init_pos, int size)
{
struct coordinates coord;
int i,j;
size=size+init_pos;
int a[size][size];
for(i=0;i<size;i++)
{
for(j=0;j<size;j++)
{
if(i>=init_pos+5 && j>=init_pos+5 && j<=size-50 && i<=init_pos+50)
a[i][j]=1;
else if(i>=init_pos+5 && j>=init_pos+35 && j<=size-20 && i<=init_pos+50)
a[i][j]=1;
else if(i>=init_pos+20 && j>=init_pos+20 && j<=init_pos+35 && i<=init_pos+35)
a[i][j]=1;
else
a[i][j]=0;
}
}
for(i=0;i<size;i++)
{
for(j=0;j<size;j++)
{
if(a[i][j]==1)
{
coord = project2D(j,i,size);
drawPixel(coord.x,coord.y,WHITE);
}
else if(a[i][j]==0)
{
coord = project2D(j,i,size);
}
}
}
}

/**************************
**   Main Function  main()
**************************/
int main (void)
{
uint32_t i, portnum = PORT_NUM;
int size =70, startPt = 15;
portnum = 0 ; /* For LCD use 1 */
if ( portnum == 0 )
	  SSP0Init();            /* initialize SSP port */
else if ( portnum == 1 )
	  SSP1Init();
for ( i = 0; i < SSP_BUFSIZE; i++ )
{
  src_addr[i] = (uint8_t)i;
  dest_addr[i] = 0;
}
//initialize LCD
lcd_init();
while(1){
int optionSel=1;
fillRect(0,0,ST7735_TFTWIDTH,ST7735_TFTHEIGHT,WHITE);
DrawSquares();
fillRect(0,0,ST7735_TFTWIDTH,ST7735_TFTHEIGHT,WHITE);
struct Point p0, p1;
uint8_t D = 15;
uint8_t temp_x = 40, temp_y = 20;
int count=3;
do {
	p0.x = temp_x;
	p0.y = temp_y;
	p1.x = temp_x;
	p1.y = temp_y + D;
	generateTrees(p0, p1, GREEN, 7);
	temp_x = rand() % 100;
	temp_y = rand() % 70;
	D = (rand() % 20) + 5;
	count--;
}while (count>0);
fillRect(0,0,ST7735_TFTWIDTH,ST7735_TFTHEIGHT,WHITE);
drawCoordinates();
drawCube(startPt,size+startPt-4);
fillCube(startPt,size);
squares2Dgraphics(size+startPt);
trees2Dgraphis(startPt,size+15);
draw_Shadow(startPt, size, -5000,0,5000);
alpha2D(startPt,size);
}
return 0;
}
/**************************
**                            End Of File
**************************/
