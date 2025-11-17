/*
Применение UVC к KHR. 8 мая 2022: Версия 1.0
*/

#include "stdio.h"
#include "kcb5.h"
#include "i2c.h"
#include "uart.h"
#include "pio.h"
#include "rom.h"
#include "timer.h"
#include "ics.h"
#include "ad.h"
#include "dac.h"
#include "math.h"

#define OFS_L 129.0         // Длина от центра до точки крепления (с учетом зазора)
#define OFS_S 213           // Смещение сервопривода
#define CHG_SVA 1718.9      // Коэффициент преобразования для сервопривода
#define HEIGHT 185          // 185 мм
#define RET_F_LEG 3         // Отступ задней ноги
#define RST_F_LEG 3         // Сброс передней ноги

//// Глобальные переменные
int16_t K0W[2];         // Правый сервопривод коленного сустава (передний)
int16_t K1W[2];         // Правый сервопривод коленного сустава (средний)
int16_t K2W[2];         // Правый сервопривод коленного сустава (задний)
int16_t HW [2];         // Сервопривод бедра
int16_t A0W[2];         // Правый сервопривод плечевого сустава (передний)
int16_t A1W[2];         // Правый сервопривод плечевого сустава (задний)
int16_t U0W[2];         // Правый сервопривод верхней части (передний)
int16_t U1W[2];         // Правый сервопривод верхней части (средний)
int16_t U2W[2];         // Правый сервопривод верхней части (задний)
int16_t EW[2];          // Сервопривод стабилизации
int16_t WESTW;          // Сервопривод поворота туловища
int16_t HEADW;          // Сервопривод поворота головы
int16_t K0R,K0RB,U0R,U0L,U1R,U1L,EWS;
int16_t K0L,K0LB,U0WB;
int16_t K0WB;
int16_t jikuasi;
int16_t motCt,motCtBak,motCtBak2,motCtdat;
int16_t mode,modeNxt,subMode,keyMode;
int16_t pitch_gyr,roll_gyr,yaw_gyr;
int16_t cycle,tst0;
int16_t walkCt,walkCtLim;       // Счетчик шагов
int16_t p_ofs,r_ofs;
int16_t ir,ip,ira,ipa;
int16_t irb,ipb,ct;
int16_t pitchs,rolls,pitch_ofs, roll_ofs, yaw, yaw_ofs;
int16_t landF,landB;

int32_t tBak, pitchi;
uint32_t tNow;

uint8_t cmd[2];
uint8_t ff[45];
uint8_t dsp[110];
uint8_t krr[4];
int8_t kn;
int8_t LEDct;   // Счетчик мигания LED

float fwctEnd,fwct,fwctUp;
float pitch,roll,pitcht,rollt;
float pitch_gyrg,         roll_gyrg;
float wk,wt;
float dyi,dyib,dyis;
float dxi,dxib,dxis;
float rollg,fw,fwi,fws,sw,freeBak,tt0;
float supportingLeg,swingLeg;   // Опорная нога, маховая нога
float footH;                    // Высота подъема стопы
float swx,swy,swMax;            // Смещение при переносе
float autoH,fh,fhMax;           // Автоматическая высота, высота стопы

//// Временные переменные ////
int32_t ii;
int16_t i,j;
float k,k0,k1,ks,kxy,kl;

////////////////////////
//// Вывод строки ////
////////////////////////
void printS(char *StringData){
    unsigned char count = 0;
    while(StringData[count] != '\0')count++;
    uart_tx(UART_COM, (unsigned char*)StringData, 0, count);
}

////////////////////////
//// Функция задержки ////
////////////////////////
void delay(int32_t t)
{
    int32_t tb=0;
    t*=1000;
    timer_write(TIMER,t);
    timer_start(TIMER);
    t=0;
    while(t >= tb){
        tb=t;
        t=timer_read(TIMER);
    }
}

/////////////////////
//// I2C доступ ////
/////////////////////
uint8_t read8(uint8_t reg )
{
    cmd[0]=reg;
    // Адрес от 0x28 сдвинут на 1 бит до 0x50 (I2C протокол KONDO)
    i2c_read(0x50, cmd, 1, ff, 1);  // BNO055_CHIP_ID_ADDR(0x28)
    return ff[0];
}

bool readLen(uint8_t reg, uint8_t len)
{
    cmd[0]=reg;
    // Адрес от 0x28 сдвинут на 1 бит до 0x50 (I2C протокол KONDO)
    i2c_read(0x50, cmd, 1, ff, len);  // BNO055_CHIP_ID_ADDR(0x28)
    return true;
}

bool write8(uint8_t reg, uint8_t dat)
{
    cmd[0]=dat;
    // Адрес от 0x28 сдвинут на 1 бит до 0x50 (I2C протокол KONDO)
    i2c_write(0x50, reg, cmd, 1);  // BNO055_CHIP_ID_ADDR(0x28)
    return true;
}

/////////////////////////////////////
//// Плавное изменение значения ////
/////////////////////////////////////
void movSv(short *s,int d){
// s: текущее положение, d: целевое положение
    if(motCt<1) *s = d;
    else        *s += (d-*s)/motCt;
}

///////////////////////////
//// Корректировка углов ////
///////////////////////////
void angAdj(void){

// **** pitch & roll ****
    if( pitchs<=ipb+1 && pitchs>=ipb-1 &&
        rolls <=irb+1 && rolls >=irb-1     ){
        ++ip;
        ipa+=pitchs;
        ira+=rolls;
    }
    else {
        ip=0;
        ipa=0;
        ira=0;
    }
    ipb=pitchs;
    irb=rolls;
    sprintf( (char *)dsp,"P:%4d R:%4d C:%4d\r",pitchs,rolls,ip );
    printS((char *)dsp);
}

///////////////////////
//// Определение угла падения ////
///////////////////////
void detAng(void){
    if( 0.35>fabs(pitch) && 0.35>fabs(roll) )return;
    sprintf( (char *)dsp," PRA:%4d %4d PRG:%4d %4d\r\n",pitchs,rolls,pitch_gyr,roll_gyr );
    printS((char *)dsp);

    ics_set_pos ( UART_SIO2, 1, 4735 +1350 );   // U0R
    ics_set_pos ( UART_SIO4, 1, 9320 -1350 );   // U0L
    ics_set_pos ( UART_SIO2, 4, 4800 +2700 );   // ER
    ics_set_pos ( UART_SIO4, 4,10150 -2700 );   // EL
    ics_set_pos ( UART_SIO1, 7, 7500 -2100 );   // K0R
    ics_set_pos ( UART_SIO3, 7, 7500 +2100 );   // K0L
    ics_set_pos ( UART_SIO1, 8, 9260 -4200 );   // HR
    ics_set_pos ( UART_SIO3, 8, 5740 +4200 );   // HL
    ics_set_pos ( UART_SIO1, 9, 7910 +2100 );   // A0R
    ics_set_pos ( UART_SIO3, 9, 7100 -2100 );   // A0L
    delay(30);
    ics_set_pos     ( UART_SIO1, 5, 0 );    // K2R
    ics_set_pos     ( UART_SIO2, 1, 0 );    // U0R
    ics_set_pos     ( UART_SIO3, 5, 0 );    // K2L
    ics_set_pos     ( UART_SIO4, 1, 0 );    // U0L
    ics_set_pos     ( UART_SIO1, 6, 0 );    // K1R
    ics_set_pos     ( UART_SIO2, 2, 0 );    // U1R
    ics_set_pos     ( UART_SIO3, 6, 0 );    // K1L
    ics_set_pos     ( UART_SIO4, 2, 0 );    // U1L
    ics_set_pos     ( UART_SIO1, 7, 0 );    // K0R
    ics_set_pos     ( UART_SIO3, 7, 0 );    // K0L
    ics_set_pos     ( UART_SIO1, 8, 0 );    // HR
    ics_set_pos     ( UART_SIO2, 4, 0 );    // ER
    ics_set_pos     ( UART_SIO3, 8, 0 );    // HL
    ics_set_pos     ( UART_SIO4, 4, 0 );    // EL
    ics_set_pos     ( UART_SIO1, 9, 0 );    // A0R
    ics_set_pos     ( UART_SIO3, 9, 0 );    // A0L
    sprintf( (char *)dsp,"turn over :%4d %4d\r\n",mode,pitchs );
    printS ( (char *)dsp );
    while(1){}
}

/////////////////////
//// UVC вспомогательная функция ////
/////////////////////
void uvcSub(void){

    // ************ В конце UVC вернуть опорную ногу в нейтральное положение ************
    if( fwct<=landF ){

        // **** Боковое смещение ****
        k = dyi/(11-fwct);
        dyi -= k;
        dyis += k;

        // **** Переднее смещение ****
        if( dxi>RST_F_LEG ){
            dxi  -= RST_F_LEG;
            dxis -= RST_F_LEG;
        }
        else if( dxi<-RST_F_LEG ){
            dxi  += RST_F_LEG;
            dxis += RST_F_LEG;
        }
        else{
            dxis -= dxi;
            dxi   = 0;
        }
    }
    if(dyis> 70)    dyis=  70;
    if(dxis<-70)    dxis= -70;
    if(dxis> 70)    dxis=  70;

    // ************ Корректировка высоты ноги ************
    if(HEIGHT>autoH){                       // Плавное изменение высоты ноги
        autoH +=(HEIGHT-autoH)*0.07;        // 0.07
    }
    else    autoH = HEIGHT;

    if( fwct>fwctEnd-landB && rollt>0){ // Учет трения при контакте с землей
        autoH -= ( fabs(dyi-dyib)+fabs(dxi-dxib) ) * 0.02;
    }
    if(140>autoH)autoH=140;                 // Минимальная высота ноги
}

void uvcSub2(void){
    float k0,k1;

    // ************ В конце UVC вернуть опорную ногу в нейтральное положение ************

    // **** Боковое смещение ****
    k1 = dyi/(fwctEnd-fwct+1);
    dyi -= k1;

    // **** Переднее смещение ****
    k0 = dxi/(fwctEnd-fwct+1);
    dxi  -= k0;

    // **** Поворот туловища ****
    wk -= wk/(fwctEnd-fwct+1);
    WESTW  -= WESTW/(fwctEnd-fwct+1);
    K2W[0]  = WESTW;
    K2W[1]  =-WESTW;

    if( fwct<=landF ){
        // **** Боковое смещение ****
        dyis += k1;

        // **** Переднее смещение ****
        dxis -= k0;
    }
    else{
        dyis -= dyis/(fwctEnd-fwct+1);
        dxis -= dxis/(fwctEnd-fwct+1);
    }

    // **** Корректировка высоты ****
    autoH += (HEIGHT-autoH)/(fwctEnd-fwct+1);

    if(dyis> 70)    dyis=  70;
    if(dxis<-70)    dxis= -70;
    if(dxis> 70)    dxis=  70;
}

/////////////////
//// UVC обработка ////
/////////////////
void uvc(void){
    float pb,rb,k;

    // ************ Игнорирование малых углов ************
    rb=roll;        // Сохранение исходных значений
    pb=pitch;
    k=sqrt(pitch*pitch+roll*roll);  // Суммарный угол наклона
    if( k>0.033 ){
        k=(k-0.033)/k;
        pitch *=k;
        roll  *=k;
    }
    else{
        pitch =0;
        roll  =0;
    }

    // ************ Применение коэффициентов к углам наклона ************
    rollt =0.25*roll;
    if(jikuasi==0)  rollt = -rollt;     // Инверсия для маховой ноги
    pitcht=0.25*pitch;

    if(fwct>landF && fwct<=fwctEnd-landB ){

        // ************ Расчет UVC ************
        k     = atan ((dyi-sw)/autoH );    // Угол колена относительно земли
        kl    = autoH/cos(k);              // Длина ноги от бедра до стопы
        ks = k+rollt;                      // Угол колена с учетом наклона
        k  = kl*sin(ks);                   // Боковое смещение от бедра до точки касания
        dyi   = k+sw;                      // Боковое смещение UVC коррекции
        autoH = kl*cos(ks);                // Обновление высоты для K1

        // **** UVC (переднее смещение) *****
        k     = atan( dxi/autoH );     // Угол наклона ноги вперёд от вертикали
        kl    = autoH/cos(k);          // Длина ноги от вертикали
        ks    = k+pitcht;              // Угол наклона с учетом pitch
        k     = kl*sin(ks);            // Переднее смещение UVC коррекции
        dxi   = k;                     // Переднее смещение UVC коррекции
        autoH = kl*cos(ks);            // Обновление высоты для K1

        // ************ Ограничения значений UVC ************
        if(dyi<  0)     dyi=   0;
        if(dyi> 45)     dyi=  45;
        if(dxi<-45)     dxi= -45;
        if(dxi> 45)     dxi=  45;

        // ************ Установка позиции маховой ноги ************

        // **** Боковое смещение *****
        dyis = dyi;                 // Y координата маховой ноги

        // **** Переднее смещение *****
        dxis = -dxi;                // X координата маховой ноги

        // ************ Корректировка ширины шага ************
        if(jikuasi==0){     // Для правой опорной ноги
            k = -sw+dyi;    // Выступ правой ноги
            ks=  sw+dyis;   // Выступ левой ноги
        }
        else{
            ks= -sw+dyi;    // Выступ левой ноги
            k =  sw+dyis;   // Выступ правой ноги
        }
        if(k+ks<0)dyis-=k+ks;  // Корректировка маховой ноги для симметрии
    }
    roll =rb;
    pitch=pb;
}

////////////////////
//// Подъем стопы ////
////////////////////
void footUp(void){

    if( fwct>landF && fwct<=(fwctEnd-landB) )fh = fhMax * sin( M_PI*(fwct-landF)/(fwctEnd-(landF+landB)) );
    else                                     fh = 0;
}

////////////////////
//// Управление переносом ////
////////////////////
void swCont(void){
    float k,t;

    k=swMax*sinf(M_PI*fwct/fwctEnd); // Синусоидальная траектория
    t=atan( (fabs(dxi)+fabs(21.5*sin(wt)))/(dyi+21.5*cos(wt)-wt) );
    if(dxi>0)   swx =  k*sin(t);
    else        swx = -k*sin(t);
    swy=k*cos(t);
}

////////////////
//// Управление руками ////
////////////////
void armCont(void){
    U1W[0]=510*dyis/70; // Движение рук в противофазе ногам
    if(U1W[0]<0)U1W[0]=0;
    U1W[1]=U1W[0];
}

////////////////////
//// Управление стопой ////
////////////////////
void footCont(float x,float y,float h,int s){
// x: смещение вперед от центра (вперед +)
// y: боковое смещение от центра (вправо +)
// h: высота от тазобедренного сустава до голеностопного (Max194.5)
// s: опорная нога 0/маховая нога 1
// Расчет смещения от K1 до A1: k = sqrt( x*x + h*h );
// K1-K0 = 40mm A0-A1 = 24.5mm, всего 64.5mm

    float k;

    k = sqrt(x*x+pow(sqrt(y*y+h*h)-64.5,2));   // Расстояние K0-A0
    if(k>129){
        autoH = sqrt(pow(sqrt(129*129-x*x)+64.5,2)-y*y); // Корректировка высоты
        k=129;
    }

    x = CHG_SVA*asin(x/k);                      // Угол коленного сустава K0
    k = CHG_SVA*acos(k/130);                    // Угол сгибания бедра
    if(k>1800)k=1800;                           // Максимум 60 градусов

    if      (2*k-HW[s]> 100) k=(HW[s]+100)/2;   // Ограничение скорости вращения
    else if (2*k-HW[s]<-100) k=(HW[s]-100)/2;

    if(mode!=0 && jikuasi==s)HW[s]  = k*2;      // Учет опорной ноги
    else                     HW[s]  = k*2;
    K0W[s]  = k+x;
    A0W[s]  = k-x;

    k = CHG_SVA*atan(y/h);                      // Угол коленного сустава K1

    if      (k-K1W[s]> 100) k=K1W[s]+100;       // Ограничение скорости вращения
    else if (k-K1W[s]<-100) k=K1W[s]-100;

    if(mode!=0 && jikuasi==s)K1W[s] = k;        // Учет опорной ноги
    else                     K1W[s] = k;
    A1W[s] = -k;
}

////////////////////
//// Управление обеими ногами ////
////////////////////
void feetCont1(float x0, float y0, float x1, float y1, int s){

    if(s==1){
        if(y0+21.5==0)      wt = 0;             // Защита от деления на ноль
        else if(jikuasi==0){
            wt = 0.5*atan( x0/(y0+21.5) ); // Угол поворота туловища
            wk=fabs(15.0*x0/45);            // 45: максимальный шаг, 15: максимальный сдвиг
        }
        else{
            wt = 0.5*atan( -x1/(y1+21.5) ); // Угол поворота туловища
            wk=fabs(15.0*x1/45);
        }

        WESTW = wt*CHG_SVA;                 // Поворот туловища (+ право)
        K2W[0]= WESTW;
        K2W[1]=-WESTW;
    }

    if(jikuasi==0){
        footCont( x0,   y0-wk,   autoH   ,   0 );
        footCont( x1,   y1-wk,   autoH-fh,   1 );
    }
    else{
        footCont( x0,   y0-wk,   autoH-fh,   0 );
        footCont( x1,   y1-wk,   autoH   ,   1 );
    }
}

void feetCont2(int s){
    if(jikuasi==0)  feetCont1( dxi  -swx, dyi  -swy,   dxis -swx, dyis +swy ,s );
    else            feetCont1( dxis -swx, dyis +swy,   dxi  -swx, dyi  -swy ,s );
}

///////////////////////////
//// Управление счетчиком ////
///////////////////////////
void counterCont(void){
    if(fwct>=fwctEnd){  // Конец цикла (смена опорной ноги)
        jikuasi^=1;
        fwct=0;

        fh=0;
        k=dyis;
        dyis=dyi;
        dyib=dyi;
        dyi=k;

        k=dxis;
        dxis=dxi;
        dxib=dxi;
        dxi=k;
    }
    else{
        fwct+=fwctUp;
        if(fwct>fwctEnd)fwct=fwctEnd;    // Ограничение максимального значения
    }
}

//##################
//#### Управление движением ####
//##################
void walk(void){

    switch(mode){

//**** A Инициализация, переход в стартовую позицию ****
case 710:
    movSv(&K0W[0],  661);
    movSv(&K1W[0],    0);
    movSv(&K2W[0],    0);
    movSv(& HW[0], 1322);
    movSv(&A0W[0],  661);
    movSv(&A1W[0],    0); // +30
    movSv(&U0W[0],-2700);
    movSv(&U1W[0],    0);
    movSv(&U2W[0],    0);
    movSv(& EW[0],    0);
    movSv(& WESTW,    0);

    movSv(&K0W[1],  661);
    movSv(&K1W[1],    0);
    movSv(&K2W[1],    0);
    movSv(& HW[1], 1322);
    movSv(&A0W[1],  661);
    movSv(&A1W[1],    0); // -30
    movSv(&U0W[1],-2760);
    movSv(&U1W[1],    0);
    movSv(&U2W[1],    0);
    movSv(& EW[1],    0);
    movSv(& HEADW,    0);

    if(motCt>0)--motCt;
    else{
        //// Переменные для коррекции углов ////
        p_ofs   =0;
        r_ofs   =0;
        ip      =0;
        ir      =0;
        ipb     =0;
        irb     =0;
        ipa     =0;
        ira     =0;

        //// Переменные для UVC ////
        dxi     =0;
        dyi     =0;
        dxis    =0;
        dyis    =0;
        dxib    =0;
        dyib    =0;

        landF   =0;
        landB   =0;
        fwctEnd =18;
        walkCt  =0;
        fwctUp  =1;
        fwct=1;
        autoH=HEIGHT;
        sw=0;
        swx=0;
        swy=0;
        jikuasi=1;

        //// Стартовая позиция ////
        footCont(0,0,HEIGHT,0);
        jikuasi=0;
        footCont(0,0,HEIGHT,1);
        mode=720;                   // Следующее состояние
        sprintf( (char *)dsp,"mode=720\r\n" );
        printS((char *)dsp);
    }
    break;

//**** B Калибровка датчиков ориентации ****
case 720:
    angAdj();
    if( ip==100 ){
        p_ofs=ipa/100;
        r_ofs=ira/100;
        mode=730;       // Следующее состояние
        sprintf( (char *)dsp,"\r\nmode=730 ** Ready ** Pa:%4d Po:%4d  Ra:%4d Ro:%4d\r\n",pitchs,p_ofs,rolls,r_ofs );
        printS((char *)dsp);
    }
    detAng();           // Обнаружение падения
    break;

//**** C Определение наклона в стартовой позиции ****
case 730:

    // **** Корректировка смещений ****
    if( ip>=50 ){
        ip=0;
        if(rolls >0) ++r_ofs;
        if(rolls <0) --r_ofs;
        if(pitchs>0) ++p_ofs;
        if(pitchs<0) --p_ofs;
    }
    else ++ip;

    feetCont2(1);

    if( fabs(roll)>0.033 || fabs(pitch)>0.044 ){
        if(roll>0)  jikuasi=1;
        else        jikuasi=0;
        fwct=1;
        mode=740;                   // Следующее состояние
        sprintf( (char *)dsp,"mode=740\r\n" );
        printS((char *)dsp);
        break;
    }
    sprintf( (char *)dsp,"P:%4d R:%4d C:%4d\r",pitchs,rolls,ip );
    printS((char *)dsp);
    break;

//**** D Запуск UVC контроля ****
case 740:
    uvc();                  // Основная обработка UVC
    uvcSub();               // Вспомогательная обработка UVC
    footUp();               // Расчет подъема стопы
    feetCont2(1);           // Управление ногами
    armCont();
    counterCont();          // Управление счетчиком
    if(fwct==0){
        mode=750;           // Следующее состояние
        sprintf( (char *)dsp,"mode=750\r\n" );
        printS((char *)dsp);
    }
    detAng();           // Обнаружение падения
    break;

//**** E UVC фаза, подготовка к переносу ****
case 750:
    feetCont2(1);
    if( fwct>30 ){
        fwct=1;

        k=sqrt(0.5*dxis*dxis+dyis*dyis);   // Амплитуда шага
        swMax=17+17*k/45;

        mode=760;       // Следующее состояние
        sprintf( (char *)dsp,"mode=760 %2d \r\n",(int)swMax );
        printS((char *)dsp);
        break;
    }
    else{
        sprintf( (char *)dsp,"C:%4d\r",(int)fwct );
        printS((char *)dsp);
        ++fwct;
    }
    detAng();           // Обнаружение падения
    break;

//**** F Фаза переноса ****
case 760:
    landF=25;
    fwctEnd=landF+25; // 15
    uvcSub2();          // Вспомогательная обработка UVC
    footUp();           // Подъем стопы
    swCont();           // Управление переносом
    feetCont2(0);       // Управление ногами
    armCont();
    counterCont();      // Управление счетчиком
    if(fwct==0){
        landF=0;
        dxi     =0;
        dyi     =0;
        dxis    =0;
        dyis    =0;
        dxib    =0;
        dyib    =0;
        landF   =0;
        landB   =0;
        fwctEnd =18;
        walkCt  =0;
        fwctUp  =1;
        fwct=1;
        autoH=HEIGHT;
        sw=0;
        swx=0;
        swy=0;
        jikuasi=1;
        mode=770;           // Следующее состояние
        sprintf( (char *)dsp,"mode=770\r\n" );
        printS((char *)dsp);
    }
    detAng();           // Обнаружение падения
    break;

//**** G Фаза после переноса, подготовка к следующему шагу ****
case 770:
    feetCont2(0);
    if( fwct>50 ){ // 50
        fwct=1;
        mode=730;       // Следующее состояние
        sprintf( (char *)dsp,"mode=730\r\n" );
        printS((char *)dsp);
        break;
    }
    else{
        sprintf( (char *)dsp,"C:%4d\r",(int)fwct );
        printS((char *)dsp);
        ++fwct;
    }
    detAng();           // Обнаружение падения
    break;

case 780:
    feetCont2(1);
    break;

case 790:
    break;

case 791:

    dxis=-dxi;
    dyis=dyi;

    feetCont2(1);
    break;

case 700:               // Отладка
    sprintf( (char *)dsp,"R:%4d  %4d  RG:%4d  %4d\r\n", (int)(roll*1000),(int)(pitcht*1000), (int)roll_gyr,(int)pitch_gyr);
    printS((char *)dsp);
    break;
    }
}


//######################
//#### ѓLЃ[“ЗЌћђ§Њд ####
//######################
void keyCont(void){

	//////////////////////////
	////// TTYѓRѓ}ѓ“ѓhЏ€—ќ //////
	//////////////////////////
	ff[0]=0;
	ff[1]=0;
	uart_rx (UART_COM, ff, 1,1);
	if(ff[0]!=0){
		sprintf( (char *)dsp,"%c \r\n",ff[0] );
		printS((char *)dsp);

		if(ff[0]==' ')keyMode=0;//ѓLЃ[ѓ‚Ѓ[ѓhѓЉѓZѓbѓg

		///////////////////////
		//// Љо–{“ь—Нѓ‚Ѓ[ѓh ////
		///////////////////////
		if(keyMode==0){
			switch(ff[0]){
				case 'r':		//ѓЉѓZѓbѓg
					motCt=100;
					mode=710;
					sprintf( (char *)dsp,"**** Reset ****\r\n" );
					printS((char *)dsp);
					break;

				case 'g':		//ЉJЋn
					sprintf( (char *)dsp,"**** Go ****\r\n" );
					printS((char *)dsp);
					break;

				case 't':		//ЋЋЊ±
					mode=790;
					sprintf( (char *)dsp,"**** angle Disp ****\r\n" );
					printS((char *)dsp);
					break;

				case 'y':		//ЋЋЊ±
					mode=791;
					sprintf( (char *)dsp,"**** debug ****\r\n" );
					printS((char *)dsp);
					break;

//				case 'm':		//ѓpѓ‰ѓЃѓ^ѓ‚Ѓ[ѓh
//					keyMode=1;
//					break;
				case 'p':		//“Б’и•Пђ”ђЭ’иѓ‚Ѓ[ѓh
					keyMode=5;
					break;
				case 'k':		//ѓTЃ[ѓ{ KѓOѓ‹Ѓ[ѓv
					keyMode=2;
					break;
				case 'u':		//ѓTЃ[ѓ{ UѓOѓ‹Ѓ[ѓv
					keyMode=3;
					break;
				case 'a':		//ѓTЃ[ѓ{ AѓOѓ‹Ѓ[ѓv
					keyMode=4;
					break;
				case 'h':		//ѓTЃ[ѓ{ HѓOѓ‹Ѓ[ѓv
					keyMode=50;
					break;
				case 'e':		//ѓTЃ[ѓ{ EѓOѓ‹Ѓ[ѓv
					keyMode=60;
					break;
				case 'z':		//ѓTЃ[ѓ{ HEAD
					keyMode=700;
					break;
				case 'w':		//ѓTЃ[ѓ{ WEST
					keyMode=800;
					break;
			}
		}


		///////////////////////////
		//// “Б’и•Пђ”ђЭ’иѓ‚Ѓ[ѓh ////
		///////////////////////////
		if(keyMode==5){
			switch(ff[0]){
				case '0':
					kn=0;
					goto dd2;
				case '1':
					kn=1;
					goto dd2;
				case '2':
					kn=2;
					goto dd2;
				case '3':
					kn=3;
					goto dd2;
				case '4':
					kn=4;
					goto dd2;
				case '5':
					kn=5;
					goto dd2;
				case '6':
					kn=6;
					goto dd2;
				case '7':
					kn=7;
					goto dd2;
				case '8':
					kn=8;
					goto dd2;
				case '9':
					kn=9;
					goto dd2;
				case '+':
				case '-':
					switch(kn){
					case 0:dxi			+= ff[0]=='+'?	1:	-1;		break;
					case 1:dyi			+= ff[0]=='+'?	1:	-1;		break;
					case 2:swMax		+= ff[0]=='+'?	1:	-1;		break;
					case 3:pitch_gyrg	+= ff[0]=='+'?	0.01:-0.01;	break;
					case 4:roll_gyrg	+= ff[0]=='+'?	0.01:-0.01;	break;
					case 5:fh			+= ff[0]=='+'?  1:  -1;		break;
					case 6:fhMax		+= ff[0]=='+'?	1:  -1;		break;
					case 7:walkCtLim	+= ff[0]=='+'?  1:  -1;		break;
					case 8:autoH		+= ff[0]=='+'?  1:  -1;		break;
					}
dd2:
					sprintf( (char *)dsp, "No:%d\r\n0 dx:%d\r\n1 dy:%d\r\n2 sw:%d\r\n3 pg:%d\r\n4 rg:%d\r\n5 fh:%d\r\n6 fh%d\r\n7  wc%d\r\n8 aH:%d\r\n"
						,kn 	,(int)dxi,(int)dyi,(int)swMax,(int)(pitch_gyrg*100),(int)(roll_gyrg*100),(int)fh,(int)fhMax,(int)walkCtLim,(int)autoH);
					printS((char *)dsp);
					break;
			}
		}


		/////////////////////////
		//// ѓTЃ[ѓ{ђЭ’иѓ‚Ѓ[ѓh ////
		/////////////////////////
		if(keyMode==2){			//ѓTЃ[ѓ{ KѓOѓ‹Ѓ[ѓv
			switch(ff[0]){
				case '0':		//K0‘I‘р
					keyMode=20;
					break;
				case '1':		//K1‘I‘р
					keyMode=21;
					break;
				case '2':		//K2‘I‘р
					keyMode=22;
					break;
			}
		}
		if(keyMode==3){			//ѓTЃ[ѓ{ UѓOѓ‹Ѓ[ѓv
			switch(ff[0]){
				case '0':		//U0‘I‘р
					keyMode=30;
					break;
				case '1':		//U1‘I‘р
					keyMode=31;
					break;
				case '2':		//U2‘I‘р
					keyMode=32;
					break;
			}
		}
		if(keyMode==4){			//ѓTЃ[ѓ{ AѓOѓ‹Ѓ[ѓv
			switch(ff[0]){
				case '0':		//A0‘I‘р
					keyMode=40;
					break;
				case '1':		//A1‘I‘р
					keyMode=41;
					break;
			}
		}

		if(keyMode>=20&&keyMode<=60){		//ѓTЃ[ѓ{ K,U,A,N,EѓOѓ‹Ѓ[ѓv
			switch(ff[0]){
				case 'r':		//K0‘I‘р
					keyMode=keyMode*10;
					break;
				case 'l':		//K1‘I‘р
					keyMode=keyMode*10+1;
					break;
				case 'b':		//K2‘I‘р
					keyMode=keyMode*10+2;
					break;
			}
		}

		if(keyMode>=200&&keyMode<=800){		//ѓTЃ[ѓ{ K,U,A,N,EѓOѓ‹Ѓ[ѓv
			i=0;
			if(ff[0]=='+')i= 30;
			if(ff[0]=='-')i=-30;
			if(ff[0]=='+'||ff[0]=='-'){
				switch(keyMode){
				case 200:	K0W[0]+=i;	break;
				case 201:	K0W[1]+=i;	break;
				case 202:	K0W[0]+=i;K0W[1]+=i;break;
				case 210:	K1W[0]+=i;	break;
				case 211:	K1W[1]+=i;	break;
				case 212:	K1W[0]+=i;K1W[1]+=i;break;
				case 220:	K2W[0]+=i;	break;
				case 221:	K2W[1]+=i;	break;
				case 222:	K2W[0]+=i;K2W[1]+=i;break;

				case 300:	U0W[0]+=i;	break;
				case 301:	U0W[1]+=i;	break;
				case 302:	U0W[0]+=i;U0W[1]+=i;break;
				case 310:	U1W[0]+=i;	break;
				case 311:	U1W[1]+=i;	break;
				case 312:	U1W[0]+=i;U1W[1]+=i;break;
				case 320:	U2W[0]+=i;	break;
				case 321:	U2W[1]+=i;	break;
				case 322:	U2W[0]+=i;U2W[1]+=i;break;

				case 400:	A0W[0]+=i;	break;
				case 401:	A0W[1]+=i;	break;
				case 402:	A0W[0]+=i;A0W[1]+=i;break;
				case 410:	A1W[0]+=i;	break;
				case 411:	A1W[1]+=i;	break;
				case 412:	A1W[0]+=i;A1W[1]+=i;break;

				case 500:	HW[0]+=i;	break;
				case 501:	HW[1]+=i;	break;
				case 502:	HW[0]+=i;HW[1]+=i;break;

				case 600:	EW[0]+=i;	break;
				case 601:	EW[1]+=i;	break;
				case 602:	EW[0]+=i;EW[1]+=i;break;

				case 700:	HEADW+=i;	break;
				case 701:	HEADW+=i;	break;

				case 800:	WESTW+=i;	break;
				case 801:	WESTW+=i;	break;
				}
				sprintf( (char *)dsp,    "Mode=%d\r\n",modeNxt );
				printS((char *)dsp);
				sprintf( (char *)dsp,    "K0:%7d %7d K1:%7d %7d K2:%7d %7d \r\n",K0W[0],K0W[1],K1W[0],K1W[1],K2W[0],K2W[1] );
				printS((char *)dsp);
				sprintf( (char *)dsp,    "H:%7d %7d \r\n",HW[0] ,HW[1]  );
				printS((char *)dsp);
				sprintf( (char *)dsp,    "A0:%7d %7d A1:%7d %7d \r\n\r\n",A0W[0],A0W[1],A1W[0],A1W[1] );
				printS((char *)dsp);
				sprintf( (char *)dsp,    "U0:%7d %7d U1:%7d %7d U2:%7d %7d \r\n",U0W[0],U0W[1],U1W[0],U1W[1],U2W[0],U2W[1] );
				printS((char *)dsp);
				sprintf( (char *)dsp,    "E:%7d %7d \r\n\r\n",EW[0] ,EW[1]  );
				printS((char *)dsp);
				sprintf( (char *)dsp,    "HD:%7d WT:%7d \r\n",HEADW ,WESTW  );
				printS((char *)dsp);
			}
		}
	}
}




// **************************************************************************
// *************** Main routine *********************************************
// **************************************************************************
int main(void){

	K0W[0]=0;			//ЊТЉЦђЯ‘OЊг•ыЊь‰EЏ‘Ќћ—p
	K1W[0]=0;			//ЊТЉЦђЯ‰Ў•ыЊь‰EЏ‘Ќћ—p
	K2W[0]=0;			//ЊТЉЦђЯ‰Ў•ыЊь‰EЏ‘Ќћ—p
	HW [0]=0;			//•GЉЦђЯ‰EЏ‘Ќћ—p
	A0W[0]=0;			//‘«ЋсЏг‰є•ыЊь‰EЏ‘Ќћ—p
	A1W[0]=0;			//‘«Ћс‰Ў•ыЊь‰EЏ‘Ќћ—p
	U0W[0]=-5400;			//ЊЁ‘OЊг•ыЊь‰EЏ‘Ќћ—p
	U1W[0]=0;			//ЊЁ‰ЎЊг•ыЊь‰EЏ‘Ќћ—p
	U2W[0]=0;			//ЊЁѓ€Ѓ[Њь‰EЏ‘Ќћ—p
	 EW[0]=0;			//•I‰EЏ‘Ќћ—p
	 WESTW=0;			//Ќ‰с“]Џ‘Ќћ—p

	K0W[1]=0;			//ЊТЉЦђЯ‘OЊг•ыЊьЌ¶Џ‘Ќћ—p
	K1W[1]=0;			//ЊТЉЦђЯ‰Ў•ыЊьЌ¶Џ‘Ќћ—p
	K2W[1]=0;			//ЊТЉЦђЯ‰Ў•ыЊьЌ¶Џ‘Ќћ—p
	HW [1]=0;			//•GЉЦђЯЌ¶Џ‘Ќћ—p
	A0W[1]=0;			//‘«ЋсЏг‰є•ыЊьЌ¶Џ‘Ќћ—p
	A1W[1]=0;			//‘«Ћс‰Ў•ыЊьЌ¶Џ‘Ќћ—p
	U0W[1]=-5400;			//ЊЁ‘OЊг•ыЊьЌ¶Џ‘Ќћ—p
	U1W[1]=0;			//ЊЁ‰ЎЊг•ыЊьЌ¶Џ‘Ќћ—p
	U2W[1]=0;			//ЊЁѓ€Ѓ[ЊьЌ¶Џ‘Ќћ—p
	 EW[1]=0;			//•IЌ¶Џ‘Ќћ—p
	 HEADW=0;			//“Є‰с“]Џ‘Ќћ—p


	///////////////////////
	//// ѓ^ѓCѓ}‚МЏ‰Љъ‰» ////
	///////////////////////
	timer_init(TIMER, TIMER_MODE_TIMER32, 1000000);//Ћж‚иЉё‚¦‚ёЋьЉъ‚P•b‚ЙђЭ’и
	delay(500);//wait 500ms ‚±‚МЋћЉФ‘Т‚Ѕ‚И‚ў‚Жѓ_ѓЃ


	///////////////////
	//// PIOЏ‰Љъ‰» ////
	///////////////////
	pio_init(PIO_LED1, PIO_SET_OUT);// PIO(LED1)‚рЏo—Н‚ЙђЭ’и
	pio_init(PIO_LED2, PIO_SET_OUT);// PIO(LED2)‚рЏo—Н‚ЙђЭ’и
	pio_init(PIO_T1, PIO_SET_IN);	// PIO(T1)‚р“ь—Н‚ЙђЭ’и
	pio_init(PIO_T2, PIO_SET_OUT);	// PIO(T2)‚рЏo—Н‚ЙђЭ’и
	pio_init(PIO_T3, PIO_SET_IN);	// PIO(T3)‚р“ь—Н‚ЙђЭ’и
	pio_init(PIO_T4, PIO_SET_IN);	// PIO(T4)‚р“ь—Н‚ЙђЭ’и
	pio_init(PIO_T5, PIO_SET_OUT);	// PIO(T5)‚рЏo—Н‚ЙђЭ’и
	pio_init(PIO_T6, PIO_SET_OUT);	// PIO(T6)‚рЏo—Н‚ЙђЭ’и
	pio_init(PIO_SW1, PIO_SET_IN);	// PIO(SW1)‚р“ь—Н‚ЙђЭ’и
	pio_init(PIO_SW2, PIO_SET_IN);	// PIO(SW2)‚р“ь—Н‚ЙђЭ’и
	pio_write (PIO_LED2, HIGH);		// —О OFF


	/////////////////////////////
	//// ѓVѓЉѓAѓ‹ѓ|Ѓ[ѓgЏ‰Љъ‰» ////
	/////////////////////////////
	uart_init(UART_COM, UART, BR115200, 8, PARITY_NONE);
	i2c_init ( 400000, I2C_MASTER );

	sio_init (UART_SIO1, BR1250000);	// SIO‚МЏ‰Љъ‰»
	sio_init (UART_SIO2, BR1250000);	// SIO‚МЏ‰Љъ‰»
	sio_init (UART_SIO3, BR1250000);	// SIO‚МЏ‰Љъ‰»
	sio_init (UART_SIO4, BR1250000);	// SIO‚МЏ‰Љъ‰»


	/////////////////////////////
	//// ѓAѓiѓЌѓOѓ|Ѓ[ѓgЏ‰Љъ‰» ////
	/////////////////////////////
	ad_init(PIO_AD1, SWEEP);		//ѓAѓiѓЌѓOѓ|Ѓ[ѓg‚PђЭ’и
	ad_init(PIO_AD2, SWEEP);		//ѓAѓiѓЌѓOѓ|Ѓ[ѓg‚QђЭ’и
	dac_init();						//ѓAѓiѓЌѓOЏo—НђЭ’и


	////////////////////////
	//// ЌXђV“ъЋћ‚р•\Ћ¦ ////
	////////////////////////
	sprintf((char *)dsp,"Version: %s %s\r\n", __DATE__, __TIME__);
	printS((char *)dsp);


	/////////////////////////
	//// BMO055 ЉJЋnЏ€—ќ ////
	/////////////////////////
	//// Make sure we have the right device ////
	if(read8(0) !=0xA0 ){		// BNO055_ID
		delay(1000);			// hold on for boot
		if(read8(0) !=0xA0 ){	// BNO055_ID
			printS("*** NG1 ***\r\n");
			return false;		// still not? ok bail
		}
	}
	//// Switch to config mode (just in case since this is the default) ////
	write8(0X3D, 0);			//BNO055_OPR_MODE_ADDR  //OPERATION_MODE_CONFIG
	delay(30);
	//// Reset ////
	write8(0X3F, 0x20);			//BNO055_SYS_TRIGGER_ADDR
	delay(500);
	while (read8(0) != 0xA0){	//BNO055_CHIP_ID_ADDR  //BNO055_ID
		delay(1000);			// hold on for boot
		if(read8(0) != 0xA0 ){			// BNO055_ID
			printS("*** NG2 ***\r\n");
			return false;		// still not? ok bail
		}
	}
	delay(50);
	//// Set to normal power mode ////
	write8(0X3E, 0X00);			//BNO055_PWR_MODE_ADDR  //POWER_MODE_NORMAL
	delay(10);
	write8(0X07, 0);			//BNO055_PAGE_ID_ADDR
	write8(0X3F, 0);			//BNO055_SYS_TRIGGER_ADDR
	delay(10);
	//// Set the requested operating mode (see section 3.3) ////
	//  _mode = 0X0C;			//OPERATION_MODE_NDOF
	write8(0X3D, 0X0C);			//BNO055_OPR_MODE_ADDR  //mode
	delay(1000);
	//// Use external crystal for better accuracy ////
	write8(0X3D, 0);			//BNO055_OPR_MODE_ADDR  //OPERATION_MODE_CONFIG
	delay(50);
	write8(0X07, 0);			//BNO055_PAGE_ID_ADDR
	write8(0x0, 0x80);			//BNO055_SYS_TRIGGER_ADDR
	delay(10);
	//// Set the requested operating mode (see section 3.3) ////
	write8(0X3D, 0X0C);			//BNO055_OPR_MODE_ADDR  //modeback
	delay(50);
	printS("*** BNO055 INIT OK ***\r\n");


	/////////////////////////////
	//// ѓTЃ[ѓ{Њ»ЌЭЉp“x ////
	/////////////////////////////
	i=ics_set_pos ( UART_SIO2, 1, 0 );	//U0Rѓoѓ“ѓUѓC€К’u
	U0W[0]=-i+4735;
	i=ics_set_pos ( UART_SIO2, 2, 0 );	//U1R +2700
	U1W[0]=-i+10110;
	i=ics_set_pos ( UART_SIO2, 3, 0 );	//U2R
	U2W[0]=-i+7500;
	i=ics_set_pos ( UART_SIO2, 4, 0 );	//ER
	EW [0]=i-4800;
	i=ics_set_pos ( UART_SIO4, 1, 0 );	//U0Lѓoѓ“ѓUѓC€К’u
	U0W[1]=i-9320;
	i=ics_set_pos ( UART_SIO4, 2, 0 );	//U1L -2700
	U1W[1]=i-4850;
	i=ics_set_pos ( UART_SIO4, 3, 0 );	//U2L
	U2W[1]=i-7500;
	i=ics_set_pos ( UART_SIO4, 4, 0 );	//EL
	EW [1]=-i+10150;
	i=ics_set_pos ( UART_SIO1, 5, 0 );	//K2R
	K2W[0]=-i+7500;
	i=ics_set_pos ( UART_SIO1, 6, 0 );	//K1R
	K1W[0]=-i+7470;
	i=ics_set_pos ( UART_SIO1, 7, 0 );	//K0R
	K0W[0]=-i+7500;
	i=ics_set_pos ( UART_SIO1, 8, 0 );	//HR +1760
	HW [0]=-i+9260;
	i=ics_set_pos ( UART_SIO1, 9, 0 );	//A0R +350
	A0W[0]=i-7910;
	i=ics_set_pos ( UART_SIO1,10, 0 );	//A1R
	A1W[0]=i-7585;
	i=ics_set_pos ( UART_SIO3, 5, 0 );	//K2L
	K2W[1]=i-7500;
	i=ics_set_pos ( UART_SIO3, 6, 0 );	//K1L
	K1W[1]=i-7500;
	i=ics_set_pos ( UART_SIO3, 7, 0 );	//K0L
	K0W[1]=i-7500;
	i=ics_set_pos ( UART_SIO3, 8, 0 );	//HL -1760
	HW [1]=i-5740;
	i=ics_set_pos ( UART_SIO3, 9, 0 );	//A0L -350
	A0W[1]=-i+7100;
	i=ics_set_pos ( UART_SIO3,10, 0 );	//A1L
	A1W[1]=-i+7530;
	i=ics_set_pos ( UART_SIO4, 0, 0 );	//HEADL
	HEADW=i-7500;
	i=ics_set_pos ( UART_SIO2, 0, 0 );	//WESTR
	WESTW=i-7500;


	/////////////////////////////
	//// ѓTЃ[ѓ{ѓXѓgѓЊѓbѓ`ђЭ’и ////
	/////////////////////////////
//	ics_set_param ( UART_SIO1, 7,ICS_STRC_SC,250);	//K0R
//	ics_set_param ( UART_SIO3, 7,ICS_STRC_SC,250);	//K0L

	ics_set_param ( UART_SIO2, 1,ICS_STRC_SC,20);	//U0R
	ics_set_param ( UART_SIO4, 1,ICS_STRC_SC,20);	//U0L

	ics_set_param ( UART_SIO2, 2,ICS_STRC_SC,20);	//U1R
	ics_set_param ( UART_SIO4, 2,ICS_STRC_SC,20);	//U1L

	ics_set_param ( UART_SIO2, 4,ICS_STRC_SC,20);	//ER
	ics_set_param ( UART_SIO4, 4,ICS_STRC_SC,20);	//EL


	////////////////////////
	//// 10msѓ^ѓCѓ}ЉJЋn ////
	////////////////////////
	timer_write(TIMER,1000000);
	timer_start(TIMER);


	////////////////////
	//// •Пђ”Џ‰Љъ‰» ////
	////////////////////

	LEDct=0;	//LED“_“”ѓJѓEѓ“ѓ^

	tBak=0;
	pitchi=0;
	tNow=0;

	p_ofs=0;
	r_ofs=0;
	ir=0;
	ip=0;
	irb=0;
	ipb=0;

	kn=0;

	motCt=100;
	keyMode=0;
	cycle=10000;
	mode=710;
	pitch_gyrg=0.08;
	roll_gyrg=0.1;

	swMax=25;//22
	fhMax=35;
	walkCtLim=3;



//----------------------------------------------------------------------------------
	////////////////////////////////////////////////
	//////////////////  MAIN LOOP  /////////////////
	////////////////////////////////////////////////
top:
	//////////////////////
	//// 10ms‘Т‚їЏ€—ќ ////
	//////////////////////
pio_write (PIO_T2, HIGH);	//OFF(waitЋћЉФЉm”F)

	do{
		tNow=timer_read(TIMER);
	}while(tNow<cycle);

	if(tNow>cycle+10){
		sprintf( (char *)dsp,"************** %d \r\n",(int)tNow);
		printS ( (char *)dsp );
	}
	timer_start(TIMER);


	////////////////////
	//// ѓTЃ[ѓ{ђЭ’и ////
	////////////////////

	//// ЉЦђЯѓЉѓ~ѓbѓg ////
	if(K1W[0]> 800)K1W[0]	= 800;
	if(K1W[0]<-450)K1W[0]	=-450;
	if(K1W[1]> 800)K1W[1]	= 800;
	if(K1W[1]<-450)K1W[1]	=-450;
	if(A0W[0]> 3500)A0W[0]	= 3500;
	if(A0W[0]<-3500)A0W[0]	=-3500;
	if(A0W[1]> 3500)A0W[1]	= 3500;
	if(A0W[1]<-3500)A0W[1]	=-3500;
	if(A1W[0]> 420)A1W[0]	= 420;	//“Y•t•i‹И‰БЌH+ѓAѓ‹ѓ~ѓ\Ѓ[ѓ‹‚Е‚МЋАЊ±Њ‹‰К
	if(A1W[0]<-900)A1W[0]	=-900;	//“Y•t•i‹И‰БЌH+ѓAѓ‹ѓ~ѓ\Ѓ[ѓ‹‚Е‚МЋАЊ±Њ‹‰К
	if(A1W[1]> 420)A1W[1]	= 420;	//“Y•t•i‹И‰БЌH+ѓAѓ‹ѓ~ѓ\Ѓ[ѓ‹‚Е‚МЋАЊ±Њ‹‰К
	if(A1W[1]<-900)A1W[1]	=-900;	//“Y•t•i‹И‰БЌH+ѓAѓ‹ѓ~ѓ\Ѓ[ѓ‹‚Е‚МЋАЊ±Њ‹‰К

	//// ICSѓfЃ[ѓ^‘—ЋуђM ////
	//// ’Ќ€Уѓ|ѓWѓVѓ‡ѓ“ѓfЃ[ѓ^‚Є10500‚р’ґ‚¦‚й‚ЖѓTЃ[ѓ{‚Є‰ћ“љ‚µ‚И‚ўЃiЉФЊ‡”Ѕ‰ћЃj
	//// ’Ќ€Уѓ|ѓWѓVѓ‡ѓ“ѓfЃ[ѓ^‚Є 3600€И‰є‚Е‚Н”Ѕ‰ћ‚Н‚·‚й‚Є“®‚©‚И‚ў

	pio_write (PIO_T2, LOW );	//ON
	ics_set_pos    ( UART_SIO1, 5, 7560 -(K2W[0])-60 );	//K2R
	tNow=timer_read(TIMER);if(tNow>10010){sprintf( (char *)dsp,"K2R**** %d \r\n",(int)tNow);printS ( (char *)dsp );}

	pio_write (PIO_T2, HIGH);	//OFF
	ii=ics_set_pos ( UART_SIO2, 1, 4735 -(U0W[0]) );	//U0Rѓoѓ“ѓUѓC€К’u
	tNow=timer_read(TIMER);if(tNow>10010){sprintf( (char *)dsp,"U0R**** %d \r\n",(int)tNow);printS ( (char *)dsp );}
	U0R=-(ii-4735);

	pio_write (PIO_T2, LOW );	//ON
	ics_set_pos    ( UART_SIO3, 5, 7500 +(K2W[1])-90 );	//K2L
	tNow=timer_read(TIMER);if(tNow>10010){sprintf( (char *)dsp,"K2L**** %d \r\n",(int)tNow);printS ( (char *)dsp );}

	pio_write (PIO_T2, HIGH);	//OFF
	ii=ics_set_pos ( UART_SIO4, 1, 9320 +(U0W[1])-60 );	//U0Lѓoѓ“ѓUѓC€К’u
	tNow=timer_read(TIMER);if(tNow>10010){sprintf( (char *)dsp,"U0L**** %d \r\n",(int)tNow);printS ( (char *)dsp );}
	U0L=(ii-9230);

	pio_write (PIO_T2, LOW );	//ON
	ics_set_pos    ( UART_SIO1, 6, 7470 -(K1W[0])-30 );	//K1R
	tNow=timer_read(TIMER);if(tNow>10010){sprintf( (char *)dsp,"K1R**** %d \r\n",(int)tNow);printS ( (char *)dsp );}

	pio_write (PIO_T2, HIGH);	//OFF
	ii=ics_set_pos ( UART_SIO2, 2,10110 -(U1W[0]) );	//U1R +2700
	tNow=timer_read(TIMER);if(tNow>10010){sprintf( (char *)dsp,"U1R**** %d \r\n",(int)tNow);printS ( (char *)dsp );}
	U1R=-(ii-10110);

	pio_write (PIO_T2, LOW );	//ON
	ics_set_pos    ( UART_SIO3, 6, 7650 +(K1W[1])-90 );//K1L
	tNow=timer_read(TIMER);if(tNow>10010){sprintf( (char *)dsp,"K1L**** %d \r\n",(int)tNow);printS ( (char *)dsp );}

	pio_write (PIO_T2, HIGH);	//OFF
	ii=ics_set_pos ( UART_SIO4, 2, 4850 +(U1W[1]) );	//U1L -2700
	tNow=timer_read(TIMER);if(tNow>10010){sprintf( (char *)dsp,"U1L**** %d \r\n",(int)tNow);printS ( (char *)dsp );}
	U1L=ii-4850;


	pio_write (PIO_T2, LOW );	//ON
	ii=ics_set_pos ( UART_SIO1, 7, 7480 -(K0W[0])-30 );	//K0R
	tNow=timer_read(TIMER);if(tNow>10010){sprintf( (char *)dsp,"K0R**** %d \r\n",(int)tNow);printS ( (char *)dsp );}
	K0R=7510-ii;

	pio_write (PIO_T2, HIGH);	//OFF
	ics_set_pos    ( UART_SIO2, 3, 7500 -(U2W[0]) );	//U2R
	tNow=timer_read(TIMER);if(tNow>10010){sprintf( (char *)dsp,"U2R**** %d \r\n",(int)tNow);printS ( (char *)dsp );}

	pio_write (PIO_T2, LOW );	//ON
	ii=ics_set_pos ( UART_SIO3, 7, 7500 +(K0W[1]) );	//K0L
	tNow=timer_read(TIMER);if(tNow>10010){sprintf( (char *)dsp,"K0L**** %d \r\n",(int)tNow);printS ( (char *)dsp );}
	K0L=ii-7500;

	pio_write (PIO_T2, HIGH);	//OFF
	ics_set_pos    ( UART_SIO4, 3, 7500 +(U2W[1]) );	//U2L
	tNow=timer_read(TIMER);if(tNow>10010){sprintf( (char *)dsp,"U2L**** %d \r\n",(int)tNow);printS ( (char *)dsp );}

	pio_write (PIO_T2, LOW );	//ON
	ics_set_pos    ( UART_SIO1, 8, 9320 -(HW [0]) );	//HR +1760
	tNow=timer_read(TIMER);if(tNow>10010){sprintf( (char *)dsp,"HR**** %d \r\n",(int)tNow);printS ( (char *)dsp );}

	pio_write (PIO_T2, HIGH);	//OFF
	ics_set_pos    ( UART_SIO2, 4, 4800 +(EW [0]+i) );	//ER
	tNow=timer_read(TIMER);if(tNow>10010){sprintf( (char *)dsp,"ER**** %d \r\n",(int)tNow);printS ( (char *)dsp );}

	pio_write (PIO_T2, LOW );	//ON
	ics_set_pos    ( UART_SIO3, 8, 5770 +(HW [1])-120 );	//HL -1760
	tNow=timer_read(TIMER);if(tNow>10010){sprintf( (char *)dsp,"HL**** %d \r\n",(int)tNow);printS ( (char *)dsp );}

	pio_write (PIO_T2, HIGH);	//OFF
	ics_set_pos    ( UART_SIO4, 4,10150 -(EW [1]+i) );	//EL
	tNow=timer_read(TIMER);if(tNow>10010){sprintf( (char *)dsp,"EL**** %d \r\n",(int)tNow);printS ( (char *)dsp );}


	i=pitch_gyrg*pitch_gyr;
	pio_write (PIO_T2, LOW );	//ON
	ics_set_pos    ( UART_SIO1, 9, 7870-10 +(A0W[0]) + i+60 );	//A0R

	pio_write (PIO_T2, HIGH);	//OFF
	ics_set_pos    ( UART_SIO2, 0, 7500 +(WESTW ) );			//WESTR

	pio_write (PIO_T2, LOW );	//ON
	ics_set_pos    ( UART_SIO3, 9, 7100    -(A0W[1]) - i );	//A0L

	pio_write (PIO_T2, HIGH);	//OFF
	ics_set_pos    ( UART_SIO4, 0, 7500 +(HEADW ) );			//HEADL

	i=roll_gyrg*roll_gyr;
	pio_write (PIO_T2, LOW );	//ON
	ics_set_pos    ( UART_SIO1,10, 7470 +(A1W[0]) - i-30 );		//A1R

	pio_write (PIO_T2, HIGH);	//OFF
	ics_set_pos    ( UART_SIO3,10, 7560 -(A1W[1]) - i-60 );		//A1L


	/////////////////////
	//// IMU“ЗЋжЏ€—ќ ////
	/////////////////////
	//      +----------+
	//      |         *| RST   PITCH  ROLL  HEADING
	//  ADR |*        *| SCL
	//  INT |*        *| SDA     ^            /->
	//  PS1 |*        *| GND     |            |
	//  PS0 |*        *| 3VO     Y    Z-->    \-X
	//      |         *| VIN
	//      +----------+

	//Њ»ЏуЃ@2020 2/6    roll : ‰E‘¤ЊX‚«Ѓi2879Ѓj Ќ¶‘¤ЊX‚«Ѓi-2879Ѓjи‡’l2879
	//                  pitch: ‘O‘¤ЊX‚«Ѓi+Ѓj    Њг‘¤ЊX‚«Ѓi-Ѓj    •вђі’l20

	readLen(0X1A, 6);	//ђв‘ОЉp“x“ЗЌћ(deg‚М16”{•\Ћ¦)
	yaw	   = ((int16_t)ff[0]) | (((int16_t)ff[1]) << 8); //’ј—§‰E‰с“]‚Е +
	pitchs = ((int16_t)ff[2]) | (((int16_t)ff[3]) << 8); //’ј—§‘OЊX‚Е   -
	rolls  = ((int16_t)ff[4]) | (((int16_t)ff[5]) << 8); //’ј—§‰EЊXЋО‚Е +
	if(rolls>0)	rolls= 2879-rolls;
	else		rolls=-2879-rolls;

	pitchs -= p_ofs;	//•вђі
	rolls  -= r_ofs;	//•вђі

	pitch = (float)pitchs*(M_PI/(180.0*16.0));	//rad‚Й•ПЉ·
	roll  = (float)rolls *(M_PI/(180.0*16.0));	//rad‚Й•ПЉ·


	readLen(0X14, 6);	//Љp‘¬“x“ЗЌћ Ѓ¦roll‚Жyaw‚ЄЋжђа‚ЖЋАЌЫ‚Є‹t
	roll_gyr 	= ((int16_t)ff[0]) | (((int16_t)ff[1]) << 8);	//’ј—§‰EЊXЋО‚Е Ѓ{
	pitch_gyr	= ((int16_t)ff[2]) | (((int16_t)ff[3]) << 8);	//’ј—§‘OЊX‚Е   Ѓ|
	yaw_gyr		= ((int16_t)ff[4]) | (((int16_t)ff[5]) << 8);	//’ј—§‰E‰с“]‚Е Ѓ{


	pio_write (PIO_T2, LOW );	//ON


	keyCont();


	walk();


	///////////////
	//// “ЄLED ////
	///////////////
	++LEDct;
	if( LEDct > 100 )LEDct = -100;				//IMU Ready

	if( mode<=720 && LEDct > 10 )LEDct = -10;	//IMU not Ready

	if( LEDct > 0   ){
		dac_write (0xffff);
		pio_write (PIO_LED1, LOW );	//ON
	}
	else{
		dac_write (0);
		pio_write (PIO_LED1, HIGH);	//OFF
	}

	goto top;
}
