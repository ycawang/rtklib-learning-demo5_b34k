/*------------------------------------------------------------------------------
* sbas.c : sbas functions
*
*          Copyright (C) 2007-2020 by T.TAKASU, All rights reserved.
*
* option : -DRRCENA  enable rrc correction
*          
* references :
*     [1] RTCA/DO-229C, Minimum operational performanc standards for global
*         positioning system/wide area augmentation system airborne equipment,
*         RTCA inc, November 28, 2001
*     [2] IS-QZSS v.1.1, Quasi-Zenith Satellite System Navigation Service
*         Interface Specification for QZSS, Japan Aerospace Exploration Agency,
*         July 31, 2009
*
* version : $Revision: 1.1 $ $Date: 2008/07/17 21:48:06 $
* history : 2007/10/14 1.0  new
*           2009/01/24 1.1  modify sbspntpos() api
*                           improve fast/ion correction update
*           2009/04/08 1.2  move function crc24q() to rcvlog.c
*                           support glonass, galileo and qzss
*           2009/06/08 1.3  modify sbsupdatestat()
*                           delete sbssatpos()
*           2009/12/12 1.4  support glonass
*           2010/01/22 1.5  support ems (egnos message service) format
*           2010/06/10 1.6  added api:
*                               sbssatcorr(),sbstropcorr(),sbsioncorr(),
*                               sbsupdatecorr()
*                           changed api:
*                               sbsreadmsgt(),sbsreadmsg()
*                           deleted api:
*                               sbspntpos(),sbsupdatestat()
*           2010/08/16 1.7  not reject udre==14 or give==15 correction message
*                           (2.4.0_p4)
*           2011/01/15 1.8  use api ionppp()
*                           add prn mask of qzss for qzss L1SAIF
*           2016/07/29 1.9  crc24q() -> rtk_crc24q()
*           2020/11/30 1.10 use integer types in stdint.h
*-----------------------------------------------------------------------------*/
#include "rtklib.h"

/* constants -----------------------------------------------------------------*/

#define WEEKOFFSET  1024        /* gps week offset for NovAtel OEM-3 */

/* sbas igp definition -------------------------------------------------------*/
static const int16_t
x1[]={-75,-65,-55,-50,-45,-40,-35,-30,-25,-20,-15,-10,- 5,  0,  5, 10, 15, 20,
       25, 30, 35, 40, 45, 50, 55, 65, 75, 85},
x2[]={-55,-50,-45,-40,-35,-30,-25,-20,-15,-10, -5,  0,  5, 10, 15, 20, 25, 30,
       35, 40, 45, 50, 55},
x3[]={-75,-65,-55,-50,-45,-40,-35,-30,-25,-20,-15,-10,- 5,  0,  5, 10, 15, 20,
       25, 30, 35, 40, 45, 50, 55, 65, 75},
x4[]={-85,-75,-65,-55,-50,-45,-40,-35,-30,-25,-20,-15,-10,- 5,  0,  5, 10, 15,
       20, 25, 30, 35, 40, 45, 50, 55, 65, 75},
x5[]={-180,-175,-170,-165,-160,-155,-150,-145,-140,-135,-130,-125,-120,-115,
      -110,-105,-100,- 95,- 90,- 85,- 80,- 75,- 70,- 65,- 60,- 55,- 50,- 45,
      - 40,- 35,- 30,- 25,- 20,- 15,- 10,-  5,   0,   5,  10,  15,  20,  25,
        30,  35,  40,  45,  50,  55,  60,  65,  70,  75,  80,  85,  90,  95,
       100, 105, 110, 115, 120, 125, 130, 135, 140, 145, 150, 155, 160, 165,
       170, 175},
x6[]={-180,-170,-160,-150,-140,-130,-120,-110,-100,- 90,- 80,- 70,- 60,- 50,
      - 40,- 30,- 20,- 10,   0,  10,  20,  30,  40,  50,  60,  70,  80,  90,
       100, 110, 120, 130, 140, 150, 160, 170},
x7[]={-180,-150,-120,- 90,- 60,- 30,   0,  30,  60,  90, 120, 150},
x8[]={-170,-140,-110,- 80,- 50,- 20,  10,  40,  70, 100, 130, 160};

EXPORT const sbsigpband_t igpband1[9][8]={ /* band 0-8 */
    {{-180,x1,  1, 28},{-175,x2, 29, 51},{-170,x3, 52, 78},{-165,x2, 79,101},
     {-160,x3,102,128},{-155,x2,129,151},{-150,x3,152,178},{-145,x2,179,201}},
    {{-140,x4,  1, 28},{-135,x2, 29, 51},{-130,x3, 52, 78},{-125,x2, 79,101},
     {-120,x3,102,128},{-115,x2,129,151},{-110,x3,152,178},{-105,x2,179,201}},
    {{-100,x3,  1, 27},{- 95,x2, 28, 50},{- 90,x1, 51, 78},{- 85,x2, 79,101},
     {- 80,x3,102,128},{- 75,x2,129,151},{- 70,x3,152,178},{- 65,x2,179,201}},
    {{- 60,x3,  1, 27},{- 55,x2, 28, 50},{- 50,x4, 51, 78},{- 45,x2, 79,101},
     {- 40,x3,102,128},{- 35,x2,129,151},{- 30,x3,152,178},{- 25,x2,179,201}},
    {{- 20,x3,  1, 27},{- 15,x2, 28, 50},{- 10,x3, 51, 77},{-  5,x2, 78,100},
     {   0,x1,101,128},{   5,x2,129,151},{  10,x3,152,178},{  15,x2,179,201}},
    {{  20,x3,  1, 27},{  25,x2, 28, 50},{  30,x3, 51, 77},{  35,x2, 78,100},
     {  40,x4,101,128},{  45,x2,129,151},{  50,x3,152,178},{  55,x2,179,201}},
    {{  60,x3,  1, 27},{  65,x2, 28, 50},{  70,x3, 51, 77},{  75,x2, 78,100},
     {  80,x3,101,127},{  85,x2,128,150},{  90,x1,151,178},{  95,x2,179,201}},
    {{ 100,x3,  1, 27},{ 105,x2, 28, 50},{ 110,x3, 51, 77},{ 115,x2, 78,100},
     { 120,x3,101,127},{ 125,x2,128,150},{ 130,x4,151,178},{ 135,x2,179,201}},
    {{ 140,x3,  1, 27},{ 145,x2, 28, 50},{ 150,x3, 51, 77},{ 155,x2, 78,100},
     { 160,x3,101,127},{ 165,x2,128,150},{ 170,x3,151,177},{ 175,x2,178,200}}
};
EXPORT const sbsigpband_t igpband2[2][5]={ /* band 9-10 */
    {{  60,x5,  1, 72},{  65,x6, 73,108},{  70,x6,109,144},{  75,x6,145,180},
     {  85,x7,181,192}},
    {{- 60,x5,  1, 72},{- 65,x6, 73,108},{- 70,x6,109,144},{- 75,x6,145,180},
     {- 85,x8,181,192}}
};

/* decode sbas message ---------------------------------------------------------
* decode sbas message frame words and check crc
* args   : gtime_t time     I   reception time
*          int    prn       I   sbas satellite prn number
*          uint32_t *word   I   message frame words (24bit x 10)
*          sbsmsg_t *sbsmsg O   sbas message
* return : status (1:ok,0:crc error)
*-----------------------------------------------------------------------------*/
extern int sbsdecodemsg(gtime_t time, int prn, const uint32_t *words,
                        sbsmsg_t *sbsmsg)
{
    int i,j;
    uint8_t f[29];
    double tow;
    
    trace(5,"sbsdecodemsg: prn=%d\n",prn);
    
    if (time.time==0) return 0;
    tow=time2gpst(time,&sbsmsg->week);
    sbsmsg->tow=(int)(tow+DTTOL);
    sbsmsg->prn=prn;
    for (i=0;i<7;i++) for (j=0;j<4;j++) {
        sbsmsg->msg[i*4+j]=(uint8_t)(words[i]>>((3-j)*8));
    }
    sbsmsg->msg[28]=(uint8_t)(words[7]>>18)&0xC0;
    for (i=28;i>0;i--) f[i]=(sbsmsg->msg[i]>>6)+(sbsmsg->msg[i-1]<<2);
    f[0]=sbsmsg->msg[0]>>6;
    
    return rtk_crc24q(f,29)==(words[7]&0xFFFFFF); /* check crc */
}
