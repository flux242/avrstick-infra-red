#include <avr/pgmspace.h>

#include "lookup.h"
#include "hidkeycodes.h"

const unsigned char lookupTable[LTSIZE] PROGMEM = {
KEY_0,     /*0*/
KEY_1,     /*1*/
KEY_2,     /*2*/
KEY_3,     /*3*/
KEY_4,     /*4*/
KEY_5,     /*5*/
KEY_6,     /*6*/
KEY_7,     /*7*/
KEY_8,     /*8*/
KEY_9,     /*9*/
0,
0,
KEY_W,     /*PWR-TOGGL*/
KEY_M,     /*MUTE*/
0,
KEY_I,     /*HELP-INFO*/
KEY_RIGHT, /*VOLUP-UP*/
KEY_LEFT,  /*VOLDOWN-DOWN*/
KEY_F2,    /*MENU*/
KEY_F9,    /*TV-RADIO*/
0,
0,// TODO X,         /*LEFT*/
0,// TODO X,         /*RIGHT*/
KEY_RETURN, /*OK*/
0,
0,
0,
0,
0,
0,
0,
0,
KEY_UP,    /*CHAN_UP*/
KEY_DOWN,  /*CHAN_DOWN*/
KEY_F,     /*PREV*/
KEY_S,     /*INPUTAB*/
0,
0,
KEY_L,     /*SLEEP*/
0,
0,
KEY_P,     /*STOP*/
0,
KEY_F5,    /*RED*/
KEY_F6,    /*GREEN*/
KEY_F7,    /*YELLOW*/
KEY_F8,    /*BLUE*/
KEY_E,     /*GUIDE*/
0,
0,
0,
0,
0,
0,
KEY_O,     /*OPTION*/
0,
KEY_V,     /*EXTERNAL*/
0,
0,
0,
KEY_T,     /*TELETEXT*/
0,
0,
0,
};

