#include <avr/pgmspace.h>

#include "lookup.h"
#include "hidkeycodes.h"

#ifdef DECODE_PHILIPS
const unsigned char lookupTable[] PROGMEM = {
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
#elif defined DECODE_NEC
const unsigned char lookupTable[] PROGMEM = {
KEY_0,     /*0*/
1,
2,
3,
4,
5,
6,
7,
KEY_8,     /*8*/
9,
10,
11,
12,
13,
14,
15,
KEY_4,     /*4*/
17,
18,
19,
20,
21,
22,
23,
24,
25,
26,
27,
KEY_F9,    /*TV-RADIO*/
29,
30,
31,
KEY_2,     /*2*/
33,
34,
35,
KEY_W,     /*PWR-TOGGL*/
37,
38,
39,
KEY_RETURN, /*OK*/
41,
42,
43,
KEY_UP,    /*CHAN_UP*/
45,
46,
47,
KEY_6,     /*6*/
49,
50,
51,
KEY_F7,    /*FREEZE*/
53,
54,
55,
56,
57,
58,
59,
KEY_DOWN,  /*CHAN_DOWN*/
61,
62,
63,
KEY_1,     /*1*/
65,
66,
67,
68,
69,
70,
71,
KEY_9,     /*9*/
73,
74,
75,
KEY_P,     /*PLAY/STOP*/
77,
78,
79,
KEY_5,     /*5*/
81,
82,
83,
84,
85,
86,
87,
88,
89,
90,
91,
92,
93,
94,
95,
KEY_3,     /*3*/
97,
98,
99,
KEY_M,     /*MUTE*/
101,
102,
103,
104,
105,
106,
107,
KEY_RIGHT, /*VOLUP-UP*/
109,
110,
111,
KEY_7,     /*7*/
113,
114,
115,
116,
117,
118,
119,
120,
121,
122,
123,
KEY_LEFT,  /*VOLDOWN-DOWN*/
125,
126,
127
};
#endif

