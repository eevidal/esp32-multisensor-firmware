/* APDS-9960 register addresses */
#define APDS9960_I2C_ADDRESS    (0x39) 

#define  ID             0x92

// Proximity register
#define  PILT           0x89
#define  PIHT           0x8B
#define  PERS           0x8C
#define  PPULSE         0x8E
#define  CONTROL        0x8F
#define  CONFIG2        0x90
#define  STATUS         0x93
#define  PDATA          0x9C
#define  POFFSET_UR     0x9D
#define  POFFSET_DL     0x9E
#define  CONFIG3        0x9F
#define  PICLEAR        0xE5
#define  AICLEAR        0xE7


//registers color

#define  ATIME          0x82
#define  WTIME          0x83
#define  AILTL          0x84
#define  AILTH          0x85
#define  AIHTL          0x86
#define  AIHTH          0x87
#define  CONFIG1        0x8D
#define  AGAIN          0x8F //<1:0>
#define  CDATAL         0x94
#define  CDATAH         0x95
#define  RDATAL         0x96
#define  RDATAH         0x97
#define  GDATAL         0x98
#define  GDATAH         0x99
#define  BDATAL         0x9A
#define  BDATAH         0x9B
#define  CICLEAR        0xE5

// Gesture control
#define  GPENTH         0xA0
#define  GEXTH          0xA1
#define  GCONF1         0xA2
#define  GCONF2         0xA3
#define  GOFFSET_U      0xA4
#define  GOFFSET_D      0xA5
#define  GOFFSET_L      0xA7
#define  GOFFSET_R      0xA9
#define  GPULSE         0xA6
#define  GCONF3         0xAA
#define  GCONF4         0xAB
#define  GFLVL          0xAE
#define  GSTATUS        0xAF
#define  IFORCE         0xE4
#define  PICLEAR        0xE5

#define  GFIFO_U        0xFC
#define  GFIFO_D        0xFD
#define  GFIFO_L        0xFE
#define  GFIFO_R        0xFF