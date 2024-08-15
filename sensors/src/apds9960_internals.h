/* APDS-9960 register addresses */
#define APDS9960_I2C_ADDRESS    (0x39) 

#define  MODE_ENABLE    0x80
#define  ID             0x92

// Proximity register
#define  PILT           0x89
#define  PIHT           0x8B
#define  PERS           0x8C
#define  CONFIG1        0x8D  //proximity and color/ACL
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

/* Bit fields */
#define PON                       0b00000001
#define AEN                       0b00000010
#define PEN                       0b00000100
#define WEN                       0b00001000
#define AIEN                      0b00010000
#define PIEN                      0b00100000
#define GEN                       0b01000000
#define GVALID                    0b00000001

/* On/Off definitions */
#define APDS9960_OFF                       0

/* Default values */
#define DEFAULT_ATIME           219     // 103ms
#define DEFAULT_WTIME           246     // 27ms
#define DEFAULT_PROX_PPULSE     0x87    // 16us, 8 pulses
#define DEFAULT_GESTURE_PPULSE  0x89    // 16us, 10 pulses
#define DEFAULT_POFFSET_UR      0       // 0 offset
#define DEFAULT_POFFSET_DL      0       // 0 offset      
#define DEFAULT_CONFIG1         0x40    // No 12x wait (WTIME) factor
#define DEFAULT_LDRIVE          LED_DRIVE_100MA
#define DEFAULT_PGAIN           PGAIN_4X
#define DEFAULT_AGAIN           AGAIN_4X
#define DEFAULT_PILT            0       // Low proximity threshold
#define DEFAULT_PIHT            50      // High proximity threshold
#define DEFAULT_AILT            0xFFFF  // Force interrupt for calibration
#define DEFAULT_AIHT            0
#define DEFAULT_PERS            0x11    // 2 consecutive prox or ALS for int.
#define DEFAULT_CONFIG2         0x01    // No saturation interrupts or LED boost  
#define DEFAULT_CONFIG3         0       // Enable all photodiodes, no SAI
#define DEFAULT_GPENTH          40      // Threshold for entering gesture mode
#define DEFAULT_GEXTH           30      // Threshold for exiting gesture mode    
#define DEFAULT_GCONF1          0x40    // 4 gesture events for int., 1 for exit
#define DEFAULT_GGAIN           GGAIN_4X
#define DEFAULT_GLDRIVE         LED_DRIVE_100MA
#define DEFAULT_GWTIME          GWTIME_2_8MS
#define DEFAULT_GOFFSET         0       // No offset scaling for gesture mode
#define DEFAULT_GPULSE          0xC9    // 32us, 10 pulses
#define DEFAULT_GCONF3          0       // All photodiodes active during gesture
#define DEFAULT_GIEN            0       // Disable gesture interrupts

