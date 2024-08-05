#include "apds9960.h"

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

typedef struct{
    i2c_bus_dev_t i2c_dev;
    uint8_t dev_addr;
    uint32_t timeout;
    apds9960_control_t _control_t; /*< config control register>*/
    apds9960_enable_t _enable_t;   /*< config enable register>*/
    apds9960_config1_t _config1_t; /*< config config1 register>*/
    apds9960_config2_t _config2_t; /*< config config2 register>*/
    apds9960_config3_t _config3_t; /*< config config3 register>*/
    apds9960_gconf1_t _gconf1_t;   /*< config gconfig1 register>*/
    apds9960_gconf2_t _gconf2_t;   /*< config gconfig2 register>*/
    apds9960_gconf3_t _gconf3_t;   /*< config gconfig3 register>*/
    apds9960_gconf4_t _gconf4_t;   /*< config gconfig4 register>*/
    apds9960_status_t _status_t;   /*< config status register>*/
    apds9960_gstatus_t _gstatus_t; /*< config gstatus register>*/
    apds9960_propulse_t _ppulse_t; /*< config pro pulse register>*/
    apds9960_gespulse_t _gpulse_t; /*< config ges pulse register>*/
    apds9960_pers_t _pers_t;       /*< config pers register>*/
    uint8_t gest_cnt;              /*< counter of gesture >*/
    uint8_t up_cnt;                /*< counter of up gesture >*/
    uint8_t down_cnt;              /*< counter of down gesture >*/
    uint8_t left_cnt;              /*< counter of left gesture >*/
    uint8_t right_cnt;             /*< counter of right gesture >*/

}apds9960_dev_t;

apds9960_t *sensor apds9960_init(i2c_bus_t *i2c_params){


};
err_t apds9960_delete(apds9960_t *sensor); // RNF.4
err_t apds9960_setup(apds9960_t *sensor, *sensor_params);
err_t apds9960_proximity_set_mode(apds9960_t *sensor, uint8_t mode);
err_t apds9960_proximity_get_mode(apds9960_t *sensor, uint8_t* mode);
err_t apds9960_set_wait_time(apds9960_t sensor, uint8_t time);
err_t apds9960_set_threshold(apds9960_t *sensor, uint8_t gesture, uint8_t threshold);
err_t apds9960_set_again(apds9960_t *sensor, uint8_t again);
err_t apds9960_set_ggain(apds9960_t *sensor, uint8_t ggain);
err_t apds9960_set_pgain(apds9960_t *sensor, uint8_t pgain);
err_t apds9960_get_again(apds9960_t *sensor, uint8_t *again);
err_t apds9960_get_ggain(apds9960_t *sensor, uint8_t *ggain);
err_t apds9960_get_pgain(apds9960_t *sensor, uint8_t *pgain);
err_t apds9960_proximity_set_threshold(apds9960_t *sensor,uint8_t threshold);
err_t apds9960_proximity_set_wait(apds9960_t *sensor, uint8_t time);
err_t apds9960_gesture_init(apds9960_t *sensor);
err_t apds9960_read_raw_data(uint8_t *gesture_data);
err_t apds9960_read_gesture(apds9960_t *sensor, gesture_t *gesture);
err_t apds9960_set_gesture_offset(apds9960_t *sensor,
uint8_t offset_up, uint8_t offset_down, uint8_t offset_left, uint8_t offset_right);
err_t apds9960_proximity_init(apds9960_t *sensor);
err_t apds9960_read_raw_proximity(apds9960_t *sensor, uint8_t *proximity);
err_t apds9960_read_proximity(apds9960_t *sensor, uint8_t *proximity);
err_t apds9960_get_color_data(apds9960_t sensor, uint16_t *r,
uint16_t *g, uint16_t *b, uint16_t *c);
err_t apds9960_enable_color_engine(apds9960_t sensor, bool en);
err_t apds9960_get_ambient_light(apds9960_t *sensor, uint16_t *l);