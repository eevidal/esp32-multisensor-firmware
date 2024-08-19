#ifndef _APDS9960_H_
#define _APDS9960_H_

#include "error_module.h"
#include <stdint.h>
#include <stdbool.h>
#include "i2c_module.h"

typedef void *apds9960_t;

/** Acceptable parameters for setMode */
typedef enum
{
    APDS9960_POWER = 0,      /** Power on*/
    APDS9960_ALS = 1,        /**ALS Enable*/
    APDS9960_PROXIMIMTY = 2, /**Proximity Detect Enable*/
    APDS9960_WAIT = 3,       /**Wait Enable*/
    APDS9960_AINT = 4,       /**ALS Interrupt Enable.*/
    APDS9960_PINT = 5,       /**Proximity Interrupt Enable. */
    APDS9960_GESTURE = 6,    /**Gesture Enable.*/
    APDS9960_ALL = 7,        /**ALL Enable*/
} apds9960_mode_t;

/** Gesture wait time values  */
/** The GWTIME controls the amount of time in a low power mode between
gesture detection cycles. Address 0xA3<2:0>*/

typedef enum
{
    APDS9960_GWTIME_0MS = 0b00000000,
    APDS9960_GWTIME_2_8MS = 0b00000001,
    APDS9960_GWTIME_5_6MS = 0b00000010,
    APDS9960_GWTIME_8_4MS = 0b00000011,
    APDS9960_GWTIME_14_0MS = 0b00000100,
    APDS9960_GWTIME_22_4MS = 0b00000101,
    APDS9960_GWTIME_30_8MS = 0b00000110,
    APDS9960_GWTIME_39_2MS = 0b00000111
} apds9960_gwtime_t;

/** ADC gain settings */
typedef enum
{
    APDS9960_AGAIN_1X = 0x00,  /**< No gain */
    APDS9960_AGAIN_4X = 0x01,  /**< 2x gain */
    APDS9960_AGAIN_16X = 0x02, /**< 16x gain */
    APDS9960_AGAIN_64X = 0x03  /**< 64x gain */
} apds9960_again_t;

/** Gain settings. For Proximity ang Gesture */
typedef enum
{
    APDS9960_GAIN_1X = 0x00, /**< 1x gain */
    APDS9960_GAIN_2X = 0x01, /**< 2x gain */
    APDS9960_GAIN_4X = 0x02, /**< 4x gain */
    APDS9960_GAIN_8X = 0x03  /**< 8x gain */
} apds9960_gain_t;

/** LED Drive Strength */
typedef enum
{
    APDS9960_LDRIVE_100MA = 0x00, /**< 100mA */
    APDS9960_LDRIVE_50MA = 0x01,  /**< 50mA*/
    APDS9960_LDRIVE_25MA = 0x02,  /**< 25mA */
    APDS9960_LDRIVE_12_5MA = 0x03 /**< 12.5mA */
} apds9960_ldrive_t;

/** Pulse Len, for proximity and gesture engine*/
typedef enum
{
    APDS9960_LEN_4US = 0x00,  /**<  4uS   */
    APDS9960_LEN_8US = 0x40,  /**<  8uS  -> Default */
    APDS9960_LEN_16US = 0x80, /**<  16uS  */
    APDS9960_LEN_32US = 0xC0  /**<  32uS  */
} apds9960_pulse_len_t;

typedef enum
{
    APDS9960_PSAT_OFF = 0x01, /**<  Disable  -> Default */
    APDS9960_PSAT_ON = 0x81,  /**<  Enable */

} apds9960_psat_int_t;

typedef enum
{
    APDS9960_PCLEAR_OFF = 0x01, /**<  Disable  -> Default */
    APDS9960_PCLEAR_ON = 0x41,  /**< Enable */

} apds9960_pclear_int_t;

typedef enum
{
    APDS9960_PSAI_OFF = 0x00, /**<  Disable  -> Default */
    APDS9960_PSAI_ON = 0x10,  /**< Enable */

} apds9960_psai_t;

typedef enum
{
    APDS9960_PCMP_OFF = 0x00, /**<  Disable  -> Default */
    APDS9960_PCMP_ON = 0x20,  /**< Enable */

} apds9960_pcmp_t;

typedef enum
{
    APDS9960_PMASK_ALL = 0b00000000,   /**< All enable*/
    APDS9960_PMASK_R   = 0b00000100,   /**< R disable */
    APDS9960_PMASK_L   = 0b00001000,   /**< L disable*/
    APDS9960_PMASK_D   = 0b00010000,  /**< D disable*/
    APDS9960_PMASK_U   = 0b00100000,   /**< U disable */
    APDS9960_PMASK_LU  = 0b00101000,  /**< */
    APDS9960_PMASK_LD  = 0b00011000,  /**< L and D disable*/
    APDS9960_PMASK_LDU = 0b00111000, /**< */
    APDS9960_PMASK_RU  = 0b00100100,   /**< */
    APDS9960_PMASK_RD  = 0b00010100,   /**< */
    APDS9960_PMASK_RDU = 0b00101100,  /**< */
    APDS9960_PMASK_RL  = 0b00001100,   /**< */
    APDS9960_PMASK_RLU = 0b00001101,  /**< */
    APDS9960_PMASK_RLD = 0b00111000,  /**< */
    APDS9960_PMASK_NONE = 0b00001111, /**< All disable*/
} apds9960_pmask_t;

typedef enum
{
    APDS9960_GEXMSK_ALL = 0b00000000,   /**< All UDLR detector data will be included in sum*/
    APDS9960_GEXMSK_R   = 0b00000100,   /**< R detector data will not be included in sum */
    APDS9960_GEXMSK_L   = 0b00001000,   /**< L detector data will not be included in sum*/
    APDS9960_GEXMSK_D   = 0b00010000,  /**< D detector data will not be included in sum*/
    APDS9960_GEXMSK_U   = 0b00100000,   /**< U detector data will not be included in sum*/
    APDS9960_GEXMSK_LU  = 0b00101000,  /**< */
    APDS9960_GEXMSK_LD  = 0b00011000,  /**< L and D detector data will not be included in sum*/
    APDS9960_GEXMSK_LDU = 0b00111000, /**< */
    APDS9960_GEXMSK_RU  = 0b00100100,   /**< */
    APDS9960_GEXMSK_RD  = 0b00010100,   /**< */
    APDS9960_GEXMSK_RDU = 0b00101100,  /**< */
    APDS9960_GEXMSK_RL  = 0b00001100,   /**< */
    APDS9960_GEXMSK_RLU = 0b00001101,  /**< */
    APDS9960_GEXMSK_RLD = 0b00111000,  /**< */
    APDS9960_GEXMSK_NONE = 0b00001111, /**< All UDLR detector data will not be included in sum*/
} apds9960_gexmsk_t;

/**Gesture Exit Persistence. When a number of consecutive “gesture end” occurrences become
equal or greater to the GEPERS value, the Gesture state machine is exited.
FIELD VALUEPERSISTENCE

/** Gesture Exit Persistence */
typedef enum
{
    APDS9960_GEXPERS_1 = 0x00, /**< 1st 'gesture end' occurrence results in gesture state machine exit. */
    APDS9960_GEXPERS_2 = 0x01, /**< 2nd 'gesture end' */
    APDS9960_GEXPERS_4 = 0x02, /**< 4th 'gesture end' */
    APDS9960_GEXPERS_7 = 0x03  /**< 7th 'gesture end' */
} apds9960_gexpers_t;


/**Gesture FIFO Threshold. This value is compared with the FIFO Level (i.e. the number of UDLR
datasets) to generate an interrupt (if enabled).*/
/** Gesture FIFO Threshold. */
typedef enum
{
    APDS9960_GFIFOTH_1 = 0x00, /**< Interrupt is generated after 1 dataset is added to FIFO */
    APDS9960_GFIFOTH_4 = 0x40, /**< Interrupt is generated after 4 datasets are added to FIFO */
    APDS9960_GFIFOTH_8 = 0x80, /**< Interrupt is generated after 8 datasets are added to FIFO */
    APDS9960_GFIFOTH_16 = 0xC0  /**< Interrupt is generated after 16 datasets are added to FIFO */
} apds9960_gfifoth_t;

typedef enum
{
    APDS9960_LBOOST_100P = 0x01, /**<  100%  -> Default */
    APDS9960_LBOOST_150P = 0x11, /**<  150%   */
    APDS9960_LBOOST_200P = 0x21, /**<  200%   */
    APDS9960_LBOOST_300P = 0x31  /**<  300%   */
} apds9960_ledboost_t;


typedef enum
{
    APDS9960_GDIM_ALL = 0x00,  /**< Both pairs are active.  */
    APDS9960_GDIM_UD = 0x01,  /**< Only the UP-DOWN pair is active.  */
    APDS9960_AGDIM_LR = 0x02, /**< Only the LEFT-RIGHT pair is active. */
    APDS9960_AGAIN_UDLR = 0x03  /**< Both pairs are active.  */
} apds9960_gdims_t;

typedef enum
{
    NONE,
    LEFT,
    RIGHT,
    UP,
    DOWN,
    NEAR,
    FAR,
    ALL
} apds9960_gesture_t;

apds9960_t *apds9960_init(i2c_bus_t *i2c_bus);

err_t apds9960_delete(apds9960_t *sensor); // RNF.4

// err_t apds9960_setup(apds9960_t *sensor, *sensor_params);

/**
 * @brief Configure work mode
 *
 * @param sensor object handle of apds9960
 * @param mode one of apds9960_mode_t struct
 *
 * @return
 *     - OK Success
 *     - FAIL on Fail
 */
err_t apds9960_set_mode(apds9960_t *sensor, apds9960_mode_t mode);

/**
 * @brief Get work mode
 *
 * @param sensor object handle of apds9960
 * @param mode one pointer to a apds9960_mode_t struct
 * @return
 *     - OK Success
 *     - FAIL on Fail
 */
err_t apds9960_get_mode(apds9960_t *sensor, apds9960_mode_t *mode);

/**
 * @brief Sets the time in low power mode between gesture detections
 *
 * @param sensor
 * @param time
 * @return err_t
 */
err_t apds9960_set_wait_time(apds9960_t sensor, uint8_t time);

err_t apds9960_set_threshold(apds9960_t *sensor, uint8_t gesture, uint8_t threshold);
err_t apds9960_set_again(apds9960_t *sensor, uint8_t again);
err_t apds9960_set_ggain(apds9960_t *sensor, uint8_t ggain);
err_t apds9960_set_pgain(apds9960_t *sensor, uint8_t pgain);
err_t apds9960_get_again(apds9960_t *sensor, uint8_t *again);
err_t apds9960_get_ggain(apds9960_t *sensor, uint8_t *ggain);
err_t apds9960_get_pgain(apds9960_t *sensor, uint8_t *pgain);
err_t apds9960_proximity_set_threshold(apds9960_t *sensor, uint8_t threshold);
err_t apds9960_proximity_set_wait(apds9960_t *sensor, uint8_t time);
err_t apds9960_gesture_init(apds9960_t *sensor);
err_t apds9960_read_raw_data(uint8_t *gesture_data);

err_t apds9960_read_gesture(apds9960_t *sensor, apds9960_gesture_t *gesture);

err_t apds9960_set_gesture_offset(apds9960_t *sensor,
                                  uint8_t offset_up, uint8_t offset_down, uint8_t offset_left, uint8_t offset_right);
err_t apds9960_proximity_init(apds9960_t *sensor);
err_t apds9960_read_raw_proximity(apds9960_t *sensor, uint8_t *proximity);
err_t apds9960_read_proximity(apds9960_t *sensor, uint8_t *proximity);
err_t apds9960_get_color_data(apds9960_t sensor, uint16_t *r,
                              uint16_t *g, uint16_t *b, uint16_t *c);
err_t apds9960_enable_color_engine(apds9960_t sensor, bool en);
err_t apds9960_get_ambient_light(apds9960_t *sensor, uint16_t *l);

// Private Functions
err_t apds9960_read_byte(apds9960_t sensor, uint8_t addr, uint8_t *data);
err_t apds9960_read(apds9960_t sensor, uint8_t addr, uint8_t *buf, uint8_t len);
err_t apds9960_write_byte(apds9960_t sensor, uint8_t addr, uint8_t data);
err_t apds9960_write(apds9960_t sensor, uint8_t addr, uint8_t *buf, uint8_t len);
err_t process_gesture(apds9960_t sensor, uint8_t data, apds9960_gesture_t *gesture);
err_t decode_gesture(apds9960_t sensor, uint8_t data, apds9960_gesture_t *gesture);
err_t apds9960_calculate_lux(apds9960_t sensor, uint16_t r,
                             uint16_t g, uint16_t b, uint16_t *l);

#endif