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
Gesture Exit Persistence */

typedef enum
{
    APDS9960_GEXPERS_1 = 0x00, /**< 1st 'gesture end' occurrence results in gesture state machine exit. */
    APDS9960_GEXPERS_2 = 0x01, /**< 2nd 'gesture end' */
    APDS9960_GEXPERS_4 = 0x02, /**< 4th 'gesture end' */
    APDS9960_GEXPERS_7 = 0x03  /**< 7th 'gesture end' */
} apds9960_gexpers_t;


/**Gesture FIFO Threshold. This value is compared with the FIFO Level (i.e. the number of UDLR
datasets) to generate an interrupt (if enabled).*/
typedef enum
{
    APDS9960_GFIFOTH_1  = 0x00, /**< Interrupt is generated after 1 dataset is added to FIFO */
    APDS9960_GFIFOTH_4  = 0x40, /**< Interrupt is generated after 4 datasets are added to FIFO */
    APDS9960_GFIFOTH_8  = 0x80, /**< Interrupt is generated after 8 datasets are added to FIFO */
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
    APDS9960_GDIM_ALL   = 0x00,  /**< Both pairs are active.  */
    APDS9960_GDIM_UD    = 0x01,  /**< Only the UP-DOWN pair is active.  */
    APDS9960_AGDIM_LR   = 0x02, /**< Only the LEFT-RIGHT pair is active. */
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


/**
 * Initializes an APDS9960 sensor for use with an I2C bus.
 *
 * @param i2c_bus Pointer to the I2C bus to use.
 * @return A pointer to the initialized APDS9960 sensor, or NULL on error.
 */
apds9960_t apds9960_init(i2c_bus_t *i2c_bus);

/**
 * Deallocates memory and resources associated with an APDS9960 sensor.
 *
 * @param sensor Pointer to the APDS9960 sensor to be deleted.
 * @return E_OK on success, error code otherwise.
 */
err_t apds9960_delete(apds9960_t sensor); // RNF.4


err_t apds9960_get_id(apds9960_t sensor,  uint8_t *const data);
/**
 * Enables the specified engine or mode on the APDS9960 sensor.
 *
 * @param sensor Pointer to the APDS9960 sensor device.
 * @param mode The engine or mode to enable.
 * @return E_OK on success, error code otherwise.
 */
err_t apds9960_enable_engine(apds9960_t sensor, apds9960_mode_t mode);


/**
 * Disables the specified engine or mode on the APDS9960 sensor.
 *
 * @param sensor Pointer to the APDS9960 sensor device.
 * @param mode The engine or mode to disable.
 * @return E_OK on success, error code otherwise.
 */
err_t apds9960_disable_engine(apds9960_t sensor, apds9960_mode_t mode);

/**
 * @brief Sets the time in low power mode between gesture detections
 *
 * @param sensor
 * @param time
 * @return err_t
 */



err_t apds9960_set_wait_time(apds9960_t sensor, uint8_t time);

/**
 * Sets the low and high threshold values for the ALS interrupt.
 *
 * @param sensor Pointer to the APDS9960 sensor device.
 * @param low The low threshold value for the ALS interrupt.
 * @param high The high threshold value for the ALS interrupt.
 * @return E_OK on success, error code otherwise.
 */
err_t apds9960_set_als_threshold(apds9960_t sensor, uint16_t low, uint16_t high);

/**
 * Sets the proximity threshold and persistence values for the APDS9960 sensor.
 *
 * @param sensor Pointer to the APDS9960 sensor device.
 * @param threshold The proximity threshold value.
 * @param persistence The proximity interrupt persistence level (0-7).
 * @return E_OK on success, error code otherwise.
 */
err_t apds9960_set_proximity_threshold(apds9960_t sensor, uint8_t t_min,  uint8_t t_max, uint8_t persistence);

/**
 * Sets the gesture threshold values for the APDS9960 sensor.
 *
 * @param sensor Pointer to the APDS9960 sensor device.
 * @param gpenth The gesture enter threshold value.
 * @param gexth The gesture exit threshold value.
 * @return E_OK on success, error code otherwise.
 */
err_t apds9960_set_gesture_threshold(apds9960_t sensor, uint8_t  gpenth, uint8_t  gexth);

/**
 * Sets the gain for the APDS9960 sensor.
 *
 * @param sensor Pointer to the APDS9960 sensor device.
 * @param again The desired gain value.
 * @return E_OK on success, error code otherwise.
 */
err_t apds9960_set_again(apds9960_t sensor, apds9960_again_t again);

/**
 * Sets the gesture gain for the APDS9960 sensor.
 *
 * @param sensor Pointer to the APDS9960 sensor device.
 * @param ggain The desired gesture gain value.
 * @return E_OK on success, error code otherwise.
 */
err_t apds9960_set_ggain(apds9960_t sensor, apds9960_gain_t ggain);

/**
 * Sets the proximity gain for the APDS9960 sensor.
 *
 * @param sensor Pointer to the APDS9960 sensor device.
 * @param pgain The desired proximity gain value.
 * @return E_OK on success, error code otherwise.
 */
err_t apds9960_set_pgain(apds9960_t sensor, apds9960_gain_t pgain);

/**
 * Sets the gesture LED drive current for the APDS9960 sensor.
 *
 * @param sensor Pointer to the APDS9960 sensor device.
 * @param gldrive The desired gesture LED drive current.
 * @return E_OK on success, error code otherwise.
 */
err_t apds9960_set_gldrive(apds9960_t sensor, apds9960_ldrive_t gldrive);


/**
 * Sets the gesture wait time for the APDS9960 sensor.
 *
 * @param sensor Pointer to the APDS9960 sensor device.
 * @param gwtime The desired gesture wait time.
 * @return E_OK on success, error code otherwise.
 */
err_t apds9960_set_gwtime(apds9960_t sensor, apds9960_gwtime_t gwtime);

/**
 * Sets the wait time for the APDS9960 sensor.
 *
 * @param sensor Pointer to the APDS9960 sensor device.
 * @param wtime The desired wait time value.
 * @return E_OK on success, error code otherwise.
 */
err_t apds9960_set_wtime(apds9960_t sensor, uint8_t wtime);

/**
 * Sets the integration time for the APDS9960 sensor.
 *
 * @param sensor Pointer to the APDS9960 sensor device.
 * @param atime The desired integration time value.
 * @return E_OK on success, error code otherwise.
 */
err_t apds9960_set_atime(apds9960_t sensor,  uint8_t atime);

/**
 * Enables the long wait time mode for the APDS9960 sensor.
 *
 * @param sensor Pointer to the APDS9960 sensor device.
 * @return E_OK on success, error code otherwise.
 */
err_t apds9960_enable_wlong(apds9960_t sensor);

/**
 * Disables the long wait time mode for the APDS9960 sensor.
 *
 * @param sensor Pointer to the APDS9960 sensor device.
 * @return E_OK on success, error code otherwise.
 */
err_t apds9960_disable_wlong(apds9960_t sensor);

/**
 * Sets the proximity pulse length and count for the APDS9960 sensor.
 *
 * @param sensor Pointer to the APDS9960 sensor device.
 * @param len The desired proximity pulse length.
 * @param pulses The number of proximity pulses.
 * @return E_OK on success, error code otherwise.
 */
err_t apds9960_set_proximity_pulse(apds9960_t sensor, apds9960_pulse_len_t len,  uint8_t pulses);

/**
 * Sets the proximity saturation interrupt mode for the APDS9960 sensor.
 *
 * @param sensor Pointer to the APDS9960 sensor device.
 * @param sat The desired proximity saturation interrupt mode.
 * @return E_OK on success, error code otherwise.
 */
err_t apds9960_set_proximity_sat_int_(apds9960_t sensor, apds9960_psat_int_t sat);

/**
 * Sets the proximity clear interrupt mode for the APDS9960 sensor.
 *
 * @param sensor Pointer to the APDS9960 sensor device.
 * @param clear The desired proximity clear interrupt mode.
 * @return E_OK on success, error code otherwise.
 */
err_t apds9960_set_proximity_clear_int_(apds9960_t sensor, apds9960_pclear_int_t clear);

/**
 * Sets the LED boost mode for the APDS9960 sensor.
 *
 * @param sensor Pointer to the APDS9960 sensor device.
 * @param boost The desired LED boost mode.
 * @return E_OK on success, error code otherwise.
 */
err_t apds9960_set_ledboost(apds9960_t sensor, apds9960_ledboost_t boost);

/**
 * Sets the upper right proximity offset for the APDS9960 sensor.
 *
 * @param sensor Pointer to the APDS9960 sensor device.
 * @param offset The desired upper right proximity offset value.
 * @return E_OK on success, error code otherwise.
 */
err_t apds9960_set_poffset_ur(apds9960_t sensor,const uint8_t *const offset);

/**
 * Sets the proximity comparator mode for the APDS9960 sensor.
 *
 * @param sensor Pointer to the APDS9960 sensor device.
 * @param pcmd_val The desired proximity comparator mode.
 * @return E_OK on success, error code otherwise.
 */
err_t apds9960_set_proximity_pcmp(apds9960_t sensor, apds9960_pcmp_t pcmd_val);

/**
 * Sets the proximity saturation interrupt mode for the APDS9960 sensor.
 *
 * @param sensor Pointer to the APDS9960 sensor device.
 * @param sai_val The desired proximity saturation interrupt mode.
 * @return E_OK on success, error code otherwise.
 */
err_t apds9960_set_proximity_sai(apds9960_t sensor, apds9960_psai_t sai_val);

/**
 * Sets the proximity mask for the APDS9960 sensor.
 *
 * @param sensor Pointer to the APDS9960 sensor device.
 * @param mask The desired proximity mask.
 * @return E_OK on success, error code otherwise.
 */
err_t apds9960_set_proximity_mask(apds9960_t sensor, apds9960_pmask_t mask);

/**
 * Sets the gesture exit mask for the APDS9960 sensor.
 *
 * @param sensor Pointer to the APDS9960 sensor device.
 * @param mask The desired gesture exit mask.
 * @return E_OK on success, error code otherwise.
 */
err_t apds9960_set_gestrure_gexmsk(apds9960_t sensor, apds9960_gexmsk_t mask);

/**
 * Sets the gesture FIFO threshold for the APDS9960 sensor.
 *
 * @param sensor Pointer to the APDS9960 sensor device.
 * @param fifoth The desired gesture FIFO threshold.
 * @return E_OK on success, error code otherwise.
 */
err_t apds9960_set_gestrure_fifoth(apds9960_t sensor, apds9960_gfifoth_t fifoth);

/**
 * Sets the gesture exit persistence for the APDS9960 sensor.
 *
 * @param sensor Pointer to the APDS9960 sensor device.
 * @param gexper The desired gesture exit persistence.
 * @return E_OK on success, error code otherwise.
 */
err_t apds9960_set_gestrure_gexpers(apds9960_t sensor, apds9960_gexpers_t gexper);

/**
 * Sets the gesture offset values for the APDS9960 sensor.
 *
 * @param sensor Pointer to the APDS9960 sensor device.
 * @param offset_up The offset value for the up gesture.
 * @param offset_down The offset value for the down gesture.
 * @param offset_left The offset value for the left gesture.
 * @param offset_right The offset value for the right gesture.
 * @return E_OK on success, error code otherwise.
 */
err_t apds9960_set_gestrure_offsets(apds9960_t sensor,const uint8_t *const offset_up, const uint8_t *const offset_down, const uint8_t *const offset_left, const uint8_t *const offset_right);

/**
 * Sets the gesture pulse length and count for the APDS9960 sensor.
 *
 * @param sensor Pointer to the APDS9960 sensor device.
 * @param len The desired gesture pulse length.
 * @param pulses The number of gesture pulses.
 * @return E_OK on success, error code otherwise.
 */
err_t apds9960_set_gesture_pulse(apds9960_t sensor, apds9960_pulse_len_t len, uint8_t pulses);

/**
 * Sets the gesture dimension mode for the APDS9960 sensor.
 *
 * @param sensor Pointer to the APDS9960 sensor device.
 * @param gdim The desired gesture dimension mode.
 * @return E_OK on success, error code otherwise.
 */
err_t apds9960_set_gesture_gdims(apds9960_t sensor, apds9960_gdims_t gdim);

/**
 * Enables the gesture interrupt for the APDS9960 sensor.
 *
 * @param sensor Pointer to the APDS9960 sensor device.
 * @return E_OK on success, error code otherwise.
 */
err_t apds9960_set_gesture_int_on(apds9960_t sensor);

/**
 * Disables the gesture interrupt for the APDS9960 sensor.
 *
 * @param sensor Pointer to the APDS9960 sensor device.
 * @return E_OK on success, error code otherwise.
 */
err_t apds9960_set_gesture_int_off(apds9960_t sensor);

/**
 * Sets the force interrupt register for the APDS9960 sensor.
 *
 * @param sensor Pointer to the APDS9960 sensor device.
 * @param iforce The value to write to the force interrupt register.
 * @return E_OK on success, error code otherwise.
 */
err_t apds9960_set_force_interrupt(apds9960_t sensor, const uint8_t *const iforce);

/**
 * Sets the proximity clear interrupt register for the APDS9960 sensor.
 *
 * @param sensor Pointer to the APDS9960 sensor device.
 * @param piclear The value to write to the proximity clear interrupt register.
 * @return E_OK on success, error code otherwise.
 */
err_t apds9960_set_proximity_clear_int(apds9960_t sensor, const uint8_t *const piclear);

/**
 * Sets the color clear interrupt register for the APDS9960 sensor.
 *
 * @param sensor Pointer to the APDS9960 sensor device.
 * @param ciclear The value to write to the color clear interrupt register.
 * @return E_OK on success, error code otherwise.
 */
err_t apds9960_set_color_clear_int(apds9960_t sensor, const uint8_t *const ciclear);

/**
 * Sets the ALS clear interrupt register for the APDS9960 sensor.
 *
 * @param sensor Pointer to the APDS9960 sensor device.
 * @param aiclear The value to write to the ALS clear interrupt register.
 * @return E_OK on success, error code otherwise.
 */
err_t apds9960_set_als_clear_int(apds9960_t sensor, const uint8_t *const aiclear);

/**
 * Checks if a valid gesture has been detected by the APDS9960 sensor.
 *
 * @param sensor Pointer to the APDS9960 sensor device.
 * @return 1 if a valid gesture is detected, 0 otherwise.
 */
uint8_t apds9960_gesture_valid(apds9960_t sensor);

/**
 * Resets the gesture count registers of the APDS9960 sensor.
 *
 * @param sensor Pointer to the APDS9960 sensor device.
 */
void apds9960_reset_counts(apds9960_t sensor);

/**
 * Reads and interprets gesture data from the APDS9960 sensor.
 *
 * This function continuously reads gesture data from the sensor until a valid gesture is detected
 * or a timeout occurs.
 *
 * @param sensor Pointer to the APDS9960 sensor device.
 * @param gesture Pointer to store the detected gesture.
 * @return E_OK on success, error code otherwise.
 */
uint8_t apds9960_read_gesture(apds9960_t sensor);

/**
 * Reads the raw proximity data from the APDS9960 sensor.
 *
 * @param sensor Pointer to the APDS9960 sensor device.
 * @param proximity Pointer to store the read proximity data.
 * @return E_OK on success, error code otherwise.
 */
err_t apds9960_read_raw_proximity(apds9960_t sensor, uint8_t *proximity);

/**
 * Reads color data from the APDS9960 sensor.
 *
 * @param sensor Pointer to the APDS9960 sensor device.
 * @param data Specifies the color data to read (CDATA, RDATA, GDATA, or BDATA).
 * @param color_data Pointer to store the read color data.
 * @return E_OK on success, error code otherwise.
 */
err_t apds9960_read_color(apds9960_t sensor, uint8_t data, uint16_t *color_data);

/**
 * Reads the red, green, blue, and clear color data from the APDS9960 sensor.
 *
 * @param sensor Pointer to the APDS9960 sensor device.
 * @param r Pointer to store the red color data.
 * @param g Pointer to store the green color data.
 * @param b Pointer to store the blue color data.
 * @param c Pointer to store the clear color data.
 * @return E_OK on success, error code otherwise.
 */
err_t apds9960_get_color_data(apds9960_t sensor, uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c);

/**
 * Initializes the APDS9960 sensor for gesture detection.
 *
 * This function configures the sensor with default settings for gesture detection.
 *
 * @param sensor Pointer to the APDS9960 sensor device.
 * @return E_OK on success, error code otherwise.
 */
err_t apds9960_gesture_init(apds9960_t sensor);

err_t apds9960_set_timeout(apds9960_t sensor, uint64_t timeout);
void apds9960_diagnose(apds9960_t sensor);
/**NOT IMPLEMENTED YET*/
err_t apds9960_read_proximity(apds9960_t sensor, uint8_t *proximity);
err_t apds9960_get_ambient_light(apds9960_t sensor, uint16_t *l);
err_t apds9960_get_again(apds9960_t sensor, uint8_t *again);
err_t apds9960_get_ggain(apds9960_t sensor, uint8_t *ggain);
err_t apds9960_get_pgain(apds9960_t sensor, uint8_t *pgain);
#endif
