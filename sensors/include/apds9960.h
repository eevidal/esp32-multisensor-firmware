#include "logs.h"

typedef void *apds9960_t;

apds9960_t *sensor apds9960_init(i2c_bus_t *i2c_params);
err_t apds9960_delete(apds9960_t *sensor); // RNF.4
err_t apds9960_setup(apds9960_t *sensor, *sensor_params);
err_t apds9960_proximity_set_mode(apds9960_t *sensor, uint8_t mode);
err_t apds9960_proximity_get_mode(apds9960_t *sensor, uint8_t* mode);
err_t apds9960_set_wait_time(apds9960_t sensor, uint8_t time);
err_t apds9960_set_threshold(apds9960_t *sensor, /
uint8_t gesture, uint8_t threshold);
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
err_t apds9960_read_gesture(apds9960_t *sensor, gesture *gesture);
err_t apds9960_set_gesture_offset(apds9960_t *sensor,
uint8_t offset_up, uint8_t offset_down, uint8_t offset_le
uint8_t offset_right);
err_t apds9960_proximity_init(apds9960_t *sensor);
err_t apds9960_read_raw_proximity(apds9960_t *sensor, uint8_t *proximity);
err_t apds9960_read_proximity(apds9960_t *sensor, uint8_t *proximity);
err_t apds9960_get_color_data(apds9960_t sensor, uint16_t *r,
uint16_t *g, uint16_t *b, uint16_t *c);
err_t apds9960_enable_color_engine(apds9960_t sensor, bool en);
err_t apds9960_get_ambient_ligth(apds9960_t *sensor, uint16_t *l);

// Private Functions 
err_t apds9960_read_byte(apds9960_t sensor, uint8_t addr,uint8_t *data);
err_t apds9960_read(apds9960_handle_t sensor, uint8_t addr, uint8_t *buf, uint8_t len);
err_t apds9960_write_byte(apds9960_t sensor, uint8_t addr,uint8_t data);
err_t apds9960_write(apds9960_t sensor, uint8_t addr, uint8_t *buf, uint8_t len)
err_t process_gesture(apds9960_t sensor, uint8_t data,gesture *gesture);
err_t decode_gesture(apds9960_t sensor, uint8_t data,gesture *gesture);
err_t apds9960_calculate_lux(apds9960_t sensor, uint16_t r,
uint16_t g, uint16_t b, uint16 *l);