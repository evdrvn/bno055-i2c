#include "bno055-i2c.h"
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <math.h>
#include <unistd.h>
#include <fcntl.h>

typedef uint8_t byte;

//#define BNO055_TRACE 
#ifdef BNO055_TRACE
#define TRACE(fmt, ...) printf("##TRACE## " fmt "\n", ##__VA_ARGS__); fflush(stdout)
#else
#define TRACE(fmt, ...) (void)sizeof(printf(fmt,##__VA_ARGS__))
#endif


static void delay(int howLong){
    usleep(howLong * 1000);
}

int bno055_writebytes(bno055_conn_t* conn, uint8_t *data, uint8_t count){
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[1];

    messages[0].addr = conn->address;
    messages[0].flags = 0;
    messages[0].len = count;
    messages[0].buf = (unsigned char *)data;

    packets.msgs = messages;
    packets.nmsgs = 1;

    return ioctl(conn->file, I2C_RDWR, &packets) >= 0;
}

int bno055_readbytes(bno055_conn_t* conn, uint8_t reg, uint8_t *dest, uint8_t count){
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[2];
    int ret = 0;
    /* write the register we want to read from */
    messages[0].addr = conn->address; 
    messages[0].flags = 0;        
    messages[0].len = 1;         
    messages[0].buf = (unsigned char *)&reg;

    /* read */
    messages[1].addr = conn->address;  
    messages[1].flags = I2C_M_RD;
    messages[1].len = count;    
    messages[1].buf = (unsigned char *)dest;

    packets.msgs = messages;
    packets.nmsgs = 2;

    ret =  ioctl(conn->file, I2C_RDWR, &packets);
   
#ifdef BNO055_TRACE
    printf("%02x :", reg);
    for(i = 0; i < count; i++){ 
        printf(" %02x", dest[i]); 
    }
    printf("\n");
#endif

    return ret >= 0;
}

static bool write8(bno055_conn_t* conn, bno055_reg_t reg, byte value){
    uint8_t buf[2];
    buf[0] = reg;
    buf[1] = value;
    bno055_writebytes(conn, buf, 2);
    /* ToDo: Check for error! */
    return true;
}

static byte read8(bno055_conn_t* conn, bno055_reg_t reg){
    byte val = 0;
    bno055_readbytes(conn, reg, &val, 1);
    return val;
}

void bno055_setmode(bno055_conn_t* conn, bno055_opmode_t mode){
    write8(conn, BNO055_OPR_MODE_ADDR, mode);
    delay(30);
}

int bno055_open(bno055_conn_t* conn, const char* i2cdev, uint8_t address, int timeout_10ms){
    int fd, ret;

    if ((fd = open (i2cdev, O_RDWR)) < 0)
        return fd;
    if ((ret = ioctl (fd, I2C_SLAVE, address)) < 0)
        return ret;
    if ((ret = ioctl (fd, I2C_TIMEOUT, timeout_10ms)) < 0)
        return ret;
 
    conn->file = fd;
    conn->address = address;

    return fd;
}

int bno055_init(bno055_conn_t* conn, uint8_t st_trigger, bno055_param_t* params, uint8_t params_num){
    uint8_t id = read8(conn, BNO055_CHIP_ID_ADDR);
    int i, ret = 0;

    if (id != BNO055_ID)
    {
        delay(1000); // hold on for boot
        id = read8(conn, BNO055_CHIP_ID_ADDR);
        if (id != BNO055_ID) {
            TRACE("BNO055_CHIP_IDADDR(0x%x) = 0x%x (expected = 0x%x)", BNO055_CHIP_ID_ADDR, id, BNO055_ID);
            return -1;  // still not? ok bail
        }
    }
    bno055_setmode(conn, OPERATION_MODE_CONFIG);
    write8(conn, BNO055_UNIT_SEL_ADDR, 0x81);
    write8(conn, BNO055_SYS_TRIGGER_ADDR, st_trigger);
    delay(1000);
    read8(conn, BNO055_SELFTEST_RESULT_ADDR);

    while (read8(conn, BNO055_CHIP_ID_ADDR) != BNO055_ID){delay(10);} // hold on for boot

    if(NULL != params){ 
        for(i = 0; i < params_num; i++){
            write8(conn, params[i].reg, params[i].value);  
        }
    }
    delay(10);
    bno055_setmode(conn, OPERATION_MODE_NDOF);
    return ret;
}

int bno055_readcalibstat(bno055_conn_t* conn, uint8_t* data){
    int ret;
    ret = bno055_readbytes(conn, BNO055_CALIB_STAT_ADDR, data, 1); 
    return ret;
}

int bno055_readquaternion(bno055_conn_t* conn, double data[4]){
    int ret = 0; 
    int16_t w, x, y, z;
    w = x = y = z = 0;
    uint8_t buffer[8] = {0};
    ret = bno055_readbytes(conn, (bno055_reg_t)BNO055_QUATERNION_DATA_W_LSB_ADDR, buffer, 8);
    if(ret < 0) return ret;
    w = (((int16_t)buffer[1]) << 8) | (int16_t)buffer[0];
    x = (((int16_t)buffer[3]) << 8) | (int16_t)buffer[2];
    y = (((int16_t)buffer[5]) << 8) | (int16_t)buffer[4];
    z = (((int16_t)buffer[7]) << 8) | (int16_t)buffer[6];
    data[0] = w / 16384.0;
    data[1] = x / 16384.0;
    data[2] = y / 16384.0;
    data[3] = z / 16384.0;
    TRACE ("w = %f", data[0]); 
    TRACE ("x = %f", data[1]); 
    TRACE ("y = %f", data[2]); 
    TRACE ("z = %f", data[3]); 
    return ret;
}

int bno055_readlinearaccel(bno055_conn_t* conn, double data[3]){
    int ret = 0; 
    int16_t x, y, z;
    x = y = z = 0;
    uint8_t buffer[6] = {0};
    ret = bno055_readbytes(conn, (bno055_reg_t)BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR, buffer, 6);
    if(ret  < 0) return ret;
    x = (((int16_t)buffer[1]) << 8) | (int16_t)buffer[0];
    y = (((int16_t)buffer[3]) << 8) | (int16_t)buffer[2];
    z = (((int16_t)buffer[5]) << 8) | (int16_t)buffer[4];
    data[0] = x / (100.0);
    data[1] = y / (100.0);
    data[2] = z / (100.0);
    TRACE ("linear accel x = %f", data[0]); 
    TRACE ("linear accel y = %f", data[1]); 
    TRACE ("linear accel z = %f", data[2]); 
    return ret;
}

int bno055_readeuler(bno055_conn_t* conn, double data[3]){
    int ret = 0; 
    double roll, pitch, yaw;
    double q[4] = {0};
    double w, x, y, z; 
    double ysqr;
    double t0, t1, t2, t3, t4;

    ret = bno055_readquaternion(conn, q);
    if(ret < 0) return ret;
    w = q[0]; 
    x = q[1]; 
    y = q[2]; 
    z = q[3]; 
    ysqr = y * y;

    // roll (x-axis rotation)
    t0 = 2.0 * (w * x + y * z);
    t1 = 1.0 - 2.0 * (x * x + ysqr);
    
    TRACE ("t0 = %f", t0); 
    TRACE ("t1 = %f", t1); 
    roll = atan2(t0, t1);

    // pitch (y-axis rotation)
    t2 = 2.0 * (w * y - z * x);
    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    TRACE ("t2 = %f", t2); 
    pitch = asin(t2);

    // yaw (z-axis rotation)
    t3 = 2.0 * (w * z + x * y);
    t4 = 1.0 - 2.0 * (ysqr + z * z);  
    TRACE ("t3 = %f", t3); 
    TRACE ("t4 = %f", t4); 
    yaw = atan2(t3, t4);

    TRACE ("r = %f", roll); 
    TRACE ("p = %f", pitch); 
    TRACE ("y = %f", yaw); 

    data[0] = roll * 180.0 / M_PI;
    data[1] = pitch * 180.0 / M_PI;
    data[2] = yaw * 180 / M_PI;

    return ret;
}
