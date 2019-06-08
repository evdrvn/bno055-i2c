#include <bno055-i2c.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <civetweb.h>
#include <evdsptc.h>
#include <math.h>

#define DEFAULT_I2C_DEVICE "/dev/i2c-1"
#define NS_AS_SEC (1000 * 1000 * 1000)
#define READ_INTERVAL_HZ (80)
#define READ_INTERVAL_NS (NS_AS_SEC / READ_INTERVAL_HZ)
#define INFORM_INTERVAL_NS (100 * 1000 * 1000)
#define MAX_WS_CLIENTS (5)
#define DEG_TO_RAD(deg) (((deg)/360)*2*M_PI)
#define RAD_TO_DEG(rad) (((rad)/2/M_PI)*360)

struct t_ws_client {
	struct mg_connection *conn;
	int state;
} static ws_clients[MAX_WS_CLIENTS];

static int running = true;
static int max = 0;
static double euler[3] = {0};
static double saccel_raw[3] = {0};
static double saccel_lpf[3] = {0};
static double saccel_hpf[3] = {0};
static double waccel[3] = {0};
static uint8_t calibstat = 0;
static pthread_mutex_t mutex;

static int createresponse(char* buf, unsigned int size){
    int ret;
    pthread_mutex_lock(&mutex);
    ret =  snprintf(buf, size - 1, "{\"calibstat\": \"%02x\", "
            "\"roll\": %f, \"pitch\": %f, \"yaw\": %f, "
            "\"waccel_x\": %f, \"waccel_y\": %f, \"waccel_z\": %f}"
            , calibstat,
            euler[0], euler[1], euler[2],
            waccel[0], waccel[1], waccel[2]); 
    pthread_mutex_unlock(&mutex);
    return ret;
}

static int handler(struct mg_connection *conn, void *ignored)
{
    char msg[BUFSIZ];
    unsigned long len;
   
    createresponse(msg, BUFSIZ);
    
    len = (unsigned long)strlen(msg);
    mg_printf(conn,
            "HTTP/1.1 200 OK\r\n"
            "Content-Length: %lu\r\n"
            "Content-Type: application/json\r\n"
            "Connection: close\r\n\r\n",
            len);

    mg_write(conn, msg, len);

    return 200;
}

static int WebSocketConnectHandler(const struct mg_connection *conn, void *cbdata)
{
	struct mg_context *ctx = mg_get_context(conn);
	int reject = 1;
	int i;

	mg_lock_context(ctx);
	for (i = 0; i < MAX_WS_CLIENTS; i++) {
		if (ws_clients[i].conn == NULL) {
			ws_clients[i].conn = (struct mg_connection *)conn;
			ws_clients[i].state = 1;
			mg_set_user_connection_data(ws_clients[i].conn,
			                            (void *)(ws_clients + i));
			reject = 0;
			break;
		}
	}
	mg_unlock_context(ctx);
	return reject;
}

static void WebSocketReadyHandler(struct mg_connection *conn, void *cbdata)
{
	char msg[BUFSIZ];
    createresponse(msg, BUFSIZ);
	struct t_ws_client *client = mg_get_user_connection_data(conn);

	mg_websocket_write(conn, WEBSOCKET_OPCODE_TEXT, msg, strlen(msg));

	client->state = 2;
}


static int WebsocketDataHandler(struct mg_connection *conn, int bits, char *data, size_t len, void *cbdata)
{
	return 1;
}


static void WebSocketCloseHandler(const struct mg_connection *conn, void *cbdata)
{
	struct mg_context *ctx = mg_get_context(conn);
	struct t_ws_client *client = mg_get_user_connection_data(conn);

	mg_lock_context(ctx);
	client->state = 0;
	client->conn = NULL;
	mg_unlock_context(ctx);
}

static void InformWebsockets(struct mg_context *ctx)
{
	bool init = false;
	char msg[BUFSIZ];
	int i;

	mg_lock_context(ctx);
	for (i = 0; i < MAX_WS_CLIENTS; i++) {
		if (ws_clients[i].state == 2) {
            if(!init){
                init = true;
                createresponse(msg, BUFSIZ);
            }
			mg_websocket_write(ws_clients[i].conn,
			                   WEBSOCKET_OPCODE_TEXT,
			                   msg,
			                   strlen(msg));
		}
	}
	mg_unlock_context(ctx);
}

static void convertSensorToWorld(double* sensor, double* euler_, double * world){
    double r11, r12, r13, r21, r22, r23, r31, r32, r33;
    double x, y ,z;
    x = DEG_TO_RAD(euler_[0]);  
    y = DEG_TO_RAD(euler_[1]);  
    z = DEG_TO_RAD(euler_[2]);
    r11 =  cos(z) * cos(y);
    r12 = -sin(z) * cos(x) + cos(z) * sin(y) * sin(x);
    r13 =  sin(z) * sin(x) + cos(z) * sin(y) * cos(x);
    r21 =  sin(z) * cos(y); 
    r22 =  cos(z) * cos(x) + sin(z) * sin(y) * sin(x); 
    r23 = -cos(z) * sin(x) + sin(z) * sin(y) * cos(x); 
    r31 = -sin(y); 
    r32 =  cos(y) * sin(x); 
    r33 =  cos(y) * cos(x); 
    world[0] = r11 * sensor[0] + r12 * sensor[1] + r13 * sensor[2];
    world[1] = r21 * sensor[0] + r22 * sensor[1] + r23 * sensor[2];
    world[2] = r31 * sensor[0] + r32 * sensor[1] + r33 * sensor[2];
}

static bool periodic_read(evdsptc_event_t* event){
    long diff; 
    int i = 0;
    static int cancel = 0;
    double euler_[3] = {0};
    struct timespec t1, t2;
    bool noerror;
    bno055_conn_t* bno055_conn;
    void** connlist;
    
    connlist = evdsptc_event_getparam(event); 
    bno055_conn = connlist[0];
    
    clock_gettime(CLOCK_MONOTONIC, &t1);
    noerror = bno055_readeuler(bno055_conn, euler_);
    noerror &= bno055_readcalibstat(bno055_conn, &calibstat);
    noerror &= bno055_readlinearaccel(bno055_conn, saccel_raw);
    clock_gettime(CLOCK_MONOTONIC, &t2);
    
    diff = (t2.tv_sec - t1.tv_sec) * 1000 * 1000 * 1000 + t2.tv_nsec - t1.tv_nsec;
    if(max < diff) max = diff;

    pthread_mutex_lock(&mutex);

    /* noise canceling */
    if( (fabs(euler[0] - euler_[0]) > 90.0 || fabs(euler[1] - euler_[1]) > 90.0 || fabs(euler[2] - euler_[2]) > 90.0) && cancel < 5 && noerror) cancel++;
    else {
        for(i = 0; i < 3; i++) euler[i] = euler_[i];
        cancel = 0;
    }

    /* accel low and hign pass filter (experimental) */
    for(i = 0; i < 3; i++){
        if(saccel_raw[i] == 1.280) saccel_raw[i] = saccel_lpf[i]; /* noise canceling */
        saccel_lpf[i] = saccel_lpf[i] * 0.98 + saccel_raw[i] * 0.02; 
        saccel_hpf[i] = saccel_raw[i] - saccel_lpf[i];
    }
    convertSensorToWorld(saccel_hpf, euler, waccel);
    
    pthread_mutex_unlock(&mutex);

    return !running;
}

static bool periodic_inform(evdsptc_event_t* event){
    static int inform_count  = 0;
    char buf[BUFSIZ];

    if(inform_count++ >= 10){
        inform_count = 0;
        pthread_mutex_lock(&mutex);
        snprintf(buf, BUFSIZ - 1, "rpy=[%8.3f,%8.3f,%8.3f], wldacc=[%8.3f,%8.3f,%8.3f], tmax = %d\n", 
                euler[0], euler[1], euler[2], 
                waccel[0], waccel[1], waccel[2], 
                max);
        pthread_mutex_unlock(&mutex);
        printf("%s", buf);
    };
    InformWebsockets((struct mg_context*)evdsptc_event_getparam(event));

    return false;
}

static void error(const char* message, const char* target){
    char buf[BUFSIZ];
    snprintf(buf, BUFSIZ - 1, "%s in \"%s\" ", message, target);
    perror(buf);
    exit(-1);
}

int main(int argc, char *argv[]){
    void* connlist[2];
    bno055_conn_t bno055_conn;
    bno055_param_t params[8];
    struct mg_context *mgctx;
	const char *options[] = { 
        "document_root", "./htdocs",
        "request_timeout_ms", "10000",
	    "websocket_timeout_ms", "30000"
    };
    const char* dev = DEFAULT_I2C_DEVICE;
    pthread_t th;
    pthread_mutexattr_t mutexattr;
    struct sched_param param;
    evdsptc_context_t read_ctx, inform_ctx;
    evdsptc_event_t read_ev, inform_ev;
    struct timespec read_interval = { read_interval.tv_sec = 0, read_interval.tv_nsec = READ_INTERVAL_NS};
    struct timespec inform_interval = { inform_interval.tv_sec = 0, inform_interval.tv_nsec = INFORM_INTERVAL_NS};
   
    mg_init_library(0);
    mgctx = mg_start(NULL, 0, options);
 
    if(argc > 1) dev = argv[1];
    if(0 > bno055_open(&bno055_conn, dev, BNO055_ADDRESS_A, 2)) error("open error", dev);

    connlist[0] = &bno055_conn;

    params[0].reg = BNO055_AXIS_MAP_CONFIG_ADDR;
    params[0].value = 0x24;
    //params[0].value = 0x21;
    params[1].reg = BNO055_AXIS_MAP_SIGN_ADDR;
    params[1].value = 0x00;
    //params[1].value = 0x01;
    
    if(0 > bno055_init(&bno055_conn,  0x81, params, 2)) error("init error", dev);

    mg_set_request_handler(mgctx, "/orientation", handler, NULL);
    mg_set_websocket_handler(mgctx,
            "/websocket",
            WebSocketConnectHandler,
            WebSocketReadyHandler,
            WebsocketDataHandler,
            WebSocketCloseHandler,
            0);

    pthread_mutexattr_init(&mutexattr);
    pthread_mutexattr_setprotocol(&mutexattr, PTHREAD_PRIO_INHERIT);
    evdsptc_setmutexattrinitializer(&mutexattr);
    pthread_mutex_init(&mutex, &mutexattr);

    evdsptc_create_periodic(&read_ctx, NULL, NULL, NULL, &read_interval);

    th = evdsptc_getthreads(&read_ctx)[0];
    param.sched_priority = 70;
    
    if(0 != pthread_setschedparam(th, SCHED_RR, &param)){
        printf("\nwarning : you get better performance to run this example as root via RT-Preempt.\n");
    }
    
    evdsptc_create_periodic(&inform_ctx, NULL, NULL, NULL, &inform_interval);
    th = evdsptc_getthreads(&inform_ctx)[0];
    param.sched_priority = 30;
    pthread_setschedparam(th, SCHED_RR, &param);
    evdsptc_event_init(&read_ev, periodic_read, connlist, false, NULL);
    
    evdsptc_post(&read_ctx, &read_ev);

    evdsptc_event_init(&inform_ev, periodic_inform, mgctx, false, NULL);
    evdsptc_post(&inform_ctx, &inform_ev);

    evdsptc_event_waitdone(&read_ev);
    evdsptc_destory(&read_ctx, true);
    evdsptc_destory(&inform_ctx, true);

    mg_stop(mgctx);
    mg_exit_library();

    return 0;
}
