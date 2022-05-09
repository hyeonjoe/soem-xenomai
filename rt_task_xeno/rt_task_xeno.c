#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>

#include <unistd.h>
#include <sched.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <pthread.h>
#include <math.h>
#include "ethercat.h"
#include "ethercattype.h"
#include "nicdrv.h"
#include "ethercatbase.h"
#include "ethercatmain.h"
#include "ethercatdc.h"
#include "ethercatcoe.h"
#include "ethercatfoe.h"
#include "ethercatconfig.h"
#include "ethercatprint.h"

#include <alchemy/timer.h>

//#include </usr/include/xenomai/alchemy/timer.h>
#include <stdint.h>
#include <xeno_config.h>
#include <boilerplate/list.h>
#include <copperplate/threadobj.h>

#include <alchemy/compat.h>

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <sys/mman.h>
#include <alchemy/sem.h>
#include <alchemy/mutex.h>
#include <alchemy/timer.h>
#include <trank/rtdk.h>
#include <pthread.h>
#include <rtdm/rtdm.h>
#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <unistd.h>
#include <sched.h>
#include <string.h>
#include <time.h>
#include <pthread.h>
#include <inttypes.h>


#define MAXSTREAM 200000
#define EC_TIMEOUTMON 500
#define JDOF 1
#define MAX_TORQUE 1000
#define ELMO_DOF 33
#define PERIOD_NS 1000000
#define INITIAL_POS 0
#define SEC_IN_NSEC 1000000000

char IOmap[4096];
pthread_t thread1;

int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;
int cnt_err = 0;



boolean printSDO = FALSE;
boolean printMAP = FALSE;
typedef struct PACKED
{
   uint8         counter;
   int16         stream[100];
} in_EBOX_streamt;

typedef struct PACKED
{
   uint8         control;
   uint8         dout;
   int16         aout[2];
   uint16        pwmout[2];
} out_EBOXt;

typedef struct PACKED
{
   uint8         control;
} out_EBOX_streamt;

struct sched_param schedp;
char IOmap[4096];
pthread_t thread1;
struct timeval tv,t1,t2;
int dorun = 0;
int deltat, tmax=0;
int64 toff;
int DCdiff;
int os;
uint32 ob;
int16 ob2;
uint8 ob3;
pthread_cond_t  cond  = PTHREAD_COND_INITIALIZER;
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
int64 integral=0;
uint32 cyclecount;
in_EBOX_streamt  *in_EBOX;
out_EBOX_streamt *out_EBOX;
double     ain[2];
int        ainc;
int        streampos;
int16      stream1[MAXSTREAM];
int16      stream2[MAXSTREAM];
uint8 newposition;
//int t; //check variation for address


RT_TASK my_task;


void task_func();

int main()
{
    int err = 0;
    RT_TASK rt_task;

    if((err = rt_task_create(&rt_task, "rt_task", 0, 50, 0)) != 0)
    {
        printf("task create failed = %d\n", err);
        return err;
    }

    if((err = rt_task_start(&rt_task, &task_func, 0)) != 0)
    {
        printf("task start failed = %d\n", err);
        return err;
    }

    while(1)
    {
        sleep(1);
    }
}

void task_func()
{
    int i = 0;
    for (i = 0; i < 10; i++)
    {
        printf("task loop %d\n", i);
        sleep(1);
    }
}




