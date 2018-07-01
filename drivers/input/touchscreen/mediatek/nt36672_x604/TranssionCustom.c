/*
* This program is created for the custom of touchpanel function and
porting .
*
* File Name: TranssionCustom.c
*
* Created: 2016-06-20
*
*/

#include "TranssionCustom.h"
#include "tpd.h"
#include "nt36xxx.h"

static int gesture_enabled = 0;
static int gesturechar_enabled =0;
static int gesturedouble_enabled=0;
static int gesturemusic_enabled=0;
static u8 g_gtp_gesture = 0;
static u8 gestures_flag[32];
static st_gesture_data gesture_data;
static struct mutex gesture_data_mutex;
static int gesture_status;

extern struct nvt_ts_data *ts;

//gesture mode
static ssize_t gesture_status_read(struct file *file,char *buffer, size_t count, loff_t *ppos)
{
    int len,ret = -1;
    char *page = NULL;
    char *ptr = NULL;
    page = kmalloc(PAGE_SIZE, GFP_KERNEL);
    if (!page)
    {
        kfree(page);
        return -ENOMEM;
    }
    ptr = page;
    ptr += sprintf(ptr,"%c:%d;\n%c:%d;\n%c:%d;\n%c:%d;\n%c:%d;\n%c:%d;\n%c:%d;\n%c:%d;\n%c:%d;\n%c:%d;\n%s:%d;\n%s:%d;\n",'w',QUERYBIT(gestures_flag,'w' ),'o',QUERYBIT(gestures_flag,'o'),'m',QUERYBIT(gestures_flag,'m'),'e',QUERYBIT(gestures_flag,'e'),'c',QUERYBIT(gestures_flag,'c'),'z',QUERYBIT(gestures_flag,'z'),'s',QUERYBIT(gestures_flag,'s'),'v',QUERYBIT(gestures_flag,'v'),'^',QUERYBIT(gestures_flag,0X5e),'>',QUERYBIT(gestures_flag,0x3e),"all",gesturechar_enabled,"cc",QUERYBIT(gestures_flag,0xcc));

    len = ptr - page;
    if(*ppos >= len)
    {
        kfree(page);
        return 0;
    }
    ret = copy_to_user(buffer,(char *)page,len);
    *ppos += len;
    if(ret)
    {
        kfree(page);
        return ret;
    }
    kfree(page);
    return len;
}

static ssize_t gesture_status_write(struct file *filp, const char __user * buf, size_t len, loff_t * off)
{
    return 0;
}

static const struct file_operations gesture_fops1 = {
    .owner = THIS_MODULE,
    .read = gesture_status_read,
    .write = gesture_status_write,
};


static ssize_t gesture_data_read(struct file *file, char * buffer, size_t size, loff_t * ppos)
{
    int len,ret = -1;
    char *page = NULL;
    char *ptr = NULL;
    page = kmalloc(PAGE_SIZE, GFP_KERNEL);
    if (!page)
    {
        kfree(page);
        return -ENOMEM;
    }
    ptr = page;
    if(gesture_data.data[0]==0xcc)
        ptr += sprintf(ptr,"%c\n",'f');
    else
        ptr += sprintf(ptr,"%c\n",gesture_data.data[0]);
    len = ptr - page;
    if(*ppos >= len)
    {
        kfree(page);
        return 0;
    }
    ret = copy_to_user(buffer,(char *)page,len);
    *ppos += len;
    NVT_LOG("CTP gesture ret:%d ppos:%d ges:%x char:%c\n",ret,(int)*ppos,gesture_data.data[0],*ptr);
    if(ret)
    {
        kfree(page);
        return ret;
    }
    kfree(page);
    return len;

}

static ssize_t gesture_data_write(struct file *filp, const char __user * buf, size_t len, loff_t * off)
{
    int tempchar3=0;
    char tempchar2,tempchar1;
    u8 tempstatus=-1;
    char buff[4]={'0','0','0','\0'};
    int ret = -1;
    ret = copy_from_user(buff, buf, 3);
    //ret=scanf(buf, "%u", buff);
    if (ret) {
        NVT_ERR("copy_from_user failed.");
        return -EPERM;
    }

    if(buff[1]!='l'&&buff[1]!='0'&&buff[1]!='a'&&buff[1]!='b'&&buff[1]!='c')
    {
        NVT_ERR("CTP-gesture bad command format 0:%c 1:%c 2:%c!!!",buff[0],buff[1],buff[2]);
        return -EPERM;

    }
    if(buff[1]=='l' && (buff[2]=='1'||buff[2]=='2'))
    {

        if(buff[2]=='2'){
                gesturechar_enabled=0;
                setgesturestatus(GESTURE_DISABLE_TOTALLY,0);
        }
        if(buff[2]=='1'){
                gesturechar_enabled=1;setgesturestatus(GESTURE_ENABLE_TOTALLY,1);
        }
        return  len;
    }
    if(buff[0]<'A'||buff[0]>'z'||(buff[2]!='1'&&buff[2]!='2')){
        NVT_ERR("CTP-gesture bad command format 0:%c 1:%c 2:%c!!!",buff[0],buff[1],buff[2]);
        return -EPERM;
    }

    tempchar1=buff[0];
    tempchar2=buff[1];
    tempstatus=buff[2];

    NVT_LOG("CTP-gesture buff 0:%c 1:%c 2:%c!!!",buff[0],buff[1],buff[2]);

    switch(tempchar2)
    {
        case '0':
            switch(tempstatus){
                case '2':
                    setgesturestatus(GESTURE_DISABLE_PARTLY,tempchar1);
                    break;
                case '1':
                    setgesturestatus(GESTURE_ENABLE_PARTLY,tempchar1);
                    break;
                default:return -EPERM;
            }
            break;
        default:
            buff[2]='\0';

            tempchar3= simple_strtoul(buff,NULL,16);

            switch(tempstatus){
                case '2':
                    if(tempchar3==0xcc)gesturedouble_enabled=0;
                    if((tempchar3==0xbb)||(tempchar3==0xba)||(tempchar3==0xab)||(tempchar3==0xaa))
                        gesturemusic_enabled=0;
                    setgesturestatus(GESTURE_DISABLE_PARTLY,tempchar3);
                    break;
                case '1':
                    if(tempchar3==0xcc)gesturedouble_enabled=1;
                    if((tempchar3==0xbb)||(tempchar3==0xba)||(tempchar3==0xab)||(tempchar3==0xaa))
                        gesturemusic_enabled=1;
                    setgesturestatus(GESTURE_ENABLE_PARTLY,tempchar3);
                    break;
                default:return -EPERM;
                }
            break;
    }

    return len;
}

static const struct file_operations gesture_fops2 = {
    .owner = THIS_MODULE,
    .read = gesture_data_read,
    .write = gesture_data_write,
};

u8 is_all_dead(u8 * longlong, s32 size)
{
	int i = 0;
	u8 sum = 0;

	for (i = 0; i < size; i++) {
		sum |= longlong[i];
	}

	return !sum;
}

void gesture_clear_data(void)
{
    mutex_lock(&gesture_data_mutex);
    memset(gesture_data.data, 0, 4);
    mutex_unlock(&gesture_data_mutex);
}

s32 setgesturestatus(unsigned int cmd,unsigned long arg)
{
    u8 ret = 0;
    u32 value = (u32) arg;
    switch (cmd & NEGLECT_SIZE_MASK) {
        case GESTURE_ENABLE_TOTALLY:
            NVT_LOG("ENABLE_GESTURE_TOTALLY");
            gesturechar_enabled=1;
            gesture_enabled=((gesturechar_enabled)||(gesturedouble_enabled)||(gesturemusic_enabled));
            break;

        case GESTURE_DISABLE_TOTALLY:
            NVT_LOG("DISABLE_GESTURE_TOTALLY");
            gesturechar_enabled=0;
            gesture_enabled=((gesturechar_enabled)||(gesturedouble_enabled)||(gesturemusic_enabled));
            break;

        case GESTURE_ENABLE_PARTLY:
            SETBIT(gestures_flag, (u8) value);
            gesture_enabled = (gesturechar_enabled ||gesturedouble_enabled||(gesturemusic_enabled));
            NVT_LOG("ENABLE_GESTURE_PARTLY, gesture = 0x%02X, gesture_enabled = %d", value, gesture_enabled);
            break;

        case GESTURE_DISABLE_PARTLY:
            ret = QUERYBIT(gestures_flag, (u8) value);
            if (!ret) {
                break;
            }
            CLEARBIT(gestures_flag, (u8) value);

            if (is_all_dead(gestures_flag, sizeof(gestures_flag))) {
                //gesture_enabled = 0;
            }
            NVT_LOG("DISABLE_GESTURE_PARTLY, gesture = 0x%02X, gesture_enabled = %d", value, gesture_enabled);
            break;
        case GESTURE_DATA_OBTAIN:
            NVT_LOG("OBTAIN_GESTURE_DATA");

            mutex_lock(&gesture_data_mutex);
            if (gesture_data.data[1] > GESTURE_MAX_POINT_COUNT) {
                gesture_data.data[1] = GESTURE_MAX_POINT_COUNT;
            }
            if (gesture_data.data[3] > 80) {
                gesture_data.data[3] = 80;
            }
            ret = copy_to_user(((u8 __user *) arg), &gesture_data.data, 4 + gesture_data.data[1] * 4 + gesture_data.data[3]);
            mutex_unlock(&gesture_data_mutex);
            if (ret) {
                NVT_ERR("ERROR when copy gesture data to user.");
                ret = -1;
            } else {
                ret = 4 + gesture_data.data[1] * 4 + gesture_data.data[3];
            }
            break;

        case GESTURE_DATA_ERASE:
            NVT_LOG("ERASE_GESTURE_DATA");
            gesture_clear_data();
            break;
    }

    return 0;
}

s32 gesture_init_node(void)
{
    struct proc_dir_entry *proc_entry = NULL;
    struct proc_dir_entry *gesture_state = NULL;
    mutex_init(&gesture_data_mutex);
    memset(gestures_flag, 0, sizeof(gestures_flag));//for debug   0-->1
    memset((u8 *) & gesture_data, 0, sizeof(st_gesture_data));

    proc_entry = proc_create(GESTURE_NODE,0666, NULL, &gesture_fops2);
    if (proc_entry == NULL) {
        NVT_ERR("[GESTURE]Couldn't create proc entry[GESTURE_NODE]!");
        return -1;
    } else {
        NVT_LOG("[GESTURE]Create proc entry[GESTURE_NODE] success!");
    }

    gesture_state = proc_create("gesture_state", 0666, NULL, &gesture_fops1);
    if (gesture_state == NULL) {
        NVT_ERR("[GESTURE]Couldn't create proc entry[gesture_state]!");
        return -1;
    } else {
        NVT_LOG("[GESTURE]Create proc entry[gesture_state] success!");
    }
    return 0;
}

void gesture_deinit_node(void)
{
    remove_proc_entry(GESTURE_NODE, NULL);
}

int SetDozeStatus(void)
{
	gesture_status = (gesturechar_enabled ||gesturedouble_enabled||(gesturemusic_enabled));
	return gesture_status;
}

int GestureIdTranslate(const int gesture_id)
{
    int ret = 0;
	NVT_LOG("[GESTURE]gesture_id=0x%x", gesture_id);
	switch(gesture_id)
	{
		case GESTURE_DOUBLECLICK:
			g_gtp_gesture=GESTURE_DC;
			break;
		case GESTURE_C:
			g_gtp_gesture=GESTURE_c;
			break;
		case GESTURE_M:
			g_gtp_gesture=GESTURE_m;
			break;
		case GESTURE_LEFT:
			g_gtp_gesture=GESTURE_LF;
			break;
		case GESTURE_RIGHT:
			g_gtp_gesture=GESTURE_RT;
			break;
		case GESTURE_UP:
			g_gtp_gesture=GESTURE_up;
			break;
		case GESTURE_DOWN:
			g_gtp_gesture=GESTURE_down;
			break;
		case GESTURE_O:
			g_gtp_gesture=GESTURE_o;
			break;
		case GESTURE_W:
			g_gtp_gesture=GESTURE_w;
			break;
		case GESTURE_E:
			g_gtp_gesture=GESTURE_e;
			break;
		case GESTURE_S:
			g_gtp_gesture=GESTURE_s;
			break;
		case GESTURE_V:
			g_gtp_gesture=GESTURE_v;
			break;
		case GESTURE_Z:
			g_gtp_gesture=GESTURE_z;
			break;
		default:
			g_gtp_gesture=-1;
			break;
	}
	ret = GestureHandle();
	return ret;
}

int GestureHandle(void)
{
    struct input_dev *input_dev = ts->input_dev;

#if defined(WAKEUP_GESTURE)
    if(ctp_gesture_fun_enable)
    {
#endif

        if (!QUERYBIT(gestures_flag, g_gtp_gesture)||(!gesturechar_enabled && g_gtp_gesture>='a'&&g_gtp_gesture<='z'))
        {
            NVT_ERR("[GESTURE]Sorry, this gesture has been disabled.");
            return -1;
        }

        if (g_gtp_gesture != -1)
        {
            gesture_data.data[0] = g_gtp_gesture;	// gesture type
            gesture_data.data[1] = 0;	// gesture points number
            gesture_data.data[2] = 0;
            gesture_data.data[3] = 0;

            NVT_LOG("[GESTURE]gesture report key");
            input_report_key(input_dev, KEY_F13, 1);
            input_sync(input_dev);
            input_report_key(input_dev, KEY_F13, 0);
            input_sync(input_dev);

            return 0;
        } else {
			//add XLLSHLSS-4 by qiang.xue 20171205 start
			NVT_ERR("[GESTURE]Sorry, g_gtp_gesture is invaild.");
			//add XLLSHLSS-4 by qiang.xue 20171205 start
			return -1;
		}

#if defined(WAKEUP_GESTURE)
    }
#endif

    return -1;
}
