

typedef void (*CB_SWVER)(const char *sw_ver);
typedef void (*CB_VRCMD)(const char *sw_ver, int cmd_id);

int isd9160_play(void);
int isd9160_stop(void);
void isd9160_reset(void);
int isd9160_init(void);
int isd9160_loop_once(void);
int register_vrcmd_callback(CB_VRCMD pfunc);
int register_swver_callback(CB_SWVER pfunc);
