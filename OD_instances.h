#include "OD.h"
#include "CO_ODinterface.h"

#ifndef MAX_MOTORS
#define MAX_MOTORS 6
#endif

static OD_RAM_t OD_RAM_instances[MAX_MOTORS];
static ODObjs_t ODObjs_instances[MAX_MOTORS];
struct OD_entry_t OD_ENTRY_instances[6][53];
static OD_t OD_instances[MAX_MOTORS];



void init_OD_RAM_t(OD_RAM_t *od_ram_instance);
void init_ODObjs_t(ODObjs_t *odobjs_instance, OD_RAM_t *od_ram_instance);
void init_ODList(OD_entry_t *od_list_instance, ODObjs_t *odobjs_instance);
void init_OD(OD_t* od_instance, OD_entry_t *od_list_instance);