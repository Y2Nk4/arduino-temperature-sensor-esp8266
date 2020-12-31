/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.2 */

#ifndef PB_TEMPERATURE_NODE_PB_H_INCLUDED
#define PB_TEMPERATURE_NODE_PB_H_INCLUDED
#include <pb.h>
#include "device_discover.pb.h"

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Struct definitions */
typedef struct _TemperatureNodeReport {
    pb_callback_t temperature_record;
    float battery_vcc;
    uint64_t device_mac;
    uint32_t measure_interval;
} TemperatureNodeReport;


/* Initializer values for message structs */
#define TemperatureNodeReport_init_default       {{{NULL}, NULL}, 0, 0, 0}
#define TemperatureNodeReport_init_zero          {{{NULL}, NULL}, 0, 0, 0}

/* Field tags (for use in manual encoding/decoding) */
#define TemperatureNodeReport_temperature_record_tag 1
#define TemperatureNodeReport_battery_vcc_tag    2
#define TemperatureNodeReport_device_mac_tag     3
#define TemperatureNodeReport_measure_interval_tag 4

/* Struct field encoding specification for nanopb */
#define TemperatureNodeReport_FIELDLIST(X, a) \
X(a, CALLBACK, REPEATED, MESSAGE,  temperature_record,   1) \
X(a, STATIC,   SINGULAR, FLOAT,    battery_vcc,       2) \
X(a, STATIC,   SINGULAR, UINT64,   device_mac,        3) \
X(a, STATIC,   SINGULAR, UINT32,   measure_interval,   4)
#define TemperatureNodeReport_CALLBACK pb_default_field_callback
#define TemperatureNodeReport_DEFAULT NULL
#define TemperatureNodeReport_temperature_record_MSGTYPE TemperatureRecord

extern const pb_msgdesc_t TemperatureNodeReport_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define TemperatureNodeReport_fields &TemperatureNodeReport_msg

/* Maximum encoded size of messages (where known) */
/* TemperatureNodeReport_size depends on runtime parameters */

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
