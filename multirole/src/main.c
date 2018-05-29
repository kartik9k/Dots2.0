#include <assert.h>
#include <string.h>
#include <stdio.h>
#include "syscfg/syscfg.h"
#include "sysinit/sysinit.h"
#include "bsp/bsp.h"
#include "os/os.h"
#include "hal/hal_gpio.h"
#ifdef ARCH_sim
#include "mcu/mcu_sim.h"
#endif
#include "nimble/ble.h"
#include "controller/ble_ll.h"
#include "host/ble_hs.h"
#include "transport/ram/ble_hci_ram.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "config/config.h"
#include "split/split.h"
#include "blecent.h"
#include "bleprph.h"

int led_pin = 11;
struct log bleprph_log;
struct log blecent_log;
int flag = 0;


static int bleprph_gap_event(struct ble_gap_event *event, void *arg);


const struct peer_chr *chr_array[10];
const struct peer* peer_array[10];
static struct os_callout blinky_callout;

static int blecent_gap_event(struct ble_gap_event *event, void *arg);
static void blecent_scan(void);
static int blecent_on_write(uint16_t conn_handle, 
                 const struct ble_gatt_error *error,
                 struct ble_gatt_attr *attr,
                 void *arg);

static void timer_ev_cb(struct os_event *ev){
    assert(ev != NULL);    
    int iter = 0;
    uint8_t value;
    for (iter = 0; iter < 1; iter += 1){
        value = 0x01;
        int rc;
        rc = ble_gattc_write_flat(peer_array[iter]->conn_handle, chr_array[iter]->chr.val_handle,
                                  &value, sizeof value, blecent_on_write, NULL);
        if (rc != 0) {

            BLECENT_LOG(ERROR, "Error: Failed to write characteristic; rc=%d\n",
                        rc);
        }               
    }
    os_callout_reset(&blinky_callout, OS_TICKS_PER_SEC/4);
}

static void init_timer(void){
    os_callout_init(&blinky_callout, os_eventq_dflt_get(),
                    timer_ev_cb, NULL);
    os_callout_reset(&blinky_callout, OS_TICKS_PER_SEC);
}

static int blecent_on_read(uint16_t conn_handle,
                const struct ble_gatt_error *error,
                struct ble_gatt_attr *attr,
                void *arg)
{
    BLECENT_LOG(INFO, "Read complete; status=%d conn_handle=%d", error->status,
                conn_handle);
    if (error->status == 0) {
        BLECENT_LOG(INFO, " attr_handle=%d value=", attr->handle);
        print_mbuf(attr->om);
    }
    BLECENT_LOG(INFO, "\n");

    return 0;
}

static int blecent_on_write(uint16_t conn_handle,
                 const struct ble_gatt_error *error,
                 struct ble_gatt_attr *attr,
                 void *arg)
{
    BLECENT_LOG(INFO, "Write complete; status=%d conn_handle=%d "
                      "attr_handle=%d\n",
                error->status, conn_handle, attr->handle);
    return 0;
}

static void blecent_read_write_subscribe(const struct peer *peer)
{
    const struct peer_chr *chr;
    uint8_t value; 
    int rc;

    chr = peer_chr_find_uuid(peer,
                             BLE_UUID128_DECLARE(0xff, 0x71, 0xa2, 0x59, 0x14, 0x58, 0xc8, 0x12,
                     0x4e, 0x99, 0x7d, 0x95, 0x12, 0x2f, 0x36, 0x59),
                             BLE_UUID128_DECLARE(0xf7, 0x6d, 0xc9, 0x07, 0x71, 0x0d, 0x16, 0xb0,
                         0xe1, 0x45, 0x7f, 0x89, 0x2e, 0x65, 0x3a, 0x5d));
    if (chr == NULL) {
        BLECENT_LOG(ERROR, "Error: Peer doesn't support the Supported New "
                           "Alert Category characteristic\n");
        goto err;
    }

    rc = ble_gattc_read(peer->conn_handle, chr->chr.val_handle,
                        blecent_on_read, NULL);
    if (rc != 0) {
        BLECENT_LOG(ERROR, "Error: Failed to read characteristic; rc=%d\n",
                    rc);
        goto err;
    }

    chr = peer_chr_find_uuid(peer,
                             BLE_UUID128_DECLARE(0xff, 0x71, 0xa2, 0x59, 0x14, 0x58, 0xc8, 0x12,
                     0x4e, 0x99, 0x7d, 0x95, 0x12, 0x2f, 0x36, 0x59),
                             BLE_UUID128_DECLARE(0xf7, 0x6d, 0xc9, 0x07, 0x71, 0x0d, 0x16, 0xb0,
                         0xe1, 0x45, 0x7f, 0x89, 0x2e, 0x65, 0x3a, 0x5d));
    if (chr == NULL) {

        BLECENT_LOG(ERROR, "Error: Peer doesn't support the Alert "
                           "Notification Control Point characteristic\n");
        goto err;
    }

    value = 0x01;
    rc = ble_gattc_write_flat(peer->conn_handle, chr->chr.val_handle,
                              &value, sizeof value, blecent_on_write, NULL);
    if (rc != 0) {

        BLECENT_LOG(ERROR, "Error: Failed to write characteristic; rc=%d\n",
                    rc);
    }

    chr_array[flag] = chr;
    peer_array[flag] = peer;

    flag = 1;
    init_timer();
    return;

err:
    ble_gap_terminate(peer->conn_handle, BLE_ERR_REM_USER_CONN_TERM);
}

static void blecent_on_disc_complete(const struct peer *peer, int status, void *arg)
{
    if (status != 0) {
        BLECENT_LOG(ERROR, "Error: Service discovery failed; status=%d "
                           "conn_handle=%d\n", status, peer->conn_handle);
        ble_gap_terminate(peer->conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        return;
    }

    BLECENT_LOG(ERROR, "Service discovery complete; status=%d "
                       "conn_handle=%d\n", status, peer->conn_handle);

    blecent_read_write_subscribe(peer);
}

static void blecent_scan(void)
{
    struct ble_gap_disc_params disc_params;
    int rc;

    disc_params.filter_duplicates = 1;

    disc_params.passive = 1;

    disc_params.itvl = 0;
    disc_params.window = 0;
    disc_params.filter_policy = 0;
    disc_params.limited = 0;

    rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER, &disc_params,
                      blecent_gap_event, NULL);
    if (rc != 0) {
        BLECENT_LOG(ERROR, "Error initiating GAP discovery procedure; rc=%d\n",
                    rc);
    }
}

static int blecent_should_connect(const struct ble_gap_disc_desc *disc)
{
    struct ble_hs_adv_fields fields;
    int rc;
    int i;

    if (disc->event_type != BLE_HCI_ADV_RPT_EVTYPE_ADV_IND &&
        disc->event_type != BLE_HCI_ADV_RPT_EVTYPE_DIR_IND) {

        return 0;
    }

    rc = ble_hs_adv_parse_fields(&fields, disc->data, disc->length_data);
    if (rc != 0) {
        return rc;
    }
 
    for (i = 0; i < fields.name_len; i++){
        if ((uint8_t)fields.name[i] == 0x70 || (uint8_t)fields.name[i] == 0x6E){
            return 1;
        }
    }

    return 0;
}

static void blecent_connect_if_interesting(const struct ble_gap_disc_desc *disc)
{
    int rc;

    if (!blecent_should_connect(disc)) {
        return;
    }

    rc = ble_gap_disc_cancel();
    if (rc != 0) {
        BLECENT_LOG(DEBUG, "Failed to cancel scan; rc=%d\n", rc);
        return;
    }
    rc = ble_gap_connect(BLE_OWN_ADDR_PUBLIC, &disc->addr, 30000, NULL,
                         blecent_gap_event, NULL);
    if (rc == BLE_HS_ENOMEM) {        
        hal_gpio_write(led_pin, 1);    
        BLECENT_LOG(ERROR, "Error: Failed to connect to device; addr_type=%d "
                           "addr=%s\n", disc->addr.type,
                           addr_str(disc->addr.val));
        return;
    }
}

static int blecent_gap_event(struct ble_gap_event *event, void *arg)
{
    struct ble_gap_conn_desc desc;
    struct ble_hs_adv_fields fields;
    int rc;

    switch (event->type) {
    case BLE_GAP_EVENT_DISC:

        rc = ble_hs_adv_parse_fields(&fields, event->disc.data,
                                     event->disc.length_data);
        if (rc != 0) {
            return 0;
        }

        print_adv_fields(&fields);

        blecent_connect_if_interesting(&event->disc);
        return 0;

    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            BLECENT_LOG(INFO, "Connection established ");

            rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
            assert(rc == 0);
            print_conn_desc(&desc);
            BLECENT_LOG(INFO, "\n");

            rc = peer_add(event->connect.conn_handle);
            if (rc != 0) {
                BLECENT_LOG(ERROR, "Failed to add peer; rc=%d\n", rc);
                return 0;
            }

            rc = ble_gap_security_initiate(event->connect.conn_handle);
            if (rc == BLE_HS_ENOMEM){
                BLECENT_LOG(ERROR, "Failed to authenticate; rc=%d\n", rc);
                return 0;                
            }
            rc = peer_disc_all(event->connect.conn_handle,
                               blecent_on_disc_complete, NULL);
            if (rc != 0) {

                BLECENT_LOG(ERROR, "Failed to discover services; rc=%d\n", rc);
                return 0;
            }   

        } else {
            BLECENT_LOG(ERROR, "Error: Connection failed; status=%d\n",
                        event->connect.status);
            blecent_scan();
        }

        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        BLECENT_LOG(INFO, "disconnect; reason=%d ", event->disconnect.reason);
        print_conn_desc(&event->disconnect.conn);
        BLECENT_LOG(INFO, "\n");
        peer_delete(event->disconnect.conn.conn_handle);
        blecent_scan();
        return 0;

    case BLE_GAP_EVENT_ENC_CHANGE:
        BLECENT_LOG(INFO, "encryption change event; status=%d ",
                    event->enc_change.status);
        rc = ble_gap_conn_find(event->enc_change.conn_handle, &desc);
        assert(rc == 0);
        print_conn_desc(&desc);
        return 0;

    case BLE_GAP_EVENT_NOTIFY_RX:
        BLECENT_LOG(INFO, "received %s; conn_handle=%d attr_handle=%d "
                          "attr_len=%d\n",
                    event->notify_rx.indication ?
                        "indication" :
                        "notification",
                    event->notify_rx.conn_handle,
                    event->notify_rx.attr_handle,
                    OS_MBUF_PKTLEN(event->notify_rx.om));
        return 0;

    case BLE_GAP_EVENT_MTU:
        BLECENT_LOG(INFO, "mtu update event; conn_handle=%d cid=%d mtu=%d\n",
                    event->mtu.conn_handle,
                    event->mtu.channel_id,
                    event->mtu.value);
        return 0;

    case BLE_GAP_EVENT_REPEAT_PAIRING:
        rc = ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc);
        assert(rc == 0);
        ble_store_util_delete_peer(&desc.peer_id_addr);

        return BLE_GAP_REPEAT_PAIRING_RETRY;

    default:
        return 0;
    }
}

static const ble_uuid128_t gatt_svr_svc_sec_test_uuid =
    BLE_UUID128_INIT(0xff, 0x71, 0xa2, 0x59, 0x14, 0x58, 0xc8, 0x12,
                     0x4e, 0x99, 0x7d, 0x95, 0x12, 0x2f, 0x36, 0x59);

static const ble_uuid128_t gatt_svr_chr_sec_test_rand_uuid =
        BLE_UUID128_INIT(0xf6, 0x6d, 0xd9, 0x07, 0x71, 0x00, 0x16, 0xb0,
                         0xe1, 0x45, 0x75, 0x89, 0x9a, 0x66, 0x3a, 0x5c);

static const ble_uuid128_t gatt_svr_chr_sec_test_static_uuid =
        BLE_UUID128_INIT(0xf7, 0x6d, 0xc9, 0x07, 0x71, 0x0d, 0x16, 0xb0,
                         0xe1, 0x45, 0x7f, 0x89, 0x2e, 0x65, 0x3a, 0x5d);

static uint8_t gatt_svr_sec_test_static_val;

static int
gatt_svr_chr_access_sec_test(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt,
                             void *arg);

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_svr_svc_sec_test_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) { {
            .uuid = &gatt_svr_chr_sec_test_rand_uuid.u,
            .access_cb = gatt_svr_chr_access_sec_test,
            .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC,
        }, {
            .uuid = &gatt_svr_chr_sec_test_static_uuid.u,
            .access_cb = gatt_svr_chr_access_sec_test,
            .flags = BLE_GATT_CHR_F_READ |
                     BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_ENC,
        }, {
            0, 
        } },
    },

    {
        0,
    },
};

static int gatt_svr_chr_write(struct os_mbuf *om, uint16_t min_len, uint16_t max_len,
                   void *dst, uint16_t *len)
{
    uint16_t om_len;
    int rc;

    om_len = OS_MBUF_PKTLEN(om);
    if (om_len < min_len || om_len > max_len) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    rc = ble_hs_mbuf_to_flat(om, dst, max_len, len);
    if (rc != 0) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    return 0;
}

static int gatt_svr_chr_access_sec_test(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt,
                             void *arg)
{
    const ble_uuid_t *uuid;
    int rand_num;
    int rc;

    uuid = ctxt->chr->uuid;
    if (ble_uuid_cmp(uuid, &gatt_svr_chr_sec_test_rand_uuid.u) == 0) {
        assert(ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR);
        rand_num = rand();
        rc = os_mbuf_append(ctxt->om, &rand_num, sizeof rand_num);
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    if (ble_uuid_cmp(uuid, &gatt_svr_chr_sec_test_static_uuid.u) == 0) {
        switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR:
            rc = os_mbuf_append(ctxt->om, &gatt_svr_sec_test_static_val,
                                sizeof gatt_svr_sec_test_static_val);
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;

        case BLE_GATT_ACCESS_OP_WRITE_CHR:
            rc = gatt_svr_chr_write(ctxt->om,
                                    sizeof gatt_svr_sec_test_static_val,
                                    sizeof gatt_svr_sec_test_static_val,
                                    &gatt_svr_sec_test_static_val, NULL);
            if (gatt_svr_sec_test_static_val == 0x01){
                // one = 1;
                blecent_scan();
            }

            return rc;

        default:
            assert(0);
            return BLE_ATT_ERR_UNLIKELY;
        }
    }

    assert(0);
    return BLE_ATT_ERR_UNLIKELY;
}

void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg){
    char buf[BLE_UUID_STR_LEN];

    switch (ctxt->op) {
    case BLE_GATT_REGISTER_OP_SVC:
        BLEPRPH_LOG(DEBUG, "registered service %s with handle=%d\n",
                    ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
                    ctxt->svc.handle);
        break;

    case BLE_GATT_REGISTER_OP_CHR:
        BLEPRPH_LOG(DEBUG, "registering characteristic %s with "
                           "def_handle=%d val_handle=%d\n",
                    ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
                    ctxt->chr.def_handle,
                    ctxt->chr.val_handle);
        break;

    case BLE_GATT_REGISTER_OP_DSC:
        BLEPRPH_LOG(DEBUG, "registering descriptor %s with handle=%d\n",
                    ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf),
                    ctxt->dsc.handle);
        break;

    default:
        assert(0);
        break;
    }
}

int gatt_svr_init(void){
    int rc;
    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }
    return 0;
}

static void bleprph_print_conn_desc(struct ble_gap_conn_desc *desc){
    BLEPRPH_LOG(INFO, "handle=%d our_ota_addr_type=%d our_ota_addr=",
                desc->conn_handle, desc->our_ota_addr.type);
    BLEPRPH_LOG(INFO, " our_id_addr_type=%d our_id_addr=",
                desc->our_id_addr.type);
    BLEPRPH_LOG(INFO, " peer_ota_addr_type=%d peer_ota_addr=",
                desc->peer_ota_addr.type);
    BLEPRPH_LOG(INFO, " peer_id_addr_type=%d peer_id_addr=",
                desc->peer_id_addr.type);
    BLEPRPH_LOG(INFO, " conn_itvl=%d conn_latency=%d supervision_timeout=%d "
                "encrypted=%d authenticated=%d bonded=%d\n",
                desc->conn_itvl, desc->conn_latency,
                desc->supervision_timeout,
                desc->sec_state.encrypted,
                desc->sec_state.authenticated,
                desc->sec_state.bonded);
}

static void bleprph_advertise(void){
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    const char *name;
    int rc;
    memset(&fields, 0, sizeof fields);
    fields.flags = BLE_HS_ADV_F_DISC_GEN |
                   BLE_HS_ADV_F_BREDR_UNSUP;

    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;


    name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    fields.uuids16 = (ble_uuid16_t[]){
        BLE_UUID16_INIT(GATT_SVR_SVC_ALERT_UUID)
    };
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        BLEPRPH_LOG(ERROR, "error setting advertisement data; rc=%d\n", rc);
        return;
    }

    memset(&adv_params, 0, sizeof adv_params);
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                           &adv_params, bleprph_gap_event, NULL);
    if (rc != 0) {
        BLEPRPH_LOG(ERROR, "error enabling advertisement; rc=%d\n", rc);
        return;
    }
}

static int
bleprph_gap_event(struct ble_gap_event *event, void *arg)
{
    struct ble_gap_conn_desc desc;
    int rc;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        BLEPRPH_LOG(INFO, "connection %s; status=%d ",
                       event->connect.status == 0 ? "established" : "failed",
                       event->connect.status);
        if (event->connect.status == 0) {
            rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
            assert(rc == 0);
            bleprph_print_conn_desc(&desc);
        }
        BLEPRPH_LOG(INFO, "\n");

        if (event->connect.status != 0) {
            bleprph_advertise();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        BLEPRPH_LOG(INFO, "disconnect; reason=%d ", event->disconnect.reason);
        bleprph_print_conn_desc(&event->disconnect.conn);
        BLEPRPH_LOG(INFO, "\n");
        bleprph_advertise();

        return 0;

    case BLE_GAP_EVENT_CONN_UPDATE:
        BLEPRPH_LOG(INFO, "connection updated; status=%d ",
                    event->conn_update.status);
        rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
        assert(rc == 0);
        bleprph_print_conn_desc(&desc);
        BLEPRPH_LOG(INFO, "\n");
        return 0;

    case BLE_GAP_EVENT_ENC_CHANGE:
        BLEPRPH_LOG(INFO, "encryption change event; status=%d ",
                    event->enc_change.status);
        rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
        assert(rc == 0);
        bleprph_print_conn_desc(&desc);
        BLEPRPH_LOG(INFO, "\n");
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        BLEPRPH_LOG(INFO, "subscribe event; conn_handle=%d attr_handle=%d "
                          "reason=%d prevn=%d curn=%d previ=%d curi=%d\n",
                    event->subscribe.conn_handle,
                    event->subscribe.attr_handle,
                    event->subscribe.reason,
                    event->subscribe.prev_notify,
                    event->subscribe.cur_notify,
                    event->subscribe.prev_indicate,
                    event->subscribe.cur_indicate);
        return 0;

    case BLE_GAP_EVENT_MTU:
        BLEPRPH_LOG(INFO, "mtu update event; conn_handle=%d cid=%d mtu=%d\n",
                    event->mtu.conn_handle,
                    event->mtu.channel_id,
                    event->mtu.value);
        return 0;

    case BLE_GAP_EVENT_REPEAT_PAIRING:
        rc = ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc);
        assert(rc == 0);
        ble_store_util_delete_peer(&desc.peer_id_addr);
        return BLE_GAP_REPEAT_PAIRING_RETRY;
    }

    return 0;
}

static void bleprph_on_reset(int reason){
    BLEPRPH_LOG(ERROR, "Resetting state; reason=%d\n", reason);
}

static void bleprph_on_sync(void){
    bleprph_advertise();
}

int main(void){
    int rc;
    memcpy(g_dev_addr, (uint8_t[6]){0x0a, 0x0a, 0x0a, 0x0a, 0x0a, 0x0a}, 6);

    sysinit();

    hal_gpio_init_out(led_pin, 0);
    log_register("multirole", &blecent_log, &log_console_handler, NULL,
                 LOG_SYSLEVEL);

    log_register("multirole_hs", &ble_hs_log, &log_console_handler, NULL,
                 LOG_SYSLEVEL);
    ble_hs_cfg.reset_cb = bleprph_on_reset;
    ble_hs_cfg.sync_cb = bleprph_on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
    ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    rc = gatt_svr_init();
    assert(rc == 0);

    rc = ble_svc_gap_device_name_set("nimble-multirole");
    assert(rc == 0);

    rc = peer_init(MYNEWT_VAL(BLE_MAX_CONNECTIONS), 64, 64, 64);
    assert(rc == 0);

    conf_load();
    while (1) {
        os_eventq_run(os_eventq_dflt_get());
    }
    return 0;
}