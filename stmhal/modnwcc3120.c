/*
 * Micro Python CC3120 Driver
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2016 Kimball Johnson
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <string.h>
#include <stdio.h>
#include <errno.h>

#include "py/nlr.h"
#include "py/objtuple.h"
#include "py/objlist.h"
#include "py/stream.h"
#include "py/runtime.h"
#include "py/mphal.h"
#include "extmod/machine_spi.h"

#include "netutils.h"
#include "modnetwork.h"
#include "pin.h"
#include "genhdr/pins.h"
#include "dma.h"
#include "irq.h"
#include "spi.h"
#include "drivers/cc3120-1.30.00.03/ti/drivers/net/wifi/netcfg.h"

#include "extint.h"

#include "drivers/cc3120-1.30.00.03/ti/drivers/net/wifi/sl_socket.h"

#define LOG_ERR(str) printf("Error: %s\n",str)
#define LOG_INFO(str) printf("Info: %s\n",str)

#define USE_HARD_SPI (0)
#define IO_TRACE (0)

#if IO_TRACE
#define IO_TRACE_PREFIX(msg) printf("[IO %010u %u] " msg, (uint)mp_hal_ticks_us(), (uint)mp_hal_pin_read(PIN_IRQ))
#else
#define IO_TRACE_PREFIX(msg)
#endif

// *** Begin simplelink interface functions
STATIC volatile SL_P_EVENT_HANDLER cc3120_IrqHandler = 0;

STATIC SPI_HandleTypeDef *SPI_HANDLE = NULL;
STATIC const pin_obj_t *PIN_CS = NULL;
STATIC const pin_obj_t *PIN_EN = NULL;
STATIC const pin_obj_t *PIN_IRQ = NULL;

#if !USE_HARD_SPI
STATIC mp_machine_soft_spi_obj_t soft_spi;
#endif

// External CC3120
Fd_t spi_Open(char* pIfName, unsigned long flags)
{
    #if USE_HARD_SPI

    spi_set_params(SPI_HANDLE, -1, 20000000, 0, 0, 8, SPI_FIRSTBIT_MSB);
    SPI_HANDLE->Init.Mode = SPI_MODE_MASTER;
    SPI_HANDLE->Init.Direction = SPI_DIRECTION_2LINES;
    SPI_HANDLE->Init.NSS = SPI_NSS_SOFT;
    SPI_HANDLE->Init.TIMode = SPI_TIMODE_DISABLED;
    SPI_HANDLE->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
    SPI_HANDLE->Init.CRCPolynomial = 0;
    #if defined(MCU_SERIES_L4) || defined(MCU_SERIES_F7)
    SPI_HANDLE->Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    #endif

    // init the SPI bus
    spi_init(SPI_HANDLE, false);

    #else

    soft_spi.delay_half = MICROPY_PY_MACHINE_SPI_MIN_DELAY;
    soft_spi.polarity = 0;
    soft_spi.phase = 0;
    soft_spi.sck = &MICROPY_HW_SPI1_SCK;
    soft_spi.mosi = &MICROPY_HW_SPI1_MOSI;
    soft_spi.miso = &MICROPY_HW_SPI1_MISO;
    mp_hal_pin_write(soft_spi.sck, soft_spi.polarity);
    mp_hal_pin_output(soft_spi.sck);
    mp_hal_pin_output(soft_spi.mosi);
    mp_hal_pin_input(soft_spi.miso);

    #endif

    #ifdef MICROPY_HW_CC3120_RST
    // take chip out of reset
    mp_hal_pin_output(&MICROPY_HW_CC3120_RST);
    mp_hal_pin_high(&MICROPY_HW_CC3120_RST);
    #endif

    // CS Pin
    mp_hal_pin_output(PIN_CS);
    mp_hal_pin_high(PIN_CS);

    HAL_Delay(50);
    return 1;
}

int spi_Close(Fd_t Fd)
{
    #if USE_HARD_SPI
    spi_deinit(SPI_HANDLE);
    #endif
    return 0;
}

int spi_TransmitReceive(unsigned char* txBuff, unsigned char* rxBuff, int Len)
{
    IO_TRACE_PREFIX("SPI ");

    mp_hal_pin_low(PIN_CS);
    #if USE_HARD_SPI
    if (txBuff == NULL) {
        // It seems that "receive only" SPI transfer does not work, at least
        // not on the STM32L475.  So for such a case we do a full tx+rx, with
        // the tx buffer being the same as the rx buffer (since we don't care
        // what data is actually sent).
        txBuff = rxBuff;
    }
    spi_transfer(spi_get_obj_from_handle(SPI_HANDLE), Len, txBuff, rxBuff, 0x1000);
    #else
    mp_machine_soft_spi_transfer(&soft_spi.base, Len, txBuff, rxBuff);
    #endif
    mp_hal_pin_high(PIN_CS);

    #if IO_TRACE
    {
        int dir;
        unsigned char *buf;
        if (rxBuff == NULL) {
            dir = 'T';
            buf = txBuff;
        } else {
            dir = 'R';
            buf = rxBuff;
        }
        printf("%cX len=%d buf=", dir, Len);
        for (int i = 0; i < 16 && i < Len; ++i) {
            printf(" %02x", buf[i]);
        }
        printf("\n");
    }
    #endif

    return HAL_OK;
}

int spi_Read(Fd_t Fd, unsigned char* pBuff, int Len)
{
    HAL_StatusTypeDef status;
    status = spi_TransmitReceive(NULL, pBuff, Len);
    if (status != HAL_OK)
        return(0);

    return Len;
}

int spi_Write(Fd_t Fd, unsigned char* pBuff, int Len)
{
    HAL_StatusTypeDef status;
    status = spi_TransmitReceive(pBuff, NULL, Len);
    if (status != HAL_OK)
        return(0);

    return Len;
}

void NwpPowerOnPreamble(void){

// IRQ pin
    mp_hal_pin_config(PIN_IRQ, MP_HAL_PIN_MODE_INPUT, MP_HAL_PIN_PULL_DOWN, 0);

//nHib pin
    mp_hal_pin_output(PIN_EN);
    mp_hal_pin_low(PIN_EN);
}
void NwpPowerOn(void){
    IO_TRACE_PREFIX("HIB=1\n");
    mp_hal_pin_high(PIN_EN);
}
void NwpPowerOff(void){
    IO_TRACE_PREFIX("HIB=0\n");
    mp_hal_pin_low(PIN_EN);
}

_u32 NwpSystemTicks(void)
{
	return HAL_GetTick();
}

STATIC mp_obj_t cc3120_callback(mp_obj_t line) {
    if (cc3120_IrqHandler != 0) {
        IO_TRACE_PREFIX("edge IRQ\n");
        (cc3120_IrqHandler)(0);
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(cc3120_callback_obj, cc3120_callback);

int NwpRegisterInterruptHandler(SL_P_EVENT_HANDLER InterruptHdl, void* pValue){
    extint_disable(PIN_IRQ->pin);
    cc3120_IrqHandler = InterruptHdl;
    extint_register((mp_obj_t)PIN_IRQ,
                    GPIO_MODE_IT_RISING,
                    GPIO_PULLDOWN,
                    (mp_obj_t)&cc3120_callback_obj,
                    true);
    extint_enable(PIN_IRQ->pin);
    return 0;
}

void NwpMaskInterrupt(){
    // only needed if IRQ is level-triggered
}

void NwpUnMaskInterrupt(){
    // It's possible that the CC3120 IRQ pin remains high because it still has
    // a pending event to proccess.  One situation when this can happen is when
    // a new event becomes available while doing a read of the previous event.
    // In that case we never get an edge and so must check if the IRQ line is
    // still high, and if so register a callback.
    if (mp_hal_pin_read(PIN_IRQ)) {
        IO_TRACE_PREFIX("level IRQ\n");
        cc3120_callback(MP_OBJ_NEW_SMALL_INT(0));
    }
}

// *** END simplelink interface functions



#define MAKE_SOCKADDR(addr, ip, port) \
    SlSockAddr_t addr; \
    addr.sa_family = SL_AF_INET; \
    addr.sa_data[0] = port >> 8; \
    addr.sa_data[1] = port; \
    addr.sa_data[2] = ip[0]; \
    addr.sa_data[3] = ip[1]; \
    addr.sa_data[4] = ip[2]; \
    addr.sa_data[5] = ip[3];

#define UNPACK_SOCKADDR(addr, ip, port) \
    port = (addr.sa_data[0] << 8) | addr.sa_data[1]; \
    ip[0] = addr.sa_data[2]; \
    ip[1] = addr.sa_data[3]; \
    ip[2] = addr.sa_data[4]; \
    ip[3] = addr.sa_data[5];


#define MAX_RX_PACKET       16000
#define MAX_TX_PACKET       1460

// This structure has the same first 3 entries as a mod_network_socket_obj_t.
// The latter entries are different so we can reuse that memory for our own purposes.
typedef struct _cc3120_socket_obj_t {
    mp_obj_base_t base;
    mp_obj_t nic;
    mod_network_nic_type_t *nic_type;
    int16_t s_fd;
    bool s_nonblocking_connect;
    uint32_t s_timeout;
} cc3120_socket_obj_t;

STATIC int cc3120_socket_settimeout(mod_network_socket_obj_t *socket, mp_uint_t timeout_ms, int *_errno);

STATIC volatile uint32_t fd_closed_state = 0;
STATIC volatile bool wlan_connected = false;
STATIC volatile bool ip_obtained = false;

STATIC int cc3120_get_fd_closed_state(int fd) {
    return fd_closed_state & (1 << fd);
}

STATIC void cc3120_set_fd_closed_state(int fd) {
    fd_closed_state |= 1 << fd;
}

STATIC void cc3120_reset_fd_closed_state(int fd) {
    fd_closed_state &= ~(1 << fd);
}


// Socket functions

// gethostbyname
STATIC int cc3120_gethostbyname(mp_obj_t nic, const char *name, mp_uint_t len, uint8_t *out_ip) {
    uint32_t ip;

    int rc = sl_NetAppDnsGetHostByName((signed char *)name, (uint16_t)len, &ip, SL_AF_INET);
    if (rc != 0) {
        return rc;
    }

    out_ip[0] = ip >> 24;
    out_ip[1] = ip >> 16;
    out_ip[2] = ip >> 8;
    out_ip[3] = ip;

    return 0;
}

STATIC void cc3120_activate(void) {
    if (cc3120_IrqHandler) {
        // Here we assume the driver was already started and just reinitialise the
        // low-level interface.
        mp_hal_pin_config(PIN_IRQ, MP_HAL_PIN_MODE_INPUT, MP_HAL_PIN_PULL_DOWN, 0);
        mp_hal_pin_output(PIN_EN);
        spi_Open(NULL, 0);
        NwpRegisterInterruptHandler(cc3120_IrqHandler, NULL);
        return;
    }

    wlan_connected = false;
    ip_obtained    = false;

    int32_t ret = -1;

    int32_t mode = sl_Start(NULL,NULL,NULL);

    static const unsigned char defaultcountry[2] = "EU";
    if (sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID, SL_WLAN_GENERAL_PARAM_OPT_COUNTRY_CODE, 2, defaultcountry)) {
        LOG_ERR("failed to set country code!");
    }

    if (mode != ROLE_STA) {
        // Configure the device into station mode
        if (mode == ROLE_AP) {
            LOG_INFO("mode: ROLE_AP");
            /* If the device is in AP mode, we need to wait for this event before doing anything */
            while (ip_obtained == false) {
                _SlNonOsHandleSpawnTask();
            }
        }

        // Select station mode, and restart to activate it
        ret = sl_WlanSetMode(ROLE_STA);
        ret = sl_Stop(100);
        mode = sl_Start(0, 0, 0);
        if (mode != ROLE_STA) {
            nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "failed to init CC3120 in STA mode"));
        }
    }

    /* Set connection policy to nothing magic  */
    LOG_INFO("Set connection policy");
    ret = sl_WlanPolicySet(SL_WLAN_POLICY_CONNECTION, SL_WLAN_CONNECTION_POLICY(0, 0, 0, 0), NULL, 0);

    /* Remove all profiles */
    LOG_INFO("Remove all profiles");
    ret = sl_WlanProfileDel(0xFF);

    // Device in station-mode. Disconnect previous connection if any
    // The function returns 0 if 'Disconnected done', negative number if already disconnected
    // Wait for 'disconnection' event if 0 is returned, Ignore other return-codes
    LOG_INFO("Disconnect from AP");
    ret = sl_WlanDisconnect();
    if (ret == 0) {
        // wait for disconnection
        while (wlan_connected == true) {
            _SlNonOsHandleSpawnTask();
        }
    }

    /* not in 1.30.00.03??
    // Enable DHCP client
    LOG_INFO("Enable DHCP");
    uint8_t val = 1;
    ret = sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE,1,1,(uint8_t *)&val);
    */

    // Set Tx power level for station mode
    // Number between 0-15, as dB offset from max power - 0 will set maximum power
    uint8_t power = 0;
    ret = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID, SL_WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (unsigned char *)&power);

    // Set PM policy to normal
    ret = sl_WlanPolicySet(SL_WLAN_POLICY_PM, SL_WLAN_NORMAL_POLICY, NULL, 0);

    // Unregister mDNS services
    ret = sl_NetAppMDNSUnRegisterService(0, 0, 0);

    ret = sl_Stop(100);

    // Initializing the CC3120 device
    ret = sl_Start(0, 0, 0);
    if (ret < 0 || ret != ROLE_STA) {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "failed to init CC3120"));
    }
}

STATIC void cc3120_deactivate(void) {
    // it doesn't seem we can call sl_Stop if the device is already stopped
    if (cc3120_IrqHandler != NULL) {
        sl_Stop(100);
        // the above call should have cleared the IRQ handler, but we do it
        // anyway because it's used to tell if we are active or not
        cc3120_IrqHandler = NULL;
    }
}

STATIC mp_obj_t cc3120_active(size_t n_args, const mp_obj_t *pos_args) {
    if (n_args == 1) {
        return mp_obj_new_bool(cc3120_IrqHandler != NULL);
    }

    if (mp_obj_is_true(pos_args[1])) {
        cc3120_activate();
    } else {
        cc3120_deactivate();
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(cc3120_active_obj, 1, 2, cc3120_active);

// Additional interface functions
// method connect(ssid, key=None, *, security=WPA2, bssid=None, timeout=90)
STATIC mp_obj_t cc3120_connect(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_ssid, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_key, MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_security, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NEW_SMALL_INT(SL_WLAN_SEC_TYPE_WPA_WPA2)} },
        { MP_QSTR_bssid, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_timeout, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NEW_SMALL_INT(90) } },
        { MP_QSTR_eapmethod, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = MP_OBJ_NEW_SMALL_INT(SL_WLAN_ENT_EAP_METHOD_TTLS_MSCHAPv2)} },
        { MP_QSTR_username,  MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
        { MP_QSTR_anonname,  MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = mp_const_none} },
    };

    // parse args
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // get ssid
    mp_uint_t ssid_len;
    const char *ssid = mp_obj_str_get_data(args[0].u_obj, &ssid_len);

    // get key and sec
    mp_uint_t key_len = 0;
    const char *key = NULL;
    mp_uint_t sec = SL_WLAN_SEC_TYPE_OPEN;
    if (args[1].u_obj != mp_const_none) {
        key = mp_obj_str_get_data(args[1].u_obj, &key_len);

        if (!MP_OBJ_IS_INT(args[2].u_obj))
            nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "invalid 'security' parameter\n"));
        sec = mp_obj_get_int(args[2].u_obj);
    }

    // get bssid
    const char *bssid = NULL;
    if (args[3].u_obj != mp_const_none) {
        bssid = mp_obj_str_get_str(args[3].u_obj);
    }

    mp_int_t timeout = -1;
    if (MP_OBJ_IS_INT(args[4].u_obj)) {
      timeout = mp_obj_get_int(args[4].u_obj) * 1000;
    }

    SlWlanSecParams_t sec_params;
    sec_params.Type = sec;
    sec_params.Key = (int8_t*)key;
    sec_params.KeyLen = key_len;

    SlWlanSecParamsExt_t sec_ext_params, *use_ext = NULL;
    if (sec == SL_WLAN_SEC_TYPE_WPA_ENT) {
        mp_uint_t len;

        memset(&sec_ext_params, 0, sizeof(sec_ext_params));
        if (!MP_OBJ_IS_INT(args[5].u_obj))
            nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "invalid 'eapmethod' parameter\n"));
        sec_ext_params.EapMethod = mp_obj_get_int(args[5].u_obj);
        if (args[6].u_obj != mp_const_none) {
            sec_ext_params.User = (_i8*)mp_obj_str_get_data(args[6].u_obj, &len);
            sec_ext_params.UserLen = len;
        }
        if (args[7].u_obj != mp_const_none) {
            sec_ext_params.AnonUser = (_i8*)mp_obj_str_get_data(args[7].u_obj, &len);
            sec_ext_params.AnonUserLen = len;
        }
        use_ext = &sec_ext_params;
    }

    // connect to AP
    printf("Connect to AP\n");
    if (sl_WlanConnect((int8_t*)ssid, ssid_len, (uint8_t*)bssid, &sec_params, use_ext)!= 0) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_OSError,
          "could not connect to ssid=%s, sec=%d, key=%s\n", ssid, sec, key));
    }

    if (timeout >= 0) {
      // Wait until connected or timeout, calling simplelink loop
      uint32_t start = HAL_GetTick();
      while (!(ip_obtained && wlan_connected) && ((HAL_GetTick() - start) < timeout) ){
        _SlNonOsHandleSpawnTask();
      }
      if (!wlan_connected || !ip_obtained) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_OSError,
          "timed out connecting to ssid=%s, sec=%d, key=%s\n", ssid, sec, key));
      }
      sl_WlanRxStatStart();
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(cc3120_connect_obj, 1, cc3120_connect);

STATIC mp_obj_t cc3120_disconnect(mp_obj_t self_in) {
    sl_WlanRxStatStop();
    sl_WlanDisconnect();
    while ((wlan_connected)){
        _SlNonOsHandleSpawnTask();
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(cc3120_disconnect_obj, cc3120_disconnect);

STATIC mp_obj_t cc3120_isconnected(mp_obj_t self_in) {
    return mp_obj_new_bool(wlan_connected && ip_obtained);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(cc3120_isconnected_obj, cc3120_isconnected);

STATIC mp_obj_t cc3120_ifconfig(mp_obj_t self_in) {
    _u16 iplen = sizeof(SlNetCfgIpV4Args_t);
    _u16 opt = 0;
    SlNetCfgIpV4Args_t ipV4 = {0};
    // Ignore the return value of the next call.  It seems that it can return non-zero
    // values even if it succeeds.  Examples of such values are: 0xff52, 0xff91.
    sl_NetCfgGet(SL_NETCFG_IPV4_STA_ADDR_MODE, &opt, &iplen, (unsigned char *)&ipV4);
    mp_obj_t tuple[4];
    tuple[0] = netutils_format_ipv4_addr((uint8_t*)&ipV4.Ip, NETUTILS_LITTLE);
    tuple[1] = netutils_format_ipv4_addr((uint8_t*)&ipV4.IpMask, NETUTILS_LITTLE);
    tuple[2] = netutils_format_ipv4_addr((uint8_t*)&ipV4.IpGateway, NETUTILS_LITTLE);
    tuple[3] = netutils_format_ipv4_addr((uint8_t*)&ipV4.IpDnsServer, NETUTILS_LITTLE);
    return mp_obj_new_tuple(MP_ARRAY_SIZE(tuple), tuple);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(cc3120_ifconfig_obj, cc3120_ifconfig);

STATIC mp_obj_t cc3120_update(mp_obj_t self_in) {

    _SlNonOsHandleSpawnTask();

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(cc3120_update_obj, cc3120_update);

STATIC mp_obj_t cc3120_sleep(mp_obj_t self_in) {

    sl_Stop(100);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(cc3120_sleep_obj, cc3120_sleep);

STATIC mp_obj_t cc3120_scan(mp_obj_t self_in) {
  int retVal;
  uint8_t configOpt = SL_WLAN_SCAN_POLICY(1, 0);
  uint8_t configVal = 60;
  LOG_INFO("Enable Scan");
  retVal = sl_WlanPolicySet(SL_WLAN_POLICY_SCAN, configOpt, &configVal, sizeof(configVal));
  mp_hal_delay_ms(1000); // Wait 1 second to ensure scan starts

  int runningIdx, numOfEntries, idx;
  SlWlanNetworkEntry_t netentry = {0};
  SlWlanNetworkEntry_t netEntries[20];
  memset(netEntries, 0, sizeof(netEntries));

  numOfEntries = 20;
  runningIdx = 0;
  idx = 0;
  retVal = sl_WlanGetNetworkList(runningIdx,numOfEntries,&netEntries[runningIdx]);

  /*
   * Because of a bug user should either read the maximum entries or read
   * entries one by one from the end and check for duplicates. Once a duplicate
   * is found process should be stopped.
   */
  /* get scan results - one by one */
  runningIdx = 20;
  numOfEntries = 1;
  memset(netEntries, 0, sizeof(netEntries));

  do {
    runningIdx--;
    retVal = sl_WlanGetNetworkList(runningIdx, numOfEntries, &netentry);
    if(retVal < numOfEntries)
      nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "Error getting AP list"));

    if(idx > 0)
    {
        if(0 == memcmp(netentry.Bssid,
                  netEntries[idx - 1].Bssid, SL_WLAN_BSSID_LENGTH))
        {
            /* Duplicate entry */
            break;
        }
    }

    memcpy(&netEntries[idx], &netentry, sizeof(SlWlanNetworkEntry_t));
    idx++;

} while (runningIdx > 0);
  mp_obj_t returnVal = mp_obj_new_list(0, NULL);
  for(int i = 0; i < idx; i++) {
    mp_obj_t tuple[5];
    tuple[0] = mp_obj_new_str((const char*)netEntries[i].Ssid, netEntries[i].SsidLen, false);
    tuple[1] = mp_obj_new_bytes(netEntries[i].Bssid, SL_WLAN_BSSID_LENGTH);
    tuple[2] = MP_OBJ_NEW_SMALL_INT(0); // channel not available on CC3120, but it is on CC3120
    tuple[3] = MP_OBJ_NEW_SMALL_INT(netEntries[i].Rssi);
    tuple[4] = MP_OBJ_NEW_SMALL_INT(netEntries[i].SecurityInfo);
    mp_obj_list_append(returnVal, mp_obj_new_tuple(5, tuple));
  }
  return returnVal;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(cc3120_scan_obj, cc3120_scan);

STATIC mp_obj_t cc3120_version(mp_obj_t self_in) {
    int32_t retVal = -1;
    _u8 pConfigOpt;
    _u16 pConfigLen;
    SlDeviceVersion_t ver;

    pConfigLen = sizeof(ver);
    pConfigOpt = SL_DEVICE_GENERAL_VERSION;
    retVal = sl_DeviceGet(SL_DEVICE_GENERAL, &pConfigOpt, &pConfigLen, (_u8 *)(&ver));

    if (retVal >= 0) {
        STATIC const qstr version_fields[] = {
            MP_QSTR_chip, MP_QSTR_mac, MP_QSTR_phy, MP_QSTR_nwp, MP_QSTR_rom, MP_QSTR_host
        };
        mp_obj_t tuple[6];
        tuple[0] = mp_obj_new_int_from_uint(ver.ChipId);

        vstr_t vstr;
        vstr_init(&vstr, 50);
        vstr_printf(&vstr, "31.%lu.%lu.%lu.%lu", ver.FwVersion[0],ver.FwVersion[1],ver.FwVersion[2],ver.FwVersion[3]);
        tuple[1] = mp_obj_new_str(vstr.buf, vstr.len, false);
        vstr_init(&vstr, 50);
        vstr_printf(&vstr, "%d.%d.%d.%d", ver.PhyVersion[0],ver.PhyVersion[1],ver.PhyVersion[2],ver.PhyVersion[3]);
        tuple[2] = mp_obj_new_str(vstr.buf, vstr.len, false);
        vstr_init(&vstr, 50);
        vstr_printf(&vstr, "%lu.%lu.%lu.%lu", ver.NwpVersion[0],ver.NwpVersion[1],ver.NwpVersion[2],ver.NwpVersion[3]);
        tuple[3] = mp_obj_new_str(vstr.buf, vstr.len, false);
        tuple[4] = mp_obj_new_int_from_uint(ver.RomVersion);
        vstr_init(&vstr, 50);
        vstr_printf(&vstr, "%lu.%lu.%lu.%lu", SL_MAJOR_VERSION_NUM,SL_MINOR_VERSION_NUM,SL_VERSION_NUM,SL_SUB_VERSION_NUM);
        tuple[5] = mp_obj_new_str(vstr.buf, vstr.len, false);

        return mp_obj_new_attrtuple(version_fields, 6, tuple);
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(cc3120_version_obj, cc3120_version);

/******************************************************************************/
// Micro Python bindings; CC3120 class

typedef struct _cc3120_obj_t {
    mp_obj_base_t base;
} cc3120_obj_t;

STATIC const cc3120_obj_t cc3120_obj = {{(mp_obj_type_t*)&mod_network_nic_type_cc3120}};

STATIC mp_obj_t cc3120_make_new(const mp_obj_type_t *type, mp_uint_t n_args, mp_uint_t n_kw, const mp_obj_t *args) {

    // Either defaults, or SPI Obj, IRQ Pin, nHIB Pin
    mp_arg_check_num(n_args, n_kw, 0, 4, false);

    //If no args given, we setup from coded defaults
    if (n_args == 0) {
#ifdef MICROPY_HW_CC3120_SPI
      SPI_HANDLE = &MICROPY_HW_CC3120_SPI;
      PIN_CS = &MICROPY_HW_CC3120_CS;
      PIN_EN = &MICROPY_HW_CC3120_HIB;
      PIN_IRQ = &MICROPY_HW_CC3120_IRQ;
#else
      nlr_raise(mp_obj_new_exception_msg(&mp_type_OSError, "No Default CC3120 definition"));
#endif
    } else {
    //Else we use the given args
       //TODO should verify the argument types
       SPI_HANDLE = spi_get_handle(args[0]);
       PIN_CS = pin_find(args[1]);
       PIN_EN = pin_find(args[2]);
       PIN_IRQ = pin_find(args[3]);
    }

    cc3120_activate();

    // register with network module
    mod_network_register_nic((mp_obj_t)&cc3120_obj);
    return (mp_obj_t)&cc3120_obj;

}

STATIC const mp_map_elem_t cc3120_locals_dict_table[] = {
    { MP_OBJ_NEW_QSTR(MP_QSTR_active),          (mp_obj_t)&cc3120_active_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_connect),         (mp_obj_t)&cc3120_connect_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_disconnect),      (mp_obj_t)&cc3120_disconnect_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_isconnected),    (mp_obj_t)&cc3120_isconnected_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_ifconfig),        (mp_obj_t)&cc3120_ifconfig_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_update),          (mp_obj_t)&cc3120_update_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_sleep),           (mp_obj_t)&cc3120_sleep_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_scan),            (mp_obj_t)&cc3120_scan_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_version),         (mp_obj_t)&cc3120_version_obj},

    // class constants
    { MP_OBJ_NEW_QSTR(MP_QSTR_SEC_OPEN), MP_OBJ_NEW_SMALL_INT(SL_WLAN_SEC_TYPE_OPEN) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_WEP), MP_OBJ_NEW_SMALL_INT(SL_WLAN_SEC_TYPE_WEP) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_WPA), MP_OBJ_NEW_SMALL_INT(SL_WLAN_SEC_TYPE_WPA) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_WPA2), MP_OBJ_NEW_SMALL_INT(SL_WLAN_SEC_TYPE_WPA_WPA2) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_WPA_ENT), MP_OBJ_NEW_SMALL_INT(SL_WLAN_SEC_TYPE_WPA_ENT) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_SEC_WEP), MP_OBJ_NEW_SMALL_INT(SL_WLAN_SEC_TYPE_WEP) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_SEC_WPA), MP_OBJ_NEW_SMALL_INT(SL_WLAN_SEC_TYPE_WPA) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_SEC_WPA2), MP_OBJ_NEW_SMALL_INT(SL_WLAN_SEC_TYPE_WPA_WPA2) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_SEC_WPA_ENT), MP_OBJ_NEW_SMALL_INT(SL_WLAN_SEC_TYPE_WPA_ENT) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_SCAN_SEC_OPEN), MP_OBJ_NEW_SMALL_INT(SL_WLAN_SEC_TYPE_OPEN) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_SCAN_SEC_WEP), MP_OBJ_NEW_SMALL_INT(SL_WLAN_SEC_TYPE_WEP) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_SCAN_SEC_WPA), MP_OBJ_NEW_SMALL_INT(SL_WLAN_SEC_TYPE_WPA) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_SCAN_SEC_WPA2), MP_OBJ_NEW_SMALL_INT(SL_WLAN_SEC_TYPE_WPA_WPA2) },

    { MP_OBJ_NEW_QSTR(MP_QSTR_SOL_SOCKET), MP_OBJ_NEW_SMALL_INT(SL_SOL_SOCKET)},
    { MP_OBJ_NEW_QSTR(MP_QSTR_SO_RCVBUF), MP_OBJ_NEW_SMALL_INT(SL_SO_RCVBUF)},
    { MP_OBJ_NEW_QSTR(MP_QSTR_SO_KEEPALIVE) , MP_OBJ_NEW_SMALL_INT(SL_SO_KEEPALIVE )},
    { MP_OBJ_NEW_QSTR(MP_QSTR_SO_RCVTIME0), MP_OBJ_NEW_SMALL_INT(SL_SO_RCVTIMEO)},
    { MP_OBJ_NEW_QSTR(MP_QSTR_SO_NONBLOCKING), MP_OBJ_NEW_SMALL_INT(SL_SO_NONBLOCKING)},
};

STATIC MP_DEFINE_CONST_DICT(cc3120_locals_dict, cc3120_locals_dict_table);

STATIC int cc3120_socket_socket(mod_network_socket_obj_t *socket_in, int *_errno) {
    if (socket_in->u_param.domain != MOD_NETWORK_AF_INET) {
        *_errno = MP_EAFNOSUPPORT;
        return -1;
    }

    mp_uint_t type;
    mp_uint_t proto = 0;
    switch (socket_in->u_param.type) {
        case MOD_NETWORK_SOCK_STREAM: type = SL_SOCK_STREAM; break;
        case MOD_NETWORK_SOCK_DGRAM: type = SL_SOCK_DGRAM; break;
        case MOD_NETWORK_SOCK_RAW: type = SL_SOCK_RAW; break;
        default: *_errno = MP_EINVAL; return -1;
    }

    /* TODO use ussl module
    if (socket_in->u_param.proto == MOD_NETWORK_SEC_SOCKET)
    {
        // SSL Socket
        if (socket_in->u_param.type != MOD_NETWORK_SOCK_STREAM ){
          *_errno = MP_EINVAL; return -1; // Only support TCP SSL
        }
        // To start we will setup ssl sockets ignoring certificates
        proto = SL_SEC_SOCKET;
    }
    */

    // open socket
    int fd = sl_Socket(SL_AF_INET, type, proto);
    if (fd < 0) {
        *_errno = -fd;
        return -1;
    }

    // clear socket state
    cc3120_reset_fd_closed_state(fd);

    // get the timeout that we need to configure the socket to
    uint32_t timeout = socket_in->u_param.timeout;

    // re-cast the socket object to use our custom fields
    cc3120_socket_obj_t *socket = (cc3120_socket_obj_t*)socket_in;

    // store state of this socket
    socket->s_fd = fd;
    socket->s_timeout = timeout;
    socket->s_nonblocking_connect = false;

    // configure the timeout
    if (cc3120_socket_settimeout(socket_in, timeout, _errno) != 0) {
        return -1;
    }

    return 0;
}

STATIC void cc3120_socket_close(mod_network_socket_obj_t *socket_in) {
    cc3120_socket_obj_t *socket = (cc3120_socket_obj_t*)socket_in;
    if (!cc3120_get_fd_closed_state(socket->s_fd)) {
        sl_Close(socket->s_fd);
        cc3120_set_fd_closed_state(socket->s_fd);
    }
}

STATIC int cc3120_socket_bind(mod_network_socket_obj_t *socket_in, byte *ip, mp_uint_t port, int *_errno) {
    cc3120_socket_obj_t *socket = (cc3120_socket_obj_t*)socket_in;
    MAKE_SOCKADDR(addr, ip, port)
    int ret = sl_Bind(socket->s_fd, &addr, sizeof(addr));
    if (ret != 0) {
        *_errno = -ret;
        return -1;
    }
    return 0;
}

STATIC int cc3120_socket_listen(mod_network_socket_obj_t *socket_in, mp_int_t backlog, int *_errno) {
    cc3120_socket_obj_t *socket = (cc3120_socket_obj_t*)socket_in;
    int ret = sl_Listen(socket->s_fd, backlog);
    if (ret != 0) {
        *_errno = -ret;
        return -1;
    }
    return 0;
}

STATIC int cc3120_socket_accept(mod_network_socket_obj_t *socket_in, mod_network_socket_obj_t *socket2_in, byte *ip, mp_uint_t *port, int *_errno) {
    cc3120_socket_obj_t *socket = (cc3120_socket_obj_t*)socket_in;
    cc3120_socket_obj_t *socket2 = (cc3120_socket_obj_t*)socket2_in;

    // accept incoming connection
    int fd;
    SlSockAddr_t addr;
    SlSocklen_t addr_len = sizeof(addr);
    if ((fd = sl_Accept(socket->s_fd, &addr, &addr_len)) < 0) {
        if (fd == SL_ERROR_BSD_EAGAIN) {
            *_errno = MP_EAGAIN;
        } else {
            *_errno = -fd;
        }
        return -1;
    }

    // clear socket state
    cc3120_reset_fd_closed_state(fd);

    // store state in new socket object
    socket2->s_fd = fd;
    socket2->s_timeout = -1; // TODO inherit timeout value?
    socket2->s_nonblocking_connect = false;

    // return ip and port
    UNPACK_SOCKADDR(addr, ip, *port);

    return 0;
}

STATIC int cc3120_socket_connect(mod_network_socket_obj_t *socket_in, byte *ip, mp_uint_t port, int *_errno) {
    cc3120_socket_obj_t *socket = (cc3120_socket_obj_t*)socket_in;

    if (cc3120_get_fd_closed_state(socket->s_fd)) {
        cc3120_socket_socket(socket_in, _errno); // Socket has been closed, we need to recreate it
    }

    MAKE_SOCKADDR(addr, ip, port)
    int ret = sl_Connect(socket->s_fd, &addr, sizeof(addr));
    if (ret != 0 && ret != SL_ERROR_BSD_ESECSNOVERIFY) {
        if (ret == SL_ERROR_BSD_EALREADY && socket->s_timeout == 0) {
            // For a non-blocking connect the CC3120 will return EALREADY the
            // first time.  Calls to sl_Select for writing can be used to poll
            // whether the connection is completed and then sl_Connect should be
            // called once more to finish the connection.  To match BSD we return
            // EINPROGRESS here and set a flag to indicate the connection is in
            // progress.
            socket->s_nonblocking_connect = true;
            *_errno = MP_EINPROGRESS;
        } else {
            *_errno = -ret;
        }
        return -1;
    }
    return 0;
}

STATIC mp_uint_t cc3120_socket_send(mod_network_socket_obj_t *socket_in, const byte *buf, mp_uint_t len, int *_errno) {
    cc3120_socket_obj_t *socket = (cc3120_socket_obj_t*)socket_in;

    if (cc3120_get_fd_closed_state(socket->s_fd)) {
        sl_Close(socket->s_fd);
        *_errno = MP_EPIPE;
        return -1;
    }

    // CC3120 does not handle fragmentation, and will overflow,
    // split the packet into smaller ones and send them out.
    mp_int_t bytes = 0;
    while (bytes < len) {
        int n = MIN((len - bytes), MAX_TX_PACKET);
        n = sl_Send(socket->s_fd, (uint8_t*)buf + bytes, n, 0);
        if (n <= 0) {
            *_errno = -n;
            return -1;
        }
        bytes += n;
    }

    return bytes;
}

STATIC mp_uint_t cc3120_socket_recv(mod_network_socket_obj_t *socket_in, byte *buf, mp_uint_t len, int *_errno) {
    cc3120_socket_obj_t *socket = (cc3120_socket_obj_t*)socket_in;

    // check the socket is open
    if (cc3120_get_fd_closed_state(socket->s_fd)) {
        // socket is closed, but CC3120 may have some data remaining in buffer, so check
        SlFdSet_t rfds;
        SL_SOCKET_FD_ZERO(&rfds);
        SL_SOCKET_FD_SET(socket->s_fd, &rfds);
        SlTimeval_t tv;
        tv.tv_sec = 0;
        tv.tv_usec = 1;
        int nfds = sl_Select(socket->s_fd + 1, &rfds, NULL, NULL, &tv);
        if (nfds == -1 || !SL_SOCKET_FD_ISSET(socket->s_fd, &rfds)) {
            // no data waiting, so close socket and return 0 data
            sl_Close(socket->s_fd);
            return 0;
        }
    }

    // cap length at MAX_RX_PACKET
    len = MIN(len, MAX_RX_PACKET);

    // do the recv
    int ret = sl_Recv(socket->s_fd, buf, len, 0);
    if (ret < 0) {
        *_errno = -ret;
        return -1;
    }

    return ret;
}

STATIC mp_uint_t cc3120_socket_sendto(mod_network_socket_obj_t *socket_in, const byte *buf, mp_uint_t len, byte *ip, mp_uint_t port, int *_errno) {
    cc3120_socket_obj_t *socket = (cc3120_socket_obj_t*)socket_in;
    MAKE_SOCKADDR(addr, ip, port)
    int ret = sl_SendTo(socket->s_fd, (byte*)buf, len, 0, (SlSockAddr_t*)&addr, sizeof(addr));
    if (ret < 0) {
        *_errno = -ret;
        return -1;
    }
    return ret;
}

STATIC mp_uint_t cc3120_socket_recvfrom(mod_network_socket_obj_t *socket_in, byte *buf, mp_uint_t len, byte *ip, mp_uint_t *port, int *_errno) {
    cc3120_socket_obj_t *socket = (cc3120_socket_obj_t*)socket_in;
    SlSockAddr_t addr;
    SlSocklen_t addr_len = sizeof(addr);
    mp_int_t ret = sl_RecvFrom(socket->s_fd, buf, len, 0, &addr, &addr_len);
    if (ret < 0) {
        *_errno = -ret;
        return -1;
    }
    UNPACK_SOCKADDR(addr, ip, *port);
    return ret;
}

STATIC int cc3120_socket_setsockopt(mod_network_socket_obj_t *socket_in, mp_uint_t level, mp_uint_t opt, const void *optval, mp_uint_t optlen, int *_errno) {
    cc3120_socket_obj_t *socket = (cc3120_socket_obj_t*)socket_in;

    if (level == MOD_NETWORK_SOL_SOCKET && opt == MOD_NETWORK_SO_REUSEADDR) {
        // It seems that CC3120 always has the behaviour of SO_REUSEADDR
        return 0;
    }

    int ret = sl_SetSockOpt(socket->s_fd, level, opt, optval, optlen);

  if (ret < 0) {
    *_errno = -ret;
      return -1;
  }
  return 0;
}

STATIC int cc3120_socket_settimeout(mod_network_socket_obj_t *socket_in, mp_uint_t timeout_ms, int *_errno) {
    cc3120_socket_obj_t *socket = (cc3120_socket_obj_t*)socket_in;
    int ret;
    if (timeout_ms == 0 || timeout_ms == -1) {
        SlSockNonblocking_t optval;
        SlSocklen_t optlen = sizeof(optval);
        if (timeout_ms == 0) {
            // set non-blocking mode
            optval.NonBlockingEnabled = 1;
        } else {
            // set blocking mode
            optval.NonBlockingEnabled = 0;
        }
        ret = sl_SetSockOpt(socket->s_fd, SL_SOL_SOCKET, SL_SO_NONBLOCKING , &optval, optlen);

    } else {
        // set timeout
        SlTimeval_t timeout;
        timeout.tv_sec = timeout_ms / 1000;
        timeout.tv_usec = timeout_ms % 1000;
        SlSocklen_t optlen = sizeof(timeout);
        ret = sl_SetSockOpt(socket->s_fd, SL_SOL_SOCKET, SL_SO_RCVTIMEO , &timeout, optlen);
    }

    if (ret != 0) {
        *_errno = -ret;
        return -1;
    }

    return 0;
}

STATIC int cc3120_socket_ioctl(mod_network_socket_obj_t *socket_in, mp_uint_t request, mp_uint_t arg, int *_errno) {
    cc3120_socket_obj_t *socket = (cc3120_socket_obj_t*)socket_in;
    mp_uint_t ret;
    if (request == MP_STREAM_POLL) {
        mp_uint_t flags = arg;
        ret = 0;
        int fd = socket->s_fd;

        // init fds
        SlFdSet_t rfds, wfds;
        SL_SOCKET_FD_ZERO(&rfds);
        SL_SOCKET_FD_ZERO(&wfds);

        // set fds if needed
        if (flags & MP_STREAM_POLL_RD) {
            SL_SOCKET_FD_SET(fd, &rfds);

            // A socked that just closed is available for reading.  A call to
            // recv() returns 0 which is consistent with BSD.
            if (cc3120_get_fd_closed_state(fd)) {
                ret |= MP_STREAM_POLL_RD;
            }
        }
        if (flags & MP_STREAM_POLL_WR) {
            SL_SOCKET_FD_SET(fd, &wfds);
        }

        // call cc3120 select with minimum timeout
        SlTimeval_t tv;
        tv.tv_sec = 0;
        tv.tv_usec = 1;
        // xfds not supported by simplelink
        int nfds = sl_Select(fd + 1, &rfds, &wfds, NULL, &tv);

        // check for error
        if (nfds < 0) {
            *_errno = -nfds;
            return -1;
        }

        // check return of select
        if (nfds > 0) {
            if (SL_SOCKET_FD_ISSET(fd, &rfds)) {
                ret |= MP_STREAM_POLL_RD;
            }
            if (SL_SOCKET_FD_ISSET(fd, &wfds)) {
                ret |= MP_STREAM_POLL_WR;
                if (socket->s_nonblocking_connect) {
                    // CC3120 is in progress of a non-blocking connect, and since
                    // the socket is now ready for writing that means it should
                    // have completed the connect.  We need to call sl_Connect once
                    // more to actually finish the connection.
                    SlSockAddr_t addr;
                    addr.sa_family = SL_AF_INET;
                    addr.sa_data[0] = 0;
                    addr.sa_data[1] = 0;
                    addr.sa_data[2] = 0;
                    addr.sa_data[3] = 0;
                    addr.sa_data[4] = 0;
                    addr.sa_data[5] = 0;
                    int ret = sl_Connect(socket->s_fd, &addr, sizeof(addr));
                    (void)ret; // TODO probably should check the return value
                    socket->s_nonblocking_connect = false; // now connected
                }
            }
        }
    } else {
        *_errno = MP_EINVAL;
        ret = -1;
    }
    return ret;
}


const mod_network_nic_type_t mod_network_nic_type_cc3120 = {
    .base = {
        { &mp_type_type },
        .name = MP_QSTR_CC3120,
        .make_new = cc3120_make_new,
        .locals_dict = (mp_obj_t)&cc3120_locals_dict,
    },
    .gethostbyname = cc3120_gethostbyname,
    .socket = cc3120_socket_socket,
    .close = cc3120_socket_close,
    .bind = cc3120_socket_bind,
    .listen = cc3120_socket_listen,
    .accept = cc3120_socket_accept,
    .connect = cc3120_socket_connect,
    .send = cc3120_socket_send,
    .recv = cc3120_socket_recv,
    .sendto = cc3120_socket_sendto,
    .recvfrom = cc3120_socket_recvfrom,
    .setsockopt = cc3120_socket_setsockopt,
    .settimeout = cc3120_socket_settimeout,
    .ioctl = cc3120_socket_ioctl,
};

// --------------------------------------------------------------------------------------
// ASYNCHRONOUS EVENT HANDLERS
// --------------------------------------------------------------------------------------

void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent) {
    switch(pWlanEvent->Id) {
        case SL_WLAN_EVENT_CONNECT: {
            LOG_INFO("[WLAN EVENT] connect");
            wlan_connected = true;
        }
        break;

        case SL_WLAN_EVENT_DISCONNECT: {
            // link down
            wlan_connected = false;
            ip_obtained = false;
            if (pWlanEvent->Data.Disconnect.ReasonCode ==  SL_WLAN_DISCONNECT_USER_INITIATED) {
                 // user initiated a disconnect request
                 //LOG_INFO("Device disconnected from the AP on application's request");
            } else {
                LOG_INFO("Device disconnected from the AP on an ERROR..!!");
            }
        }
        break;

        default:
            LOG_INFO("[WLAN EVENT] Unexpected event");
            printf("pWlanEvent->Id: %d\n", (int)pWlanEvent->Id);
            break;
    }
}

void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent) {
    switch (pNetAppEvent->Id) {
        case SL_NETAPP_EVENT_IPV4_ACQUIRED: {
            LOG_INFO("[NETAPP EVENT] IP acquired");
            ip_obtained = true;

            SlIpV4AcquiredAsync_t *pEventData = NULL;
            pEventData = &pNetAppEvent->Data.IpAcquiredV4;
            uint8_t ip1,ip2,ip3,ip4;
            ip1 = pEventData->Ip & 0xff;
            ip2 = (pEventData->Ip >> 8) & 0xff;
            ip3 = (pEventData->Ip >> 16) & 0xff;
            ip4 = (pEventData->Ip >> 24) & 0xff;
            printf("IP:  %d.%d.%d.%d\n",ip4,ip3,ip2,ip1);
            ip1 = pEventData->Gateway & 0xff;
            ip2 = (pEventData->Gateway >> 8) & 0xff;
            ip3 = (pEventData->Gateway >> 16) & 0xff;
            ip4 = (pEventData->Gateway >> 24) & 0xff;
            printf("GW:  %d.%d.%d.%d\n",ip4,ip3,ip2,ip1);
            ip1 = pEventData->Dns & 0xff;
            ip2 = (pEventData->Dns >> 8) & 0xff;
            ip3 = (pEventData->Dns >> 16) & 0xff;
            ip4 = (pEventData->Dns >> 24) & 0xff;
            printf("DNS: %d.%d.%d.%d\n",ip4,ip3,ip2,ip1);

            break;
        }
        default:
            LOG_INFO("[NETAPP EVENT] Unexpected event");
            printf("pWlanEvent->Id: %d\n", (int)pNetAppEvent->Id);
            break;
    }
}

void SimpleLinkNetAppRequestEventHandler(SlNetAppRequest_t *pNetAppRequest, SlNetAppResponse_t *pNetAppResponse) {
    LOG_INFO("[NETAPP REQUEST EVENT]");
}

void SimpleLinkNetAppRequestMemFreeEventHandler(_u8 *buffer) {
    LOG_INFO("[NETAPP REQUEST MEMFREE EVENT]");
}

void SimpleLinkHttpServerEventHandler(SlNetAppHttpServerEvent_t *event, SlNetAppHttpServerResponse_t *response) {
    LOG_INFO("[HTTP EVENT]");
}

void SimpleLinkFatalErrorEventHandler(SlDeviceFatal_t *event) {
    switch (event->Id) {
        default:
            LOG_INFO("[FATAL EVENT]");
            printf("%d\n", (int)event->Id);
            break;
    }
}

void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent) {
    switch (pDevEvent->Id) {
        default:
            LOG_INFO("[GENERAL EVENT]");
            printf("%d\n", (int)pDevEvent->Id);
            break;
    }
}

void SimpleLinkSockEventHandler(SlSockEvent_t *pSock) {
    switch (pSock->Event) {
        default:
            printf("[SOCK EVENT] Unexpected event %d\n", (int)pSock->Event);
            break;
    }
}

void SimpleLinkSocketTriggerEventHandler(SlSockTriggerEvent_t* pSlSockTriggerEvent) {
    LOG_INFO("[SOCK TRIGGER EVENT]");
}

int _SlNonOsSemSet(_SlNonOsSemObj_t* pSemObj , _SlNonOsSemObj_t Value) {
    *pSemObj = Value;
    return NONOS_RET_OK;
}

int _SlNonOsSemGet(_SlNonOsSemObj_t* pSyncObj, _SlNonOsSemObj_t WaitValue, _SlNonOsSemObj_t SetValue, unsigned int Timeout) {
    /* If timeout 0 configured, just detect the value and return */
    if ((Timeout ==0) && (WaitValue == *pSyncObj)) {
        *pSyncObj = SetValue;
        return NONOS_RET_OK;
    }

    while (Timeout > 0) {
        if (WaitValue == *pSyncObj) {
            *pSyncObj = SetValue;
            break;
        }
        if (Timeout != NONOS_WAIT_FOREVER) {
            Timeout--;
        }
        tiDriverSpawnCallback();
        mp_hal_delay_ms(1);
    }

    if (Timeout == 0) {
        return NONOS_RET_ERR;
    } else {
        return NONOS_RET_OK;
    }
}
