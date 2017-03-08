# CC3100 driver

import time, struct
from machine import Pin, SPI

# configure these to let the CC3100 connect to your local wifi AP
AP_SSID = 'ssid'
AP_PASS = 'pass'

class CC3100:
    def __init__(self, spi, cs, irq, hib, rst):
        self.spi = spi
        self.cs = cs
        self.irq = irq
        self.hib = hib
        self.rst = rst
        self.seq_num = 0

        self.hib(0)
        self.rst(0)
        time.sleep_ms(100)

        self.buf4 = bytearray(4)
        self.buf8 = bytearray(8)

    def spi_tx(self, buf):
        self.cs(0)
        self.spi.write(buf)
        self.cs(1)

    def spi_rx(self, buf):
        self.cs(0)
        self.spi.readinto(buf)
        self.cs(1)

    def sync(self, cmd=None, timeout=1000):
        start = time.ticks_ms()
        while self.irq.value() == 0:
            if time.ticks_diff(time.ticks_ms(), start) > timeout:
                raise OSError("timeout in sync")
        self.spi_tx(b'\x65\x87\x78\x56')
        start = time.ticks_ms()
        while self.irq.value() == 1:
            if time.ticks_diff(time.ticks_ms(), start) > timeout:
                raise OSError("timeout in sync")
        self.spi_rx(self.buf8)
        assert self.buf8[0] == 0xbc | self.seq_num
        assert self.buf8[1] == 0xdc
        assert self.buf8[2] == 0xcd
        assert self.buf8[3] == 0xab
        if cmd is not None:
            assert self.buf8[4] == cmd & 0xff
            assert self.buf8[5] == (cmd >> 8) & 0x7f
        len = self.buf8[6] | self.buf8[7] << 8
        self.seq_num = (self.seq_num + 1) & 3
        self.spi_rx(self.buf4) # get first 4 bytes of response
        return len

    def calc_pad(self, l):
        # need to pad to make the payload a multiple of 4 bytes
        if l & 3 == 0:
            return 0
        else:
            return 4 - (l & 3)

    def start(self):
        self.rst.value(1)
        time.sleep_ms(800)
        self.hib.value(1)
        self.sync(0x0008, timeout=2000)
        self.spi_rx(self.buf8)
        print(self.buf8, self.irq())
        self.spi_rx(self.buf4)
        print(self.buf4, self.irq())
        self.spi_rx(self.buf4)
        print(self.buf4, self.irq())

    def set_country_code(self, country):
        assert len(country) == 2
        self.spi_tx(b'\x21\x43\x34\x12')
        self.spi_tx(b'\xb5\x8c\x0c\x00')
        self.spi_tx(b'\x00\x00\x01\x00\x09\x00\x02\x00')
        self.spi_tx(country + b'\x00\x00')
        self.sync(0x8cb5)
        self.spi_rx(self.buf4)

    def set_connection_policy(self):
        self.spi_tx(b'\x21\x43\x34\x12')
        self.spi_tx(b'\x86\x8c\x04\x00')
        self.spi_tx(b'\x10\x30\x00\x00')
        self.sync(0x8c86)
        self.spi_rx(self.buf4)

    def wlan_profile_del_all(self):
        self.spi_tx(b'\x21\x43\x34\x12')
        self.spi_tx(b'\x85\x8c\x04\x00')
        self.spi_tx(b'\xff\x52\x05\x08') # \xff means del all
        self.sync(0x8c85)
        self.spi_rx(self.buf4)

    def wlan_disconnect(self):
        self.spi_tx(b'\x21\x43\x34\x12')
        self.spi_tx(b'\x81\x8c\x00\x00')
        self.sync(0x8c81)
        self.spi_rx(self.buf4)

    def get_version(self):
        self.spi_tx(b'\x21\x43\x34\x12')
        self.spi_tx(b'\x66\x84\x08\x00') # \x66\x84 is SL_OPCODE_DEVICE_DEVICEGET
        self.spi_tx(b'\x00\x00\x01\x00\x0c\x00\x00\x08')
        self.sync(0x8466)

        # get first part of response
        self.spi_rx(self.buf8)
        assert self.buf8[6] == 44 # we expect 44 bytes of data

        # get data
        buf44 = bytearray(44)
        self.spi_rx(buf44)
        version = struct.unpack('<IIIIIBBBBIIIII', buf44)

        # print version info
        print('chip: 0x%x' % version[0])
        print('mac:  31.%u.%u.%u.%u' % version[1:5])
        print('phy:  %u.%u.%u.%u' % version[5:9])
        print('nwp:  %u.%u.%u.%u' % version[9:13])
        print('rom:  0x%x' % version[13])

    def get_ipv4_info(self):
        self.spi_tx(b'\x21\x43\x34\x12')
        self.spi_tx(b'\x33\x84\x08\x00')
        self.spi_tx(b'\x01\x00\x03\x00\x00\x00\x10\x00')
        self.sync(0x8433)

        # get first part of response
        self.spi_rx(self.buf8)
        assert self.buf8[6] == 16 # we expect 16 bytes of data

        # get data
        buf= bytearray(16)
        self.spi_rx(buf)

        # print ifconfig
        print('ip:   %u.%u.%u.%u' % (buf[3], buf[2], buf[1], buf[0]))
        print('mask: %u.%u.%u.%u' % (buf[7], buf[6], buf[5], buf[4]))
        print('gw:   %u.%u.%u.%u' % (buf[11], buf[10], buf[9], buf[8]))
        print('dns:  %u.%u.%u.%u' % (buf[15], buf[14], buf[13], buf[12]))

    def wlan_set_scan_policy(self):
        self.spi_tx(b'\x21\x43\x34\x12')
        self.spi_tx(b'\x86\x8c\x08\x00')
        self.spi_tx(b'\x20\x34\x01\x01\x3c\x81\x99\x7d')
        self.sync(0x8c86)
        self.spi_rx(self.buf4)

    def wlan_get_network_list(self, index):
        self.spi_tx(b'\x21\x43\x34\x12')
        self.spi_tx(b'\x8c\x8c\x04\x00')
        self.buf4[0] = index
        self.buf4[1] = 1 # number of entries to get
        self.buf4[2] = 0
        self.buf4[3] = 0
        self.spi_tx(self.buf4)
        self.sync(0x8c8c)
        self.spi_rx(self.buf4)
        assert self.buf4[0] == 1 # number of entries returned

        # get the result
        buf44 = bytearray(44)
        self.spi_rx(buf44)
        ssid, ssid_len, sec_type, bssid, rssi, _, _, _ = struct.unpack('32sBB6sbbbb', buf44)
        ssid = ssid[0:ssid_len]
        return (ssid, sec_type, bssid, rssi)

    def wlan_connect(self, ssid, password):
        ssid = bytes(ssid, 'utf8')
        password = bytes(password, 'utf8')
        n = 9 + len(ssid) + len(password)
        n_pad = self.calc_pad(n)
        self.spi_tx(b'\x21\x43\x34\x12')
        buf = self.buf4
        buf[0] = 0x80
        buf[1] = 0x8c
        buf[2] = n + n_pad
        buf[3] = 0x00
        self.spi_tx(buf)
        buf = bytearray(9)
        buf[0] = 2 # sec_type
        buf[1] = len(ssid)
        #buf[2:8] = bssid
        buf[8] = len(password)
        self.spi_tx(buf + ssid + password + b'\x00' * n_pad)
        self.sync(0x8c80, timeout=5000)
        self.spi_rx(self.buf4)
        print(self.buf4)

        # wait for async connection event
        self.sync(0x0880, timeout=5000)
        self.spi_rx(bytearray(76))
        print('got wlan connect event')

        # wait for async IP acquired event
        self.sync(0x1825, timeout=5000)
        buf = bytearray(12)
        self.spi_rx(buf)
        print('got IP address')
        print('ip:  %u.%u.%u.%u' % (buf[3], buf[2], buf[1], buf[0]))
        print('gw:  %u.%u.%u.%u' % (buf[7], buf[6], buf[5], buf[4]))
        print('dns: %u.%u.%u.%u' % (buf[11], buf[10], buf[9], buf[8]))

    def get_host_by_name(self, host):
        host = bytes(host, 'utf8')
        n = 4 + len(host)
        n_pad = self.calc_pad(n)
        self.spi_tx(b'\x21\x43\x34\x12')
        buf = self.buf4
        buf[0] = 0x20
        buf[1] = 0x9c
        buf[2] = n + n_pad
        buf[3] = 0
        self.spi_tx(buf)
        buf[0] = len(host)
        buf[1] = 0
        buf[2] = 2
        buf[3] = 0
        self.spi_tx(buf + host + b'\x00' * n_pad)
        self.sync(0x9c20, timeout=2000)
        self.spi_rx(self.buf4)

        # wait for async event
        self.sync(0x9820, timeout=5000)
        buf = bytearray(8)
        self.spi_rx(buf)
        return '%u.%u.%u.%u' % (buf[7], buf[6], buf[5], buf[4])

    def socket_create(self):
        self.spi_tx(b'\x21\x43\x34\x12')
        self.spi_tx(b'\x01\x94\x04\x00')
        self.spi_tx(b'\x02\x01\x00\x00')
        self.sync(0x9401)
        self.spi_rx(self.buf4)
        return self.buf4[0] | self.buf4[1] << 8

    def socket_connect(self, s, ip, port):
        ip = [int(x) for x in ip.split('.')]
        port = ((port & 0xff) << 8) | ((port >> 8) & 0xff) # swap to network order
        self.spi_tx(b'\x21\x43\x34\x12')
        self.spi_tx(b'\x06\x94\x0c\x00')
        family = 0x20
        self.spi_tx(struct.pack('<HBBHHBBBB', 0, s, family, port, 0, ip[0], ip[1], ip[2], ip[3]))
        self.sync(0x9406)
        self.spi_rx(self.buf4)
        ret_code = self.buf4[0] | self.buf4[1] << 8
        assert ret_code == 0
        assert self.buf4[2] == s

        # wait for async event
        self.sync(0x1006, timeout=5000)
        buf = bytearray(8)
        self.spi_rx(buf)
        ret_code, = struct.unpack('<h', buf)
        assert buf[2] == s
        if ret_code != 0:
            raise OSError(-ret_code)

    def socket_send(self, s, buf):
        n = 4 + len(buf)
        n_pad = self.calc_pad(n)
        self.spi_tx(b'\x21\x43\x34\x12')
        self.spi_tx(struct.pack('<HHHBB', 0x940c, n + n_pad, len(buf), s, 8))
        self.spi_tx(buf)
        self.spi_tx(b'\x00' * n_pad)
        # no sync!
        return len(buf)

    def socket_recv(self, s, n):
        self.spi_tx(b'\x21\x43\x34\x12')
        self.spi_tx(struct.pack('<HHHBB', 0x940a, 4, n, s, 0))

        # wait for async event
        self.sync(0x100a, timeout=10000)
        self.spi_rx(self.buf4)
        ret_code, = struct.unpack('<h', self.buf4)
        assert self.buf4[2] == s
        if ret_code < 0:
            raise OSError(-ret_code)
        buf = bytearray(ret_code)
        self.spi_rx(buf)
        return bytes(buf)

    def socket_close(self, s):
        # doesn't work if the socket was closed by the other side
        self.spi_tx(b'\x21\x43\x34\x12')
        self.spi_tx(struct.pack('<HHBBBB', 0x9402, 4, s, 0, 0, 0))
        self.sync(0x1402)
        self.spi_rx(self.buf4)
        ret_code, = struct.unpack('<h', self.buf4)
        assert self.buf4[2] == s
        if ret_code < 0:
            raise OSError(-ret_code)

# pin and spi connection
p_rst = Pin('X2', Pin.OUT)
p_hib = Pin('X3', Pin.OUT)
p_irq = Pin('X4', Pin.IN)
p_cs = Pin('X5', Pin.OUT)
if 1:
    # hardware SPI
    spi = SPI('X', baudrate=1000000, phase=0, polarity=0)
else:
    # software SPI
    p_sck = Pin('X6', Pin.OUT)
    p_miso = Pin('X7', Pin.IN)
    p_mosi = Pin('X8', Pin.OUT)
    spi = SPI(baudrate=2000000000, sck=p_sck, mosi=p_mosi, miso=p_miso)
print(spi)

cc3100 = CC3100(spi, p_cs, p_irq, p_hib, p_rst)

# basic initialisation
cc3100.start()
cc3100.set_country_code(b'EU')
cc3100.set_connection_policy()
cc3100.wlan_profile_del_all()
cc3100.wlan_disconnect()
cc3100.get_version()
cc3100.get_ipv4_info()

# do a scan of AP's
cc3100.wlan_set_scan_policy()
time.sleep_ms(1000) # wait for scan to finish
last_ap = None
for i in range(19, -1, -1):
    data = cc3100.wlan_get_network_list(i)
    if last_ap is not None and data == last_ap:
        # got a duplicate so that means it's the end of the list
        break
    print(data)
    last_ap = data

# connect to an AP
cc3100.wlan_connect(AP_SSID, AP_PASS)
cc3100.get_ipv4_info()

# lookup some IP addresses
for host in (
    'micropython.org',
    'bbc.com',
    'google.com',
    '0.0.0.0',
    '192.168.0.1'):
    print(host, cc3100.get_host_by_name(host))

# create a socket and use it
for i in range(2):
    s = cc3100.socket_create()
    print('socket descriptor:', s)
    cc3100.socket_connect(s, '176.58.119.26', 80)
    #cc3100.socket_connect(s, '192.168.1.105', 8000)
    cc3100.socket_send(s, b'GET /ks/test.html HTTP/1.0\r\nHost: micropython.org\r\n\r\n')
    print(cc3100.socket_recv(s, 100))
    print(cc3100.socket_recv(s, 200))
    cc3100.socket_close(s)
