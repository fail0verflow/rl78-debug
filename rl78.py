from pyftdi.gpio import GpioController
import serial
import time, struct, binascii, code, os

def delay(amount):
    now = start = time.perf_counter()
    while True:
        now = time.perf_counter()
        if now - start >= amount:
            return

# for C232HM-DDHSL-0 cable
WIRE_ORANGE = 1 << 0
WIRE_YELLOW = 1 << 1
WIRE_GREEN = 1 << 2
WIRE_BROWN = 1 << 3
WIRE_GRAY = 1 << 4
WIRE_PURPLE = 1 << 5
WIRE_WHITE = 1 << 6
WIRE_BLUE = 1 << 7

class Reset:
    def __init__(s, url):
        # init gpio mode with gray (conncted to RESET) and green (TOOL0) as outputs
        s.gpio = GpioController()
        s.gpio.open_from_url(url, direction = WIRE_GRAY | WIRE_GREEN)

    def enter_rom(s):
        s.gpio.set_direction(WIRE_GRAY | WIRE_GREEN)
        # RESET=0, TOOL0=0
        s.gpio.write_port(0)
        delay(.04)
        # RESET=1, TOOL0=0
        s.gpio.write_port(WIRE_GRAY)
        delay(.001)
        # RESET=1, TOOL0=1
        s.gpio.write_port(WIRE_GRAY | WIRE_GREEN)
        delay(.01)
        # stop driving TOOL0 (with this ftdi device - another one takes over)
        s.gpio.set_direction(WIRE_GRAY)

def read_all(port, size):
    data = b''
    while len(data) < size:
        data += port.read(size - len(data))
    assert len(data) == size
    return data

def size8(size):
    if size <= 0 or size > 0x100: return None
    if size == 0x100: size = 0
    return size

def pack24(x):
    assert x < (1 << 24)
    return struct.pack('<HB', x & 0xffff, x >> 16)

class ProtoA:
    SOH = 0x01
    STX = 0x02
    ETB = 0x17
    ETX = 0x03

    COM_RESET           = 0x00
    COM_19              = 0x19 # undocumented cmd. sets FSSQ=2
    COM_ERASE           = 0x22
    COM_PROG            = 0x40
    COM_VERIFY          = 0x13
    COM_BLANK_CHECK     = 0x32
    COM_BAUDRATE_SET    = 0x9a
    COM_SILICON_SIG     = 0xc0
    COM_SEC_SET         = 0xa0
    COM_SEC_GET         = 0xa1
    COM_SEC_RLS         = 0xa2
    COM_CHECKSUM        = 0xb0

    ST_COM_NUM_ERR  = 0x04
    ST_PARAM_ERR    = 0x05
    ST_ACK          = 0x06
    ST_SUM_ERR      = 0x07
    ST_VERIFY_ERR   = 0x0f
    ST_PROTECT_ERR  = 0x10
    ST_NACK         = 0x15
    ST_ERASE_ERR    = 0x1a
    ST_BLANK_ERR    = 0x1b
    ST_WRITE_ERR    = 0x1c

    def __init__(s, port):
        s.port = port

    def read_all(s, size):
        return read_all(s.port, size)

    def _checksum(s, data):
        csum = 0
        for d in data:
            csum -= d
            csum &= 0xff
        return csum

    def _checksum16(s, data):
        csum = 0
        for d in data:
            csum -= d
            csum &= 0xffff
        return csum

    def recv_frame(s):
        while s.port.read() != bytes([s.STX]):
            pass
        len_b = s.port.read()
        LEN = size8(struct.unpack('B', len_b)[0])
        recv_len = LEN + 2
        data = s.read_all(recv_len)
        #print('recv %s' % (binascii.hexlify(data)))
        if s._checksum(len_b + data[:LEN]) != data[LEN]:
            print('bad checksum')
        if data[LEN+1] != s.ETX:
            print('bad footer')
        return data[:LEN]

    def _send_frame(s, data, is_cmd = True, last_data = True):
        header = s.SOH if is_cmd else s.STX
        trailer = s.ETX if last_data else s.ETB
        LEN = size8(len(data))
        SUM = s._checksum(struct.pack('B', LEN) + data)
        cmd = struct.pack('BB%dBBB' % (len(data)), header, LEN, *data, SUM, trailer)
        #print('send %s' % (binascii.hexlify(cmd)))
        s.port.write(cmd)
        # discard the loopback bytes
        s.read_all(len(cmd))
        return s.recv_frame()

    def send_frame(s, data, is_cmd = True, last_data = True):
        while True:
            r = s._send_frame(data, is_cmd, last_data)
            if r[0] != s.ST_SUM_ERR:
                return r

    def reset(s):
        return s.send_frame(struct.pack('B', s.COM_RESET))

    def set_baudrate(s, baudrate, voltage):
        return s.send_frame(struct.pack('BBB', s.COM_BAUDRATE_SET, baudrate, voltage))

    def silicon_sig(s):
        r = s.send_frame(struct.pack('B', s.COM_SILICON_SIG))
        if r[0] != s.ST_ACK: return None
        return s.recv_frame()

    def security_get(s):
        r = s.send_frame(struct.pack('B', s.COM_SEC_GET))
        if r[0] != s.ST_ACK: return None
        return s.recv_frame()

    def security_set(s, sec):
        r = s.send_frame(struct.pack('B', s.COM_SEC_SET))
        if r[0] != s.ST_ACK: return None
        return s.send_frame(sec, False)[0] == s.ST_ACK

    def verify(s, addr, data):
        assert len(data) > 0
        SA = pack24(addr)
        EA = pack24(addr + len(data) - 1)
        r = s.send_frame(struct.pack('B', s.COM_VERIFY) + SA + EA)
        if r[0] != s.ST_ACK: return False
        for i in range(0, len(data), 0x100):
            last_data = len(data) - i <= 0x100
            r = s.send_frame(data[i:i+0x100], False, last_data)
        return r[0] == s.ST_ACK and r[1] == s.ST_ACK

    def checksum(s, addr, size):
        assert size > 0
        SA = pack24(addr)
        EA = pack24(addr + size - 1)
        r = s.send_frame(struct.pack('B', s.COM_CHECKSUM) + SA + EA)
        if r[0] != s.ST_ACK: return None
        return struct.unpack('<H', s.recv_frame())[0]

    def blank_check(s, addr, size=0x400):
        assert size > 0
        SA = pack24(addr)
        EA = pack24(addr + size - 1)
        # XXX
        D01 = struct.pack('B', 0)
        r = s.send_frame(struct.pack('B', s.COM_BLANK_CHECK) + SA + EA + D01)
        if r[0] not in (s.ST_ACK, s.ST_BLANK_ERR):
            return None
        # True means it is blank
        return r[0] == s.ST_ACK

    def invert_boot_cluster(s):
        # XXX can't be set via protoA :'(
        sec = s.security_get()
        sec = bytes([sec[0] ^ 1, *sec[1:]])
        return s.security_set(sec)

    def cmd19(s):
        # this is standalone "internal verify"
        addr = 0
        size = 0x400
        assert (((addr >> 8) & 0xff) & 3) == 0
        assert ((((addr + size - 1) >> 8) & 0xff) & 3) == 3
        SA = pack24(addr)
        EA = pack24(addr + size - 1)
        return s.send_frame(struct.pack('B', s.COM_19) + SA + EA)

    def erase_block(s, addr):
        return s.send_frame(struct.pack('B', s.COM_ERASE) + pack24(addr))

    def program(s, addr, data):
        SA = pack24(addr)
        EA = pack24(addr + len(data) - 1)
        r = s.send_frame(struct.pack('B', s.COM_PROG) + SA + EA)
        if r[0] != s.ST_ACK: return False
        for i in range(0, len(data), 0x100):
            last_data = len(data) - i <= 0x100
            r = s.send_frame(data[i:i+0x100], False, last_data)
        if r[0] != s.ST_ACK or r[1] != s.ST_ACK:
            return False
        # iverify status
        return s.recv_frame()

    def write(s, addr, data):
        # erase block = 0x400, everything else can use 0x100
        if addr % 0x400 or len(data) % 0x400:
            return False
        for i in range(0, len(data), 0x400):
            s.erase_block(addr + i)
        # XXX should be able to handle multiple blocks, not sure why it hangs
        #s.program(addr, data)
        for i in range(0, len(data), 0x100):
            s.program(addr + i, data[i:i+0x100])
        return s.verify(addr, data)

class ProtoOCD:
    SYNC = 0x00
    PING = 0x90
    UNLOCK = 0x91
    READ = 0x92
    WRITE = 0x93
    EXEC = 0x94
    EXIT_RETI = 0x95
    EXIT_RAM = 0x97

    PONG = bytes([3, 3])

    ST_UNLOCK_ALREADY = 0xf0
    ST_UNLOCK_LOCKED = 0xf1
    ST_UNLOCK_OK = 0xf2
    ST_UNLOCK_SUM = 0xf3
    ST_UNLOCK_NG = 0xf4

    def __init__(s, port):
        s.port = port
    def read_all(s, size):
        return read_all(s.port, size)
    def checksum(s, data):
        csum = 0
        for d in data:
            csum += d
            csum &= 0xff
        csum -= 1
        csum &= 0xff
        return csum
    def send_cmd(s, cmd):
        #print('send %s' % (binascii.hexlify(cmd)))
        s.port.write(cmd)
        # discard the loopback bytes
        s.read_all(len(cmd))
    def wait_ack(s):
        while s.read_all(1) != bytes([s.SYNC]):
            pass
    def sync(s):
        s.send_cmd(struct.pack('B', s.SYNC))
        s.wait_ack()
    def ping(s):
        s.send_cmd(struct.pack('B', s.PING))
        return s.read_all(len(s.PONG)) == s.PONG
        #return s.read_all(len(ping_result)) == ping_result
    def unlock(s, ocd_id, corrupt_sum = False):
        s.send_cmd(struct.pack('B', s.UNLOCK))
        status = s.read_all(1)[0]
        # f0: already unlocked
        # f1: need to send
        if status == s.ST_UNLOCK_ALREADY:
            print('already unlocked')
            return True
        if status != s.ST_UNLOCK_LOCKED:
            print('unexpected status')
            return False
        csum = s.checksum(ocd_id)
        if corrupt_sum:
            csum += 1
            csum &= 0xff
        s.send_cmd(struct.pack('10BB', *ocd_id, csum))
        status = s.read_all(1)[0]
        # f2: success
        # f3: checksum mismatch
        # f4: checksum matched but ocd_id did not (could trigger flash erase?)
        if status != s.ST_UNLOCK_OK:
            print('unlock failed: %x' % (status))
        return status == s.ST_UNLOCK_OK
    def read(s, offset, size):
        size8_ = size8(size)
        if size8_ is None: return None
        s.send_cmd(struct.pack('<BHB', s.READ, offset, size8_))
        return s.read_all(size)
    def write(s, addr, data):
        size = size8(len(data))
        if size is None: return None
        s.send_cmd(struct.pack('<BHB%dB' (len(data)), s.WRITE, addr, size, *data))
        return s.read_all(1)[0] == s.WRITE
    def call_f07e0(s):
        s.send_cmd(struct.pack('B', s.EXEC))
        return s.read_all(1)[0] == s.EXEC
    def leave(s, to_ram = False):
        cmd = s.EXIT_RAM if to_ram else s.EXIT_RETI
        s.send_cmd(struct.pack('B', cmd))
        return s.read_all(1)[0] == cmd

class RL78:
    MODE_A_1WIRE = b'\x3a'
    MODE_A_2WIRE = b'\x00'
    MODE_OCD = b'\xc5'
    BAUDRATE_INIT = 115200
    BAUDRATE_FAST = 1000000
    def __init__(s, gpio_url, uart_port):
        s.reset_ctl = Reset(gpio_url)
        s.port = serial.Serial(uart_port, baudrate=s.BAUDRATE_INIT, timeout=0, stopbits=2)
        s.a = ProtoA(s.port)
        s.ocd = ProtoOCD(s.port)
        s.mode = None
    def reset(s, mode):
        s.mode = mode
        s.port.baudrate = s.BAUDRATE_INIT
        s.reset_ctl.enter_rom()
        s.port.write(s.mode)
        # we'll see the reset as a null byte. discard it and the init byte
        read_all(s.port, 2)
        # send baudrate cmd (required) & sync
        baudrate = s.BAUDRATE_FAST if s.mode != s.MODE_OCD else s.BAUDRATE_INIT
        rl78_br = {115200: 0, 250000: 1, 500000: 2, 1000000: 3}[baudrate]
        # 21 = 2.1v
        # really just sets internal voltage regulator to output 1.7, 1.8 or 2.1 volts
        # regulator seems to auto-adjust anyways...
        # feeding with 1.7v uses slower mode, 1.8v and 2.1v are same, slightly faster speed
        r = s.a.set_baudrate(rl78_br, 21)
        s.port.baudrate = baudrate
        if r[0] != ProtoA.ST_ACK: return False
        delay(.01)
        if s.mode != s.MODE_OCD:
            r = s.a.reset()
            if r[0] != ProtoA.ST_ACK: return False
        else:
            s.ocd.wait_ack()
            if not s.ocd.ping(): return False
        return True

if __name__ == '__main__':
    rl78 = RL78('ftdi://ftdi:232h/0', 'COM5')
    if not rl78.reset(RL78.MODE_A_1WIRE):
        print('failed to init a')
        exit()
    print('sig', binascii.hexlify(rl78.a.silicon_sig()))
    print('sec', binascii.hexlify(rl78.a.security_get()))
    code.InteractiveConsole(locals = locals()).interact('Entering shell...')
