import sigrokdecode as srd
from math import floor

class Ann:
    (START_BIT, DATA_BIT, STOP_BIT, BYTE_VAL, SYNC,
     UPDI_OPCODE, UPDI_ADDRESS, UPDI_DATA, UPDI_ACK,
     UPDI_COMMAND) = range(10)

class UPDI:
    (DATA8, DATA16) = range(2)
    (KEY8, KEY16) = range(2)
    (PTR, PTRINC, PTRREG) = range(3)

    OP_LDS    = 0x00
    OP_LD     = 0x20
    OP_STS    = 0x40
    OP_ST     = 0x60
    OP_LDCS   = 0x80
    OP_REPEAT = 0xA0
    OP_STCS   = 0xC0

    OP_KEY     = 0xE0
    OP_READSIB = 0xE5

    def opname(cmd):
        if cmd == UPDI.OP_KEY:
            return "KEY"
        elif cmd == UPDI.OP_READSIB:
            return "READSIB"

        opcode = cmd & 0xE0
        if opcode == UPDI.OP_LDS:
            return "LDS"
        elif opcode == UPDI.OP_LD:
            return "LD"
        elif opcode == UPDI.OP_STS:
            return "STS"
        elif opcode == UPDI.OP_ST:
            return "ST"
        elif opcode == UPDI.OP_LDCS:
            return "LDCS"
        elif opcode == UPDI.OP_REPEAT:
            return "REPEAT"
        elif opcode == UPDI.OP_STCS:
            return "STCS"

        return None

class Decoder(srd.Decoder):
    api_version = 3
    id = 'avr_updi'
    name = 'AVR UPDI'
    longname = 'Microchip Universal Programming and Debug Interface'
    desc = 'Asynchronous one-wire serial bus carrying programming and debug instructions'
    license = 'gplv2+'
    inputs = ['logic']
    outputs = ['updi']
    channels = (
        {'id': 'updi', 'name': 'UPDI', 'desc': 'UPDI one-wire line'},
    )
    annotations = (
        ('start-bit', 'UPDI start bit'), # START_BIT
        ('data-bit', 'UPDI bits'),       # DATA_BIT
        ('stop-bit', 'UPDI stop bit'),   # STOP_BIT
        ('byte-value', 'Byte'),          # BYTE_VAL
        ('sync', 'Sync byte'),           # SYNC
        ('updi-opcode', 'UPDI opcode'),  # UPDI_OPCODE
        ('updi-addr', 'UPDI address'),   # UPDI_ADDRESS
        ('updi-data', 'UPDI data'),      # UPDI_DATA
        ('updi-ack', 'UPDI ACK'),        # UPDI_ACK
        ('updi', 'UPDI command'),        # UPDI_COMMAND
    )
    annotation_rows = (
        ('bits', 'Bits', (Ann.START_BIT, Ann.DATA_BIT, Ann.STOP_BIT,)),
        ('bytes', 'Byte values', (Ann.BYTE_VAL,)),
        ('fields', 'UPDI fields', (Ann.SYNC, Ann.UPDI_OPCODE, Ann.UPDI_ADDRESS, Ann.UPDI_DATA, Ann.UPDI_ACK,)),
        ('updi', 'UPDI', (Ann.UPDI_COMMAND,)),
    )

    def __init__(self):
        self.reset()

    def reset(self):
        self.samplerate = None
        self.samples_per_bit = None

    def start(self):
        self.out_ann = self.register(srd.OUTPUT_ANN)

    def metadata(self, key, value):
        if key == srd.SRD_CONF_SAMPLERATE:
            self.samplerate = value

    def putopcode(self, name):
        self.put(self.starttime, self.samplenum, self.out_ann,
                 [Ann.UPDI_OPCODE, ["Opcode: {0}".format(name), name]])

    def putcmd(self, name, operands=None):
        if operands is None:
            value = [name]
        else:
            value = ["{0} {1}".format(name, operands), name]

        self.put(self.cmdstarttime, self.samplenum, self.out_ann,
                 [Ann.UPDI_COMMAND, value])

    def wait_for_sync(self):
        """Waits for a valid SYNC condition, returning True when it finds one
        Returns False to wait again.
        As a side-effect will set self.samples_per_bit
        """
        # TODO: Calculate maximum start bit time based on last known baudrate
        max_bittime = 200

        # SYNC starts on a falling edge
        self.wait([{0: 'f'}])

        # start
        syncstart = self.samplenum
        self.wait([{0: 'r'}, {'skip': max_bittime * 3}])
        bittime = self.samplenum - syncstart

        if bittime > max_bittime:
            return False

        self.put(syncstart, self.samplenum, self.out_ann,
                 [Ann.START_BIT, ['Start bit', 'Start', 'S']])

        # now find 7 more edges and hope they're compatible
        for idx in range(7):
            bitstart = self.samplenum
            self.wait([{0: 'e'}, {'skip': max_bittime * 3}])
            bittime = self.samplenum - bitstart
            if bittime > max_bittime:
                return False
            self.put(bitstart, self.samplenum, self.out_ann, [Ann.DATA_BIT, ['x']])

        avgtime = floor((self.samplenum - syncstart + 4) / 8)

        self.samples_per_bit = avgtime

        # Read the 8th bit and the stop bits
        self.wait_for_data_bit('x')

        self.wait_for_stop()

        self.put(syncstart, self.samplenum, self.out_ann, [Ann.BYTE_VAL, ['55']])
        self.put(syncstart, self.samplenum, self.out_ann, [Ann.SYNC, ['SYNC']])

        self.syncstarttime = syncstart

        return True

    def wait_for_start(self):
        """Waits for a start condition.
        Sets self.starttime as a side-effect."""
        self.wait([{0: 'f'}])
        self.starttime = self.samplenum
        self.wait([{'skip': self.samples_per_bit}])
        self.put(self.starttime, self.samplenum, self.out_ann,
                 [Ann.START_BIT, ['Start bit', 'Start', 'S']])

    def wait_for_data_bit(self, ann_name=None):
        """Waits for a single bit time and returns the value of the bit.
        """
        bitstart = self.samplenum
        # Sample at midpoint of the bit time
        (bit,) = self.wait([{'skip': floor(self.samples_per_bit / 2)}])
        self.wait([{'skip': floor((self.samples_per_bit + 1) / 2)}])
        if ann_name is None:
            self.put(bitstart, self.samplenum, self.out_ann, [Ann.DATA_BIT, ["{0}".format(bit)]])
        else:
            self.put(bitstart, self.samplenum, self.out_ann, [Ann.DATA_BIT, [ann_name]])
        return bit

    def wait_for_stop(self):
        """Waits for both stop bits.
        """
        bitstart = self.samplenum
        (bit,) = self.wait([{'skip': floor(self.samples_per_bit / 2)}])
        self.wait([{'skip': self.samples_per_bit + floor((self.samples_per_bit + 1) / 2)}])
        ## TODO: Assert stopbit is low
        self.put(bitstart, self.samplenum, self.out_ann,
                 [Ann.STOP_BIT, ['Stop bit', 'Stop', 'T']])

    def wait_for_byte(self):
        self.wait_for_start()
        val = 0
        for idx in range(8):
            # UART is LSB-first
            val |= self.wait_for_data_bit() << idx
        self.wait_for_stop()
        self.bytestarttime = self.starttime
        self.put(self.bytestarttime, self.samplenum, self.out_ann, [Ann.BYTE_VAL, ["{:02X}".format(val)]])
        return val

    def wait_for_ACK(self):
        self.wait_for_byte()
        self.put(self.bytestarttime, self.samplenum, self.out_ann, [Ann.UPDI_ACK, ['ACK']])

    def wait_for_value(self, size, ann_id, ann_name):
        if size == UPDI.DATA8:
            val = self.wait_for_byte()
            wordstarttime = self.bytestarttime
            valstr = "{:02X}".format(val)
        elif size == UPDI.DATA16:
            val = self.wait_for_byte()
            wordstarttime = self.bytestarttime
            val |= self.wait_for_byte() << 8
            valstr = "{:04X}".format(val)
        else:
            raise Exception("TODO: size too big")

        self.put(self.bytestarttime, self.samplenum, self.out_ann,
                 [ann_id, ["{0}: {1}".format(ann_name, valstr), valstr]])

        return (val, valstr)

    def wait_for_data(self, size):
        return self.wait_for_value(size, Ann.UPDI_DATA, 'Data')

    def wait_for_addr(self, size):
        return self.wait_for_value(size, Ann.UPDI_ADDRESS, 'Addr')

    def handle_LDS(self, cmd):
        addrsize = (cmd >> 2) & 0x03
        size = cmd & 0x03

        (addr, addrstr) = self.wait_for_addr(addrsize)
        (val, valstr) = self.wait_for_data(size)

        self.putcmd("LDS", "{0} <- {1}".format(addrstr, valstr))

    def handle_LD(self, cmd):
        ptr = (cmd >> 2) & 0x03
        size = cmd & 0x03

        (val, valstr) = self.wait_for_data(cmd & 0x03)

        if self.cmdstarttime is not None:
            if ptr == UPDI.PTR:
                operands = '[P] <- '+valstr
            elif ptr == UPDI.PTRINC:
                operands = '[P+] <- '+valstr
            else:
                raise Exception("TODO: ptr reserved")

            self.putcmd("LD", operands)
        else:
            # subsequent bytes of a REPEAT LD ...
            self.put(self.starttime, self.samplenum, self.out_ann,
                     [Ann.UPDI_COMMAND, ['... <- '+valstr, valstr]])

    def handle_ST(self, cmd):
        ptr = (cmd >> 2) & 0x03
        size = cmd & 0x03

        (val, valstr) = self.wait_for_data(size)

        if ptr == UPDI.PTR:
            operands = '[P], '+valstr
        elif ptr == UPDI.PTRINC:
            operands = '[P+], '+valstr
        elif ptr == UPDI.PTRREG:
            operands = 'P, '+valstr
        else:
            raise Exception("TODO: ptr reserved")

        self.wait_for_ACK()

        self.putcmd("ST", operands)

    def handle_LDCS(self, cmd):
        addr = cmd & 0x0F

        (val, _) = self.wait_for_data(UPDI.DATA8)

        self.putcmd("LDCS", "[{0:02X}] <- {1:02X}".format(addr, val))

    def handle_REPEAT(self, cmd):
        size = cmd & 0x03

        (count, _) = self.wait_for_value(size, Ann.UPDI_DATA, 'Count')

        self.putcmd("REPEAT", str(count))

        while not self.wait_for_sync():
            pass

        self.cmdstarttime = self.syncstarttime

        cmd = self.wait_for_byte()

        opname = UPDI.opname(cmd)

        if opname is None:
            self.putopcode('??')
            return
        elif (cmd & 0xE0) == UPDI.OP_REPEAT:
            # REPEAT REPEAT is not allowed
            self.putopcode('REPEAT!')
            return

        self.putopcode(opname)
        handle = getattr(self, "handle_{0}".format(opname))

        for _ in range(count + 1):
            handle(cmd)
            self.cmdstarttime = None

    def handle_STCS(self, cmd):
        addr = cmd & 0x0F

        (val, _) = self.wait_for_data(UPDI.DATA8)

        self.putcmd("STCS", "[{0:02X}], {1:02X}".format(addr, val))

    def handle_KEY(self, cmd):
        size = cmd & 0x03

        if size == UPDI.KEY8:
            count = 8
        elif size == UPDI.KEY16:
            count = 16
        else:
            raise Exception("TODO size too big")

        key = ""
        keystarttime = None
        for _ in range(count):
            key += "{:02X}".format(self.wait_for_byte())
            if keystarttime is None:
                keystarttime = self.bytestarttime

        self.put(keystarttime, self.samplenum, self.out_ann,
                 [Ann.UPDI_DATA, ["Key: {0}".format(key), key]])

        self.putcmd("KEY", "0x{0}".format(key))

    def handle_READSIB(self, cmd):
        sib = b''
        sibstarttime = None
        for idx in range(16):
            sib += self.wait_for_byte().to_bytes(1)
            if sibstarttime is None:
                sibstarttime = self.bytestarttime

        sib_in_hex = sib.hex().upper()
        self.put(sibstarttime, self.samplenum, self.out_ann,
                 [Ann.UPDI_DATA, ["SIB: {0}".format(sib_in_hex), sib_in_hex]])

        # TODO: This does presume valid ASCII bytes
        sib_family = sib[0:7].decode('ascii').strip()
        sib_nvmver = sib[8:11].decode('ascii')
        sib_ocdver = sib[11:14].decode('ascii')
        sib_dbgosc = sib[15:16].decode('ascii')

        self.putcmd("READSIB", "family=\"{0}\" nvmver={1} ocdver={2} dbgosc={3}".format(sib_family, sib_nvmver, sib_ocdver, sib_dbgosc))

    def decode(self):
        if not self.samplerate:
            raise Exception('Cannot decode without samplerate.')

        while True:
            while not self.wait_for_sync():
                pass

            self.cmdstarttime = self.syncstarttime

            cmd = self.wait_for_byte()

            opname = UPDI.opname(cmd)

            if opname is not None:
                self.putopcode(opname)
                getattr(self, "handle_{0}".format(opname))(cmd)
            else:
                self.putopcode('??')
                # TODO: If we have data bytes coming soon, decode them?
