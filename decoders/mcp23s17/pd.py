import sigrokdecode as srd

ann_select, ann_addr, ann_value, ann_warning = range(4)

# Register names in IOCON.BANK=0 mode
registers = {
    0x00: 'IODIRA',
    0x01: 'IODIRB',
    0x02: 'IPOLA',
    0x03: 'IPOLB',
    0x04: 'GPINTENA',
    0x05: 'GPINTENB',
    0x06: 'DEFVALA',
    0x07: 'DEFVALB',
    0x08: 'INTCONA',
    0x09: 'INTCONB',
    0x0A: 'IOCON',
    0x0B: 'IOCON',
    0x0C: 'GPPUA',
    0x0D: 'GPPUB',
    0x0E: 'INTFA',
    0x0F: 'INTFB',
    0x10: 'INTCAPA',
    0x11: 'INTCAPB',
    0x12: 'GPIOA',
    0x13: 'GPIOB',
    0x14: 'OLATA',
    0x15: 'OLATB',
}

class Decoder(srd.Decoder):
    api_version = 2
    id = 'mcp23s17'
    name = 'MCP23S17'
    longname = 'Microchip MCP23S17'
    desc = '16-bit GPIO expander'
    license = 'gplv2+'
    inputs = ['spi']
    outputs = ['mcp23x17']
    annotations = (
        ('select', 'Slave select byte'),
        ('addr', 'Registers addresses'),
        ('value', 'Values written to/read from the device'),
        ('warnings', 'Human-readable warnings'),
    )
    annotation_rows = (
        ('registers', 'Registers', (ann_select, ann_addr, ann_value)),
        ('warnings', 'Warnings', (ann_warning,)),
    )

    def start(self):
        self.out_ann = self.register(srd.OUTPUT_ANN)

    def putselect(self, ss, es, slave, direction):
        self.put(ss, es, self.out_ann, [ann_select,
            ['%s %02X' % (direction[0].upper() + direction[1:], slave)]])

    def putaddr(self, ss, es, addr):
        self.put(ss, es, self.out_ann, [ann_addr,
            ['Addr %02X' % addr]])

    def putv(self, ss, es, name, val):
        self.put(ss, es, self.out_ann, [ann_value,
            ['%s: %s' % (name, val)]])

    def putwarn(self, ss, es, msg):
        self.put(ss, es, self.out_ann, [ann_warning, [msg]])

    def decode(self, ss, es, data):
        if data[0] != 'TRANSFER':
            return

        _, mosi, miso = data

        # Ignore zero-byte packets
        if not len(mosi):
            return

        # Read the slave-select byte
        slavesel = mosi[0].val
        slave = slavesel >> 1
        direction = 'read' if slavesel & 1 else 'write'

        self.putselect(mosi[0].ss, mosi[0].es, slave, direction)

        if len(mosi) < 2:
            return

        addr = mosi[1].val

        self.putaddr(mosi[1].ss, mosi[1].es, addr)

        if direction == 'read':
            values = miso
        else:
            values = mosi

        values.pop(0)
        values.pop(0)

        while values:
            value = values.pop(0)

            if addr not in registers:
                self.putwarn(value.ss, value.es,
                    msg='Unknown register %02X' % addr)
                addr += 1
                continue

            name = registers[addr]

            # Try to merge 16bit transfers into singles
            if addr % 2 == 0 and len(values):
                value_hi = values.pop(0)
                val = value.val | (value_hi.val << 8)

                self.putv(value.ss, value_hi.es, name[:-1],
                    '%04X' % val)

                addr += 2
            else:
                self.putv(value.ss, value.es, name,
                    '%02X' % value.val)

                addr += 1
