##
## This file is part of the libsigrokdecode project.
##
## Copyright (C) 2015 Paul Evans <leonerd@leonerd.org.uk>
##
## This program is free software; you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation; either version 2 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program; if not, write to the Free Software
## Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
##

from collections import namedtuple

import sigrokdecode as srd

SII_BITNAME = (
    '', '', 'XA1', 'XA0', 'BS1', 'WR#', 'OE#', 'BS2', 'PAGEL',
)

Command = namedtuple('Command', ['name'])

COMMANDS = {
    0x80: Command('CE'),
    0x40: Command('WFUSE'),
    0x20: Command('WLOCK'),
    0x10: Command('WFLASH'),
    0x11: Command('WEEP'),
    0x08: Command('RSIG'),   # Also ROSC
    0x04: Command('RFUSE'),  # Also RLOCK
    0x02: Command('RFLASH'),
    0x03: Command('REEP'),
}

(ann_sii_bits, ann_sdi_bits, ann_sdo_bits, ann_sii, ann_sdi, ann_sdo,
    ann_instr, ann_warn) = range(8)

class Decoder(srd.Decoder):
    api_version = 2
    id = 'avr_hvsp'
    name = 'AVR HVSP'
    longname = 'AVR High-Voltage Serial Programming'
    desc = 'High-Voltage Serial Programming of AVR ATtiny microcontrollers'
    license = 'gplv2'
    inputs = ['logic']
    outputs = ['avr_hvsp']
    channels = (
        {'id': 'sci', 'name': 'SCI', 'desc': 'Clock'},
        {'id': 'sii', 'name': 'SII', 'desc': 'Serial Instructions In'},
        {'id': 'sdi', 'name': 'SDI', 'desc': 'Serial Data In'},
        {'id': 'sdo', 'name': 'SDO', 'desc': 'Serial Data Out'},
    )
    annotations = (
        ('sii-bits', 'SII bits'),
        ('sdi-bits', 'SDI bits'),
        ('sdo-bits', 'SDO bits'),
        ('sii', 'SII'),
        ('sdi', 'SDI'),
        ('sdo', 'SDO'),
        ('instr', 'Instructions'),
        ('warnings', 'Warnings'),
    )
    annotation_rows = (
        ('sii-bits', 'SII bits', (ann_sii_bits,)),
        ('sdi-bits', 'SDI bits', (ann_sdi_bits,)),
        ('sdo-bits', 'SDO bits', (ann_sdo_bits,)),
        ('sii', 'SII', (ann_sii,)),
        ('sdi', 'SDI', (ann_sdi,)),
        ('sdo', 'SDO', (ann_sdo,)),
        ('instr', 'Instructions', (ann_instr,)),
        ('warnings', 'Warnings', (ann_warn,)),
    )

    def start(self):
        self.out_ann = self.register(srd.OUTPUT_ANN)

        self.oldpins = None
        self.oldsci = None
        self.bitnum = None
        self.bit_hightime = None
        self.bit_lowtime = None

        self.cur_cmd = 0
        self.cur_addr = 0
        self.cur_val = 0

    def putbit(self, ss, es, ann, val, name=None):
        if name is None:
            self.put(ss, es, self.out_ann, [ann, ["%d" % val]])
            return
        if name.endswith('#'):
            val = not val
            name = name[:-1]
        self.put(ss, es, self.out_ann, [ann, [name if val else ' ']])

    def putx(self, ss, es, ann, val):
        self.put(ss, es, self.out_ann, [ann, ["%02X" % val]])

    def puti(self, ss, es, val):
        self.put(ss, es, self.out_ann, [ann_instr, [val]])

    def putwarn(self, ss, es, warning):
        self.put(ss, es, self.out_ann, [ann_warn, [warning]])

    def do_load_addr(self, ss, es, bs, sdi):
        if bs == 0:
            self.puti(ss, es, "LLA %02X" % sdi)
            self.cur_addr = (self.cur_addr & 0xFF00) | sdi
        elif bs == 1:
            self.puti(ss, es, "LHA %02X" % sdi)
            self.cur_addr = (sdi << 8) | (self.cur_addr & 0x00FF)
        else:
            self.putwarn(ss, es, "LA invalid BS=%d" % bs)

    def do_load_data(self, ss, es, bs, sdi):
        if bs == 0:
            self.puti(ss, es, "LLD %02X" % sdi)
            self.cur_val = (self.cur_val & 0xFF00) | sdi
        elif bs == 1:
            self.puti(ss, es, "LHD %02X" % sdi)
            self.cur_val = (sdi << 8) | (self.cur_val & 0x00FF)

    def do_load_command(self, ss, es, sdi):
        if sdi not in COMMANDS:
            self.putwarn("CMD unknown")
            self.cur_cmd = None
            return

        self.cur_cmd = COMMANDS[sdi]
        self.puti(ss, es, "CMD %s" % self.cur_cmd.name)

    def do_word(self, ss, es, sii, sdi):
        # The SII bits map to hardware control lines in an interesting manner
        # 6   5   4   3   2   1   0
        # XA1 XA0 BS1 WR# OE# BS2 PAGEL
        xa = (sii & 0x60) >> 5;
        bs = (sii & 0x10) >> 4 | (sii & 0x02)
        wr = not (sii & 0x08)
        oe = not (sii & 0x04)
        pagel = (sii & 0x01)

        # WR, OE and PAGEL are only allowed if XA==3
        if xa != 3:
            if wr:
                self.putwarn(ss, es, "Invalid WR")
                return
            if oe:
                self.putwarn(ss, es, "Invalid OE")
                return
            if pagel:
                self.putwarn(ss, es, "Invalid PAGEL")
                return

        # At most one of wr, oe and pagel are allowed
        if wr and oe:
            self.putwarn(ss, es, "Invalid WR+OE")
            return
        if wr and pagel:
            self.putwarn(ss, es, "Invalid WR+PAGEL")
            return
        if oe and pagel:
            self.putwarn(ss, es, "Invalid OE+PAGEL")
            return

        if xa == 0:
            self.do_load_addr(ss, es, bs, sdi)
        elif xa == 1:
            self.do_load_data(ss, es, bs, sdi)
        elif xa == 2:
            if bs != 0:
                self.putwarn(ss, es, "Invalid BS")
            else:
                self.do_load_command(ss, es, sdi)
        else:
            self.putwarn(ss, es, "TODO: XA=3")

    def sci_rise(self, pins, samplenum, bitnum):
        # SII / SDI are clocked on rising edge
        self.sii, self.sdi, _ = pins

        self.bit_old_hightime = self.bit_hightime
        self.bit_hightime = samplenum

        if bitnum == 0:
            self.word_ss = samplenum

        # Only bits 1 to 8 are interesting
        if bitnum < 1 or bitnum > 8:
            return

        # Now we have the extent of the SDO bit
        self.putbit(self.bit_old_hightime, samplenum,
            ann_sdo_bits, self.sdo)

        if bitnum == 1:
            self.word_out_ss = self.bit_old_hightime
            self.sdo_word = 0

        self.sdo_word = (self.sdo_word << 1) | self.sdo

        if bitnum == 8:
            self.putx(self.word_out_ss, samplenum, ann_sdo, self.sdo_word)

    def sci_fall(self, pins, samplenum, bitnum):
        # SDO is clocked on falling edge
        _, _, self.sdo = pins

        self.bit_old_lowtime = self.bit_lowtime
        self.bit_lowtime = samplenum

        if bitnum == 10:
            self.do_word(self.word_ss, samplenum,
                self.sii_word, self.sdi_word)

        # Only bits 1 to 8 are interesting
        if bitnum < 1 or bitnum > 8:
            return

        # We now have the extent of the SII/SDI bit
        if bitnum > 1:
            self.putbit(self.bit_old_lowtime, samplenum,
                ann_sii_bits, self.sii, name=SII_BITNAME[bitnum])
        self.putbit(self.bit_old_lowtime, samplenum,
            ann_sdi_bits, self.sdi)

        if bitnum == 1:
            self.word_in_ss = self.bit_old_lowtime
            self.sii_word = 0
            self.sdi_word = 0

        self.sii_word = (self.sii_word << 1) | self.sii
        self.sdi_word = (self.sdi_word << 1) | self.sdi

        if bitnum == 8:
            self.putx(self.word_in_ss, samplenum, ann_sii, self.sii_word)
            self.putx(self.word_in_ss, samplenum, ann_sdi, self.sdi_word)

    def decode(self, ss, es, data):
        for samplenum, pins in data:
            if pins == self.oldpins:
                continue
            self.oldpins = pins

            sci, sii, sdi, sdo = pins

            if self.oldsci is None:
                self.oldsci = sci
                self.bitnum = -1
                continue

            if sci == self.oldsci:
                continue
            self.oldsci = sci

            if sci:
                self.bitnum += 1
                if self.bitnum > 10:
                    self.bitnum = 0;

                self.sci_rise((sii, sdi, sdo), samplenum, self.bitnum)
            else:
                self.sci_fall((sii, sdi, sdo), samplenum, self.bitnum)
