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

import sigrokdecode as srd

ann_sii_bits, ann_sdi_bits, ann_sdo_bits, ann_sii, ann_sdi, ann_sdo = range(6)

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
    )
    annotation_rows = (
        ('sii-bits', 'SII bits', (ann_sii_bits,)),
        ('sdi-bits', 'SDI bits', (ann_sdi_bits,)),
        ('sdo-bits', 'SDO bits', (ann_sdo_bits,)),
        ('sii', 'SII', (ann_sii,)),
        ('sdi', 'SDI', (ann_sdi,)),
        ('sdo', 'SDO', (ann_sdo,)),
    )

    def start(self):
        self.out_ann = self.register(srd.OUTPUT_ANN)

        self.oldpins = None
        self.oldsci = None
        self.bitnum = None
        self.bit_hightime = None
        self.bit_lowtime = None

    def putbit(self, ss, es, ann, val):
        self.put(ss, es, self.out_ann, [ann, ["%d" % val]])

    def putx(self, ss, es, ann, val):
        self.put(ss, es, self.out_ann, [ann, ["%02X" % val]])

    def sci_rise(self, pins, samplenum, bitnum):
        # SII / SDI are clocked on rising edge
        self.sii, self.sdi, _ = pins

        self.bit_old_hightime = self.bit_hightime
        self.bit_hightime = samplenum

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

        # Only bits 1 to 8 are interesting
        if bitnum < 1 or bitnum > 8:
            return

        # We now have the extent of the SII/SDI bit
        self.putbit(self.bit_old_lowtime, samplenum,
            ann_sii_bits, self.sii)
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
