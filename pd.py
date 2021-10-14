##
## This file is part of the libsigrokdecode project.
##
## Copyright (C) 2011-2014 Uwe Hermann <uwe@hermann-uwe.de>
## Copyright (C) 2016 Gerhard Sittig <gerhard.sittig@gmx.net>
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
## along with this program; if not, see <http://www.gnu.org/licenses/>.
##

# Note the implementation details:
#
# Although the Atmel literature suggests (does not explicitly mandate,
# but shows in diagrams) that two stop bits are used in the protocol,
# the decoder loses synchronization with ATxmega generated responses
# when it expects more than one stop bit. Since the chip's hardware is
# fixed, this is not an implementation error in some programmer software.
# Since this is a protocol decoder which does not participate in the
# communication (does not actively send data), we can read the data
# stream with one stop bit, and transparently keep working when two
# are used.
#
# Annotations in the UART fields level differ from Atmel literature.
# Wrong parity bits are referred to as "parity error". Low stop bits are
# referred to as "frame error".
#
# The PDI component in the device starts disabled. Enabling PDI
# communication is done by raising DATA and clocking RESET with a
# minimum frequency. PDI communication automatically gets disabled when
# RESET "is inactive" for a certain period of time. The specific timing
# conditions are rather fuzzy in the literature (phrased weakly), and
# are device dependent (refer to the minumum RESET pulse width). This
# protocol decoder implementation internally prepares for but currently
# does not support these enable and disable phases. On the one hand it
# avoids excess external dependencies or wrong results for legal input
# data. On the other hand the decoder works when input streams start in
# the middle of an established connection.
#
# Communication peers detect physical collisions. The decoder can't.
# Upon collisions, a peer will cease any subsequent transmission, until
# a BREAK is seen. Synchronization can get enforced by sending two BREAK
# conditions. The first will cause a collision, the second will re-enable
# the peer. The decoder has no concept of physical collisions. It stops
# the interpretation of instructions when BREAK is seen, and assumes
# that a new instruction will start after BREAK.
#
# This protocol decoder only supports PDI communication over UART frames.
# It lacks support for PDI over JTAG. This would require separation into
# multiple protocol decoder layers (UART physical, JTAG physical, PDI
# instructions, optionally device support on top of PDI. There is some
# more potential for future extensions:
# - The JTAG physical has dedicated TX and RX directions. This decoder
#   only picks up communicated bytes but does not check which "line"
#   they are communicated on (not applicable to half duplex UART).
# - PDI over JTAG uses "special frame error" conditions to communicate
#   additional symbols: BREAK (0xBB with parity 1), DELAY (0xDB with
#   parity 1), and EMPTY (0xEB with parity 1).
# - Another "device support" layer might interpret device specific
#   timings, and might map addresses used in memory access operations
#   to component names, or even register names and bit fields(?). It's
#   quite deep a rabbithole though...

import sigrokdecode as srd
from collections import namedtuple

class Ann:
    '''Annotation and binary output classes.'''
    (
        BIT, START, DATA, PARITY_OK, PARITY_ERR,
        STOP_OK, STOP_ERR, BREAK,
        OPCODE, DATA_PROG, DATA_DEV, PDI_BREAK,
        ENABLE, DISABLE, COMMAND,
    ) = range(15)
    (
        BIN_BYTES,
    ) = range(1)

Bit = namedtuple('Bit', 'val ss es')

class PDI:
    '''PDI protocol instruction opcodes, and operand formats.'''
    (
        OP_LDS, OP_LD, OP_STS, OP_ST,
        OP_LDCS, OP_REPEAT, OP_STCS, OP_KEY,
    ) = range(8)
    pointer_format_nice = [
        '*(ptr)',
        '*(ptr++)',
        'ptr',
        'ptr++ (rsv)',
    ]
    pointer_format_terse = [
        '*p',
        '*p++',
        'p',
        '(rsv)',
    ]
    # ctrl_reg_name = {
    #     0: 'status',
    #     1: 'reset',
    #     2: 'ctrl',
    # }
    ctrl_reg_name = {
        0: 'STATUS A',
        1: 'STATUS B',
        2: 'CTRL A',
        3: 'CTRL B',
        7: 'ASI KEY STATUS',
        8: 'ASI RESET REQ',
        9: 'ASI CTRL A',
        10: 'ASI SYS CTRL A',
        11: 'ASI SYS STAT',
        12: 'ASI CRC STATUS'
    }

class Decoder(srd.Decoder):
    api_version = 3
    id = 'avr_updi'
    name = 'AVR UPDI'
    longname = 'Atmel Program and Debug Interface'
    desc = 'Atmel ATxmega Program and Debug Interface (PDI) protocol.'
    license = 'gplv2+'
    # inputs = ['logic']
    inputs = ['uart']
    outputs = []
    tags = ['Debug/trace']
    # channels = (
    #     {'id': 'reset', 'name': 'RESET', 'desc': 'RESET / PDI_CLK'},
    #     {'id': 'data', 'name': 'DATA', 'desc': 'PDI_DATA'},
    # )
    annotations = (
        ('uart-bit', 'UART bit'),
        ('start-bit', 'Start bit'),
        ('data-bit', 'Data bit'),
        ('parity-ok', 'Parity OK bit'),
        ('parity-err', 'Parity error bit'),
        ('stop-ok', 'Stop OK bit'),
        ('stop-err', 'Stop error bit'),
        ('break', 'BREAK condition'),
        ('opcode', 'Instruction opcode'),
        ('data-prog', 'Programmer data'),
        ('data-dev', 'Device data'),
        ('pdi-break', 'BREAK at PDI level'),
        ('enable', 'Enable PDI'),
        ('disable', 'Disable PDI'),
        ('cmd-data', 'PDI command with data'),
    )
    annotation_rows = (
        ('uart_bits', 'UART bits', (Ann.BIT,)),
        ('uart_fields', 'UART fields', (Ann.START, Ann.DATA, Ann.PARITY_OK,
                                        Ann.PARITY_ERR, Ann.STOP_OK, Ann.STOP_ERR, Ann.BREAK)),
        ('pdi_fields', 'PDI fields', (Ann.OPCODE, Ann.DATA_PROG, Ann.DATA_DEV,
                                      Ann.PDI_BREAK)),
        ('pdi_cmds', 'PDI commands', (Ann.ENABLE, Ann.DISABLE, Ann.COMMAND)),
    )
    binary = (
        ('bytes', 'PDI protocol bytes'),
    )

    def __init__(self):
        self.reset()

    def reset(self):
        self.samplerate = None
        self.clear_state()

    def clear_state(self):
        # Track bit times and bit values.
        self.ss_last_fall = None
        self.data_sample = None
        self.ss_curr_fall = None
        # Collect UART frame bits into byte values.
        self.bits = []
        self.zero_count = 0
        self.zero_ss = None
        self.break_ss = None
        self.break_es = None
        self.clear_insn()

    def clear_insn(self):
        # Collect instructions and their arguments,
        # properties of the current instructions.
        self.insn_rep_count = 0
        self.insn_opcode = None
        self.insn_wr_counts = []
        self.insn_rd_counts = []
        # Accumulation of data items as bytes pass by.
        self.insn_dat_bytes = []
        self.insn_dat_count = 0
        self.insn_ss_data = None
        # Next layer "commands", instructions plus operands.
        self.cmd_ss = None
        self.cmd_insn_parts_nice = []
        self.cmd_insn_parts_terse = []

    def metadata(self, key, value):
        if key == srd.SRD_CONF_SAMPLERATE:
            self.samplerate = value

    def start(self):
        self.out_ann = self.register(srd.OUTPUT_ANN)
        self.out_binary = self.register(srd.OUTPUT_BINARY)

    def put_ann_bit(self, bit_nr, ann_idx):
        b = self.bits[bit_nr]
        self.put(b.ss, b.es, self.out_ann, [ann_idx, [str(b.val)]])

    def put_ann_data(self, bit_nr, ann_data):
        b = self.bits[bit_nr]
        self.put(b.ss, b.es, self.out_ann, ann_data)

    def put_ann_row_val(self, ss, es, row, value):
        self.put(ss, es, self.out_ann, [row, value])

    def put_bin_bytes(self, ss, es, row, value):
        self.put(ss, es, self.out_binary, [row, value])

    def handle_byte(self, ss, es, byteval):
        '''Handle a byte at the PDI protocol layer.'''

        # Handle BREAK conditions, which will abort any
        # potentially currently executing instruction.
        is_break = byteval is None
        if is_break:
            self.cmd_insn_parts_nice.append('BREAK')
            self.cmd_insn_parts_terse.append('BRK')
            self.insn_rep_count = 0
            # Will FALLTHROUGH to "end of instruction" below.


        if not self.insn_dat_count and byteval in [0, 0x55]:
            return

        # Decode instruction opcodes and argument sizes
        # from the first byte of a transaction.
        if self.insn_opcode is None and not is_break:
            opcode = (byteval & 0xe0) >> 5
            arg30 = byteval & 0x0f
            arg32 = (byteval & 0x0c) >> 2
            arg10 = byteval & 0x03
            self.insn_opcode = opcode
            self.cmd_ss = ss
            mnemonics = None
            if opcode == PDI.OP_LDS:
                # LDS: load data, direct addressing.
                # Writes an address, reads a data item.
                width_addr = arg32 + 1
                width_data = arg10 + 1
                self.insn_wr_counts = [width_addr]
                self.insn_rd_counts = [width_data]
                mnemonics = [
                    'Insn: LDS a{:d}, m{:d}'.format(width_addr, width_data),
                    'LDS a{:d}, m{:d}'.format(width_addr, width_data), 'LDS',
                ]
                self.cmd_insn_parts_nice = ['LDS']
                self.cmd_insn_parts_terse = ['LDS']
            elif opcode == PDI.OP_LD:
                # LD: load data, indirect addressing.
                # Reads a data item, with optional repeat.
                ptr_txt = PDI.pointer_format_nice[arg32]
                ptr_txt_terse = PDI.pointer_format_terse[arg32]
                width_data = arg10 + 1
                self.insn_wr_counts = []
                self.insn_rd_counts = [width_data]
                if self.insn_rep_count:
                    self.insn_rd_counts.extend(self.insn_rep_count * [width_data])
                    self.insn_rep_count = 0
                mnemonics = [
                    'Insn: LD {:s} m{:d}'.format(ptr_txt, width_data),
                    'LD {:s} m{:d}'.format(ptr_txt, width_data), 'LD',
                ]
                self.cmd_insn_parts_nice = ['LD', ptr_txt]
                self.cmd_insn_parts_terse = ['LD', ptr_txt_terse]
            elif opcode == PDI.OP_STS:
                # STS: store data, direct addressing.
                # Writes an address, writes a data item.
                width_addr = arg32 + 1
                width_data = arg10 + 1
                self.insn_wr_counts = [width_addr, width_data]
                self.insn_rd_counts = []
                mnemonics = [
                    'Insn: STS a{:d}, i{:d}'.format(width_addr, width_data),
                    'STS a{:d}, i{:d}'.format(width_addr, width_data), 'STS',
                ]
                self.cmd_insn_parts_nice = ['STS']
                self.cmd_insn_parts_terse = ['STS']
            elif opcode == PDI.OP_ST:
                # ST: store data, indirect addressing.
                # Writes a data item, with optional repeat.
                ptr_txt = PDI.pointer_format_nice[arg32]
                ptr_txt_terse = PDI.pointer_format_terse[arg32]
                width_data = arg10 + 1
                self.insn_wr_counts = [width_data]
                self.insn_rd_counts = []
                if self.insn_rep_count:
                    self.insn_wr_counts.extend(self.insn_rep_count * [width_data])
                    self.insn_rep_count = 0
                mnemonics = [
                    'Insn: ST {:s} i{:d}'.format(ptr_txt, width_data),
                    'ST {:s} i{:d}'.format(ptr_txt, width_data), 'ST',
                ]
                self.cmd_insn_parts_nice = ['ST', ptr_txt]
                self.cmd_insn_parts_terse = ['ST', ptr_txt_terse]
            elif opcode == PDI.OP_LDCS:
                # LDCS: load control/status.
                # Loads exactly one byte.
                reg_num = arg30
                reg_txt = PDI.ctrl_reg_name.get(reg_num, 'r{:d}'.format(reg_num))
                reg_txt_terse = '{:d}'.format(reg_num)
                self.insn_wr_counts = []
                self.insn_rd_counts = [1]
                mnemonics = [
                    'Insn: LDCS {:s}, m1'.format(reg_txt),
                    'LDCS {:s}, m1'.format(reg_txt), 'LDCS',
                ]
                self.cmd_insn_parts_nice = ['LDCS', reg_txt]
                self.cmd_insn_parts_terse = ['LDCS', reg_txt_terse]
            elif opcode == PDI.OP_STCS:
                # STCS: store control/status.
                # Writes exactly one byte.
                reg_num = arg30
                reg_txt = PDI.ctrl_reg_name.get(reg_num, 'r{:d}'.format(reg_num))
                reg_txt_terse = '{:d}'.format(reg_num)
                self.insn_wr_counts = [1]
                self.insn_rd_counts = []
                mnemonics = [
                    'Insn: STCS {:s}, i1'.format(reg_txt),
                    'STCS {:s}, i1'.format(reg_txt), 'STCS',
                ]
                self.cmd_insn_parts_nice = ['STCS', reg_txt]
                self.cmd_insn_parts_terse = ['STCS', reg_txt_terse]
            elif opcode == PDI.OP_REPEAT:
                # REPEAT: sets repeat count for the next instruction.
                # Reads repeat count from following bytes.
                width_data = arg10 + 1
                self.insn_wr_counts = [width_data]
                self.insn_rd_counts = []
                mnemonics = [
                    'Insn: REPEAT i{:d}'.format(width_data),
                    'REPEAT i{:d}'.format(width_data), 'REP',
                ]
                self.cmd_insn_parts_nice = ['REPEAT']
                self.cmd_insn_parts_terse = ['REP']
            elif opcode == PDI.OP_KEY:
                # KEY: set activation key (enables PDIBUS mmap access).
                # Writes a sequence of 8 bytes, fixed length.
                width_data = 8
                self.insn_wr_counts = [width_data]
                self.insn_rd_counts = []
                mnemonics = [
                    'Insn: KEY i{:d}'.format(width_data),
                    'KEY i{:d}'.format(width_data), 'KEY',
                ]
                self.cmd_insn_parts_nice = ['KEY']
                self.cmd_insn_parts_terse = ['KEY']

            # Emit an annotation for the instruction opcode.
            self.put_ann_row_val(ss, es, Ann.OPCODE, mnemonics)

            # Prepare to write/read operands/data bytes.
            self.insn_dat_bytes = []
            if self.insn_wr_counts:
                self.insn_dat_count = self.insn_wr_counts[0]
                return
            if self.insn_rd_counts:
                self.insn_dat_count = self.insn_rd_counts[0]
                return
            # FALLTHROUGH.
            # When there are no operands or data bytes to read,
            # then fall through to the end of the instruction
            # handling below (which emits annotations).

        # Read bytes which carry operands (addresses, immediates)
        # or data values for memory access.
        if self.insn_dat_count and not is_break:

            # Accumulate received bytes until another multi byte
            # data item is complete.
            if not self.insn_dat_bytes:
                self.insn_ss_data = ss
            self.insn_dat_bytes.append(byteval)
            self.insn_dat_count -= 1
            if self.insn_dat_count:
                return

            # Determine the data item's duration and direction,
            # "consume" its length spec (to simplify later steps).
            data_ss = self.insn_ss_data
            data_es = es
            if self.insn_wr_counts:
                data_ann = Ann.DATA_PROG
                data_width = self.insn_wr_counts.pop(0)
            elif self.insn_rd_counts:
                data_ann = Ann.DATA_DEV
                data_width = self.insn_rd_counts.pop(0)

            # PDI communicates multi-byte data items in little endian
            # order. Get a nice textual representation of the number,
            # wide and narrow for several zoom levels.
            self.insn_dat_bytes.reverse()
            data_txt_digits = ''.join(['{:02x}'.format(b) for b in self.insn_dat_bytes])
            data_txt_hex = '0x' + data_txt_digits
            data_txt_prefix = 'Data: ' + data_txt_hex
            data_txts = [data_txt_prefix, data_txt_hex, data_txt_digits]
            self.insn_dat_bytes = []

            # Emit an annotation for the data value.
            self.put_ann_row_val(data_ss, data_es, data_ann, data_txts)

            # Collect detailled information which describes the whole
            # command when combined (for a next layer annotation,
            # spanning the complete command).
            self.cmd_insn_parts_nice.append(data_txt_hex)
            self.cmd_insn_parts_terse.append(data_txt_digits)

            # Send out write data first until exhausted,
            # then drain expected read data.
            if self.insn_wr_counts:
                self.insn_dat_count = self.insn_wr_counts[0]
                return
            if self.insn_rd_counts:
                self.insn_dat_count = self.insn_rd_counts[0]
                return

            # FALLTHROUGH.
            # When all operands and data bytes were seen,
            # terminate the inspection of the instruction.

        # Postprocess the instruction after its operands were seen.
        cmd_es = es
        cmd_txt_nice = ' '.join(self.cmd_insn_parts_nice)
        cmd_txt_terse = ' '.join(self.cmd_insn_parts_terse)
        cmd_txts = [cmd_txt_nice, cmd_txt_terse]
        self.put_ann_row_val(self.cmd_ss, cmd_es, Ann.COMMAND, cmd_txts)
        if self.insn_opcode == PDI.OP_REPEAT and not is_break:
            # The last communicated data item is the repeat
            # count for the next instruction (i.e. it will
            # execute N+1 times when "REPEAT N" is specified).
            count = int(self.cmd_insn_parts_nice[-1], 0)
            self.insn_rep_count = count

        # Have the state for instruction decoding cleared, but make sure
        # to carry over REPEAT count specs between instructions. They
        # start out as zero, will be setup by REPEAT instructions, need
        # to get passed to the instruction which follows REPEAT. The
        # instruction which sees a non-zero repeat count which will
        # consume the counter and drop it to zero, then the counter
        # remains at zero until the next REPEAT instruction.
        save_rep_count = self.insn_rep_count
        self.clear_insn()
        self.insn_rep_count = save_rep_count


    def decode(self, ss, es, data):
        try:
            # debug_print("decode")
            ptype, rxtx, pdata = data

            # Ignore unknown/unsupported ptypes.
            # if ptype not in ('STARTBIT', 'DATA', 'STOPBIT'):
            if ptype not in ('DATA'):
                return

            self.handle_byte(ss, es, pdata[0])

        except Exception as e:
            pass
