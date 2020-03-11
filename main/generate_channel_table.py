#! /usr/bin/env python3

import sys, os

class GenerateChannelTable:
    def __init__(self, filename):
        self.filename = filename
        self.channels = {}
        self.datarates = {}
        self.frequency_range = []
        self.groups = []
        self.max_datarates = 0
        self.bandwidth_bins = []

    def calc_freq(self, freq):
        frf = round(freq / self.pll_step);
        msb = int(frf) / 65536;
        mid = (int(frf) / 256) % 256;
        lsb = int(frf) % 256;
        return [ msb, mid, lsb ]

    def process_crystal(self, line, lines):
        self.pll_step = float(lines[line][1]) / 2**19
        return line + 1

    def process_frequencies(self, line, lines):
        self.frequency_range = [ int(lines[line][1]), int(lines[line][2]) ]
        return line + 1

    def process_bandwidth_bins(self, line, lines):
        done = False
        while not done and line < len(lines):
            if lines[line][0] == "end":
                done = True
                line = line + 1
            else:
                fields = lines[line]
                # print("datarates: %s" % fields)
                line += 1
                bandwidth = int(fields[0])
                self.bandwidth_bins.append(bandwidth)

        return line

    def process_channels(self, line, lines):
        # Build the channel table
        fields = lines[line][1:]
        # print("process_channels: %s" % fields)
        line = line + 1
        group = fields[0]
        first_channel = int(fields[1])
        last_channel= int(fields[2])
        start_freq = int(fields[3])
        step_size = int(fields[4])

        freq = start_freq
        for c in range(first_channel, last_channel + 1):
            if c in self.channels:
                print("channel %d already defined group '%s'" % (c, self.channels[c]["group"]), file=sys.stderr)
            else:
                self.channels[c] = { "group": group, "freq": self.calc_freq(freq), "hz": freq }
                freq += step_size

        self.groups.append(group)

        # get the datarates
        done = False
        num_datarates = 0
        while not done and line < len(lines):
            if lines[line][0] == "end":
                done = True
                line = line + 1
            else:
                fields = lines[line]
                # print("datarates: %s" % fields)
                line += 1
                if fields[0] in [ "datarate", "dr" ]:
                    if group not in self.datarates:
                        self.datarates[group] = {}

                    dr = int(fields[1])
                    if dr+1 > num_datarates:
                        num_datarates = dr + 1
                    sf = None
                    bw = None
                    txpower = None
                    payload = None
                    f = 2
                    while f < len(fields):
                        if fields[f] in [ "sf", "spread-factor" ]:
                            sf = int(fields[f+1])
                            f += 2
                        elif fields[f] in [ "bw", "bandwidth" ]:
                            bw = int(fields[f+1])
                            if bw in self.bandwidth_bins:
                                bw = self.bandwidth_bins.index(bw)
                            else:
                                print("Invalid bandwidth '%s' in datarate group '%s'" % (bw, group), file=sys.stderr)
                                bw = 0
                            
                            f += 2
                        elif fields[f] in [ "tx", "txpower" ]:
                            txpower = int(fields[f+1])
                            f += 2
                        elif fields[f] in [ "payload" ]:
                            payload = int(fields[f+1])
                            f += 2

                    if dr in self.datarates[group]:
                        print("datarate %d already defined in group '%s'" % (dr, group), file=sys.stderr)
                    elif sf == None:
                        print("sf not defined for group '%s' datarate %d" % (group, dr), file=sys.stderr)
                    elif bw == None:
                        print("bw not defined for group '%s' datarate %d" % (group, dr), file=sys.stderr)
                    elif txpower == None:
                        print("txpower not defined for group '%s' datarate %d" % (group, dr), file=sys.stderr)
                    elif payload == None:
                        print("payload not defined for group '%s' datarate %d" % (group, dr), file=sys.stderr)

                    else:
                        self.datarates[group][dr] = { "sf": sf, "bw": bw, "tx": txpower, "payload": payload }

        if num_datarates > self.max_datarates:
            self.max_datarates = num_datarates

        return line

    def run(self):
        lines = []

        if self.filename == None:
            input_file = sys.stdin
        else:
            try:
                input_file = open(self.filename, "r")
            except:
                input_file = None

        if input_file == None:
            return -1
        else:
            for line in input_file:
                line = line.strip().split();

                # Skip comment lines
                if len(line) > 0 and line[0][0] != "#":
                    lines.append(line)

            if input_file != sys.stdin:
               input_file.close()

        # print(lines)

        line = 0
        while line < len(lines):
            if lines[line][0] == "crystal":
                line = self.process_crystal(line, lines)

            elif lines[line][0] == "bandwidth_bins":
                line = self.process_bandwidth_bins(line+1, lines)

            elif lines[line][0] == "channels":
                line = self.process_channels(line, lines)

            elif lines[line][0] == "frequency":
                line = self.process_frequencies(line, lines)

            else:
                print("Invalid line: %s" % (lines[line]), file=sys.stderr)
                line = line + 1

        # Generate the channel table
        print("/*")
        print(" * Channel table (constructed from %s" % (self.filename if self.filename else "Standard Input"))
        print(" *    !!! DO NOT MODIFY !!!")
        print(" */")
        print("typedef struct channel_entry_struct {")
        print("    uint8_t  freq_high;")
        print("    uint8_t  freq_mid;")
        print("    uint8_t  freq_low;")
        print("    uint8_t  datarate_group;")
        print("} channel_entry_t;")
        print("")
        print("typedef struct datarate_entry_struct {")
        print("    uint8_t  sf;")
        print("    uint8_t  bw;")
        print("    uint8_t  tx;")
        print("    int      payload;")
        print("} datarate_entry_t;")
        print("")
        print("typedef struct channel_table_struct {")
        print("    channel_entry_t   channels[%d];" % len(self.channels))
        print("    datarate_entry_t  datarates[%d][%d];" % (len(self.groups), self.max_datarates))
        print("    long              bandwidth_bins[%d];" % (len(self.bandwidth_bins)))
        print("} channel_table_t;")
        print("")
        print("channel_table_t channel_table = {")

        for channel in self.channels:
            chinfo = self.channels[channel]
            group = chinfo["group"]
            datarates = self.datarates[chinfo["group"]]
            print("    .channels[%d] = {" % channel)
            print("        .freq_high = %d," % (chinfo["freq"][0]))
            print("        .freq_mid = %d," % (chinfo["freq"][1]))
            print("        .freq_low = %d," % (chinfo["freq"][2]))
            print("        .datarate_group = %d," % (self.groups.index(group)))
            print("    },")

        for group in self.groups:
            print("    .datarates[%d] = {" % self.groups.index(group))
            for dr in self.datarates[group]:
                print("        [%d] = {" % (dr))
                for field in self.datarates[group][dr]:
                    print("            .%s = %d," % (field, self.datarates[group][dr][field]))
                print("        },")
            print("    },")   

        for bw in self.bandwidth_bins:
            print("    .bandwidth_bins[%d] = %d," % (self.bandwidth_bins.index(bw), int(bw)))

        print("};")


def main ():
     return GenerateChannelTable(sys.argv[1] if len(sys.argv) > 1 else None).run()

if __name__ == "__main__":
    main ()
