#! /usr/bin/env python3

import sys
import os
import argparse

class GenerateChannelTable:
    def __init__(self, input_name, output_name, device, debug):
        self.input_name = input_name
        self.input_file = None
        self.output_name = output_name
        self.output_file = None
        self.device = device
        self.debug = debug
        self.channels = {}
        self.datarates = {}
        self.frequency_range = []
        self.groups = []
        self.max_datarates = 0
        self.bandwidth_bins = []
        self.crystal = 0
        self.pll_step = 0

    def calc_freq(self, freq):
        frf = round(freq / self.pll_step);
        msb = int(frf) / 65536;
        mid = (int(frf) / 256) % 256;
        lsb = int(frf) % 256;
        return [ msb, mid, lsb ]

    def process_crystal(self, line, lines):
        self.crystal = float(lines[line][1])
        self.pll_step = self.crystal / 2**19

        if self.debug:
            print("process_crystal: crystal %.f pll_step %.f" % (self.crystal, self.pll_step), file=sys.stderr)

        return line + 1

    def process_device(self, line, lines):
        if self.debug:
            print("process_device: %s" % lines[line], file=sys.stderr)

        # Save only if not overriden
        if not self.device:
            self.device = lines[line][1]

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
        if self.debug:
            print("process_channels: %s" % fields, file=sys.stdout)
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
                    cr = 5  # default
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
                        elif fields[f] in [ "cr", "coding-rate" ]:
                            cr = int(fields[f+1])
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
                        self.datarates[group][dr] = { "sf": sf, "bw": bw, "cr": cr, "tx": txpower, "payload": payload }

        if num_datarates > self.max_datarates:
            self.max_datarates = num_datarates

        return line

    def run(self):
        rc = 0

        if self.output_name:
            try:
                self.output_file = open(self.output_name, "w")
            except:
                rc = -2
                print("Unable to create %s" % self.output_name, file=sys.stderr)

        if rc == 0 and self.input_name:
            try:
                self.input_file = open(self.input_name, "r")
            except:
                rc = 3
                print("Unable to open %s" % self.input_name, file=sys.stderr)

        if rc == 0:
            lines = []

            for line in self.input_file:
                line = line.strip().split();

                # Skip comment lines
                if len(line) > 0 and line[0][0] != "#":
                    lines.append(line)

            # Close now before we do the work
            if self.input_name != None:
               self.input_file.close()
               self.input_file = None

            # print(lines)
    
            line = 0
            while line < len(lines):
                if lines[line][0] == "crystal":
                    line = self.process_crystal(line, lines)
    
                elif lines[line][0] == "device":
                    line = self.process_device(line, lines)
    
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
            print("/*",                                                                                                             file=self.output_file)
            print(" * Channel table (constructed from %s"          % (self.input_name if self.input_name else "Standard Input"),    file=self.output_file)
            print(" *    !!! DO NOT MODIFY !!!",                                                                                    file=self.output_file)
            print(" */",                                                                                                            file=self.output_file)
            print("typedef struct channel_entry_struct {",                                                                          file=self.output_file)
            print("    uint8_t  freq_high;",                                                                                        file=self.output_file)
            print("    uint8_t  freq_mid;",                                                                                         file=self.output_file)
            print("    uint8_t  freq_low;",                                                                                         file=self.output_file)
            print("    uint8_t  datarate_group;",                                                                                   file=self.output_file)
            print("} channel_entry_t;",                                                                                             file=self.output_file)
            print("",                                                                                                               file=self.output_file)
            print("typedef struct datarate_entry_struct {",                                                                         file=self.output_file)
            print("    uint8_t  sf;",                                                                                               file=self.output_file)
            print("    uint8_t  bw;",                                                                                               file=self.output_file)
            print("    uint8_t  cr;",                                                                                               file=self.output_file)
            print("    uint8_t  tx;",                                                                                               file=self.output_file)
            print("    int      payload;",                                                                                          file=self.output_file)
            print("} datarate_entry_t;",                                                                                            file=self.output_file)
            print("",                                                                                                               file=self.output_file)
            print("typedef struct channel_table_struct {",                                                                          file=self.output_file)
            print("    const char*       device;",                                                                                  file=self.output_file)
            print("    float             crystal;",                                                                                 file=self.output_file)
            print("    channel_entry_t   channels[%d];"             % len(self.channels),                                           file=self.output_file)
            print("    datarate_entry_t  datarates[%d][%d];"        % (len(self.groups), self.max_datarates),                       file=self.output_file)
            print("    long              bandwidth_bins[%d];"       % (len(self.bandwidth_bins)),                                   file=self.output_file)
            print("} channel_table_t;",                                                                                             file=self.output_file)
            print("",                                                                                                               file=self.output_file)
            print("channel_table_t channel_table = {",                                                                              file=self.output_file)
    
            print("    .device = %s,"                               % (('"' + self.device + '"') if self.device else "NULL"),       file=self.output_file)
            print("    .crystal = %.f,"                             % (self.crystal),                                               file=self.output_file)
            for channel in self.channels:
                chinfo = self.channels[channel]
                group = chinfo["group"]
                datarates = self.datarates[chinfo["group"]]
                print("    .channels[%d] = {"                       % channel,                                                      file=self.output_file)
                print("        .freq_high = %d,"                    % (chinfo["freq"][0]),                                          file=self.output_file)
                print("        .freq_mid = %d,"                     % (chinfo["freq"][1]),                                          file=self.output_file)
                print("        .freq_low = %d,"                     % (chinfo["freq"][2]),                                          file=self.output_file)
                print("        .datarate_group = %d,"               % (self.groups.index(group)),                                   file=self.output_file)
                print("    },",                                                                                                     file=self.output_file)
    
            for group in self.groups:
                print("    .datarates[%d] = {"                      % self.groups.index(group),                                     file=self.output_file)
                for dr in self.datarates[group]:
                    print("        [%d] = {"                        % (dr),                                                         file=self.output_file)
                    for field in self.datarates[group][dr]:
                        print("            .%s = %d,"               % (field, self.datarates[group][dr][field]),                    file=self.output_file)
                    print("        },",                                                                                             file=self.output_file)
                print("    },",                                                                                                     file=self.output_file)
    
            for bw in self.bandwidth_bins:
                print("    .bandwidth_bins[%d] = %d,"               % (self.bandwidth_bins.index(bw), int(bw)),                             file=self.output_file)
    
            print("};",                                                                                                                     file=self.output_file)

        if self.input_file:
            self.input_file.close()

        if self.output_file:
            self.output_file.close()
        return rc


def main ():
    parser = argparse.ArgumentParser(usage="%(prog)s <channels file> [options]")

    parser.add_argument("-D", "--debug" , action="store_true", help="Enable debug messages to stderr")

    parser.add_argument("-d", "--device", type=str, help="specify a particular device", default=None)

    parser.add_argument("-o", "--output", type=str, help="specify a particular output file", default=None)

    (options, args) = parser.parse_known_args()

    if len(args) > 0:

        input_file = args[0]
 
        gt = GenerateChannelTable(input_file, options.output, options.device, options.debug)

        rc = gt.run()
    else:
        print("Missing channel file name", file=sys.stderr)
        rc = -1

    return rc

if __name__ == "__main__":
    main ()
