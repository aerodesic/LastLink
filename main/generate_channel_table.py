#! /usr/bin/env python3

import sys
import os
import argparse

class GenerateChannelTable:
    def __init__(self, input_name, cfile_name, hfile_name, output_name, device, debug):
        self.input_name = input_name
        self.input_file = None
        self.cfile_name = cfile_name
        self.cfile = None
        self.hfile_name = hfile_name
        self.hfile = None
        self.output_name = output_name
        self.device = "sample"
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

        # If output_name specified, then cfile_name and hfile_name are ignored.  both outputs to to one file.
        if self.output_name:
            try:
               self.cfile = open(self.output_name, "w")
               self.hfile = self.cfile
          
            except:
                rc = -2
                print("Unable to create %s" % self.output_name, file=sys.stderr)

        else:
            if self.cfile_name:
                try:
                    self.cfile = open(self.cfile_name, "w")
                except:
                    rc = -2
                    print("Unable to create %s" % self.cfile_name, file=sys.stderr)
            else:
               self.cfile = sys.stdout

            if self.hfile_name:
                try:
                    self.hfile = open(self.hfile_name, "w")
                except:
                    rc = -2
                    print("Unable to create %s" % self.hfile_name, file=sys.stderr)

        # If no names specified, send both to stdout
        if self.output_name == None and self.cfile_name == None and self.hfile_name == None:
            self.cfile = self.hfile = sys.stdout

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
    
            # Generate the channel table h file
            print("/*",                                                                                                             file=self.hfile)
            print(" * Channel table (constructed from %s"           % (self.input_name if self.input_name else "Standard Input"),   file=self.hfile)
            print(" *    !!! DO NOT MODIFY !!!",                                                                                    file=self.hfile)
            print(" */",                                                                                                            file=self.hfile)
            print("",                                                                                                               file=self.hfile)
            if self.hfile_name != None:
                print("#ifndef __%s_included__"                     % self.hfile_name,                                              file=self.hfile)
                print("#define __%s_included__"                     % self.hfile_name,                                              file=self.hfile)
            print("",                                                                                                               file=self.hfile)
            print("typedef struct channel_entry_%s_struct {"        % self.device,                                                  file=self.hfile)
            print("    uint8_t  freq_high;",                                                                                        file=self.hfile)
            print("    uint8_t  freq_mid;",                                                                                         file=self.hfile)
            print("    uint8_t  freq_low;",                                                                                         file=self.hfile)
            print("    uint8_t  datarate_group;",                                                                                   file=self.hfile)
            print("} channel_entry_%s_t;"                           % self.device,                                                  file=self.hfile)
            print("",                                                                                                               file=self.hfile)
            print("typedef struct datarate_entry_%s_struct {"       % self.device,                                                  file=self.hfile)
            print("    uint8_t  sf;",                                                                                               file=self.hfile)
            print("    uint8_t  bw;",                                                                                               file=self.hfile)
            print("    uint8_t  cr;",                                                                                               file=self.hfile)
            print("    uint8_t  tx;",                                                                                               file=self.hfile)
            print("    int      payload;",                                                                                          file=self.hfile)
            print("} datarate_entry_%s_t;"                          % self.device,                                                  file=self.hfile)
            print("",                                                                                                               file=self.hfile)
            print("typedef struct channel_table_%s_struct {"        % self.device,                                                  file=self.hfile)
            print("    const char*          device;",                                                                               file=self.hfile)
            print("    int                  low_domain_freq;",                                                                      file=self.hfile)
            print("    int                  high_domain_freq;",                                                                     file=self.hfile)
            print("    float                crystal;",                                                                              file=self.hfile)
            print("    channel_entry_%s_t   channels[%d];"          % (self.device, len(self.channels)),                            file=self.hfile)
            print("    datarate_entry_%s_t  datarates[%d][%d];"     % (self.device, len(self.groups), self.max_datarates),          file=self.hfile)
            print("    long                 bandwidth_bins[%d];"    % (len(self.bandwidth_bins)),                                   file=self.hfile)
            print("} channel_table_%s_t;"                           % self.device,                                                  file=self.hfile)
            print("",                                                                                                               file=self.hfile)
            print("",                                                                                                               file=self.hfile)
            if self.hfile_name != None:
                print("#endif /* __%s_included__ */"                % self.hfile_name,                                              file=self.hfile)

            # Generate the c file
            if self.hfile_name != None:
                print("#include \"%s\""                             % self.hfile_name,                                              file=self.cfile)
            print("",                                                                                                               file=self.cfile)
            print("const channel_table_%s_t channel_table_%s = {"   % (self.device, self.device),                                   file=self.cfile)
            print("    .device = %s,"                               % (('"' + self.device + '"') if self.device else "NULL"),       file=self.cfile)
            print("    .low_domain_freq = %d,"                      % (self.frequency_range[0]),                                    file=self.cfile)
            print("    .high_domain_freq = %d,"                     % (self.frequency_range[1]),                                    file=self.cfile)
            print("    .crystal = %.f,"                             % (self.crystal),                                               file=self.cfile)
            for channel in self.channels:
                chinfo = self.channels[channel]
                group = chinfo["group"]
                datarates = self.datarates[chinfo["group"]]
                print("    .channels[%d] = {"                       % channel,                                                      file=self.cfile)
                print("        .freq_high = %d,"                    % (chinfo["freq"][0]),                                          file=self.cfile)
                print("        .freq_mid = %d,"                     % (chinfo["freq"][1]),                                          file=self.cfile)
                print("        .freq_low = %d,"                     % (chinfo["freq"][2]),                                          file=self.cfile)
                print("        .datarate_group = %d,"               % (self.groups.index(group)),                                   file=self.cfile)
                print("    },",                                                                                                     file=self.cfile)
    
            for group in self.groups:
                print("    .datarates[%d] = {"                      % self.groups.index(group),                                     file=self.cfile)
                for dr in self.datarates[group]:
                    print("        [%d] = {"                        % (dr),                                                         file=self.cfile)
                    for field in self.datarates[group][dr]:
                        print("            .%s = %d,"               % (field, self.datarates[group][dr][field]),                    file=self.cfile)
                    print("        },",                                                                                             file=self.cfile)
                print("    },",                                                                                                     file=self.cfile)
    
            for bw in self.bandwidth_bins:
                print("    .bandwidth_bins[%d] = %d,"               % (self.bandwidth_bins.index(bw), int(bw)),                     file=self.cfile)
    
            print("};",                                                                                                             file=self.cfile)
            print("",                                                                                                               file=self.cfile)
            print("/* Create non-specific typedef for local use */",                                                                file=self.cfile)
            print("typedef channel_table_%s_t channel_table_t;"    % (self.device),                                                 file=self.cfile)
            print("#define channel_table channel_table_%s"         % (self.device),                                                 file=self.cfile)

        if self.input_file:
            self.input_file.close()

        if self.hfile == self.cfile:
            # Just close one below
            self.hfile = None

        elif self.cfile and self.cfile != sys.stdout:
            self.cfile.close()

        if self.hfile and self.hfile != sys.stdout:
            self.hfile.close()

        return rc


def main ():
    parser = argparse.ArgumentParser(usage="%(prog)s <channels file> [options]")

    parser.add_argument("-D", "--debug" , action="store_true", help="Enable debug messages to stderr")

    parser.add_argument("-d", "--device", type=str, help="specify a particular device", default=None)

    parser.add_argument("-C", "--cfile", type=str, help="specify a particular output file for .c code", default=None)

    parser.add_argument("-H", "--hfile", type=str, help="specify a particular output file for .h headers", default=None)

    parser.add_argument("-o", "--output", type=str, help="specify a particular output file for combined header and code ", default=None)

    (options, args) = parser.parse_known_args()

    if len(args) > 0:

        input_file = args[0]
 
        gt = GenerateChannelTable(input_file, options.cfile, options.hfile, options.output, options.device, options.debug)

        rc = gt.run()
    else:
        print("Missing channel file name", file=sys.stderr)
        rc = -1

    return rc

if __name__ == "__main__":
    main ()
