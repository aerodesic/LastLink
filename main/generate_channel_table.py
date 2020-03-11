#! /usr/bin/env python

import sys, os

class GenerateChannelTable:
    def __init__(self, filename):
        self.filename = filename
        self.channels = {}
        self.datarates = {}
        self.frequency_range = []

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

    def process_channels(self, line, lines):
        # Build the channel table
        fields = lines[line][1:]
        print("process_channels: %s" % fields)
        line = line + 1
        group = fields[0]
        first_channel = int(fields[1])
        last_channel= int(fields[2])
        start_freq = int(fields[3])
        step_size = int(fields[4])

        freq = start_freq
        for c in range(first_channel, last_channel + 1):
            if c in self.channels:
                print("channel %d already defined group '%s'" % c, self.channels[c]["group"])
            else:
                self.channels[c] = { "group": group, "freq": self.calc_freq(freq), "hz": freq }
                freq += step_size

        # get the datarates
        done = False
        while not done and line < len(lines):
            if lines[line][0] == "end":
                done = True
                line = line + 1
            else:
                fields = lines[line]
                print("datarates: %s" % fields)
                line += 1
                if fields[0] in [ "datarate", "dr" ]:
                    if group not in self.datarates:
                        self.datarates[group] = {}

                    dr = int(fields[1])
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
                            f += 2
                        elif fields[f] in [ "tx", "txpower" ]:
                            txpower = int(fields[f+1])
                            f += 2
                        elif fields[f] in [ "payload" ]:
                            payload = int(fields[f+1])
                            f += 2

                    if dr in self.datarates[group]:
                        print("datarate %d already defined in group '%s'" % (dr, group))
                    elif sf == None:
                        print("sf not defined for group '%s' datarate %d" % (group, dr))
                    elif bw == None:
                        print("bw not defined for group '%s' datarate %d" % (group, dr))
                    elif txpower == None:
                        print("txpower not defined for group '%s' datarate %d" % (group, dr))
                    elif payload == None:
                        print("payload not defined for group '%s' datarate %d" % (group, dr))

                    else:
                        self.datarates[group][dr] = { "sf": sf, "bw": bw, "tx": txpower, "payload": payload }

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

        print(lines)

        line = 0
        while line < len(lines):
            if lines[line][0] == "crystal":
                line = self.process_crystal(line, lines)

            elif lines[line][0] == "channels":
                line = self.process_channels(line, lines)

            elif lines[line][0] == "frequency":
                line = self.process_frequencies(line, lines)

            else:
                print("Invalid line: %s" % lines[line])
                line = line + 1

        for channel in self.channels:
            # print("    channel %d group '%s' frequency %.0f freq %s" % (channel, self.channels[channel]["group"],  self.channels[channel]["hz"], self.channels[channel]["freq"]))
            print("channel %s: %s" % (channel, self.channels[channel]))

        for group in self.datarates:
            print("Datarate group '%s'" % group)
            for datarate in self.datarates[group]:
                sf = self.datarates[group][datarate]["sf"]
                bw = self.datarates[group][datarate]["bw"]
                txpower = self.datarates[group][datarate]["tx"]
                payload = self.datarates[group][datarate]["payload"]
                print("    datarate %d spread-factor %d bandwidth %d txpower %d payload %d" % (datarate, sf, bw, txpower, payload))



def main ():
     return GenerateChannelTable(sys.args[1] if len(sys.argv) > 1 else None).run()

if __name__ == "__main__":
    main ()
