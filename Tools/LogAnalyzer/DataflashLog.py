#
# Code to abstract the parsing of APM Dataflash log files, currently only used by the LogAnalyzer
#
# Initial code by Andrew Chapman (amchapman@gmail.com), 16th Jan 2014
#

from __future__ import print_function
import collections
import os
import numpy
import bisect
import sys
import ctypes

from VehicleType import VehicleType, VehicleTypeString

class Format(object):
    '''Data channel format as specified by the FMT lines in the log file'''
    def __init__(self,msgType,msgLen,name,types,labels):
        self.NAME    = 'FMT'
        self.msgType = msgType
        self.msgLen  = msgLen
        self.name    = name
        self.types   = types
        self.labels  = labels.split(',')

    def __str__(self):
        return "%8s %s" % (self.name, `self.labels`)

    @staticmethod
    def trycastToFormatType(value,valueType):
        '''using format characters from libraries/DataFlash/DataFlash.h to cast strings to basic python int/float/string types
        tries a cast, if it does not work, well, acceptable as the text logs do not match the format, e.g. MODE is expected to be int'''
        try:
            if valueType in "fcCeELd":
                return float(value)
            elif valueType in "bBhHiIMQq":
                return int(value)
            elif valueType in "nNZ":
                return str(value)
        except:
            pass
        return value

    def to_class(self):
        members = dict(
            NAME = self.name,
            labels = self.labels[:],
        )

        fieldtypes = [i for i in self.types]
        fieldlabels = self.labels[:]

        # field access
        for (label, _type) in zip(fieldlabels, fieldtypes):
            def createproperty(name, format):
                # extra scope for variable sanity
                # scaling via _NAME and def NAME(self): return self._NAME / SCALE
                propertyname = name
                attributename = '_' + name
                p = property(lambda x:getattr(x, attributename),
                             lambda x, v:setattr(x,attributename, Format.trycastToFormatType(v,format)))
                members[propertyname] = p
                members[attributename] = None
            createproperty(label, _type)

        # repr shows all values but the header
        members['__repr__'] = lambda x: "<{cls} {data}>".format(cls=x.__class__.__name__, data = ' '.join(["{}:{}".format(k,getattr(x,'_'+k)) for k in x.labels]))

        def init(a, *x):
            if len(x) != len(a.labels):
                raise ValueError("Invalid Length")
            #print(list(zip(a.labels, x)))
            for (l,v) in zip(a.labels, x):
                try:
                    setattr(a, l, v)
                except Exception as e:
                    print("{} {} {} failed".format(a,l,v))
                    print(e)

        members['__init__'] = init

        # finally, create the class
        cls = type(\
            'Log__{:s}'.format(self.name),
            (object,),
            members
        )
        #print(members)
        return cls


class logheader(ctypes.LittleEndianStructure):
    _fields_ = [ \
        ('head1', ctypes.c_uint8),
        ('head2', ctypes.c_uint8),
        ('msgid', ctypes.c_uint8),
    ]
    def __repr__(self):
        return "<logheader head1=0x{self.head1:x} head2=0x{self.head2:x} msgid=0x{self.msgid:x} ({self.msgid})>".format(self=self)


class BinaryFormat(ctypes.LittleEndianStructure):
    NAME = 'FMT'
    MSG = 128
    SIZE = 0
    FIELD_FORMAT = {
        'b': ctypes.c_int8,
        'B': ctypes.c_uint8,
        'h': ctypes.c_int16,
        'H': ctypes.c_uint16,
        'i': ctypes.c_int32,
        'I': ctypes.c_uint32,
        'f': ctypes.c_float,
        'd': ctypes.c_double,
        'n': ctypes.c_char * 4,
        'N': ctypes.c_char * 16,
        'Z': ctypes.c_char * 64,
        'c': ctypes.c_int16,# * 100,
        'C': ctypes.c_uint16,# * 100,
        'e': ctypes.c_int32,# * 100,
        'E': ctypes.c_uint32,# * 100,
        'L': ctypes.c_int32,
        'M': ctypes.c_uint8,
        'q': ctypes.c_int64,
        'Q': ctypes.c_uint64,
    }

    FIELD_SCALE = {
        'c': 100,
        'C': 100,
        'e': 100,
        'E': 100,
    }

    _packed_ = True
    _fields_ = [ \
        ('head', logheader),
        ('type', ctypes.c_uint8),
        ('length', ctypes.c_uint8),
        ('name', ctypes.c_char * 4),
        ('types', ctypes.c_char * 16),
        ('labels', ctypes.c_char * 64),
    ]
    def __repr__(self):
        return "<{cls} {data}>".format(cls=self.__class__.__name__, data = ' '.join(["{}:{}".format(k,getattr(self,k)) for (k,_) in self._fields_[1:]]))

    def to_class(self):
        members = dict(
            NAME = self.name,
            MSG = self.type,
            SIZE = self.length,
            labels = self.labels.split(",") if self.labels else [],
            _pack_ = True)

        fieldtypes = [i for i in self.types]
        fieldlabels = self.labels.split(",")
        if self.labels and (len(fieldtypes) != len(fieldlabels)):
            print("Broken FMT message for {} .. ignoring".format(self.name), file=sys.stderr)
            return None

        fields = [('head',logheader)]

        # field access
        for (label, _type) in zip(fieldlabels, fieldtypes):
            def createproperty(name, format):
                # extra scope for variable sanity
                # scaling via _NAME and def NAME(self): return self._NAME / SCALE
                propertyname = name
                attributename = '_' + name
                scale = BinaryFormat.FIELD_SCALE.get(format, None)
                p = property(lambda x:getattr(x, attributename))
                if scale is not None:
                    p = property(lambda x:getattr(x, attributename) / scale) 
                members[propertyname] = p
                try:
                    fields.append((attributename, BinaryFormat.FIELD_FORMAT[format]))
                except KeyError:
                    print('ERROR: Failed to add FMT type: {}, with format: {}'.format(attributename, format))
                    raise
            createproperty(label, _type)
        members['_fields_'] = fields

        # repr shows all values but the header
        members['__repr__'] = lambda x: "<{cls} {data}>".format(cls=x.__class__.__name__, data = ' '.join(["{}:{}".format(k,getattr(x,k)) for k in x.labels]))

        # finally, create the class
        cls = type(\
            'Log__{:s}'.format(self.name),
            (ctypes.LittleEndianStructure,),
            members
        )

        if ctypes.sizeof(cls) != cls.SIZE:
            print("size mismatch for {} expected {} got {}".format(cls, ctypes.sizeof(cls), cls.SIZE), file=sys.stderr)
#            for i in cls.labels:
#                print("{} = {}".format(i,getattr(cls,'_'+i)))
            return None

        return cls

BinaryFormat.SIZE = ctypes.sizeof(BinaryFormat)

class Channel(object):
    '''storage for a single stream of data, i.e. all GPS.RelAlt values'''

    # TODO: rethink data storage, but do more thorough regression testing before refactoring it
    # TODO: store data as a scipy spline curve so we can more easily interpolate and sample the slope?

    def __init__(self):
        self.dictData = {} #  dict of linenum->value      # store dupe data in dict and list for now, until we decide which is the better way to go
        self.listData = [] #  list of (linenum,value)     # store dupe data in dict and list for now, until we decide which is the better way to go
    def getSegment(self, startLine, endLine):
        '''returns a segment of this data (from startLine to endLine, inclusive) as a new Channel instance'''
        segment = Channel()
        segment.dictData = {k:v for k,v in self.dictData.iteritems() if k >= startLine and k <= endLine}
        return segment
    def min(self):
        return min(self.dictData.values())
    def max(self):
        return max(self.dictData.values())
    def avg(self):
        return numpy.mean(self.dictData.values())
    def getNearestValueFwd(self, lineNumber):
        '''Returns (value,lineNumber)'''
        index = bisect.bisect_left(self.listData, (lineNumber,-99999))
        while index<len(self.listData):
            line  = self.listData[index][0]
            #print "Looking forwards for nearest value to line number %d, starting at line %d" % (lineNumber,line) # TEMP
            if line >= lineNumber:
                return (self.listData[index][1],line)
            index += 1
        raise Exception("Error finding nearest value for line %d" % lineNumber)
    def getNearestValueBack(self, lineNumber):
        '''Returns (value,lineNumber)'''
        index = bisect.bisect_left(self.listData, (lineNumber,-99999)) - 1
        while index>=0:
            line  = self.listData[index][0]
            #print "Looking backwards for nearest value to line number %d, starting at line %d" % (lineNumber,line) # TEMP
            if line <= lineNumber:
                return (self.listData[index][1],line)
            index -= 1
        raise Exception("Error finding nearest value for line %d" % lineNumber)
    def getNearestValue(self, lineNumber, lookForwards=True):
        '''find the nearest data value to the given lineNumber, defaults to first looking forwards. Returns (value,lineNumber)'''
        if lookForwards:
            try:
                return self.getNearestValueFwd(lineNumber)
            except:
                return self.getNearestValueBack(lineNumber)
        else:
            try:
                return self.getNearestValueBack(lineNumber)
            except:
                return self.getNearestValueFwd(lineNumber)
        raise Exception("Error finding nearest value for line %d" % lineNumber)
    def getInterpolatedValue(self, lineNumber):
        (prevValue,prevValueLine) = self.getNearestValue(lineNumber, lookForwards=False)
        (nextValue,nextValueLine) = self.getNearestValue(lineNumber, lookForwards=True)
        if prevValueLine == nextValueLine:
            return prevValue
        weight = (lineNumber-prevValueLine) / float(nextValueLine-prevValueLine)
        return ((weight*prevValue) + ((1-weight)*nextValue))
    def getIndexOf(self, lineNumber):
        '''returns the index within this channel's listData of the given lineNumber, or raises an Exception if not found'''
        index = bisect.bisect_left(self.listData, (lineNumber,-99999))
        #print "INDEX of line %d: %d" % (lineNumber,index)
        #print "self.listData[index][0]: %d" % self.listData[index][0]
        if (self.listData[index][0] == lineNumber):
            return index
        else:
            raise Exception("Error finding index for line %d" % lineNumber)

class LogIterator:
    '''Smart iterator that can move through a log by line number and maintain an index into the nearest values of all data channels'''
    # TODO: LogIterator currently indexes the next available value rather than the nearest value, we should make it configurable between next/nearest

    class LogIteratorSubValue:
        '''syntactic sugar to allow access by LogIterator[lineLabel][dataLabel]'''
        logdata   = None
        iterators = None
        lineLabel = None
        def __init__(self, logdata, iterators, lineLabel):
            self.logdata = logdata
            self.lineLabel = lineLabel
            self.iterators = iterators
        def __getitem__(self, dataLabel):
            index = self.iterators[self.lineLabel][0]
            return self.logdata.channels[self.lineLabel][dataLabel].listData[index][1]

    iterators   = {}      # lineLabel -> (listIndex,lineNumber)
    logdata     = None
    currentLine = None

    def __init__(self, logdata, lineNumber=0):
        self.logdata = logdata
        self.currentLine = lineNumber
        for lineLabel in self.logdata.formats:
            if lineLabel in self.logdata.channels:
                self.iterators[lineLabel] = ()
        self.jump(lineNumber)
    def __iter__(self):
        return self
    def __getitem__(self, lineLabel):
        return LogIterator.LogIteratorSubValue(self.logdata, self.iterators, lineLabel)
    def next(self):
        '''increment iterator to next log line'''
        self.currentLine += 1
        if self.currentLine > self.logdata.lineCount:
            return self
        for lineLabel in self.iterators.keys():
            # check if the currentLine has gone past our the line we're pointing to for this type of data
            dataLabel = self.logdata.formats[lineLabel].labels[0]
            (index, lineNumber) = self.iterators[lineLabel]
            # if so, and it is not the last entry in the log, then increment the indices for all dataLabels under that lineLabel
            if (self.currentLine > lineNumber) and (index < len(self.logdata.channels[lineLabel][dataLabel].listData)-1):
                index += 1
                lineNumber = self.logdata.channels[lineLabel][dataLabel].listData[index][0]
                self.iterators[lineLabel] = (index,lineNumber)
        return self
    def jump(self, lineNumber):
        '''jump iterator to specified log line'''
        self.currentLine = lineNumber
        for lineLabel in self.iterators.keys():
            dataLabel = self.logdata.formats[lineLabel].labels[0]
            (value,lineNumber) = self.logdata.channels[lineLabel][dataLabel].getNearestValue(self.currentLine)
            self.iterators[lineLabel] = (self.logdata.channels[lineLabel][dataLabel].getIndexOf(lineNumber), lineNumber)


class DataflashLogHelper:
    '''helper functions for dealing with log data, put here to keep DataflashLog class as a simple parser and data store'''

    @staticmethod
    def getTimeAtLine(logdata, lineNumber):
        '''returns the nearest GPS timestamp in milliseconds after the given line number'''
        if not "GPS" in logdata.channels:
            raise Exception("no GPS log data found")
        # older logs use 'TIme', newer logs use 'TimeMS'
        timeLabel = "TimeMS"
        if "Time" in logdata.channels["GPS"]:
            timeLabel = "Time"
        while lineNumber <= logdata.lineCount:
            if lineNumber in logdata.channels["GPS"][timeLabel].dictData:
                return logdata.channels["GPS"][timeLabel].dictData[lineNumber]
            lineNumber = lineNumber + 1

        sys.stderr.write("didn't find GPS data for " + str(lineNumber) + " - using maxtime\n")
        return logdata.channels["GPS"][timeLabel].max()

    @staticmethod
    def findLoiterChunks(logdata, minLengthSeconds=0, noRCInputs=True):
        '''returns a list of (to,from) pairs defining sections of the log which are in loiter mode. Ordered from longest to shortest in time. If noRCInputs == True it only returns chunks with no control inputs'''
        # TODO: implement noRCInputs handling when identifying stable loiter chunks, for now we're ignoring it

        def chunkSizeCompare(chunk1, chunk2):
            chunk1Len = chunk1[1]-chunk1[0]
            chunk2Len = chunk2[1]-chunk2[0]
            if chunk1Len == chunk2Len:
                return 0
            elif chunk1Len > chunk2Len:
                return -1
            else:
                return 1

        od = collections.OrderedDict(sorted(logdata.modeChanges.items(), key=lambda t: t[0]))
        chunks = []
        for i in range(len(od.keys())):
            if od.values()[i][0] == "LOITER":
                startLine = od.keys()[i]
                endLine   = None
                if i == len(od.keys())-1:
                    endLine = logdata.lineCount
                else:
                    endLine = od.keys()[i+1]-1
                chunkTimeSeconds = (DataflashLogHelper.getTimeAtLine(logdata,endLine)-DataflashLogHelper.getTimeAtLine(logdata,startLine)+1) / 1000.0
                if chunkTimeSeconds > minLengthSeconds:
                    chunks.append((startLine,endLine))
                    #print "LOITER chunk: %d to %d, %d lines" % (startLine,endLine,endLine-startLine+1)
                    #print "  (time %d to %d, %d seconds)" % (DataflashLogHelper.getTimeAtLine(logdata,startLine), DataflashLogHelper.getTimeAtLine(logdata,endLine), chunkTimeSeconds)
        chunks.sort(chunkSizeCompare)
        return chunks

    @staticmethod
    def isLogEmpty(logdata):
        '''returns an human readable error string if the log is essentially empty, otherwise returns None'''
        # naive check for now, see if the throttle output was ever above 20%
        throttleThreshold = 20
        if logdata.vehicleType == VehicleType.Copter:
            throttleThreshold = 200 # copter uses 0-1000, plane+rover use 0-100
        if "CTUN" in logdata.channels:
            try:
                maxThrottle = logdata.channels["CTUN"]["ThrOut"].max()
            except KeyError as e:
                # ThrOut was shorted to ThO at some stage...
                maxThrottle = logdata.channels["CTUN"]["ThO"].max()
                # at roughly the same time ThO became a range from 0 to 1
                throttleThreshold = 0.2
            if maxThrottle < throttleThreshold:
                return "Throttle never above 20%"
        return None


class DataflashLog(object):
    '''APM Dataflash log file reader and container class. Keep this simple, add more advanced or specific functions to DataflashLogHelper class'''
    
    knownHardwareTypes = ["APM", "PX4", "MPNG"]
    intTypes   = "bBhHiIM"
    floatTypes = "fcCeEL"
    charTypes  = "nNZ"    

    def __init__(self, logfile=None, format="auto", ignoreBadlines=False):
        self.filename = None

        self.vehicleType     = None # from VehicleType enumeration; value derived from header
        self.vehicleTypeString = None # set at same time has the enum value
        self.firmwareVersion = ""
        self.firmwareHash    = ""
        self.freeRAM         = 0
        self.hardwareType    = "" # APM 1, APM 2, PX4, MPNG, etc What is VRBrain? BeagleBone, etc? Needs more testing
    
        self.formats     = {} # name -> Format
        self.parameters  = {} # token -> value
        self.messages    = {} # lineNum -> message
        self.modeChanges = {} # lineNum -> (mode,value)
        self.channels    = {} # lineLabel -> {dataLabel:Channel}
    
        self.filesizeKB   = 0
        self.durationSecs = 0
        self.lineCount    = 0
        self.skippedLines = 0
        self.backpatch_these_modechanges = []

        if logfile:
            self.read(logfile, format, ignoreBadlines)

    def getCopterType(self):
        '''returns quad/hex/octo/tradheli if this is a copter log'''
        if self.vehicleType != VehicleType.Copter:
            return None
        motLabels = []
        if "MOT" in self.formats: # not listed in PX4 log header for some reason?
            motLabels = self.formats["MOT"].labels
        if "GGain" in motLabels:
            return "tradheli"
        elif len(motLabels) == 4:
            return "quad"
        elif len(motLabels) == 6:
            return "hex"
        elif len(motLabels) == 8:
            return "octo"
        else:
            return ""

    def read(self, logfile, format="auto", ignoreBadlines=False):
        '''returns on successful log read (including bad lines if ignoreBadlines==True), will throw an Exception otherwise'''
        # TODO: dataflash log parsing code is pretty hacky, should re-write more methodically
        self.filename = logfile
        if self.filename == '<stdin>':
            f = sys.stdin
        else:
            f = open(self.filename, 'r')

        if format == 'bin':
            head = '\xa3\x95\x80\x80'
        elif format == 'log':
            head = ""
        elif format == 'auto':
            if self.filename == '<stdin>':
                # assuming TXT format
#                raise ValueError("Invalid log format for stdin: {}".format(format))
                head = ""
            else:
                head = f.read(4)
                f.seek(0)
        else:
            raise ValueError("Unknown log format for {}: {}".format(self.filename, format))

        if head == '\xa3\x95\x80\x80':
            numBytes, lineNumber = self.read_binary(f, ignoreBadlines)
            pass
        else:
            numBytes, lineNumber = self.read_text(f, ignoreBadlines)

        # gather some general stats about the log
        self.lineCount  = lineNumber
        self.filesizeKB = numBytes / 1024.0
        # TODO: switch duration calculation to use TimeMS values rather than GPS timestemp
        if "GPS" in self.channels:
            # the GPS time label changed at some point, need to handle both
            timeLabel = None
            for i in 'TimeMS','TimeUS','Time':
                if i in self.channels["GPS"]:
                    timeLabel = i
                    break
            firstTimeGPS = int(self.channels["GPS"][timeLabel].listData[0][1])
            lastTimeGPS  = int(self.channels["GPS"][timeLabel].listData[-1][1])
            if timeLabel == 'TimeUS':
                firstTimeGPS /= 1000
                lastTimeGPS /= 1000
            self.durationSecs = (lastTimeGPS-firstTimeGPS) / 1000

        # TODO: calculate logging rate based on timestamps
        # ...

    msg_vehicle_to_vehicle_map = {
        "ArduCopter": VehicleType.Copter,
        "APM:Copter": VehicleType.Copter,
        "ArduPlane": VehicleType.Plane,
        "ArduRover": VehicleType.Rover
    }

    # takes the vehicle type supplied via "MSG" and sets vehicleType from
    # the VehicleType enumeration
    def set_vehicleType_from_MSG_vehicle(self, MSG_vehicle):
        ret = self.msg_vehicle_to_vehicle_map.get(MSG_vehicle, None)
        if ret is None:
            raise ValueError("Unknown vehicle type (%s)" % (MSG_vehicle))
        self.vehicleType = ret
        self.vehicleTypeString = VehicleTypeString[ret]

    def handleModeChange(self, lineNumber, e):
        if self.vehicleType == VehicleType.Copter:
            try:
                modes = {0:'STABILIZE',
                    1:'ACRO',
                    2:'ALT_HOLD',
                    3:'AUTO',
                    4:'GUIDED',
                    5:'LOITER',
                    6:'RTL',
                    7:'CIRCLE',
                    9:'LAND',
                    10:'OF_LOITER',
                    11:'DRIFT',
                    13:'SPORT',
                    14:'FLIP',
                    15:'AUTOTUNE',
                    16:'HYBRID',}
                if hasattr(e, 'ThrCrs'):
                    self.modeChanges[lineNumber] = (modes[int(e.Mode)], e.ThrCrs)
                else:
                    # assume it has ModeNum:
                    self.modeChanges[lineNumber] = (modes[int(e.Mode)], e.ModeNum)
            except:
                if hasattr(e, 'ThrCrs'):
                    self.modeChanges[lineNumber] = (e.Mode, e.ThrCrs)
                else:
                    # assume it has ModeNum:
                    self.modeChanges[lineNumber] = (e.Mode, e.ModeNum)
        elif self.vehicleType in [VehicleType.Plane, VehicleType.Copter, VehicleType.Rover]:
            self.modeChanges[lineNumber] = (e.Mode, e.ModeNum)
        else:
            # if you've gotten to here the chances are we don't
            # know what vehicle you're flying...
            raise Exception("Unknown log type for MODE line vehicletype=({}) line=({})".format(self.vehicleTypeString, repr(e)))

    def backPatchModeChanges(self):
        for (lineNumber, e) in self.backpatch_these_modechanges:
            self.handleModeChange(lineNumber, e)

    def process(self, lineNumber, e):
        if e.NAME == 'FMT':
            cls = e.to_class()
            if cls is not None: # FMT messages can be broken ...
                if hasattr(e, 'type') and e.type not in self._formats: # binary log specific
                    self._formats[e.type] = cls
                if cls.NAME not in self.formats:
                    self.formats[cls.NAME] = cls
        elif e.NAME == "PARM":
            self.parameters[e.Name] = e.Value
        elif e.NAME == "MSG":
            if not self.vehicleType:
                tokens = e.Message.split(' ')
                self.set_vehicleType_from_MSG_vehicle(tokens[0]);
                self.backPatchModeChanges()
                self.firmwareVersion = tokens[1]
                if len(tokens) == 3:
                    self.firmwareHash = tokens[2][1:-1]
            else:
                self.messages[lineNumber] = e.Message
        elif e.NAME == "MODE":
            if self.vehicleType is None:
                self.backpatch_these_modechanges.append( (lineNumber, e) )
            else:
                self.handleModeChange(lineNumber, e)
        # anything else must be the log data
        else:
            groupName = e.NAME

            # first time seeing this type of log line, create the channel storage
            if not groupName in self.channels:
                self.channels[groupName] = {}
                for label in e.labels:
                    self.channels[groupName][label] = Channel()

            # store each token in its relevant channel
            for label in e.labels:
                value = getattr(e, label)
                channel = self.channels[groupName][label]
                channel.dictData[lineNumber] = value
                channel.listData.append((lineNumber, value))


    def read_text(self, f, ignoreBadlines):
        self.formats = {'FMT':Format}
        lineNumber = 0
        numBytes = 0
        knownHardwareTypes = ["APM", "PX4", "MPNG"]
        for line in f:
            lineNumber = lineNumber + 1
            numBytes += len(line) + 1
            try:
                #print "Reading line: %d" % lineNumber
                line = line.strip('\n\r')
                tokens = line.split(', ')
                # first handle the log header lines
                if line == " Ready to drive." or line == " Ready to FLY.":
                    continue
                if line == "----------------------------------------":  # present in pre-3.0 logs
                    raise Exception("Log file seems to be in the older format (prior to self-describing logs), which isn't supported")
                if len(tokens) == 1:
                    tokens2 = line.split(' ')
                    if line == "":
                        pass
                    elif len(tokens2) == 1 and tokens2[0].isdigit(): # log index
                        pass
                    elif len(tokens2) == 3 and tokens2[0] == "Free" and tokens2[1] == "RAM:":
                        self.freeRAM = int(tokens2[2])
                    elif tokens2[0] in knownHardwareTypes:
                        self.hardwareType = line      # not sure if we can parse this more usefully, for now only need to report it back verbatim
                    elif (len(tokens2) == 2 or len(tokens2) == 3) and tokens2[1][0].lower() == "v":  # e.g. ArduCopter V3.1 (5c6503e2)
                        self.set_vehicleType_from_MSG_vehicle(tokens2[0])
                        self.firmwareVersion = tokens2[1]
                        if len(tokens2) == 3:
                            self.firmwareHash    = tokens2[2][1:-1]
                    else:
                        errorMsg = "Error parsing line %d of log file: %s" % (lineNumber, self.filename)
                        if ignoreBadlines:
                            print(errorMsg + " (skipping line)", file=sys.stderr)
                            self.skippedLines += 1
                        else:
                            raise Exception("")
                else:
                    if not tokens[0] in self.formats:
                        raise ValueError("Unknown Format {}".format(tokens[0]))
                    e = self.formats[tokens[0]](*tokens[1:])
                    self.process(lineNumber, e)
            except Exception as e:
                print("BAD LINE: " + line, file=sys.stderr)
                if not ignoreBadlines:
                    raise Exception("Error parsing line %d of log file %s - %s" % (lineNumber,self.filename,e.args[0]))
        return (numBytes,lineNumber)

    def read_binary(self, f, ignoreBadlines):
        lineNumber = 0
        numBytes = 0
        for e in self._read_binary(f, ignoreBadlines):
            lineNumber += 1
            if e is None:
                continue
            numBytes += e.SIZE
#            print(e)
            self.process(lineNumber, e)
        return (numBytes,lineNumber)

    def _read_binary(self, f, ignoreBadlines):
        self._formats = {128:BinaryFormat}
        data = bytearray(f.read())
        offset = 0
        while len(data) > offset + ctypes.sizeof(logheader):
            h = logheader.from_buffer(data, offset)
            if not (h.head1 == 0xa3 and h.head2 == 0x95):
                if ignoreBadlines == False:
                    raise ValueError(h)
                else:
                    if h.head1 == 0xff and h.head2 == 0xff and h.msgid == 0xff:
                        print("Assuming EOF due to dataflash block tail filled with \\xff... (offset={off})".format(off=offset), file=sys.stderr)
                        break
                    offset += 1
                    continue

            if h.msgid in self._formats:
                typ = self._formats[h.msgid]
                if len(data) <= offset + typ.SIZE:
                    break
                try:
                    e = typ.from_buffer(data, offset)
                except:
                    print("data:{} offset:{} size:{} sizeof:{} sum:{}".format(len(data),offset,typ.SIZE,ctypes.sizeof(typ),offset+typ.SIZE))
                    raise
                offset += typ.SIZE
            else:
                raise ValueError(str(h) + "unknown type")
            yield e
