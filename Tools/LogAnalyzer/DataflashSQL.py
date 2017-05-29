#!/usr/bin/env python

import logging
import sys

log = logging.getLogger()
log.setLevel(logging.DEBUG)

ch = logging.StreamHandler(sys.stdout)
ch.setLevel(logging.DEBUG)
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
ch.setFormatter(formatter)
log.addHandler(ch)

import DataflashLog

import sqlalchemy
from sqlalchemy import Table, MetaData, Column, ForeignKey, Integer, String, Float
from sqlalchemy.orm import mapper
from sqlalchemy.schema import ForeignKeyConstraint

class mappings(object):
    pass

class DataflashSQL(object):
    """Use SQLAlchemy to convert a DataflashLog to a sqlite Database
    Tables are created per log type when reading FMT messages and have the same name
    as indicated in the FMT message
    Columns are learned of the FMT messages as well,
    the column _{tablename} is special -the primary key- useful for self joins
    Mapped Tables can be accessed via DataflashSQL.m.NAME
    Tables are mapped and of type sqlalchemy.schema.Table, so you have to use table.c for the columns
    e.g: db.m.ATUN.c.line
    *special* tables/views are prefixed with _

    _record is a list of all parsed lines, with the TimeMS of the closest message carrying a TimeMS value
    GPS TimeMS values currently get ignored, as their TimeMS value drifts different to the other TimeMS values

    _timems is a table "unioning" all available TimeMS values (but GPS)

    _mode is a view on the flight modes used, adding Start/Stop line and TimeMS values
    """
    def __init__(self, db):
        self.engine = sqlalchemy.create_engine('sqlite:///{}'.format(db))#, isolation_level='SERIALIZABLE', echo=True)
        Session = sqlalchemy.orm.sessionmaker(bind=self.engine)
        self.session = Session()
        self.m = mappings()
        self.metadata = MetaData()

        # create mappings for FMT _record _timems tables
        self._init_FMT()

        # learn formats from DB
        q = self.session.query(self.m.FMT)
        r = q.all()
        for obj in r:
            if hasattr(self.m, obj.name) == True or obj.name == 'FMT':
                continue

            tbl = DataflashSQL.table_from_FMT(obj, self.metadata)
            attr = mapper(
                self.class_from_FMT(obj),
                tbl,
                inherits=self.m._record.class_,
                polymorphic_identity=obj.name,
                primary_key=tbl.c.line,
            )
            setattr(self.m, obj.name, attr)

    def _init_FMT(self):
        if hasattr(self.m, '_record') == False:

            class SQLRecord(object):
                def __init__(self, line, type):
                    self.line = line
                    self.type = type

            tbl = Table('_record', self.metadata,
                        Column('line', Integer, primary_key=True, nullable=False),
                        Column('type', String(8), nullable=False, index=True),
                        Column('_TimeMS', Integer, nullable=True),
                        )
            setattr(self.m, '_record',
                    mapper(
                        SQLRecord,
                        tbl,
                        polymorphic_on=tbl.c.type,
                        with_polymorphic='*'
                    )
                )
        self.metadata.create_all(self.engine, tables=[self.m._record.local_table])

        if hasattr(self.m, 'FMT') == False:
            class SQLFormat(SQLRecord):
                def __init__(self, line, length, name, types, labels):
                    SQLRecord.__init__(self, line, 'FMT')
                    self.length = length
                    self.name = name
                    self.types = types
                    self.labels = labels
                @staticmethod
                def from_object(line, obj):
                    l = getattr(obj,'length', -1) # Text Log files lack length
                    return SQLFormat(line, l, obj.name, obj.types, obj.labels)
            setattr(self.m, 'FMT',
                    mapper(
                        SQLFormat,
                        Table('fmt', self.metadata,
                              Column('_fmt', Integer, nullable=False, primary_key=True),
                              Column('line', Integer, nullable=False, index=True),
                              Column('type', String(8), nullable=False, default='FMT'),
                              Column('length', Integer, nullable=True),
                              Column('name', String(4), nullable=False),
                              Column('types', String(16), nullable=False),
                              Column('labels', String(64), nullable=False),
                              ForeignKeyConstraint(['line', 'type'], ['_record.line', '_record.type'])
                              ),
                        inherits=self.m._record.class_, # aka SQLRecord
                        polymorphic_identity='FMT'
                        )
                    )
        self._handle_FMT(SQLFormat(-1, -1, '_timems', 'I', 'TimeMS'))
        self.metadata.create_all(self.engine, tables=[self.m.FMT.local_table])
        self.session.commit()

    def _handle_FMT(self, obj):
        if hasattr(self.m, obj.name) == False: # we can't map the same table twice
            setattr(self.m,
                    obj.name,
                    mapper(
                        self.class_from_FMT(obj),
                        DataflashSQL.table_from_FMT(obj, self.metadata),
                        inherits=self.m._record.class_,
                        polymorphic_identity=obj.name
                        )
                    )
        self.metadata.create_all(self.engine, tables=[getattr(self.m, obj.name).local_table])

    @staticmethod
    def table_from_FMT(obj, metadata):
        cols = []
        fieldtypes = [i for i in obj.types]
        fieldlabels = obj.labels.split(",")

        for (label, _type) in zip(fieldlabels, fieldtypes):
            if _type in "fcCeEL":
                t = Float
            elif _type in "bBhHiIM":
                t = Integer
            elif _type in "nNZ":
                t = String
            cols.append(Column(label, t))#, nullable=False))
        return Table(obj.name.lower(),
                     metadata,
                     Column('_{}'.format(obj.name.lower()), Integer, nullable=False, primary_key=True),
                     Column('line', Integer, nullable=False, index=True),
                     Column('type', String(8), nullable=False, default=obj.name),
                     ForeignKeyConstraint(['line', 'type'], ['_record.line', '_record.type']),
                     *cols)

    def class_from_FMT(self, obj):
        labels = obj.labels.split(",")
        def init(a, line, fmt, obj):
            self.m._record.class_.__init__(a, line, fmt)
            for l in labels:
                try:
                    setattr(a, l, getattr(obj,l))
                except Exception as e:
                    print("{} {} failed".format(a,l))
                    print(e)

        members = {}
        members['__init__'] = init
        members['__repr__'] = lambda x: "<{cls} {data}>".format(cls=x.__class__.__name__, data = ' '.join(["{}:{}".format(k,getattr(x,k,None)) for k in labels]))

        # create the class
        cls = type(\
            'SQL__{:s}'.format(obj.name),
            (self.m._record.class_,),
            members
        )
        return cls

    def convert(self, logfile, format='auto'):
        _self = self
        class Import(DataflashLog.DataflashLog):
            def __init__(self, logfile, format):
                DataflashLog.DataflashLog.__init__(self)
                self.logfile=logfile
                self.format=format
            def run(self):
                self.read(self.logfile, self.format, True)

            def process(self, lineNumber, e):
                if e.NAME == 'FMT':
                    cls = e.to_class()
                    if cls is not None: # FMT messages can be broken ...
                        if hasattr(e, 'type') and e.type not in self._formats: # binary log specific
                            self._formats[e.type] = cls
                        if cls.NAME not in self.formats:
                            self.formats[cls.NAME] = cls
                try:
                    cls = getattr(_self.m, e.NAME, None)
                    if cls is None:
                        return
                    if e.NAME == 'FMT':
                        a = _self.m.FMT.class_.from_object(lineNumber, e)
                        _self.session.add(a)
                        if e.name == 'FMT':
                            return
                        _self._handle_FMT(e)
                        _self.session.commit()
                    else:
                        _self.session.add(cls.class_(lineNumber, e.NAME, e))
                except Exception as ex:
                    log.exception(ex)
                    raise ex

        i = Import(logfile, format)
        i.run()
        self.session.commit()
        self.finish()

    def finish(self):
        self._finish_timems()
        self._finish_record()
        self._finish_mode()

    def _finish_timems(self):
        # update timems table
        # a view is possible but very slow
        names = []
        for i in dir(self.m):
            cls = getattr(self.m, i)
            if not type(cls) == type(self.m._record):
                continue

            if i.startswith("GPS"):
                # gps timestamps differ to much and do not drift as the other timestamps do
                continue
            try:
                x = cls.c.TimeMS
                names.append(cls)
            except:
                continue
        if len(names) == 0:
            return
        stmts = []
        for i in names:
            q = """SELECT
    line AS line,
    type AS type,
    timems AS timems
FROM
    "{cls}"
""".format(cls=i.local_table.name)
            stmts.append(q)

        stmt = "INSERT INTO _timems (line,type,TimeMS) {stmts}".format(stmts="\nUNION\n".join(stmts))
#        self.session.execute("DELETE FROM _timems")
        self.session.execute(stmt)
        self.session.commit()

    def _finish_record(self):
        # set _TimeMS guess in _record
        self.session.execute("""UPDATE
    _record
SET
    _TimeMS =
        CASE WHEN
            IFNULL(abs(_record.line-(SELECT line from _timems WHERE line <= _record.line ORDER BY line DESC LIMIT 1)),999999) <
            IFNULL(abs(_record.line-(SELECT line from _timems WHERE line >= _record.line ORDER BY line ASC LIMIT 1)),999999)
        THEN
            (SELECT TimeMS from _timems WHERE line <= _record.line ORDER BY line DESC LIMIT 1)
        ELSE
            (SELECT TimeMS from _timems WHERE line >= _record.line ORDER BY line ASC LIMIT 1)
        END;""")
        self.session.commit()

    def _finish_mode(self):
        # create _mode View
        self.session.execute("""CREATE VIEW _mode AS SELECT
    begin.Mode,
    begin.line AS StartLine,
    IFNULL(end.line, (SELECT MAX(r.line) FROM _record AS r)) AS StopLine,
    rb._TimeMS AS StartTime,
    IFNULL(re._TimeMS,(SELECT MAX(r._TimeMS) FROM _record AS r)) AS StopTime,
    (IFNULL(re._TimeMS,(SELECT MAX(r._TimeMS) FROM _record AS r))-rb._TimeMS) / 1000 AS interval
FROM
    mode as begin
    LEFT OUTER JOIN mode AS end ON( (begin._mode+1) = end._mode)
    JOIN _record AS rb on (rb.line = begin.line)
    LEFT OUTER JOIN _record AS re ON( re.line = end.line )""")
        self.session.commit()


# FOREIGN KEYS break multi row inserts in SQLite
#
#@sqlalchemy.event.listens_for(sqlalchemy.engine.Engine, "connect")
#
#def on_connect(con, cr):
#    cursor = con.cursor()
#    cursor.execute("PRAGMA foreign_keys=ON")
#    cursor.close()


class ExampleAutotune(object):
    AUTOTUNE_INITIALISED       = 30
    AUTOTUNE_OFF               = 31
    AUTOTUNE_RESTART           = 32
    AUTOTUNE_SUCCESS           = 33
    AUTOTUNE_FAILED            = 34
    AUTOTUNE_REACHED_LIMIT     = 35
    AUTOTUNE_PILOT_TESTING     = 36
    AUTOTUNE_SAVEDGAINS        = 37
    def __init__(self):
        pass

    def run(self, e, verbose):
        if getattr(e.m, 'ATUN', None) is None:
            return

        sessions = e.session.execute("""SELECT
    Id,
    begin.line AS StartLine,
    IFNULL(MIN(end.line)-1,MAX(end.max)) AS StopLine
FROM
    ev as begin,
    (SELECT e.line AS line,r.line AS max from _record AS r LEFT OUTER JOIN ev AS e USING(line) where e.Id is null or e.Id = {id} ) AS end
WHERE
    begin.Id = {id} AND
    IFNULL(end.line,end.max) > begin.line
GROUP BY
    begin.line""".format(id=self.AUTOTUNE_INITIALISED))

        for s in sessions:
            EV = e.m.EV

            q = e.session.query(EV.c.Id)
            q = q.filter(EV.c.line >= s.StartLine, EV.c.line <= s.StopLine) # limit to session
            q = q.filter(EV.c.Id >= self.AUTOTUNE_INITIALISED , EV.c.Id <= self.AUTOTUNE_SAVEDGAINS) # only AUTOTUNE related Id

            events = set([i for (i,) in q.all()])

            if self.AUTOTUNE_SUCCESS in events:
                print("Autotune {}-{} +++".format(s.StartLine,s.StopLine))
            else:
                print("Autotune {}-{} ---".format(s.StartLine,s.StopLine))
            if verbose:
                _record = e.m._record

                q = e.session.query(_record)
                q = q.with_polymorphic([e.m.EV, e.m.ATUN]) # polymorphic lookup only for tables required
                q = q.filter(_record.c.line >= s.StartLine, _record.c.line <= s.StopLine) # limit to session
                q = q.filter(_record.c.type.in_(['ATUN','EV'])) # we only want ATUN & EV
                q = q.order_by(sqlalchemy.asc(_record.c.line)) # order by line ascending

                for i in q.all():
                    print(i)

def main():
    import argparse
    parser = argparse.ArgumentParser(description='Analyze an APM Dataflash log for known issues')
    parser.add_argument('--logfile', type=argparse.FileType('r'), help='path to Dataflash log file (or - for stdin)', default=None)
    parser.add_argument('--db', type=str, help='path to Dataflash Database file ', default=":memory:")
    parser.add_argument('-f', '--format',  metavar='', type=str, action='store', choices=['bin','log','auto'], default='auto')
    parser.add_argument('-v', '--verbose', metavar='', action='store_const', const=True, help='verbose output')
    args = parser.parse_args()

    db = DataflashSQL(args.db)

    if args.logfile:
        db.convert(args.logfile.name, args.format)

    x = ExampleAutotune()
    x.run(db, args.verbose)

if __name__ == '__main__':
    main()

