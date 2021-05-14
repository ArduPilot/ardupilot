#!/usr/bin/env python
'''
run Replay over a set of logs to check for code regressions
'''

import optparse, os, sys

parser = optparse.OptionParser("CheckLogs")
parser.add_option("--logdir", type='string', default='testlogs', help='directory of logs to use')
parser.add_option("--create-checked-logs", action='store_true', default=False, help="created logs with CHEK messages")
parser.add_option("--tolerance-euler", type=float, default=3, help="tolerance for euler angles in degrees");
parser.add_option("--tolerance-pos", type=float, default=2, help="tolerance for position angles in meters");
parser.add_option("--tolerance-vel", type=float, default=2, help="tolerance for velocity in meters/second");

opts, args = parser.parse_args()

def run_cmd(cmd, dir=".", show=False, output=False, checkfail=True):
    '''run a shell command'''
    from subprocess import call, check_call,Popen, PIPE
    if show:
        print("Running: '%s' in '%s'" % (cmd, dir))
    if output:
        return Popen([cmd], shell=True, stdout=PIPE, cwd=dir).communicate()[0]
    elif checkfail:
        return check_call(cmd, shell=True, cwd=dir)
    else:
        return call(cmd, shell=True, cwd=dir)

def run_replay(logfile):
    '''run Replay on one logfile'''
    print("Processing %s" % logfile)
    cmd = "./Replay.elf -- --check %s --tolerance-euler=%f --tolerance-pos=%f --tolerance-vel=%f " % (
        logfile,
        opts.tolerance_euler,
        opts.tolerance_pos,
        opts.tolerance_vel)
    run_cmd(cmd, checkfail=False)

def get_log_list():
    '''get a list of log files to process'''
    import glob, sys
    pattern = os.path.join(opts.logdir, "*-checked.bin")
    file_list = glob.glob(pattern)
    print("Found %u logs to processs" % len(file_list))
    if len(file_list) == 0:
        print("No logs to process matching %s" % pattern)
        sys.exit(1)
    return file_list

def create_html_results():
    '''create a HTML file with results'''
    error_count = 0

    git_version = run_cmd('git log --pretty=oneline HEAD~1..HEAD', output=True)
    
    f = open("replay_results.html", "w")
    f.write(
'''<!DOCTYPE html PUBLIC "-//W3C//DTD XHTML 1.0 Strict//EN"
        "http://www.w3.org/TR/xhtml1/DTD/xhtml1-strict.dtd">
<html lang="en" xmlns="http://www.w3.org/1999/xhtml">
<head>
<title>Replay results</title>
<meta charset="utf-8"/>
</head>
<body>
<h1>Replay Results</h1>
Git version: %s
<p>
<table border="1">
<tr bgcolor="lightgrey">
 <th>Filename</th>
 <th>RollError(deg)</th>
 <th>PitchError(deg)</th>
 <th>YawError(deg)</th>
 <th>PosError(m)</th>
 <th>VelError(m/s)</th>
</tr>
''' % git_version)
    infile = open("replay_results.txt", "r")

    line_count = 0
    line_errors = 0
    
    for line in infile:
        line = line.strip()
        line_count += 1
        a = line.split("\t")
        if len(a) != 6:
            print("Invalid line: %s" % line)
            error_count += 1
            continue
        tolerances = [opts.tolerance_euler,
                      opts.tolerance_euler,
                      opts.tolerance_euler,
                      opts.tolerance_pos,
                      opts.tolerance_vel]
        f.write('''<tr><td><a href="%s">%s</a></td>''' % (a[0],a[0]))
        error_in_this_log = False
        for i in range(1,6):
            tol = tolerances[i-1]
            if a[i] == "FPE":
                bgcolor = "red"
                error_count += 1
                error_in_this_log = True
            elif float(a[i]) > tol:
                bgcolor = "red"
                error_count += 1
                error_in_this_log = True
            else:
                bgcolor = "white"
            f.write('''<td bgcolor="%s" align="right">%s</td>\n''' % (bgcolor, a[i]))

        if error_in_this_log:
            line_errors += 1
        f.write('''</tr>\n''')

    f.write('''</table>\n''')

    # write summary
    f.write(
'''<h2>Summary</h2>
<p>Processed %u logs<br/>
%u errors from %u logs<br/>
<hr>
<p>Tolerance Euler: %.3f degrees<br/>
Tolerance Position: %.3f meters<br/>
Tolerance Velocity: %.3f meters/second
''' % (line_count, error_count, line_errors,
       opts.tolerance_euler,
       opts.tolerance_pos,
       opts.tolerance_vel))

    # add trailer
    f.write(
'''
</body>
</html>
''')
    f.close()
    infile.close()

def check_logs():
    '''run log checking'''
    log_list = get_log_list()

    # remove old results file
    try:
        os.unlink("replay_results.txt")
    except Exception as ex:
        print(ex)

    for logfile in log_list:
        run_replay(logfile)

    create_html_results()

def create_checked_logs():
    '''create a set of CHEK logs'''
    import glob, sys
    if os.path.isfile(opts.logdir):
        full_file_list = [opts.logdir]
    else:
        pattern = os.path.join(opts.logdir, "*.bin")
        full_file_list = glob.glob(pattern)
    file_list = []
    for f in full_file_list:
        if not f.endswith("-checked.bin"):
            file_list.append(f)
    if len(file_list) == 0:
        print("No files to process")
        sys.exit(1)
    for f in file_list:
        print("Processing %s" % f)
        log_list_current = set(glob.glob("logs/*.BIN"))
        cmd = "./Replay.elf -- --check-generate %s" % f
        run_cmd(cmd, checkfail=True)
        log_list_after = set(glob.glob("logs/*.BIN"))
        changed = log_list_after.difference(log_list_current)
        if len(changed) != 1:
            print("Failed to generate log for %s" % f)
            sys.exit(1)
        outlog = list(changed)[0]
        name, ext = os.path.splitext(f)
        newname = name + '-checked.bin'
        os.rename(outlog, newname)
        print("Created %s" % newname)

if opts.create_checked_logs:
    create_checked_logs()
    sys.exit(0)

check_logs()
