# Script Created by Tang Dynasty.
proc step_begin { step } {
  set stopFile ".stop.f"
  if {[file isfile .stop.f]} {
    puts ""
    puts " #Halting run"
    puts ""
    return -code error
  }
  set beginFile ".$step.begin.f"
  set platform "$::tcl_platform(platform)"
  set user "$::tcl_platform(user)"
  set pid [pid]
  set host ""
  if { [string equal $platform unix] } {
    if { [info exists ::env(HOSTNAME)] } {
      set host $::env(HOSTNAME)
    }
  } else {
    if { [info exists ::env(COMPUTERNAME)] } {
      set host $::env(COMPUTERNAME)
    }
  }
  set ch [open $beginFile w]
  puts $ch "<?xml version=\"1.0\"?>"
  puts $ch "<ProcessHandle Version=\"1\" Minor=\"0\">"
  puts $ch "    <Process Ownner=\"$user\" Host=\"$host\" Pid=\"$pid\">"
  puts $ch "    </Process>"
  puts $ch "</ProcessHandle>"
  close $ch
}
proc step_end { step } {
  set endFile ".$step.end.f"
  set ch [open $endFile w]
  close $ch
}
proc step_error { step } {
  set errorFile ".$step.error.f"
  set ch [open $errorFile w]
  close $ch
}
step_begin read_design
set ACTIVESTEP read_design
set rc [catch {
  open_project {top.prj}
  import_device ef2_s4.db -package EF2M45VG81C
  elaborate -top {top}
  read_adc ../../../source/top.adc
  export_db {top_elaborate.db}
} RESULT]
if {$rc} {
  step_error read_design
  return -code error $RESULT
} else {
  step_end read_design
  unset ACTIVESTEP
}
step_begin opt_rtl
set ACTIVESTEP opt_rtl
set rc [catch {
  optimize_rtl
  report_area -file top_rtl.area
  export_db {top_rtl.db}
} RESULT]
if {$rc} {
  step_error opt_rtl
  return -code error $RESULT
} else {
  step_end opt_rtl
  unset ACTIVESTEP
}
step_begin opt_gate
set ACTIVESTEP opt_gate
set rc [catch {
  read_sdc ../../../source/top.sdc
  optimize_gate -maparea top_gate.area
  legalize_phy_inst
  export_db {top_gate.db}
} RESULT]
if {$rc} {
  step_error opt_gate
  return -code error $RESULT
} else {
  step_end opt_gate
  unset ACTIVESTEP
}
