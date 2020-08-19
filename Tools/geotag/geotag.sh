#!/bin/bash

# Advanced geotagging tool by André Kjellstrup
######################################################################  VARIABLES ########################

version="v1.1"
mode=1 # 0=geotag, 1=jpg distance(s) , 2=CAM distance in sec
loglinenum=1 # last read log line number
loglinenummatched=0 # last read log line number used - reverting to this +1 when looking back
logline="start" #current line, placeholder for start
logtimemsprev=0
logended=0 #true if log ended.
logtarget=0 #true if we got log type we wanted
logmatch=1 # true if we accepted lo line as matching
loggotpos=0 #true if log contains pos messages (enables fallback)
typical_offset="notset" #contains time offset of first picture vs log. 
jpgtot=0  #number of jpg files in folder
jpgtagged=0 #number of correctly tagged photos
jpgmiss=0 #number of missing photos
jpgnotinlog=0 #counting jpg's with timestamp outside log
camindex=0 #number of last CAM message used.
camtot=0  #number of CAM lines found
trigtot=0 #number of TRIG lines found
logmiss=0 #missed CAM messages
revjumps=0 #count how many time we have backed up
difflimit=10 # POS message difflimit 10 gives 0.06sec , 20 has proved useful 
running=1
cmdarg=$* #save command line parameters
FILE=jpglist
ML="false"
logsdone=0 #counts processed logs
SKIPIMG=0



###########################

while [[ $# > 0 ]]
do
key="$1"

case $key in
    -w|--write)
    WRITE="true"
    ;;
    -r|--readonly)
    WRITE="false"
    ;;
    -f|--find)
    WRITE="false"
    FIND="true"
    ;;
    -p|--pos)
    MODE="POS"
    typical_offset="$2"   
    shift # get past argument
    ;; 
    -dl|--difflimit)
    difflimit="$2"
    shift # get past argument
    ;;
    -c|--cam)
    MODE="CAM"
    ;;      
    -t|--trig)
    MODE="TRIG"
    ;;
    -ns|--nosubsec)
    NS="true"
    ;;
    -ml|--multilog)
    ML="true"
    ;;
    -del|--delorg)
    DELORG="true"
    ;;   
    -sp|--skipphoto)
    SKIPIMG="$2"
    shift # get past argument
    ;;
    -sc|--skipcam)
    SKIPCAM="$2"
    shift # get past argument
    ;;
    -st|--skiptrig)
    SKIPTRIG="$2"
    shift # get past argument
    ;;
    -sl|--skiplog)
    SKIPLOG="$2"
    shift # get past argument
    ;;

    -h|--help)
    HELP="true"
    ;;
    *)
	# unknown option
    ;;
esac
shift # past argument or value
done

#echo FILE EXTENSION  = "${EXTENSION}"
#echo SEARCH PATH     = "${SEARCHPATH}"
#echo LIBRARY PATH    = "${LIBPATH}"

if [[ "$HELP" == "true" ]] || [[ "$WRITE" == "" ]] #|| [[ "$MODE" == "" ]] 
then
echo "Geotagging script by André Kjellstrup.  $version"
echo "execute inside a folder containing .JPG and .BIN"
echo "example: geotag.sh -c -sp 2 -r"
echo ""
echo "-w or --write Enables photo processing"
echo "-r or --readonly Enables photo processing"
echo "-f or --readonly, finds CAM to exif spacing for +/- 10 skipped photos, and +/- 10 skipped CAM messages"
echo "-sp x or --skipphoto x Skips x photos counting from the start"
echo "-sc x or --skipcam x Skips x CAM entrys from the start"
echo "-sl x or --skiplog x Skips x POS entrys from the start"
echo "-st x or --skiptrig x Skips x TRIG entrys from the start"
echo "-p x or --pos x Use POS log only (if camera was independendly trigged) x=offset in seconds"
echo "-dl x or --difflimit x Acceptable limit in 10ms steps for POS-log vs photo differance 10=0.1s (default)"
echo "-c or --cam Use CAM log only (do not fall back to POS pessages)"
echo "-t or --trig Use TRIG log only (if camera shutter is sensed)"
echo "-ns or --nosubsec Do not look for subsecond EXIF tag."
echo "-ml or --multilog Grab and combine all .BIN files (for processing photos from more then one flight."
echo "-del or --delorg : Delete JPG_original (backups)."
echo "EXAMPLE:"
echo "Multiple flights dataset from S100; geotag.sh -p 0 -ns -dl 30 -w -ml -del"
echo "simgle CAM flight; geotag.sh -c -w -del"
exit 1
fi

##ADD DEPENDENCIES HERE
command -v mavlogdump.py >/dev/null 2>&1 || { echo >&2 "I require mavlogdump.py but it's not installed.  Aborting."; exit 1; }
command -v exiftool >/dev/null 2>&1 || { echo >&2 "I require exiftool but it's not installed.  Aborting."; exit 1; }
##command -v foo >/dev/null 2>&1 || { echo >&2 "I require foo but it's not installed.  Aborting."; exit 1; }

if [[ "$WRITE" == "true" ]]
then
echo "INFO: will backup and modify photos" | tee geotag.log
fi

if [[ "$WRITE" == "false" ]]
then
echo "INFO: dry run only, no photos will be modified" | tee geotag.log
fi

if [[ "$MODE" == "POS" ]]
then
echo "INFO: use POS records only, expected offset of $typical_offset , poslimit (time of photo vs log is $poslimit)"   | tee geotag.log
fi

if [[ "$MODE" == "CAM" ]]
then
echo "INFO: use CAM records only" | tee geotag.log
fi


if [[ "$MODE" == "TRIG" ]]
then
echo "INFO: use TRIG records only" | tee geotag.log
fi

if [[ "$NS" == "true" ]]
then
echo "INFO: Do not look for subsecond-time in EXIF" | tee geotag.log
fi


if [[ "$SKIPIMG" -gt "0" ]]
then
echo "INFO: will skip first $SKIPIMG photos" | tee geotag.log
fi

if [[ "$SKIPCAM" -gt "0" ]]
then
echo "INFO: will skip first $SKIPCAM CAM logs" | tee geotag.log
fi

if [[ "$SKIPTRIG" -gt "0" ]]
then
echo "INFO: will skip first $SKIPTRIG TRIG logs" | tee geotag.log
fi


if [[ "$SKIPLOG" -gt "0" ]]
then
echo "INFO: will skip first $SKIPLOG POS logs" | tee geotag.log
loglinenum=$SKIPLOG
fi

if [ "$difflimit" != 10 ]
then
echo "INFO: Difflimit is changed to $difflimit (10=0.1s)" | tee geotag.log
fi


echo "INFO: using arguments $cmdarg " | tee geotag.log

jpglistindex=$(( 1 + $SKIPIMG)) #which file to read from jpg list


#=========================  FUNCTION readexif ======================================================
# get exif data for filename in $jpgname,
# return: $jpgdate 20-12-2010 ,$jpgtime 14:10:20.22 ,$jpgtimems 0.22 (220ms)
function readexif   
{
	if [[ "$NS" == "true" ]]
	then
	  jpgdatetime=$(exiftool -ee -p '$datetimeoriginal' "$jpgname" -n)
	  jpgdatetime="$jpgdatetime.50"
	else
	  jpgdatetime=$(exiftool -ee -p '$subsecdatetimeoriginal' "$jpgname" -n)
	fi


  jpgdate=$(echo -n "$jpgdatetime" | head -c10)
  jpgdate=$(echo "$jpgdate" | tr : -)   #replace : with -
  jpgtime=$(echo -n "$jpgdatetime" | tail -c-11)
  jpgtimems=$(echo -n "$jpgtime" | tail -c-2)
  jpgtimems=$(echo "scale=2; ($jpgtimems /100)" | bc -l) 
#echo "DEBUG $jpgname date $jpgdate time $jpgtime " 
##read -p "Press any key to continue... " -n1 -s
######browse log until we find CAM message
}

#=========================  FUNCTION readlog ======================================================
# extract needed date from logline string $logline
# return: $logdate 20-12-2010 ,$logtime 14:10:20.22 ,$logtimems 0.22 (220ms)
# return: $lat $lon $alt(m AMSL) 
#
###### We Prefer TRIG , then CAM, then fallback to POS
# in log they look alike:
#1970-01-01 01:01:12.55: CAM {TimeUS : 72553489, GPSTime : 0, GPSWeek : 0, Lat : 0.0, Lng : 0.0, Alt : -0.01, RelAlt : -0.01, GPSAlt : 0.0, Roll : 1.4, Pitch : 0.41, Yaw : 82.49}
#1970-01-01 01:01:15.80: TRIG {TimeUS : 75803571, GPSTime : 0, GPSWeek : 0, Lat : 0.0, Lng : 0.0, Alt : -0.01, RelAlt : -0.01, GPSAlt : 0.0, Roll : 1.43, Pitch : 0.43, Yaw : 82.62}

function readlog
{
logdate=$(echo "$logline"| grep -o '^\S*')   #till first space
logtime=$(echo "$logline"| awk -F" " '{print $2}')
logtime=$(echo -n "$logtime" | head -c-1) #remove trailing :
logtimems=$(echo -n "$logtime" | tail -c-2) 
logtimems=$(echo "scale=2; ($logtimems /100)" | bc -l) 

if [[ "$logline" == *POS* ]] 
    then
    lat=$(echo "$logline"| grep -o -P '(?<=Lat : ).*(?=, Lng)')
    lon=$(echo "$logline"| grep -o -P '(?<=Lng : ).*(?=, Alt)')
    alt=$(echo "$logline"| grep -o -P '(?<= Alt : ).*(?=, RelHomeAlt)')
fi
 

if [[ "$logline" == *CAM* ]] 
    then 
    lat=$(echo "$logline"| grep -o -P '(?<=Lat : ).*(?=, Lng)')
    lon=$(echo "$logline"| grep -o -P '(?<=Lng : ).*(?=, Alt)')
    alt=$(echo "$logline"| grep -o -P '(?<=Alt : ).*(?=, RelAlt)')
    fi


#rol=$(echo "$logline"| grep -o -P '(?<=Roll : ).*(?=, Pitch)') 
#pit=$(echo "$logline"| grep -o -P '(?<=Pitch : ).*(?=, Yaw)') 
#yaw=$(echo "$logline"| grep -o -P '(?<=Yaw : ).*(?=})')  
#echo "$logdate $logtime  CAM=$cam Lat=$lat	Lng=$lng  Alt= $alt"

}

#=========================  FUNCTION geotag ======================================================
# converts to WGS84  and tags photos with data found in lat,lon,alt.
function geotag

{
#jpgname="test.JPG"
#lat=69.6041115
#lon=23.2757452
#alt=122.8374

posWGS84=( $(echo "$lon $lat $alt" | cs2cs +proj=latlong +datum=WGS84))   # returns position as 22d16'32.683"E 69d36'14.801"N 123.837 in {posWGS84[0...2]}
lonWGS84ref=$(echo -n "${posWGS84[0]}" | tail -c-1) 
lonWGS84pos=$(echo -n "${posWGS84[0]}" | head -c-1) 
latWGS84ref=$(echo -n "${posWGS84[1]}" | tail -c-1) 
latWGS84pos=$(echo -n "${posWGS84[1]}" | head -c-1) 

#echo $lonWGS84pos
#echo $lonWGS84ref
#echo $latWGS84pos
#echo $latWGS84ref
#echo "original:"
#echo ${posWGS84[0]}
#echo ${posWGS84[1]}
#echo ${posWGS84[2]}

if [[ "$WRITE" == "true" ]]
then
exiftool -exif:gpsmapdatum="WGS-84" -exif:gpsaltitude="$alt" -exif:gpsaltituderef="Above Sea Level" -exif:gpslongitude="$lonWGS84pos" -exif:gpslongituderef="$lonWGS84ref" -exif:gpslatitude="$latWGS84pos"  -exif:gpslatituderef="$latWGS84ref" "$jpgname"  >/dev/null &
fi 
let "jpgtagged++"
echo "ALT= "$alt" LON= "$lonWGS84pos" LATref= "$lonWGS84ref" LAT= "$latWGS84pos" LATref= "$latWGS84ref""
}


#=========================  FUNCTION report ======================================================
function report   
{
echo "Report:" | tee -a geotag.log
echo "Found $jpgtot photos" | tee -a geotag.log
echo "Found $camtot CAM log lines" | tee -a geotag.log
echo "Found $trigtot TRIG log lines" | tee -a geotag.log
echo "Time between first photo and log was ""$typical_offset""s" | tee -a geotag.log

if [[ "$WRITE" == "true" ]]
then
	echo "Tagged $jpgtagged photos" | tee -a geotag.log
else
	echo "Could have tagged $jpgtagged photos" | tee -a geotag.log
fi 

if [[ "$MODE" != "POS" ]]
then
	echo "Detected $jpgmiss missing photos" | tee -a geotag.log
	echo "Detected $logmiss missing $MODE messages" | tee -a geotag.log
else
	echo "(Unable to detect and report missing pictures and CAM messages in POS mode)" | tee -a geotag.log
fi
if [[ "$jpgnotinlog" -gt 0 ]]
then
	echo "FAILED: to tag $jpgnotinlog jpg file(s) where no POS log matched (maybe increase difflimit?)" | tee -a geotag.log
fi


if [[ "$DELORG" == "true" ]]
then
	echo "INFO: Deleting original .JPG that were cloned by EXIF" | tee -a geotag.log
    sleep 1s
	rm *.JPG_original
fi
	
if [[ "$WRITE" == "true" ]]
then
	mv geotag.log geotagwrite.log
fi

}


#=========================  FUNCTION load log to memory ======================================================
function loadlogdump
{
echo "INFO: Loading logdump into memory..." | tee -a geotag.log
OLDIFS=$IFS
IFS=$'\012'  # \012 is a linefeed
logarray=( $(<logdump) )
IFS=$OLDIFS
logarraysize=${#logarray[@]}
echo "INFO: done, loaded $logarraysize lines." | tee -a geotag.log
}

#=========================  FUNCTION load camdump to memory ======================================================
function loadcamdump
{
echo "INFO: Loading camdump into memory..." | tee -a geotag.log
OLDIFS=$IFS
IFS=$'\012'  # \012 is a linefeed
logarray=( $(<camdump) )
IFS=$OLDIFS
logarraysize=${#logarray[@]}
echo "INFO: done, loaded $logarraysize lines." | tee -a geotag.log
}


##############################################################################################################
##############################################################################################################
##############################################################################################################
##############################################################################################################
##############################################################################################################
##############################################################################################################
#=========================  FUNCTION CAM only loop ======================================================
function posloop
{
echo "INFO: Searching within POS log..." | tee -a geotag.log
loggotpos=0


###############get JPG file#########
while [[ "$running" == 1 ]]
do

	if [ "$logmatch" == 1 ]  
		then  #read next picture only if log line accepted 
		jpgname=$(awk 'NR=="'"$jpglistindex"'"{print;exit}' jpglist)

	if [ "$jpgname" == "" ] 
		then
		echo "END: Last photo processed" | tee -a geotag.log
		running=0
		report
		exit 1
	fi
	readexif
fi

###############lookup LOG#########
logtarget=0 #reset, for new search
if [[ "$loglinereverse" -eq 1 || "$camindex" -eq "$camtot" || "$MODE" == "POS" ]] 
then 
POSONLY="true"
else
POSONLY="false"
fi

while [ "$logtarget" -eq 0 ]
do
let "loglinenum++"

#### check if we are out of log messages.
if [[ "$loglinenum" -gt "$logarraysize" ]]
then
echo "ERROR: Log ended at $loglinenum of $logarraysize while looking for $MODE message for filename $jpgname , maybe this photo is not a part of dataset, or in log, ignoring." | tee -a geotag.log
	
	#read next picture only if log line accepted
	jpgname=$(awk 'NR=="'"$jpglistindex"'"{print;exit}' jpglist)
	let "loglinenum = loglinenummatched" # revert to last known good position in log - or to START, to mach photos in random order..
	let "jpglistindex++" #move on to next
		if [ "$jpgname" == "" ] 
		then
		echo "END: Last photo processed" | tee -a geotag.log
		running=0
		report
		exit 1
		fi
	readexif
fi

logline=${logarray["$loglinenum"]}


    if [[ "$logline" == *CAM* ]] 
    then 
        logtype="CAM"
        let "camindex++"
        logtarget=1
        readlog    
        #	echo "DEBUG: at $loglinenum $logline" 
    fi
    if [[ "$logline" == *POS* ]] && [[ "$POSONLY" == "true" || "$loglinereverse" -eq 1 ]]
    then
        loggotpos=1
        logtype="POS"
        logtarget=1
        readlog  
        #echo "DEBUG: at $loglinenum $logline"  
    fi 

if [[ "$SKIPCAM" -gt 0 ]] && [[ "$logtarget" -eq 1 ]] 
    then
    let "SKIPCAM--"
    logtarget=0
fi



done

#### check if we are out of suitable log messages.
if [[ "$loglinenum" -gt "$logarraysize" ]]
then
echo "ERROR: Log ended at $loglinenum of $logarraysize while looking for CAM message for filename $jpgname maybe there are too many photos" | tee -a geotag.log
running=0
report
exit 1
fi

#echo "DEBUG relevant logline found loglinenumber=$loglinenum  logline=$logline "
######process log line 

######## Calculate offset
diff=$(( ( $(date -ud "$jpgdate $jpgtime" +'%s') - $(date -ud "$logdate $logtime" +'%s') ) )) #get differance in seconds (integer)
difftot=$(echo "scale=2; ($diff+($jpgtimems - $logtimems))" | bc -l) # return floting point
######## Calculate time since last trigger command
differr=$(( ( $(date -ud "$logdate $logtime" +'%s') - $(date -ud "$logdateprev $logtimeprev" +'%s') ) )) #get differance in seconds (integer)
difftoterr=$(echo "scale=2; ($differr+($logtimems - $logtimemsprev))" | bc -l) # return floting point
difftotoff=$(echo "scale=2; ($difftot - $typical_offset)" | bc -l) 
difftoterroff=$(echo "scale=2; ($differr - $typical_offset)" | bc -l) # return floting point 
 if [ "$typical_offset" == "notset" ]
 then
	 typical_offset=$difftot
     echo "INFO: Expected time offset between camera and log is ""$typical_offset""s (positive = camera is ahead)" | tee -a geotag.log
 fi
 

 
###############compare offset to previous offset to detect skipped photos
   
   diffeee=$(echo "(($difftot - $typical_offset)*100)" | bc -l | cut -f1 -d"." ) #keep integer
    if [[ "$diffeee" -gt "$difflimit" ]] || [[ "$diffeee" -lt -"$difflimit" ]]
		
		then    
		    if [[ "$MODE" == "POS" ]]  #Do fail analysis ONLY in CAM mode
            then  ##Speed up POS mode
			logmatch=0
                if [[ "$diffeee" -gt 50 ]]   # are we more than 1 sec behind ?
                then
                let "loglinenum=loglinenum+25"
                #echo "DEBUG: > 1 sec behind, jumping ahead"
                fi
                if [[ "$diffeee" -lt -25 ]]   # are we more than 0.5 sec ahead ?
                then
                let "loglinenum=loglinenum-30"
                let "revjumps++"
                #echo "DEBUG: > 0.5 sec ahead, jumping back"
                
					if [[ "$revjumps" -gt 30 ]]  ## we have backed up too many times, the image must be invalid for this log !
					then
					echo "ERROR: Failed to find log time for image $jpgname, it's not in log or does not belong to this dataset.( maybe try higher difflimit ?)" | tee -a geotag.log
					let "jpgnotinlog++"
					let "jpglistindex++" #move on to next
					let "revjumps=0"
					logmatch=1 #trigger lookup of next picture
								
					fi
				fi
                    
              ##do CAM mode analysis
                
            fi
            #logmatch=1 ###############################################################ACCEPT ANYTHING
            #let "jpglistindex=jpglistindex+1"  ###############################################################ACCEPT ANYTHING

		else  ## PHOTO AND LOG MATCHING
            
            percent=$(echo "scale=2; (($jpglistindex/$jpgtot)*100)" | bc -l | cut -f1 -d".") 
			echo "MATCHED: ""$percent""% Done, time diff.: ""$difftotoff""s between $jpgname $jpgdate $jpgtime & $logtype log $loglinenum $logdate $logtime"  | tee -a geotag.log
			let "revjumps=0"
            geotag
            logmatch=1
            #logmatcgfailinrow=0
            let "jpglistindex++"
            let "loglinenummatched = loglinenum + 1"  # if we need to go back for POS message, go here
            #loglinereverse=0
    fi    #save first image camera vs log offset

logdateprev=$logdate  #for calculation of picture spacing
logtimeprev=$logtime  #for calculation of picture spacing
logtimemsprev=$logtimems #for calculation of picture spacing
logtarget=0 #reset so we are looking for next logline next time

done  #running
}

##### END POS loop ###########################################################################################
##############################################################################################################
##############################################################################################################
##############################################################################################################
##############################################################################################################
##############################################################################################################

##############################################################################################################
##############################################################################################################
##############################################################################################################
##############################################################################################################
##############################################################################################################
##############################################################################################################
#=========================  FUNCTION CAM only loop ======================================================
# In case of missing photo: try next log entry
# in case of missing log entry:  do not tag photo
function camloop
{
echo "INFO: Searching for CAM only..." | tee -a geotag.log
loggotpos=0


###############get JPG file#########
while [[ "$running" == 1 ]] ; do
	if [ "$logmatch" == 1 ]  
		then  #read next picture only if log line accepted
		jpgname=$(awk 'NR=="'"$jpglistindex"'"{print;exit}' jpglist)
		if [ "$jpgname" == "" ] ; then
			echo "END: Last photo processed" | tee -a geotag.log
			running=0
			report
			exit 1
		fi
	readexif
	fi

###############lookup LOG#########
logtarget=0 #reset, for new search

while [ "$logtarget" -eq 0 ] ; do
let "loglinenum++"
logline=${logarray["$loglinenum"]}

    if [[ "$logline" == *"$MODE"* ]] ; then
		logtype="$MODE"
        logtarget=1
        readlog    
        #echo "DEBUG: at $loglinenum $logline" 
    fi

#skip CAM lines if needded
if [[ "$MODE" == "CAM"  && "$SKIPCAM" -gt 0 ]] && [[ "$logtarget" -eq 1 ]] ; then
    let "SKIPCAM--"
    echo "INFO: Skipped a CAM log line." | tee -a geotag.log
    logtarget=0
fi

#skip TRIG lines if needded
if [[ "$MODE" == "TRIG"  && "$SKIPTRIG" -gt 0 ]] && [[ "$logtarget" -eq 1 ]] ; then
    let "SKIPTRIG--"
    logtarget=0
fi

done

#### check if we are out of suitable log messages.
if [[ "$loglinenum" -gt "$logarraysize" ]] ; then
	echo "ERROR: Log ended at $loglinenum of $logarraysize while looking for $MODE message for filename $jpgname maybe there are too many photos" | tee -a geotag.log
	running=0
	report
	exit 1
fi

######## Calculate offset
diff=$(( ( $(date -ud "$jpgdate $jpgtime" +'%s') - $(date -ud "$logdate $logtime" +'%s') ) )) #get differance in seconds (integer)
difftot=$(echo "scale=2; ($diff+($jpgtimems - $logtimems))" | bc -l) # return floting point
######## Calculate time since last trigger command
differr=$(( ( $(date -ud "$logdate $logtime" +'%s') - $(date -ud "$logdateprev $logtimeprev" +'%s') ) )) #get differance in seconds (integer)
difftoterr=$(echo "scale=2; ($differr+($logtimems - $logtimemsprev))" | bc -l) # return floting point
difftotoff=$(echo "scale=2; ($difftot - $typical_offset)" | bc -l) 
difftoterroff=$(echo "scale=2; ($differr - $typical_offset)" | bc -l) # return floting point 
 if [ "$typical_offset" == "notset" ] ; then
	typical_offset=$difftot
    echo "INFO: Expected time offset between camera and log is ""$typical_offset""s (positive = camera is ahead)" | tee -a geotag.log
 fi
 
difflimit=100 # CAM difflimit 1sec variation between command and exif


###############compare offset to previous offset to detect skipped photos
    diffeee=$(echo "(($difftot - $typical_offset)*100)" | bc -l | cut -f1 -d"." ) #keep integer
    if [[ "$diffeee" -gt "$difflimit" ]] || [[ "$diffeee" -lt -"$difflimit" ]]
		then    ##TOO MUCH DIFFERENCE
        logmatch=0 
            if [[ "$diffeee" -lt -100 ]] ; then  # are we more than 1 sec ahead ?
                #let "loglinenum=loglinenum-10"
                echo "WARNING: Found extra photo for which there is no $MODE event. I am at logline $loglinenum (Details below)"
                echo "WARNING: Big Time diff.: ""$difftotoff""s between $jpgname date $jpgtime and $logtype logline $loglinenum $logdate $logtime Time since last $MODE event was ""$difftoterr""s"  | tee -a geotag.log
                logmatch=1  #make the log-fetcher get next picture 
                let "loglinenum=loglinenummatched"  #make log-fetcher keep the log line for next picture
                let "jpglistindex++" #move on to next picture
                let "logmiss++"  #count missing log items
                
            fi
        
        
		else  ## PHOTO AND LOG MATCHING
            
            if [[ $logmatch -eq 0 ]] && [[ "$logmatcgfailinrow" -lt 5 ]] ; then
                let "jpgmiss++"
                echo "ERROR: MISSING PHOTO #$jpgmiss detected, matched current photo with next logged $logtype logline" | tee -a geotag.log
			fi
            if [[ $logmatch -eq 0 ]] && [[ "$logmatcgfailinrow" -gt 4 ]] ; then
                let "logmiss++"
                echo "ERROR: MISSING $MODE log entry detected, matched current photo with logged $logtype logline" | tee -a geotag.log
			fi

            percent=$(echo "scale=2; (($jpglistindex/$jpgtot)*100)" | bc -l | cut -f1 -d".") 
			echo "MATCHED: ""$percent""% Done, time diff.: ""$difftotoff""s between $jpgname $jpgdate $jpgtime & $logtype log $loglinenum $logdate $logtime Time since last $MODE command ""$difftoterr""s"  | tee -a geotag.log
            geotag
            logmatch=1
            logmatcgfailinrow=0
            let "jpglistindex++"
            let "loglinenummatched = loglinenum + 1"  # if we need to go back for POS message, go here
            loglinereverse=0
    fi    #save first image camera vs log offset

logdateprev=$logdate  #for calculation of picture spacing
logtimeprev=$logtime  #for calculation of picture spacing
logtimemsprev=$logtimems #for calculation of picture spacing
logtarget=0 #reset so we are looking for next logline next time

done  #running
}

##### END CAM only loop ######################################################################################
##############################################################################################################
##############################################################################################################
##############################################################################################################
##############################################################################################################
##############################################################################################################


######################################################################  List JPG's
find . -maxdepth 1 -name "*.[Jj][Pp][Gg]" | sort >jpglist
jpgtot=$(wc -l < jpglist)
echo "INFO: Found $jpgtot photos" | tee -a geotag.log



######################################################################  look for logdump, create if needed
if [ -f ./logdump ]; then
    echo "INFO: found logdump"
else
    binname=$(find . -maxdepth 1 -name "*.[Bb][Ii][Nn]")
    if [[ "$binname" == "" ]]; then
        echo "ERROR: .BIN log not found, make sure directory contain a .BIN or .bin log or logdump file." | tee -a geotag.log
        exit 1
    fi
    for binname in $(find . -maxdepth 1 -name "*.[Bb][Ii][Nn]"); do
        if [[ "$ML" == "true" ]]; then
            echo "INFO: adding $binname to logdump..." | tee -a geotag.log
            mavlogdump.py --types POS,CAM,TRIG --format standard "$binname" >> logdump
        else
            if [[ "$logsdone" < 1 ]]; then 
                echo "INFO: dumping $binname to logdump..." | tee -a geotag.log
                mavlogdump.py --types POS,CAM,TRIG --format standard "$binname" > logdump
            fi
        fi
        let "logsdone ++"
    done
fi
echo "INFO: done" | tee -a geotag.log

######################################################################  DUMP camdump
if [ ! -f ./camdump ] && [[ "$MODE" != "POS" ]]; then
    echo "INFO: no camdump found, exporting loglines from .bin to camdump..." | tee -a geotag.log
    mavlogdump.py --types CAM,TRIG --format standard "$binname" > camdump
    echo "INFO: done" | tee -a geotag.log
else
    echo "INFO: camdump found" | tee -a geotag.log
fi


######################################################################  DUMP exiflog
#if [ ! -f ./exifdump ]; then
#    echo "INFO: no exifdump found, exporting exif names and timestamps exifdump..." | tee -a geotag.log
#    #mavlogdump.py --types POS,CAM,TRIG --format standard "$binname" > logdump
#    echo "INFO: done" | tee -a geotag.log
#else
#    echo "INFO: exifdump found" | tee -a geotag.log
#fi


######################################################################  Count CAM events
camtot=$(grep CAM logdump | wc -l)
echo "INFO: Found $camtot CAM log lines (camera shutter sensed)" | tee -a geotag.log

######################################################################  Count TRIG events
trigtot=$(grep TRIG logdump | wc -l)
echo "INFO: Found $trigtot TRIG log lines (commands to shoot)" | tee -a geotag.log



######################################################################  LOAD exifs to memory
#echo "INFO: Loading Photo data into memory..." | tee -a geotag.log
#OLDIFS=$IFS
#IFS=$'\012'  # \012 is a linefeed
#logarray=( $(<exifdump) )
#IFS=$OLDIFS
#logarraysize=${#exifarray[@]}
#echo "INFO: done, loaded $exifarraysize lines." | tee -a geotag.log



######################################################################  Are we searching or processing ?
#if [ "$FIND" == "true" ]
#then
#    loadcamdump
#    camloop
#else
  loadlogdump
    if [[ "$MODE" == "POS" ]]; then
        posloop
    else   
        camloop
    fi
#fi

