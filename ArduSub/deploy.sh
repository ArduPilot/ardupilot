# Deployment script for ArduSub binaries

BINARYDIR='../binaries/Sub'

BOARDS=(px4-v2)
TARGETS=(bluerov vectored vectored6dof simplerov)

MONTH=$(date +"%Y-%m")

DATETIME=$(date +"%Y-%m-%d-%H:%M")

echo $DATETIME

COMMIT=$(git rev-parse --short HEAD)

echo $COMMIT

mkdir $BINARYDIR/$MONTH
mkdir $BINARYDIR/$MONTH/$DATETIME

for board in "${BOARDS[@]}"
do
	for target in "${TARGETS[@]}"
	do
		make $board-$target
		if [ $? -eq 0 ]; then
			mkdir $BINARYDIR/$MONTH/$DATETIME/PX4-$target
			cp ArduSub-v2.px4 $BINARYDIR/$MONTH/$DATETIME/PX4-$target/ArduSub-v2.px4
			git log -1 > $BINARYDIR/$MONTH/$DATETIME/PX4-$target/git-version.txt
			echo "\nAPMVERSION: ArduSub V3.4-dev" >> $BINARYDIR/$MONTH/$DATETIME/PX4-$target/git-version.txt
		fi
		echo $board-$target
	done
done

# Copy to "latest" folder
rm -rf $BINARYDIR/latest/*
cp -r $BINARYDIR/$MONTH/$DATETIME/* $BINARYDIR/latest/