while getopts p:d: flag
do
    case "${flag}" in
        p) pfile=${OPTARG};;
        d) dynamics=${OPTARG};;
    esac
done
echo "pfile: $name";
echo "dynamics: $age";