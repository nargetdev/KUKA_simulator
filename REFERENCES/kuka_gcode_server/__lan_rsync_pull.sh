#!/opt/homebrew/bin/fish
# Pull robot files using rsync with detailed change information
# --itemize-changes shows what files are being modified:
#   >f+++ = new file
#   >f.st = file with changed size and time
#   >f... = existing file, no changes

rsync --itemize-changes /Volumes/172.31.1.147/KRC/ROBOTER/KRC/R1/*{dat,src,sub} .
rsync --itemize-changes /Volumes/172.31.1.147/KRC/ROBOTER/Config/User/Common/EthernetKRL/*.xml EKI/
rsync --itemize-changes /Volumes/172.31.1.147/KRC/ROBOTER/Config/User/Common/KLIConfig.xml CONFIG/
