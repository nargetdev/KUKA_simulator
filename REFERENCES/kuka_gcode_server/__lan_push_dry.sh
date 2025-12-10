#!/opt/homebrew/bin/fish

# Show what would be copied for data/source/sub files (using -n for dry-run)
rsync -avn --itemize-changes *{dat,src,sub} /Volumes/172.31.1.147/KRC/ROBOTER/KRC/R1/

# Show what would be copied for XML files
rsync -avn --itemize-changes EKI/*.xml /Volumes/172.31.1.147/KRC/ROBOTER/Config/User/Common/EthernetKRL/