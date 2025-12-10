#!/opt/homebrew/bin/fish

# Show what would be copied for data/source/sub files (using -n for dry-run)
rsync -avn --itemize-changes /Volumes/172.31.1.147/KRC/ROBOTER/KRC/R1/*{dat,src,sub} .

# Show what would be copied for XML files
rsync -avn --itemize-changes /Volumes/172.31.1.147/KRC/ROBOTER/Config/User/Common/EthernetKRL/*.xml EKI/

# Show what would be copied for KLIConfig
rsync -avn --itemize-changes /Volumes/172.31.1.147/KRC/ROBOTER/Config/User/Common/KLIConfig.xml CONFIG/