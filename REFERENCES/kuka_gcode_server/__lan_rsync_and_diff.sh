#!/opt/homebrew/bin/fish

# Store the changed files in temporary files for later processing
set dat_src_changes (rsync -avn --itemize-changes /Volumes/172.31.1.147/KRC/ROBOTER/KRC/R1/*{dat,src,sub} . | grep '^>' | awk '{print $2}')
set eki_changes (rsync -avn --itemize-changes /Volumes/172.31.1.147/KRC/ROBOTER/Config/User/Common/EthernetKRL/*.xml EKI/ | grep '^>' | awk '{print "EKI/"$2}')
set config_changes (rsync -avn --itemize-changes /Volumes/172.31.1.147/KRC/ROBOTER/Config/User/Common/KLIConfig.xml CONFIG/ | grep '^>' | awk '{print "CONFIG/"$2}')

# Combine all changed files
set all_changes $dat_src_changes $eki_changes $config_changes

# If there are changes, offer to review them
if test (count $all_changes) -gt 0
    echo "Changed files detected. Would you like to review the differences? [Y/n]"
    read -l confirm
    if test "$confirm" = "" -o "$confirm" = "y" -o "$confirm" = "Y"
        for file in $all_changes
            set source_path ""
            switch $file
                case "EKI/*"
                    set source_path "/Volumes/172.31.1.147/KRC/ROBOTER/Config/User/Common/EthernetKRL/"(basename $file)
                case "CONFIG/*"
                    set source_path "/Volumes/172.31.1.147/KRC/ROBOTER/Config/User/Common/KLIConfig.xml"
                case "*"
                    set source_path "/Volumes/172.31.1.147/KRC/ROBOTER/KRC/R1/"(basename $file)
            end
            
            echo "Showing diff for $file"
            echo "Press q to move to next file"
            diff -u $file $source_path | less
        end
    end
end