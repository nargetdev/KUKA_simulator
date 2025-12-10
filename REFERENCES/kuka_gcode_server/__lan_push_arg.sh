#!/opt/homebrew/bin/fish

for file in $argv
    echo "Copying $file..."
    cp $file /Volumes/172.31.1.147/KRC/ROBOTER/KRC/R1/
end