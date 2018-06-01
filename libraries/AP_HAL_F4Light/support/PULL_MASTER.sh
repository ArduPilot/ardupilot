git checkout  master && \
git pull --rebase git://github.com/ArduPilot/ardupilot.git master

# Step 2: Merge the changes and update on GitHub.

git checkout Revo && \ 
git rebase master && git push --force origin Revo

