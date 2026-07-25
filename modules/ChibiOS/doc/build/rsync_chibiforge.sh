#!/bin/bash
export BRANCH=21.11
rsync -avzP -e "ssh -p7822" --delete --exclude=.* ../manuals/html/rt/ chibiforge.org:~/public_html/doc/${BRANCH}/rt
rsync -avzP -e "ssh -p7822" --delete --exclude=.* ../manuals/html/nil/ chibiforge.org:~/public_html/doc/${BRANCH}/nil
rsync -avzP -e "ssh -p7822" --delete --exclude=.* ../manuals/html/hal/ chibiforge.org:~/public_html/doc/${BRANCH}/hal
rsync -avzP -e "ssh -p7822" --delete --exclude=.* ../manuals/html/ex/ chibiforge.org:~/public_html/doc/${BRANCH}/ex
rsync -avzP -e "ssh -p7822" --delete --exclude=.* ../manuals/html/full_rm/ chibiforge.org:~/public_html/doc/${BRANCH}/full_rm

