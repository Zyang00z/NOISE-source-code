#!/bin/sh
# Generate front+back mill + cut gcode
#
# offset 0.125 (for isolation) can be tuned if pcb has room
pcb2gcode \
  --back *-B_Cu.gbr \
  --outline *-Edge_Cuts.gbr \
  --fill-outline \
  --voronoi \
  --drill *.drl \
  --cutter-diameter=0.6 \
  --milldrill-diameter=0.6 \
  --cut-infeed=0.1 \
  --cut-feed=250 \
  --cut-speed=2 \
  --cut-vertfeed=128 \
  --milldrill \
  --drill-feed=100 \
  --drill-speed=0 \
  --metric \
  --metricoutput \
  --mill-feed=100 \
  --mill-speed=2 \
  --mill-vertfeed=254 \
  --zchange=2 \
  --zcut=-1.75 \
  --zdrill=-1.8 \
  --zsafe=1.2 \
  --zwork=-0.11 \
  --onedrill \
  --drill-side back \
  --cut-side=back \
  --nog64 \
  | tee pcb2gcode_output
  
#  --zero-start \

# process back (remove unsupported g-codes, retain hold)
cat back.ngc | grep -v "^G64" back.ngc | grep -v "^M6" > board.ngc
rm back.ngc

# process drill file (remove tool changes, hold)
cat milldrill.ngc | grep -v "^T" | grep -v "^M0" > drill-cut.ngc
rm milldrill.ngc

# process outline (remove unsupported g-codes, hold)
cat outline.ngc | grep -v "^G64" | grep -v "^M0" >> drill-cut.ngc
rm outline.ngc

# tidy svgs
mkdir -p svg
mv *.svg svg
