#Processes and Plots all segments
 #Must first create 'filename'.m with python/batch program
 #Set current directory to ...\data
  #DU 6/17/19: Copied from plot_segments.m
  #DU 5/20/19: Modified to drive plot parameters from this top-level program
              #Plots to be made/saved specified by "whichplots"
              #Specify which segments to be plotted
#INPUT PARAMETERS              
label = 'Joetest3'  #filename (*.m)
whichsegments = [4];   #Which segments to plot? ([] for "all")
#Runway pilot station location and heading
 # rhdg is runway heading: 0 is North, 90 is East
 # latitude is converted to Y axis in meters,longitude to X axis in meters
  # AAM East pilot station/runway:
  #origin=[39.8420194 -105.2123333 1804]; rhdg=106
  # AAM West pilot station/runway:  
  #origin=[39.8420888 -105.2126722 1805]; rhdg=106;
  # LAMA Runway:
  origin=[40.06403056 -104.9204722 1536]; rhdg=75.68;
#Plot parameters:
  rTol=20; #wings out-of-roll tolerance for color to be "red"
  posIndex=21; #posIndex=21 helps when pressure/altitude sensor bad (normal=2)
  pThresh=88; 
  levelThresh=10;
  whichplots=[0 0 1 0]; #Specify which plots: [3D, eulerVMP, Manuever, RPY]
    #Note: plot types (.jpg, .ofig set in plot_maneuver.m subfunction savefig()
  i = 1;  #Start with this segment  

#Process Data: create .work file and load into workspace
if !exist([label ".work"], "file")
  if !exist([label ".m"])
    disp(sprintf(["Running " "dflog2octave2.bat " label]));
    system(["..\\setup\\dflog2octave2.bat " label]);
  endif
  buildWorkspace(label,2,0)
endif
disp(sprintf("Loading %s", [label ".work"]));
load ([label ".work"])

#Plot segments 
if (length(whichsegments) == 0)  #plot all segments
  N = length(segments);  #Note: segments loaded from *.work file
  disp(sprintf("Plotting all N segments: %s", N));
  while (i < N)  #DU 5/20/19
    plot_track_color(segments, data, label, i, ...  
        origin, rhdg, rTol, posIndex, pThresh, levelThresh, whichplots)
  # pause(5);
    i = i + 1
  endwhile
else #only plot specified segments
  for i=1:length(whichsegments)
    disp(sprintf("Plotting segment: %s", whichsegments(i)));
    plot_track_color(segments, data, label, whichsegments(i), ...  
        origin, rhdg, rTol, posIndex, pThresh, levelThresh, whichplots)
  endfor
endif
close all
clear