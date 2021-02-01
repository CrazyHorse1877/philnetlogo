extensions [matrix]

breed [targets target]
breed [rescuers rescue-chopper]
breed [random-ants random-ant]
breed [square-uavs square-uav]
breed [parallel-uavs parallel-uav]
breed [greedy-uavs greedy-uav]
breed [centres centre]


patches-own [
  target?              ;; true while at target, false elsewhere
  home?                ;; true while at home, false elsewhere
  home-prob           ;; nuber that is higher closer to home
  squad-probability
  xoffset ;;future
  yoffset ;;future
]

globals [
prob-ahead
prob-right
prob-left
zoom            ;; Future
num-found       ;; Number of targets currently located
agentcounter    ;; number of agents to be launched
done            ;; Have we found them all?
g_action        ;; Global Action command
]

turtles-own [
turn
xcord           ;; Future
ycord           ;; Future
]


targets-own[
desired-heading
;turn-rate
last-turn-L?
speedaction

found?
not-found
]

rescuers-own [
desired-heading
;turn-rate
last-turn-L?
speed
action

found              ;; did this agent find something
gpsx               ;; Real x location
gpsy               ;; Real y location
]
random-ants-own [
;sensor-range    ;;distance sensor can see
desired-heading
current-heading
;turn-rate
turnto
last-turn-L?
speed
action

ticker
maxpro
found
gpsx
gpsy
]
square-uavs-own [
;sensor-range    ;;distance sensor can see
desired-heading
current-heading
;turn-rate
last-turn-L?
speed
action
ticker
turncount
found
gpsx
gpsy
]
parallel-uavs-own [
;sensor-range    ;;distance sensor can see
desired-heading
current-heading
;turn-rate
last-turn-L?
speed
action

ticker
found
gpsx
gpsy
]
greedy-uavs-own [
;sensor-range    ;;distance sensor can see
desired-heading
current-heading
;turn-rate
last-turn-L?
speed
action
ticker
turnto
found
gpsx
gpsy
maxpro
]


;;;;;;;;;;;;;;;;;;;;;;;;
;;; Setup procedures ;;;
;;;;;;;;;;;;;;;;;;;;;;;;

;;;;;;;;;;;;;;;;;;;;;;;
;;       MAIN        ;;
;;;;;;;;;;;;;;;;;;;;;;;

to setup
  clear-all
  clear-plot
  clear-drawing
  clear-turtles
  ;import-pcolors-rgb "darwin_large.bmp"

  set greedy_team 1
  set  num-found 0
 
  ;; Automatic calulation of number ov vehicles reqiuired based on the varience of sensors and target
  ;set greedy_team ceiling ( ( ( (V_X + V_Y) / 2 ) / ( ( 100 - overlap ) / 100 ) ) / sensor-range )

  set-default-shape targets "x"
  set-default-shape rescuers "airplane"
  set-default-shape random-ants "airplane"
  set-default-shape square-uavs "airplane"
  set-default-shape greedy-uavs "airplane"
    
  ;; Spawn Turtles / Agents
  launch-enemy enemy
  launch-squad squad
  set agentcounter team

  ;; Set a turtle at origin, = easy calculations later on
  create-centres 1 [set size 1 setxy ( target_x ) ( target_y ) ]
                    
  setup-patches

  ;set steerrange 2.5
  set g_action "go"
  ask turtles [ifelse trails? [ pd ][ pu ] ]
  
end

;; RESET ALL
to reset-all
   clear-turtles
   clear-drawing
   
   set  num-found 0

   ;set team ceiling ( ( ( (V_X + V_Y) / 2 ) / ( ( 100 - overlap ) / 100 ) ) / sensor-range )

   setup-patches
   launch-squad squad
   launch-enemy enemy

   set agentcounter team
   create-centres 1 [set size 1
     setxy ( target_x ) ( target_y ) ]

   ask turtles [ifelse trails? [ pd ][ pu ] ]
   set g_action "go"

end

;; Setup the home location
to setup-home 
    ask hangers [ set size 5 ]
    set g_action "go-home"
end

;;;;;;;;;;;;;;;;;;;;;;;
;;   SETUP PATCHES   ;;
;;;;;;;;;;;;;;;;;;;;;;;
to setup-patches
  
  ifelse background? [import-drawing "darwin_large.bmp"] ;; Load the raster graphic Map
    [ ask patches [ if ( PDF? and not background? ) [ recolor-patch ] ] ]
    
  ask patches [ setup_probabliity ( target_x ) ( target_y ) V_X V_Y ] ;; Setup initial target probabilities
  normalise ;; ensure we are working with a normalised initial prior

  ask patches [ set home? ((distancexy  (max-pxcor * 0.9) (max-pycor * 0.1) ) < 2 )]   ;; Set home location
  ask patches [ set home-prob 1000 - distancexy  (max-pxcor * 0.9) (max-pycor * 0.1) ] ;; Set home tracer
end


;; Create some grid based 2D Gaussian functions
to setup_probabliity [ x0 y0 sigma_x sigma_y ]
 let prob-val max-pxcor

 let A 1
 let theta  0
 let a1 ( cos ( theta ) ^ 2 / 2 / sigma_x ^ 2 + sin ( theta ) ^ 2 / 2 / sigma_y ^ 2 )
 let b1 (- sin ( 2 * theta ) / 4 / sigma_x ^ 2  + sin ( 2 * theta ) / 4 / sigma_y ^ 2 )
 let c1 (sin ( theta ) ^ 2 / 2 / sigma_x ^ 2 + cos ( theta ) ^ 2 / 2 / sigma_y ^ 2 )
 
 let Z ( A * exp ( - ( a1 * ( ( pxcor * worldscale ) - ( x0 * worldscale ) ) ^ 2 + 2 * b1 * ( ( pxcor * worldscale ) - ( x0 * worldscale ) ) * ( (  pycor * worldscale ) - ( y0 * worldscale ) ) + c1 * ( ( pycor * worldscale ) - ( y0 * worldscale ) ) ^ 2 ) ) );
 
 set target-probability Z
 set square-probability Z
 set squad-probability Z
 set parallel-probability Z
  
end


;;  Setup the sensor observation model using a gaussing 2D PDF
to setup-sensor

  ;let m matrix:from-row-list [[1 2 3] [4 5 6]]   ;; future
  let sigma_x sensor-range   ;; x variance 
  let sigma_y sensor-range   ;; y variance 
  let x0 xcor                ;; X mean 
  let y0 ycor                ;; y mean 
  let a1 ( cos ( heading ) ^ 2 / 2 / sigma_x ^ 2 + sin ( heading ) ^ 2 / 2 / sigma_y ^ 2 )
  let b1 (- sin ( 2 * heading ) / 4 / sigma_x ^ 2  + sin ( 2 * heading ) / 4 / sigma_y ^ 2 )
  let c1 (sin ( heading ) ^ 2 / 2 / sigma_x ^ 2 + cos (heading ) ^ 2 / 2 / sigma_y ^ 2 )
  ask patches in-radius ( 4 * ( sensor-range / sensorscale ) ) 
    [
      
      let Z ( Weight * exp ( - ( a1 * ( pxcor - x0 ) ^ 2 + 2 * b1 * ( pxcor - x0 ) * (  pycor - y0 ) + c1 * (  pycor - y0 ) ^ 2 ) ) );
      set squad-probability squad-probability * ( 1 - Z )
    ]
end

to recolor-patch  ;; patch procedure
  ;; scale color to show target probability
   
  if show-probability = 1  [ if pcolor != white [ set pcolor scale-color lime squad-probability   0 0.001]  ]

end


;; greedy-uavs config
to launch-greedy-uavs [number sendtocenter? tcolor ]
  create-greedy-uavs number [ ;; setup greedy algorithm agents
    set size 2
    set pen-size 2
    set color ( 15 + 10 * ( random 5 ) )
    setxy (max-pxcor * 0.98) ( max-pycor * 0.36 )
    set heading 270
    set speed ( 110 + random 10 ) / 100
    ifelse sendtocenter? [ set action "centre" ] [ set action "search" ]  ;; Setup Action
  ]
end

;; square search config
to launch-square-search [number]
  create-square-uavs number [ ;; setup expandign square agents
    set size 2 
    set color 104
    setxy (max-pxcor * 0.9) (max-pycor * 0.1)
    set heading ( random 90 ) + 270
    set last-turn-L? true
    set desired-heading 0
    set speed ( 110 + random 10 ) / 100
    set action "centre"
    set turncount 0
    set ticker 0
  ]
end

;;  Setup and scatter lost targets
to launch-enemy [ number ]
  create-targets number
  [ set size 0                   ;; Missing target size
    set color orange             ;; Missing target colour
    ;if V_X > 20 [ set V_X 20 ]  ;; Limit variance
    ;if V_Y > 20 [ set V_Y 20 ]  ;; Limit variance
    let x-pos abs ( random-normal abs( target_x ) (V_X / worldscale ) )
    let y-pos abs ( random-normal abs( target_y ) (V_Y / worldscale ) )
    setxy x-pos y-pos
    set found? false
  ]
end

;; instruch agents to go home
to return-to-home  ;; turtle procedure
  ifelse home?
  [ 
    set action "home"
    stop]
  [ send-me-home ] 
end


to look-for-target  ;; turtle procedure
  if ( [who] of targets in-radius (( sensor-range / sensorscale   ) * 2.5 ) ) != [] ;nobody 
    [
      ask targets in-radius ( ( sensor-range / sensorscale  ) * 2.5 )[ if not found? [ set found? true
        set time2rescue time2rescue - 1
        set num-found num-found + 1
        if show_targets? [ set color green set size 4]
        
      ] ]
      ; if set action "loiter"
      ; set found [ who ] of targets in-radius sensor-range / sensorscale 
    ]
end

to search-greedy  ;; go in direction of strongest probabliity
    ifelse patch-ahead (( sensor-range / sensorscale ) * steerrange ) != nobody  
      [
        ifelse ( [ pcolor ] of patch-ahead (( sensor-range / sensorscale ) * steerrange ) != white )
        [ 
          ask patch-ahead (( sensor-range / sensorscale   ) * steerrange ) [ set prob-ahead target-probability ] 
        ]
        [
          set prob-ahead   ( - 100 ) 
        ]
      ]
      [ set prob-ahead   ( - 100 ) ]
 
      
    ;set turn "no"
    foreach [ 15 35 35  95  125 ]
    [
      ;  Add navagation aids, get probability
      ifelse patch-right-and-ahead ? (( sensor-range / sensorscale   ) * steerrange ) != nobody
      [
        ifelse ( [ pcolor ] of patch-right-and-ahead ? (( sensor-range / sensorscale ) * steerrange )  != white ) 
        [ 
          ask patch-right-and-ahead ? (( sensor-range / sensorscale   ) * steerrange )
          [
            if steerdot [ set pcolor cyan ]
            set prob-right target-probability
          ]
        ]
        [ set prob-right ( -2 )]
      ]
      [ set prob-right ( -2)]
        
      if (prob-right > prob-ahead and prob-right > maxpro)
        [
          set turn "right" 
          set maxpro prob-right 
          set turnto ?
        ]
    ]
    foreach [ 10 30 60 90 120 ]
    [
      ;  Add navagation aids, get probability
      ifelse patch-left-and-ahead ? (( sensor-range / sensorscale   ) * steerrange ) != nobody
      [
        ifelse ( [ pcolor ] of patch-left-and-ahead ? (( sensor-range / sensorscale ) * steerrange ) != white  ) 
        [ 
          ask patch-left-and-ahead ? (( sensor-range / sensorscale   ) * steerrange )
          [
            if steerdot [ set pcolor yellow ] 
            set prob-left target-probability ]
          ]
        [ set prob-left -3 ]
      ]
      [ set prob-left -3 ]
      
      if (prob-left > prob-ahead) and (prob-left > maxpro)
        [
          set turn "left"
          set turnto ?
        ]
    ]
    if not can-move? 1 [ wiggle 20 ]
        
    if turn = "right" [ifelse turnto < turn-rate * sensorscale  [ rt turnto * sensorscale ] [ rt turn-rate * sensorscale ] ]
    if turn = "left" [ifelse turnto < turn-rate * sensorscale  [ lt turnto * sensorscale ] [ lt turn-rate * sensorscale ] ]
    set turn "no"
    if false
    [
      show max-one-of patches [target-probability]
      show max [target-probability] of patches
    ]
    set maxpro 0
    set prob-right 0
    set prob-left 0
    set turnto 0
end

to search-like-ant 
  
  if true[
    ifelse patch-ahead (( sensor-range / sensorscale ) * steerrange ) != nobody [
      ask patch-ahead (( sensor-range / sensorscale ) * steerrange ) [ set prob-ahead squad-probability ]]
    [ set prob-ahead -1 ]
    ;set turn "no"
    foreach [ 5 15 20 25 30 45 60 90 ][
      ;  Add navagation aids, get probability
      ifelse patch-right-and-ahead ? (( sensor-range / sensorscale ) * steerrange ) != nobody [
        ask patch-right-and-ahead ? (( sensor-range / sensorscale ) * steerrange )
        [
          ;if not background? [ set pcolor white ]
          set prob-right squad-probability 
          ]
        ]
      [ set prob-right -1.0 ]
      if (prob-right > prob-ahead) [ set turn "right" 
        set maxpro prob-right 
        set turnto ?]
    ]
    foreach [ 5 15 20 25 30 45 60 90 ][
      ;  Add navagation aids, get probability
      ifelse patch-left-and-ahead ? (( sensor-range / sensorscale ) * steerrange ) != nobody [
        ask patch-left-and-ahead ? (( sensor-range / sensorscale ) * steerrange )
        [ 
          ;if not background? [ set pcolor yellow ] 
          set prob-left squad-probability 
        ]
      ]
      [ set prob-left 0 ]
      if (prob-left > prob-ahead) and (prob-left > maxpro) [ set turn "left"
        set turnto ?]
    ]
    if not can-move? 1 [ wiggle 20 ]
  ]
  if turn = "right" [ifelse turnto < turn-rate [ rt turnto / 2 ] [ rt turn-rate / 10] ]
  if turn = "left" [ifelse turnto < turn-rate [ lt turnto / 2 ] [ lt turn-rate / 10] ]
  set turn "no"
  if false[
    show max-one-of patches [squad-probability]
    show max [squad-probability] of patches
  ]
end



;; Enable drawing of "No-Go" Zones
to patch-draw
  if mouse-down? [    ;; reports true or false to indicate whether mouse button is down
    ask patch mouse-xcor mouse-ycor  [ 
      ask patches in-radius 5 [ set pcolor [ white ] of myself
      ]
    ]
  ]
end



;; Realy basic (fast) controller to send agents home
to send-me-home  ;; turtle procedure
  set prob-ahead home-prob-at-angle   0
  set prob-right home-prob-at-angle  45
  set prob-left  home-prob-at-angle -45
  if (prob-right > prob-ahead) or (prob-left > prob-ahead)
  [ ifelse prob-right > prob-left
    [ rt turn-rate ]
    [ lt turn-rate ] ]
  if not can-move? 1 [ rt turn-rate ]
  fd speed
end

;; make the agent circle - FUTURE
to loiter
  fd 0.5
  rt 5
end

;; allows tracking once object is found - FUTURE
to rescue-target
  stop ;;future
end

;; create some randomness for mouse prominent 
to wiggle [ randomness ]
  rt random randomness
  lt random randomness
  if not can-move? 1 [ rt 10 ]
  ;;]
end


to-report home-prob-at-angle [angle]
  let p patch-right-and-ahead angle 1
  if p = nobody [ report 0 ]
  report [home-prob] of p
end

to-report target-probability-prob-at-angle [angle]
  let p patch-right-and-ahead angle 1
  if p = nobody [ report 0 ]
  report [target-probability] of p
end

to goto-centreX
  if distance centre 2 < 2 [ set action "search"  set ticker ticks]
  set current-heading heading
  face centre 2
  set desired-heading heading
  set heading current-heading
  ifelse current-heading = desired-heading [] [
    ifelse desired-heading > 180
      [ lt turn-rate * ( 1 - (abs ( current-heading - desired-heading ) ) / 180 )]
      [ rt turn-rate * ( 1 - (abs ( current-heading - desired-heading ) ) / 180 )]
  ]
  fd speed
end

to goto-point [ pointx pointy ]
 ask one-of centres [ setxy pointx pointy ]
 ifelse distance one-of centres < 1 [ set action "search" set desired-heading 0 set ticker ticks] 
   [
     let tocentre towards one-of centres
     let headign_error subtract-headings heading tocentre
     ifelse headign_error = 0 [] [  ifelse headign_error > 0 
       
      [ ifelse ( headign_error < turn-rate) [ lt abs headign_error] [ lt turn-rate ] ] 
      [ ifelse ( headign_error < turn-rate) [ rt abs headign_error] [ rt turn-rate ] ] 
    ]
   ]
 fd speed
end

to normalise
  if show-probability = 1   [   ;;  Normalise target likelyhood array
    let total sum [squad-probability] of patches
    ask patches [ set squad-probability squad-probability * 1 / total ]
  ]
  
end  


;; Update Sensor information
to do-update
  let sigma_x sensor-range ; * 0.7
  let sigma_y sensor-range ; * 0.7 
  let x0 xcor
  let y0 ycor
  let a1 ( cos ( heading ) ^ 2 / 2 / sigma_x ^ 2 + sin ( heading ) ^ 2 / 2 / sigma_y ^ 2 )
  let b1 (- sin ( 2 * heading ) / 4 / sigma_x ^ 2  + sin ( 2 * heading ) / 4 / sigma_y ^ 2 )
  let c1 (sin ( heading ) ^ 2 / 2 / sigma_x ^ 2 + cos (heading ) ^ 2 / 2 / sigma_y ^ 2 )
  ask patches in-radius ( 4 * ( sensor-range / sensorscale  ) ) 
    [
      let Z ( Weight * exp ( - ( a1 * ( ( pxcor * sensorscale ) - ( x0 * sensorscale ) ) ^ 2 + 2 * b1 * ( ( pxcor * sensorscale ) - ( x0 * sensorscale ) ) * (  ( pycor * sensorscale ) - ( y0 * sensorscale ) ) + c1 * (  ( pycor * sensorscale ) - ( y0 * sensorscale ) ) ^ 2 ) ) );
      set target-probability target-probability * ( 1 - Z )   
    ]
end



;;;;;;;;;;;;;;;;;;;;;
;;;     MAIN      ;;;
;;;;;;;;;;;;;;;;;;;;;

to go  ;; go button  

     ask army_squad 
     [ 
       if alive? [ walk ] ;; Look for targets in range
       
       ifelse action = "centre" [ goto-point ( target_x ) ( target_y ) ] ;; Send the agents to last known position first
       [
         ifelse action = "search"
         [ wiggle 40               ;; Add some randomness
           search-like-ant         ;; Use stochasic technique as pheromone replacement
           fd speed                ;; Move forward t
           do-update               ;; update probability map with sensor data
         ]
         [ return-to-home ]  ;  All Found
       ]
     ]
     
     every updatespeed [   if ( PDF? and not background? ) [ ask patches  [ recolor-patch ] ] ]  ;; View updated target liklyhood

     tick
     if ticks mod 5 = 0 [ plot time2rescue ]              ;; Periodically update plot
end
@#$#@#$#@
GRAPHICS-WINDOW
214
22
1292
695
-1
-1
3.55
1
10
1
1
1
0
0
0
1
0
300
0
180
1
1
1
ticks

BUTTON
6
36
74
69
NIL
setup
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL

SLIDER
1421
648
1610
681
Prediction
Prediction
0
1
0
.001
1
NIL
HORIZONTAL

BUTTON
85
37
197
70
Run Simulation
go
T
1
T
OBSERVER
NIL
NIL
NIL
NIL

BUTTON
8
396
196
429
Watch one of the search UAVs
let mylist list (one-of greedy-uavs) (one-of random-ants)\nwatch ( one-of mylist )
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL

BUTTON
8
435
197
468
Follow one of the search UAVs
follow one-of greedy-uavs
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL

TEXTBOX
12
17
201
35
Cluster Search and Rescue Simulation
11
0.0
0

MONITOR
12
513
96
558
NIL
subject
3
1
11

BUTTON
106
521
196
554
Clear Drawing
clear-drawing\n    ;ifelse background [import-pcolors-rgb \"darwin_large.bmp\"]\n    ;[ if PDF? [ ask patches [ recolor-patch ] ] ]\n     if PDF? [ ask patches [ recolor-patch ] ] 
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL

BUTTON
8
472
97
505
subject trails on
if subject != nobody\n[ ask subject [ pd ] ]
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL

BUTTON
102
472
198
507
NIL
reset-perspective
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL

PLOT
1419
109
1609
280
Time to rescue
Time
Target
0.0
1.0
0.0
1.0
true
false
PENS
"time2rescue" 1.0 0 -6459832 true

CHOOSER
5
79
97
124
square-team
square-team
0 1 2
0

CHOOSER
104
127
196
172
random-team
random-team
0 1 2 3 4 5 6 7 8 9 10 11 12 13 141 15 16 17 18 19 20 30 40 50 60 70 80 90 100
0

CHOOSER
6
262
98
307
lost-target
lost-target
0 1 2 3 4 10 20 40 60 80 100 500 1000
10

CHOOSER
104
262
196
307
show-probability
show-probability
1 2 3 4 5
1

SWITCH
1423
570
1549
603
background?
background?
0
1
-1000

BUTTON
106
179
196
212
Add Random
launch-random-ants 1 Goto_Center?\nask random-ants [ ifelse trails? [pd][pu] ]
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL

BUTTON
6
179
99
212
Add Greedy
launch-greedy-uavs 1 Goto_Center? white\nask greedy-uavs [ ifelse trails? [pd][pu] ]
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL

CHOOSER
104
78
196
123
parallel-team
parallel-team
0 1 2
0

MONITOR
1520
55
1601
100
NIL
current_pen
17
1
11

SLIDER
1463
463
1608
496
V_X
V_X
3.5
20
13.5
0.5
1
NIL
HORIZONTAL

SLIDER
1463
499
1608
532
V_Y
V_Y
3.5
20
13.5
0.5
1
NIL
HORIZONTAL

SWITCH
1420
609
1611
642
do_normalise
do_normalise
0
1
-1000

CHOOSER
1419
55
1511
100
max_pen
max_pen
1 2 3 4 5 6 7 8 9 10 50 100
11

BUTTON
1423
14
1607
47
Export
export-plot \"Time to rescue\" user-new-file
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL

MONITOR
5
214
97
259
Search Agents
count greedy-uavs
17
1
11

SLIDER
1421
320
1513
353
Sensor_vx
Sensor_vx
0
20
10
1
1
NIL
HORIZONTAL

SWITCH
9
565
99
598
trails?
trails?
0
1
-1000

BUTTON
109
565
198
598
update pen
ask turtles [ifelse trails? [ pd ][ pu ] ]
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL

SLIDER
1421
355
1608
388
Weight
Weight
0
3
3
.001
1
NIL
HORIZONTAL

SLIDER
1422
390
1608
423
sensor-range
sensor-range
1
10
2.5
.1
1
NIL
HORIZONTAL

SLIDER
1516
319
1608
352
Sensor_vy
Sensor_vy
0
20
10
1
1
NIL
HORIZONTAL

SWITCH
7
312
117
345
searchable?
searchable?
0
1
-1000

SLIDER
1463
426
1607
459
target_x
target_x
0
max-pxcor
70
10
1
NIL
HORIZONTAL

SLIDER
1423
426
1456
540
target_y
target_y
0
max-pycor
65
1
1
NIL
VERTICAL

MONITOR
104
215
199
260
ants
count random-ants
17
1
11

SWITCH
3
353
118
386
Goto_Center?
Goto_Center?
1
1
-1000

MONITOR
1422
276
1510
321
Num_Sensors
ceiling ( ( ( ( ( V_X * worldscale ) + ( V_Y * worldscale ) ) / 2 ) / ( ( 100 - overlap ) / 100 ) ) / ( sensor-range * worldscale ) )
4
1
11

SLIDER
1422
538
1514
571
p-min
p-min
-10
100
-10
1
1
NIL
HORIZONTAL

SLIDER
1516
537
1608
570
p-max
p-max
0
110
110
1
1
NIL
HORIZONTAL

SLIDER
1514
284
1606
317
overlap
overlap
0
70
30
1
1
NIL
HORIZONTAL

INPUTBOX
117
335
210
395
greedy_team
2
1
0
Number

SLIDER
1370
31
1403
680
worldscale
worldscale
0.5
3
0.99
.01
1
NIL
VERTICAL

SLIDER
1309
728
1429
761
updatespeed
updatespeed
0
5
2.476
.001
1
NIL
HORIZONTAL

SLIDER
1327
29
1360
680
sensorscale
sensorscale
.2
3
1.186
0.001
1
NIL
VERTICAL

SLIDER
1430
689
1602
722
turn-rate
turn-rate
0
50
22
1
1
NIL
HORIZONTAL

SLIDER
1434
727
1606
760
steerrange
steerrange
2
4
3.248
.001
1
NIL
HORIZONTAL

SWITCH
118
311
211
344
show_targets?
show_targets?
1
1
-1000

BUTTON
9
608
94
641
NO FLY ZONE
patch-draw\n
T
1
T
OBSERVER
NIL
NIL
NIL
NIL

BUTTON
106
607
201
640
Clear no fly zone
ask patches\n  [\n    if pcolor = white\n    [\n      set pcolor 0\n    ]\n  ]
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL

BUTTON
4
653
99
686
Import Map
import-pcolors-rgb \"darwin_large.bmp\"
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL

SWITCH
105
652
208
685
PDF?
PDF?
0
1
-1000

CHOOSER
7
129
99
174
max_agents
max_agents
0 1 2 3 4 5 6 7 8 9 10 11 12 13 141 15 16 17 18 19 20 30 40 50 60 70 80 90 100
2

SWITCH
1506
571
1609
604
steerdot
steerdot
1
1
-1000

BUTTON
10
697
203
730
NIL
ask one-of greedy-uavs [die]
NIL
1
T
OBSERVER
NIL
NIL
NIL
NIL

@#$#@#$#@
WHAT IS IT?
-----------
This is simulation of an autonomous team of Unmanned Aerial search and rescue UAVs conduction a maratime search and rescue mission to find a number of lost targets. The simulation includs NetLogo's perspective features. Try it in both 2D and 3D.

In this project, the UAVs search for a lost target, or a number of lost targets set by the user. The output is plotted in a graph that measures teh realive time to be rescued. There is currently no fixed distance or time measurements as is used as part of the varifycation and validation process of testing the optimal control algorithm.

HOW TO USE IT
-------------
Click the SETUP. Click the GO button to start the simulation.

Press the WATCH ONE-OF UAVs and FOLLOW ONE-OF UAVs buttons to focus on the behavior of one specific UAV and the area directly surrounding it. While watching or following a  UAV press the SUBJECT TRAIL button.  This leaves a trail behind the subject so you can see where its been. Press the RESET-PERSPECTIVE button to return to the broad view of the world.


CREDITS AND REFERENCES
-----------------------

This model was developed at the University of New South Wales as part of the System in teh loop Sumulation of multiple aerial vehicle thesis as part of a masters of engineering. See website for more information (http://www.simulation.unsw.edu.au)

To refer to this model in academic publications, please use:
Sammons, P. (2011).  NetLogo Cluster Search and Rescue.  http://www.simulation.unsw.edu.au/netlogo/  Moddeling and simulation group, University of New South Wales.

In other publications, please use:  Copyright 2010 Phil Sammons.  All rights reserved.
Please contact j.page@unsw.edu.au to request permissions to reproduce any part of this document.
@#$#@#$#@
default
true
0
Polygon -7500403 true true 150 5 40 250 150 205 260 250

airplane
true
0
Polygon -7500403 true true 150 0 135 15 120 60 120 105 15 165 15 195 120 180 135 240 105 270 120 285 150 270 180 285 210 270 165 240 180 180 285 195 285 165 180 105 180 60 165 15

arrow
true
0
Polygon -7500403 true true 150 0 0 150 105 150 105 293 195 293 195 150 300 150

box
false
0
Polygon -7500403 true true 150 285 285 225 285 75 150 135
Polygon -7500403 true true 150 135 15 75 150 15 285 75
Polygon -7500403 true true 15 75 15 225 150 285 150 135
Line -16777216 false 150 285 150 135
Line -16777216 false 150 135 15 75
Line -16777216 false 150 135 285 75

bug
true
0
Circle -7500403 true true 96 182 108
Circle -7500403 true true 110 127 80
Circle -7500403 true true 110 75 80
Line -7500403 true 150 100 80 30
Line -7500403 true 150 100 220 30

butterfly
true
0
Polygon -7500403 true true 150 165 209 199 225 225 225 255 195 270 165 255 150 240
Polygon -7500403 true true 150 165 89 198 75 225 75 255 105 270 135 255 150 240
Polygon -7500403 true true 139 148 100 105 55 90 25 90 10 105 10 135 25 180 40 195 85 194 139 163
Polygon -7500403 true true 162 150 200 105 245 90 275 90 290 105 290 135 275 180 260 195 215 195 162 165
Polygon -16777216 true false 150 255 135 225 120 150 135 120 150 105 165 120 180 150 165 225
Circle -16777216 true false 135 90 30
Line -16777216 false 150 105 195 60
Line -16777216 false 150 105 105 60

car
false
0
Polygon -7500403 true true 300 180 279 164 261 144 240 135 226 132 213 106 203 84 185 63 159 50 135 50 75 60 0 150 0 165 0 225 300 225 300 180
Circle -16777216 true false 180 180 90
Circle -16777216 true false 30 180 90
Polygon -16777216 true false 162 80 132 78 134 135 209 135 194 105 189 96 180 89
Circle -7500403 true true 47 195 58
Circle -7500403 true true 195 195 58

circle
false
0
Circle -7500403 true true 0 0 300

circle 2
false
0
Circle -7500403 true true 0 0 300
Circle -16777216 true false 30 30 240

cow
false
0
Polygon -7500403 true true 200 193 197 249 179 249 177 196 166 187 140 189 93 191 78 179 72 211 49 209 48 181 37 149 25 120 25 89 45 72 103 84 179 75 198 76 252 64 272 81 293 103 285 121 255 121 242 118 224 167
Polygon -7500403 true true 73 210 86 251 62 249 48 208
Polygon -7500403 true true 25 114 16 195 9 204 23 213 25 200 39 123

cylinder
false
0
Circle -7500403 true true 0 0 300

dot
false
0
Circle -7500403 true true 90 90 120

face happy
false
0
Circle -7500403 true true 8 8 285
Circle -16777216 true false 60 75 60
Circle -16777216 true false 180 75 60
Polygon -16777216 true false 150 255 90 239 62 213 47 191 67 179 90 203 109 218 150 225 192 218 210 203 227 181 251 194 236 217 212 240

face neutral
false
0
Circle -7500403 true true 8 7 285
Circle -16777216 true false 60 75 60
Circle -16777216 true false 180 75 60
Rectangle -16777216 true false 60 195 240 225

face sad
false
0
Circle -7500403 true true 8 8 285
Circle -16777216 true false 60 75 60
Circle -16777216 true false 180 75 60
Polygon -16777216 true false 150 168 90 184 62 210 47 232 67 244 90 220 109 205 150 198 192 205 210 220 227 242 251 229 236 206 212 183

fish
false
0
Polygon -1 true false 44 131 21 87 15 86 0 120 15 150 0 180 13 214 20 212 45 166
Polygon -1 true false 135 195 119 235 95 218 76 210 46 204 60 165
Polygon -1 true false 75 45 83 77 71 103 86 114 166 78 135 60
Polygon -7500403 true true 30 136 151 77 226 81 280 119 292 146 292 160 287 170 270 195 195 210 151 212 30 166
Circle -16777216 true false 215 106 30

flag
false
0
Rectangle -7500403 true true 60 15 75 300
Polygon -7500403 true true 90 150 270 90 90 30
Line -7500403 true 75 135 90 135
Line -7500403 true 75 45 90 45

flower
false
0
Polygon -10899396 true false 135 120 165 165 180 210 180 240 150 300 165 300 195 240 195 195 165 135
Circle -7500403 true true 85 132 38
Circle -7500403 true true 130 147 38
Circle -7500403 true true 192 85 38
Circle -7500403 true true 85 40 38
Circle -7500403 true true 177 40 38
Circle -7500403 true true 177 132 38
Circle -7500403 true true 70 85 38
Circle -7500403 true true 130 25 38
Circle -7500403 true true 96 51 108
Circle -16777216 true false 113 68 74
Polygon -10899396 true false 189 233 219 188 249 173 279 188 234 218
Polygon -10899396 true false 180 255 150 210 105 210 75 240 135 240

house
false
0
Rectangle -7500403 true true 45 120 255 285
Rectangle -16777216 true false 120 210 180 285
Polygon -7500403 true true 15 120 150 15 285 120
Line -16777216 false 30 120 270 120

leaf
false
0
Polygon -7500403 true true 150 210 135 195 120 210 60 210 30 195 60 180 60 165 15 135 30 120 15 105 40 104 45 90 60 90 90 105 105 120 120 120 105 60 120 60 135 30 150 15 165 30 180 60 195 60 180 120 195 120 210 105 240 90 255 90 263 104 285 105 270 120 285 135 240 165 240 180 270 195 240 210 180 210 165 195
Polygon -7500403 true true 135 195 135 240 120 255 105 255 105 285 135 285 165 240 165 195

line
true
0
Line -7500403 true 150 0 150 300

line half
true
0
Line -7500403 true 150 0 150 150

pentagon
false
0
Polygon -7500403 true true 150 15 15 120 60 285 240 285 285 120

person
false
0
Circle -7500403 true true 110 5 80
Polygon -7500403 true true 105 90 120 195 90 285 105 300 135 300 150 225 165 300 195 300 210 285 180 195 195 90
Rectangle -7500403 true true 127 79 172 94
Polygon -7500403 true true 195 90 240 150 225 180 165 105
Polygon -7500403 true true 105 90 60 150 75 180 135 105

plant
false
0
Rectangle -7500403 true true 135 90 165 300
Polygon -7500403 true true 135 255 90 210 45 195 75 255 135 285
Polygon -7500403 true true 165 255 210 210 255 195 225 255 165 285
Polygon -7500403 true true 135 180 90 135 45 120 75 180 135 210
Polygon -7500403 true true 165 180 165 210 225 180 255 120 210 135
Polygon -7500403 true true 135 105 90 60 45 45 75 105 135 135
Polygon -7500403 true true 165 105 165 135 225 105 255 45 210 60
Polygon -7500403 true true 135 90 120 45 150 15 180 45 165 90

square
false
0
Rectangle -7500403 true true 30 30 270 270

square 2
false
0
Rectangle -7500403 true true 30 30 270 270
Rectangle -16777216 true false 60 60 240 240

star
false
0
Polygon -7500403 true true 151 1 185 108 298 108 207 175 242 282 151 216 59 282 94 175 3 108 116 108

target
false
0
Circle -7500403 true true 0 0 300
Circle -16777216 true false 30 30 240
Circle -7500403 true true 60 60 180
Circle -16777216 true false 90 90 120
Circle -7500403 true true 120 120 60

tree
false
0
Circle -7500403 true true 118 3 94
Rectangle -6459832 true false 120 195 180 300
Circle -7500403 true true 65 21 108
Circle -7500403 true true 116 41 127
Circle -7500403 true true 45 90 120
Circle -7500403 true true 104 74 152

triangle
false
0
Polygon -7500403 true true 150 30 15 255 285 255

triangle 2
false
0
Polygon -7500403 true true 150 30 15 255 285 255
Polygon -16777216 true false 151 99 225 223 75 224

truck
false
0
Rectangle -7500403 true true 4 45 195 187
Polygon -7500403 true true 296 193 296 150 259 134 244 104 208 104 207 194
Rectangle -1 true false 195 60 195 105
Polygon -16777216 true false 238 112 252 141 219 141 218 112
Circle -16777216 true false 234 174 42
Rectangle -7500403 true true 181 185 214 194
Circle -16777216 true false 144 174 42
Circle -16777216 true false 24 174 42
Circle -7500403 false true 24 174 42
Circle -7500403 false true 144 174 42
Circle -7500403 false true 234 174 42

turtle
true
0
Polygon -10899396 true false 215 204 240 233 246 254 228 266 215 252 193 210
Polygon -10899396 true false 195 90 225 75 245 75 260 89 269 108 261 124 240 105 225 105 210 105
Polygon -10899396 true false 105 90 75 75 55 75 40 89 31 108 39 124 60 105 75 105 90 105
Polygon -10899396 true false 132 85 134 64 107 51 108 17 150 2 192 18 192 52 169 65 172 87
Polygon -10899396 true false 85 204 60 233 54 254 72 266 85 252 107 210
Polygon -7500403 true true 119 75 179 75 209 101 224 135 220 225 175 261 128 261 81 224 74 135 88 99

wheel
false
0
Circle -7500403 true true 3 3 294
Circle -16777216 true false 30 30 240
Line -7500403 true 150 285 150 15
Line -7500403 true 15 150 285 150
Circle -7500403 true true 120 120 60
Line -7500403 true 216 40 79 269
Line -7500403 true 40 84 269 221
Line -7500403 true 40 216 269 79
Line -7500403 true 84 40 221 269

x
false
0
Polygon -7500403 true true 270 75 225 30 30 225 75 270
Polygon -7500403 true true 30 75 75 30 270 225 225 270

@#$#@#$#@
NetLogo 4.1.3
@#$#@#$#@
need-to-manually-make-preview-for-this-model
@#$#@#$#@
@#$#@#$#@
@#$#@#$#@
@#$#@#$#@
default
0.0
-0.2 0 0.0 1.0
0.0 1 1.0 0.0
0.2 0 0.0 1.0
link direction
true
0
Line -7500403 true 150 150 90 180
Line -7500403 true 150 150 210 180

@#$#@#$#@
0
@#$#@#$#@
