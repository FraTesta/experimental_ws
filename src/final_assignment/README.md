# Final Assignment
## RUN

Launch the complete project
```
roslaunch final_assignment robotModel.launch 
```
to launch also Rviz
```
roslaunch final_assignment robotModel.launch use_rviz:=true
```

## Track
Aggiunto controllo per quando viene attivato l'action ma non vede più la palla, allora il robot si gira a destra e poi a sinistra per 9 iteraioni dopo di che 
abortisce il goal 

## Move base sattings

yaw_goal_tolerance: 6.26 to totolay make sure that the robot dose not consider the orientation of the goal. 

xy_goal_tolerance: 0.5 instead  0.3 to avoid goal on top a wall 

sim_time: 2.2   # 2.0

vx_samples: 9   # 10

vtheta_samples: 35  # 40 

to make less rapid curves and at same time reduce the computation load

## Gmapping settings 

Problema è che con rapidi cambiamenti di direzione la mappa veniva ruotata di molto rispetto al odom frame o corretta in modo errato perchè fraintendeva i riferimenti.
E perchè aggiornava troppo spesso la mappa e con uno score threshold troppo basso. Dato che in spazi aperti lo score diminuiva molto perchè non si avevano riscontri buoni -> reload della mappa in modo errato

"maxUrange" value="35.0"/> # 16. in order to increase the quality of the map but we can get good result reducing the range as well. (Per il problema che negli spazi aperti non riusciva a metchare bene -> low score -> resampling errato )

<param name="iterations" value="8"/> # The number of iterations of the scanmatcher 5 (incrementarlo per avere match più accurati)

<param name="ogain" value="5.0"/> # Gain to be used while evaluating the likelihood, for smoothing the resampling effects 3

      <param name="temporalUpdate" value="1.0"/> <!-- before was 3.0 Process a scan if the last scan processed is older than the update time in seconds. A value less than zero will turn time based updates off. --> Perchè a volte si aggiungono errori quindi e bene usare frequentemente i dati degli scan per aggiornare la mappa (potrebbe essere aumentato).

Si può inoltre ridurre il numero di particles per velocizzare il tutto. 

## Problems
When it reaches two balls it start to track only the second one. We should implement a sort of buffer in the callback of the roomDetection but we also save the position of the first ball when it was detected, in order to go back to such position when the robot will reach the second one or vice versa 



 

