# Control System Design Using Fuzzy Gain Scheduling of PD with Kalman Filter for Railway Automatic Train Operation

The development of train control systems has progressed towards following the rapid growth of railway 
transport demands. To further increase the capacity of railway systems, Automatic Train Operation (ATO) 
systems have been widely adopted in metros and gradually applied to mainline railways to replace drivers in 
controlling the movement of trains with optimised running trajectories for punctuality and energy saving. Many 
controller design methods have been studied and applied in ATO systems. However, most researchers paid less 
attention to measurement noise in the development of ATO control system, whereas such noise indeed exists in 
every single instrumentation device and disturbs the process output of ATO. Thus, this thesis attempts to 
address such issues.

In order to overcome measurement error, the author develops Fuzzy gain scheduling of PD (proportional and 
derivative) control assisted by a Kalman filter that is able to maintain the train speed within the specified 
trajectory and stability criteria in normal and noisy conditions due to measurement noise. Docklands Light 
Railway (DLR) in London is selected as a case study to implement the proposed idea. The MRes project work 
is summarised as follows: (1) analysing literature review, (2) modelling the train dynamics mathematically, (3) 
designing PD controller and Fuzzy gain scheduling, (4) adding a Gaussian white noise as measurement error, 
(5) implementing a Kalman filter to improve the controllers, (6) examining the entire system in an artificial 
trajectory and a real case study, i.e. the DLR, and (7) evaluating all based on strict objectives, i.e. a ±3% 
allowable error limit, a punctuality limit of no later and no earlier than 30 seconds, Integrated Absolute Error 
(IAE) and Integrated Squared Error (ISE) performances.

The results show that Fuzzy gain scheduling of PD control can cope well with the examinations in normal
situations. However, such discovery is not found in noisy conditions. Nevertheless, after the introduction to 
Kalman filter, all control objectives are then satisfied in not only normal but also noisy conditions. The case 
study implemented using DLR data including on the route from Stratford International to Woolwich Arsenal 
indicates a satisfactory performance of the designed controller for ATO systems.

The thesis document can be seen [here](https://etheses.bham.ac.uk/id/eprint/8478/5/Utomo18MRes.pdf)
