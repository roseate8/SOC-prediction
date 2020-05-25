# SOC-prediction
Abstract: State of charge (SOC) estimation is of great significance for the safe operation of lithium-ion battery (LIB) packs. Improving the accuracy of SOC estimation results and reducing the algorithm complexity are important for the state estimation. For the purpose of this report, we discuss and explore around the use of Kalman Filter for prediction of SoC of batteries – implementation, results, advantages and disadvantages


1. Introduction

  1.1. Rechargeable Batteries
  A rechargeable battery is also known as a storage battery, secondary cell or accumulator. It is a type of electrical
  battery which can be charged, discharged into a load, and recharged many times. Primary batteries are supplied
  fully charged and discarded after use. A rechargeable battery is composed of one or more electrochemical cells.
  The term ”accumulator” signifies the fact that it accumulates and stores energy. This is done through a
  reversible electrochemical reaction. Rechargeable batteries are produced in many different shapes and sizes,
  ranging from button cells to megawatt systems connected to stabilize an electrical distribution network. [1]

  1.2. State of Charge (SoC)
  The SOC is defined as the available capacity expressed as a percentage of some reference, sometimes its rated
  capacity but more likely its current (i.e. at the latest charge-discharge cycle) capacity but this ambiguity can
  lead to confusion and errors. It is not usually an absolute measure in Coulombs, kWh or Ah of the energy left
  in the battery which would be less confusing. An alternative form of the same measure is the depth of discharge
  (DoD), the inverse of SoC (100% = empty; 0% = full). SoC is normally used when discussing the current state
  of a battery in use, while DoD is most often seen when discussing the lifetime of the battery after repeated use.
  The preferred SOC reference should be the rated capacity of a new cell rather than the current capacity of
  the cell. This is because the cell capacity gradually reduces as the cell ages.
  The optimization of state of charge for Lithium batteries presents the main effect on their internal states
  in several applications [2]. It leads to maintain the activities of applications in permanent way. The state of
  charge (SoC) is meaningful parameter that is defined in case of discharge of the battery [3]. It presents the shift
  time of battery capacity through the following expression [4]:
  with Ib: the current of the battery, Q: the nominal capacity of the battery and : Time of energy storage. The
  SoC optimization is a difficult issue for different domains due to its dependency on some factors such as battery
  capacitance, temperature and internal resistance and also the problem of defining it easily [4]. Therefore, many
  researches have focused on the possibility to estimate the SoC of the Lithium cells through different techniques

  1.3. Li-ion batteries
  Lithium-ion batteries, also known as Li-ion battery (LIB) is a type of rechargeable battery. The electrochemistry
  involves lithium ions move from the negative electrode to the positive electrode during discharge and back when
  charging. Metallic lithium is used in a non-rechargeable lithium battery. Li-ion batteries use an intercalated
  lithium compound as one electrode material unlike the former. Other constituent components of a lithium-ion
  battery cell are the electrolyte and the electrodes. Li-ion is a popular choice among rechargeable batteries for
  portable electronics. The desirable properties found are :
  • High energy density
  • Negligible memory effect
  • Low self-discharge
  LIBs are also growing in popularity for military, battery electric vehicle and aerospace applications.
    1.3.1. Chemistry
    The reactants in the electrochemical reactions in a lithium-ion cell are materials of anode and cathode, both of
    which are compounds containing lithium atoms. During discharge an oxidation reaction at the anode produces
    positively charged lithium ions and negatively charged electrons, as well as uncharged material that remains
    at the anode; after transport of the lithium ions through the electrolyte and of electrons through an external
    circuit, they recombine at the cathode together with the cathode material in a reduction reaction.
    Both electrodes allow lithium ions to move in and out of their structures with a process called insertion
    (intercalation) or extraction (deintercalation), respectively. As the lithium ions ’rock’ back and forth between
    the two electrodes, these batteries are also known as ’rocking-chair batteries’ or ’swing batteries’.[5] During
    discharge, the (positive) lithium ions move from the negative electrode (anode) (usually graphite as below) to
    the positive electrode (cathode) (forming a lithium compound) through the electrolyte while the electrons flow
    through the external circuit in the same direction.[6] When the cell is charging, the reverse occurs with the
    lithium ions and electrons move back into the negative electrode in a net higher energy state.
    The following equations exemplify the chemistry.
    The positive electrode (cathode) half-reaction in the lithium-doped cobalt oxide substrate is:
    CoO2 + Li+ + e
    − <=> LiCoO2
    The negative electrode (anode) half-reaction for the graphite is:
    LiC6 <=> C6 + Li+ + e
    −
    The full reaction (left to right: discharging, right to left: charging) being:
    LiC6 + CoO2 <=> C6 + LiCoO2
    The overall reaction has its limits. Over-discharging supersaturates lithium cobalt oxide, leading to the
    production of lithium oxide, possibly by the following irreversible reaction:
    Li+ + e
    − + LiCoO2− > Li2O + CoO
    Overcharging up to 5.2 volts leads to the synthesis of cobalt(IV) oxide, as evidenced by x-ray diffraction:
    LiCoO2− > Li+ + CoO2 + e
    −
    Each gram of lithium represents Faraday’s constant/6.941 or 13,901 coulombs. At 3 V, this gives 41.7 kJ
    per gram of lithium, or 11.6 kWh per kg. [7]
    1.3.2. Performance
    • Volumetric energy density : 250 to 620 Wh/L (900 to 2230 J/cm)
    • Specific power density : 300 to 1500 W/kg (at 20 seconds and 285 Wh/L
    • Specific energy: 100 - 265 Wh/kg (0.36 - 0.875 MJ/kg)
    • Nominal cell voltage: 3.6 / 3.7 / 3.8 / 3.85 V, LiFePO 4 3.2 V
    • Energy/consumer-price: 3.6 Wh/US$
    2
    • Specific power: 250 - 340 W/kg
    • Charge/discharge efficiency: 80 - 90%
    • Cycle durability: 400 - 1,200 cycles
    Because lithium-ion batteries can have a variety of positive and negative electrode materials, the energy density
    and voltage vary accordingly. Performance of manufactured batteries has improved over time. For example,
    from 1991 to 2005 the energy capacity per price of lithium ion batteries improved more than ten-fold, from 0.3
    Wh per dollar to over 3 Wh per dollar.[132] In the period from 2011-2017, progress has averaged 7.5% annually.
    1.3.3. Applications
    Automobile starters, portable consumer devices, some light vehicles, power-tools, uninterruptible power supplies,
    and battery storage power stations are some areas which actively employ rechargeable batteries. Applications
    which are upcoming include hybrid and electric drive vehicles. Based on the size, the rechargeable battery can
    find its application. Light-duty products can power portable electronic devices, power tools and appliances.
    Heavy-duty batteries power electric vehicles, ranging from scooters to locomotives and ships. An interesting
    upcoming application is their use in distributed electricity generation and in stand-alone power systems.
    The Li-ion technology has been developed on the basis of the existing primary Li batteries. To circumvent
    the safety issues caused by Li-dendrite formation in metallic Li batteries, several alternative approaches were
    pursued in which either the electrolyte or the negative electrode was modified. Capitalized on earlier findings,
    carbonaceous materials were proposed as anode material which finally led to the creation of the C/LiCoO2
    rocking-chair cell commercialized by Sony Corporation in June 1991. These batteries with a configuration of
    carbonaceous material as negative electrode and lithiated metal oxides as positive electrode were called Li- ion
    batteries.
    Li-ion batteries have an unmatchable combination of high energy and power density, making it the technology
    of choice for portable electronics, power tools, and hybrid/full electric vehicles [8].

2. Methodology

  2.1. Types of methods for Determining SoC
  Usually, SoC cannot be measured directly but it can be estimated from direct measurement variables in two
  ways: offline and online. In offline techniques, the battery desires to be charged and discharged in constant rate
  such as Coulomb-counting. This method gives precise estimation of battery SoC, but they are protracted, costly,
  and interrupt main battery performance. Therefore, researchers are looking for some online techniques.[1] In
  general there are five methods to determine SoC indirectly:[2] [3] [9]
  • chemical
  • voltage
  • current integration
  • kalman filtering
  • pressure
  Chemical method
  This method works only with batteries that offer access to their liquid electrolyte, such as non-sealed lead acid
  batteries. The specific gravity or pH of the electrolyte can be used to indicate the SoC of the battery.
  Hydrometers are used to calculate the specific gravity of a battery. To find specific gravity, it is necessary to
  measure out volume of the electrolyte and to weigh it. Then specific gravity is given by (mass of electrolyte [g]/
  volume of electrolyte [ml])/ (Density of Water, i.e. 1g/1ml). To find SoC from specific gravity, a look-up table
  of SG vs SoC is needed.

3
Voltage method
This method converts a reading of the battery voltage to SoC, using the known discharge curve (voltage vs.
SoC) of the battery. However, the voltage is more significantly affected by the battery current (due to the
battery’s electrochemical kinetics) and temperature. This method can be made more accurate by compensating
the voltage reading by a correction term proportional to the battery current, and by using a look-up table of
battery’s open circuit voltage vs. temperature. In fact, it is a stated goal of battery design to provide a voltage
as constant as possible no matter the SoC, which makes this method difficult to apply.
Current integration method
This method, also known as ”coulomb counting”, calculates the SoC by measuring the battery current and
integrating it in time. Since no measurement can be perfect, this method suffers from long-term drift and lack
of a reference point: therefore, the SoC must be re-calibrated on a regular basis, such as by resetting the SoC
to 100% when a charger determines that the battery is fully charged (using one of the other methods described
here).
Combined approaches
Maxim Integrated touts a combined voltage and charge approach that is claimed superior to either method
alone; it is implemented in their ModelGauge m3 series of chips, such as MAX17050,[4][5] which is used in the
Nexus 6 and Nexus 9 Android devices, for example.[6]
Kalman filtering*
To overcome the shortcomings of the voltage method and the current integration method, a Kalman filter can
be used. The battery can be modeled with an electrical model which the Kalman filter will use to predict the
over-voltage, due to the current. In combination with coulomb counting, it can make an accurate estimation of
the state of charge. The strength of a Kalman filter is that it is able to adjust its trust of the battery voltage
and coulomb counting in real time. [10] [8]
Pressure method
This method can be used with certain NiMH batteries, whose internal pressure increases rapidly when the
battery is charged. More commonly, a pressure switch indicates if the battery is fully charged. This method
may be improved by taking into account Peukert’s law which is a function of charge/discharge rate or ampere.
3. Kalman Filter in state-of-charge prediction
In this report we discuss the results of prediction of SoC for batteries using Kalman Filters.
Kalman filtering, also known as linear quadratic estimation (LQE), is an algorithm that uses a series of measurements observed over time, containing statistical noise and other inaccuracies, and produces estimates of unknown
variables that tend to be more accurate than those based on a single measurement alone, by estimating a joint
probability distribution over the variables for each timeframe. [11]
Kalman filter algorithm is a closed-loop formbased upon a feedback mechanism. It adjusts the SOC value
dynamically according to the voltage error between the measured voltage value and the estimated voltage value
from the battery model. Then the adjusted SOC value and the current feed back to the battery model, to
generate a new estimated battery voltage. After much iteration, the output voltage of the model will achieve
a dynamical equilibrium, approximately equal to the measured terminal voltage. An additional benefit of the
Kalman filter is that it automatically provides dynamic error bounds on the estimation as well. This method
provides the safety information specifics of the battery pack, reducing the chance of overcharge or overdischarge.
The drawbacks of this method lay in the computational complexity and more stringent requirements for the
accuracy of the battery model. [12]
The algorithm works in a two-step process. In the prediction step, the Kalman filter produces estimates of the
current state variables, along with their uncertainties. Once the outcome of the next measurement (necessarily
corrupted with some amount of error, including random noise) is observed, these estimates are updated using a
weighted average, with more weight being given to estimates with higher certainty. The algorithm is recursive.
It can run in real time, using only the present input measurements and the previously calculated state and its
uncertainty matrix; no additional past information is required. [13]

4
Python implementation of Kalman Filter :
def KalmanFilter() :
for n in range(measurements):
x = A * x + B * u[n]
P = A * P * A.T + Q
# Measurement Update (Correction)
# Compute the Kalman Gain
S = H * P * H.T + R
K = (P * H.T) * np.linalg.pinv(S)
# Update the estimate via z
Z = mx[n]
y = Z - (H * x) # Innovation or Residual
x = x + (K * y)
# Update the error covariance
P = (I - (K * H))*P
Kalman filter well optimize the SoC of the battery. It provides good state of charge estimation error thanks
to its accuracy for parameter estimation. Hence, the stability between the model and the observer has been
perfectly ensured by Kalman filter. Besides, the dispersion of error of SoC estimation for fractional kalman filter
is not totally concentrated at zero error. It is more dispersed and tends to -0.005 due to the problem of choice
of initial parameters. This explains that the optimal values of SoC are not exactly equal to real SoC values.
Extended Kalman Filter
In the extended Kalman filter (EKF), the state transition and observation models need not be linear functions
of the state but may instead be nonlinear functions.These functions are of differentiable type.
xk = f(xk−1, uk) + wk
z k = h(xk) + vk
If the system is nonlinear, a linearization process at each time step will be necessary to approximate the
non-linear system. The EKF will play a great role in these systems. Based on the error between the model
estimated voltage and the measured voltage, the EKF adjusts the SOC to change the model output voltage to
minimize the voltage error. After a certain number of iterations, this error will converge to zero, while the SOC
will converge to its optimal value.
Unscented Kalman Filter
When the state transition and observation models – that is, the predict and update functions are highly
nonlinear, the extended Kalman filter can give particularly poor performance.[14] This is because the covariance
is propagated through linearization of the underlying nonlinear model. The unscented Kalman filter (UKF)
uses a deterministic sampling technique known as the unscented transformation (UT) to pick a minimal set
of sample points (called sigma points) around the mean. The sigma points are then propagated through the
nonlinear functions, from which a new mean and covariance estimate are then formed. The resulting filter
depends on how the transformed statistics of the UT are calculated and which set of sigma points are used. It
should be remarked that it is always possible to construct new UKFs in a consistent way [15]
3.1. Physical model approach
(using Simulink on MATLAB)[16]
This model shows how to estimate the state of charge (SOC) of lithium battery, use multiple experiments,
mix of model parameters identification and simulation of extended kalman filter(EKF). (The results shown are
based on a simulation model made by Alter Wang [17] publically available on github)
Li-Battery model building, parameters identification and verification, SOC estimation using extended kalman
filter(EKF) through two ways:
5
1. Simulinks (EKF only)
2. Scripts (EKF and UKF)
Thevenin equivalent circuit model and extended kalman filter are included in the simulation file ”EKFSimR2016.slx”,
of which the structure is shown in the snapshot below.
Figure 1: Structure of EKFSim R2016.slx
Results
Figure 2: Output of EKFSim R2016.slx : The estimated curve has distinct divergences in the current pulse
areas and it converges to the true value in the constant current discharge areas. The estimated SOC and update
Up(voltage of RC element in Thevenin ECM) change synchronously due to the same state vector that they are
in, that can be seen in the function block ’EKF’.
6
Figure 3: UL (voltage on the load) variation
3.2. Data driven approach
(using kalman filter and implemented on python) [18]
(Please refer to appendix for the Source code)
kalman.py :
Class that defines the algorithm for the Extended Kalman Fliter (EKF)
battery.py :
Class that implements the SOC Estimator for a battery cell. To use this class, create an instance using Battery.
All other parameters are in SI units. To update the state of this instance of a battery cell, call. This function
will return the estimated State of Charge.
protocol.py :
Script that shows an instantiation of cell object corresponding to a battery pack. Tests the program by reading
in simulated data from a file and generating an output timeseries of SOC for a single cell.
main.py :
File that declares global look-up tables corresponding to battery parameters.
utils.py :
Script that defines the polynomial function
Results
Rise Time for SoC estimation: The desired state of charge by Kalman filter observer converges slowly to
real state. Thus, it isnt considered for real time applications.
Kalman filter owns limits for convergence of the variables. It takes a long period to reach to real values of
the following state of charge (SoC) due to its slowly operations. According to figure 11, the estimated state
has been firstly fluctuated in narrow range and then it has been converged to real SoC. Hence, Kalman Filter
observer is very slow method and it owns long rise time for SoC estimation.
So, Kalman filter doesnt operate perfectly in term of estimation rise time and it yields to get bad quality of
state optimization for electrochemical systems. It has high time consumption due to the long time of calculation
of the covariances
7
(a) Voltage (V) v time (t) (b) SoC v time (t) (c) current (I) v time (t)
Figure 4: Graphs printed out from the simulation
4. Conclusion
Kalman filter observer can estimate the state of charge of Lithium battery despite of its limits. In fact, it has
high time consumption which depends on two reasons:
• The first reason: It is related to calculation of feedback coefficients. For Kalman filter, its gain is determined through the equations related to prediction and correction steps.
• The second reason: There is long time of calculation of complex covariances which makes a big time
consumption for Kalman filter.
• The Kalman filter is suitable for linear systems, unfortunately LIBs show obvious nonlinear dynamic characteristics. The Extended Kalman filter (EKF) and unscented Kalman filter (UKF), which are algorithms
developed based on the Kalman filter, better solve the state estimation problems for the nonlinear LIB
[12], and can also obtain ideal estimation results. [19]
Besides, Kalman filter has high estimation rise time due to its slow responses for SoC prediction of Lithium
batteries. In order to get better the state estimation for batteries, the extension form of these proposed
techniques can be considered.
5. Outlook and future
Future goals of this work is overcoming the limits of Kalman filter by considering its extension form or a robust
and fast observer tool for SoC estimation which is Proportional Integral Observer (PIO). It well defines through
its operations the actual states of electrochemical systems.
In order to further improve the accuracy of the LIB state estimation, some related technologies, such as
observer theory, adaptive algorithms [16,17], bio-inspired algorithms [1,18] have been combined with the Kalman
filter algorithm and obtained the expected effects.
References
[1] Wikipedia contributors, Rechargeable battery — Wikipedia, the free encyclopedia, [Online; accessed 29-November-2019]
(2019).
URL https://en.wikipedia.org/w/index.php?title=Rechargeablebatteryoldid = 927851868
[2] Y. M.Ichise, T.Kojima, An analog simulation of non-integer order transfer functions for analysis of electrode processes, in:
Journal of Electroanalytical Chemistry and Interfacial Electrochemistry, Vol. 33, 1971, pp. 253 – 265.
[3] D. N. A. K. P. Christopher Manning, Eli White, Development of a Plug-In Hybrid Electric Vehicle Control Strategy Employing Software-In-the-Loop Techniques, in: SAE 2013 World Congress Exhibition, 2013.
[4] P. L. J. A. Garcia-Valle, Rodrigo, Electric Vehicle Integration into Modern Power Networks, in: Springe-Verlag New York
2013, 2013.
[5] B. Megahed, Sid; Scrosati, Lithium-ion rechargeable batteries, Journal of Power Sources 51 ((12)) (1994) 79–104.
doi:10.1016/0378-7753(94)01956-8.
8
[6] HowStuffWorks, How lithium-ion batteries work, https://electronics.howstuffworks.com/everyday-tech/lithium-ion-battery1.htm
(2006).
[7] G. M. J. P. E. K. V. T. Younesi, Reza; Veith, Lithium salts for advanced lithium batteries: Limetal, lio 2, and lis, Energy
Environ. Sci. 8. doi:10.1039/c5ee01215e.
[8] N. Nitta, F. Wu, J. T. Lee, G. Yushin, Li-ion battery materials: present and future, Materials Today 18 (5) (2015) 252 –
264. doi:https://doi.org/10.1016/j.mattod.2014.10.040.
URL http://www.sciencedirect.com/science/article/pii/S1369702114004118
[9] Wikipedia, State of charge, https://en.wikipedia.org/wiki/Stateofcharge(2019).
[10] J. Zhang, J. Lee, A review on prognostics and health monitoring of li-ion battery, Journal of Power Sources 196 (15) (2011)
6007 – 6014. doi:https://doi.org/10.1016/j.jpowsour.2011.03.101.
URL http://www.sciencedirect.com/science/article/pii/S0378775311007865
[11] Wikipedia contributors, Kalman filter — Wikipedia, the free encyclopedia, [Online; accessed 27-November-2019] (2019).
URL https://en.wikipedia.org/w/index.php?title=Kalmanf ilteroldid = 927164805
[12] B. Mo, J. Yu, D. Tang, H. Liu, J. Yu, A remaining useful life prediction approach for lithium-ion batteries using kalman filter
and an improved particle filter, in: 2016 IEEE International Conference on Prognostics and Health Management (ICPHM),
2016, pp. 1–5. doi:10.1109/ICPHM.2016.7542847.
[13] R. E. Kalman, A New Approach to Linear Filtering and Prediction Problems, Journal of Basic Engineering
82 (1) (1960) 35–45. arXiv:https://asmedigitalcollection.asme.org/fluidsengineering/article-pdf/82/1/35/5518977/35 1.pdf,
doi:10.1115/1.3662552.
URL https://doi.org/10.1115/1.3662552
[14] H. M. T. Menegaz, J. Y. Ishihara, G. A. Borges, A. N. Vargas, A systematization of the unscented kalman filter theory,
IEEE Transactions on Automatic Control 60 (10) (2015) 2583–2598. doi:10.1109/TAC.2015.2404511.
[15] S. J. Julier, J. K. Uhlmann, New extension of the Kalman filter to nonlinear systems, in: I. Kadar (Ed.), Signal Processing,
Sensor Fusion, and Target Recognition VI, Vol. 3068, International Society for Optics and Photonics, SPIE, 1997, pp. 182
– 193. doi:10.1117/12.280797.
URL https://doi.org/10.1117/12.280797
[16] MATLAB, Matlab documentation, https://in.mathworks.com/help/sldo/examples/estimate-model-parameters-per-experiment-code.html,
accessed on 2019-11-11 (2018).
[17] A. Wang, Battery soc estimation, https://github.com/AlterWL/Battery SOC Estimation, accessed on 2019-11-11 (2019).
[18] Rudram, Soc prediction using kalman filter, https://github.com/roseate8/SOC-prediction (2019).
[19] Z. yu, R. Huai, L. Xiao, State-of-charge estimation for lithium-ion batteries using a kalman filter based on local linearization,
Energies 8 (2015) 7854–7873. doi:10.3390/en8087854.
Appendix A.
Source code :
Using Kalman filter built on python - https://github.com/roseate8/SOC-prediction
Using Simulink with MATLAB - https://github.com/AlterWL/Battery SOC Estimation
9
