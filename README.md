# Yaw Damper Design for Level I Dutch-Roll Flying Qualities

**Course project:** Flight Dynamics / Control Engineering
**Tools:** MATLAB Control System Toolbox and Simulink  
**Design candidate:** yaw-damper gain ($K_1=0.25$), washout time constant ($\tau=2 \mathrm{s}$)

## 1. Objective and Scope

The objective is to evaluate the supplied aircraft yaw-rate dynamics, assess the baseline Dutch-roll damping, design a yaw-damper feedback configuration, and verify its behaviour using MATLAB and Simulink. The analysis includes open-loop modal characteristics, root-locus and gain selection, closed-loop poles, step and pulse responses with the damper disabled and enabled, washout time-constant sensitivity, and an explicitly assumption-based MIL-F-8785C modal-criteria assessment.

The model is linear and transfer-function based. The results establish performance for the specified model and tested inputs; they do not constitute certification of a real aircraft.

## 2. System Models and Open-Loop Characteristics

### 2.1 Aircraft, servo, and washout models

The aircraft yaw-rate plant is:

$$ P(s)=\frac{-4.5s+0.75}{s^2+1.1s+4.2} $$

The rudder servo is:

$$ G_s(s)=\frac{-10}{s+10} $$

and the nominal washout filter is:

$$ H_w(s)=\frac{s}{s+0.5}=\frac{s}{s+1/\tau}, \qquad \tau=2 \mathrm{s} $$

For the final yaw-damper configuration, the closed-loop transfer function used in MATLAB is:

$$ T_{\mathrm{on}}(s)=\frac{G_s(s)P(s)}{1+K_1G_s(s)P(s)H_w(s)} $$

implemented as `feedback(Gs*P, K1*Hw)`. With the damper disabled, $T_{\mathrm{off}}(s)=G_s(s)P(s)$. The aircraft plant, servo, and washout filter must be kept distinct when interpreting the different root-locus configurations.

### 2.2 Open-loop poles, zero, and modal parameters

The aircraft denominator is $s^2+1.1s+4.2$, giving poles:

$$ s_{1,2}=-0.55 \pm j1.9742 $$

The plant zero follows from $-4.5s+0.75=0$:

$$ s_z=+0.16667 $$

This right-half-plane zero is associated with non-minimum-phase behaviour and helps explain the inverse response in the plant's step response. The poles have negative real parts, so the plant's natural mode is stable and oscillatory.

Comparing the denominator with $s^2+2\zeta\omega_n s+\omega_n^2$ gives:

$$ \omega_n=\sqrt{4.2}=2.0494 \mathrm{rad/s}, \qquad \zeta=\frac{1.1}{2\sqrt{4.2}}=0.2684 $$

and $\omega_d \approx 1.9742 \mathrm{rad/s}$. The oscillation period is approximately $2\pi/\omega_d=3.18 \mathrm{s}$, and the exponential decay time constant is $1/0.55 \approx 1.82 \mathrm{s}$.

### 2.3 Open-loop step and steady-state check

For a unit-step input, $R(s)=P(s)/s$. The final value theorem gives:

$$ r(\infty)=\lim_{s\to0} sR(s)=P(0)=\frac{0.75}{4.2}=+0.17857 $$

The response contains a decaying oscillatory transient and approaches this positive steady-state value while the input remains applied. The initial inverse-response behaviour is consistent with the right-half-plane zero. This open-loop plant response is a baseline; it is not the same as the servo-plus-aircraft damper-off response, whose DC value is negative because $G_s(0)=-1$.

MATLAB checks can be performed with `pole(P)`, `zero(P)`, `damp(P)`, `dcgain(P)`, and `step(P,20)`.

## 3. Root Locus Without the Washout Filter

To isolate the effect of the washout filter, the no-washout comparison retains both the servo and aircraft plant, and applies the gain in a unity negative-feedback loop:

$$ L_{\mathrm{no\ washout}}(s)=G_s(s)P(s), \qquad T(s)=\frac{K L_{\mathrm{no\ washout}}(s)}{1+K L_{\mathrm{no\ washout}}(s)} $$

For this comparison, the closed-loop characteristic polynomial is:

$$ s^3+11.1s^2+(15.2+45K)s+(42-7.5K)=0 $$

The table gives the poles calculated from this characteristic polynomial for representative gains. The complex-pair damping ratios are calculated from $\zeta=-\Re(s)/|s|$. 

| Gain ($K$) | Closed-loop poles (approx.) | Complex-pair damping ratio | Stability |
| :--- | :--- | :--- | :--- |
| 0.05 | -9.7479, -0.6760 $\pm$ j1.9527 | 0.327 | Stable |
| 0.10 | -9.4811, -0.8095 $\pm$ j1.9224 | 0.388 | Stable |
| 0.15 | -9.1965, -0.9517 $\pm$ j1.8812 | 0.452 | Stable |
| 0.20 | -8.8904, -1.1048 $\pm$ j1.8262 | 0.517 | Stable |
| 0.25 | -8.5569, -1.2715 $\pm$ j1.7528 | 0.588 | Stable |
| 0.30 | -8.1877, -1.4562 $\pm$ j1.6536 | 0.661 | Stable |
| 0.40 | -7.2721, -1.9140 $\pm$ j1.3037 | 0.827 | Stable |
| 0.50 | -5.5455, -3.6804, -1.8741 | No complex pair | Stable |

```matlab
s = tf('s');
P = (-4.5*s + 0.75)/(s^2 + 1.1*s + 4.2);
Gs = -10/(s + 10);
L_no_washout = Gs*P;
figure;
rlocus(L_no_washout);
grid on;
title('Root Locus Without Washout Filter');
```

In this configuration, the complex pair becomes more damped over the listed gains, and all listed poles remain in the left half-plane. 

![image alt](https://github.com/grim-reaper-original/Flight-Dynamics-7th-Sem-EL-Project/blob/3826e59b1e7ca15f96c4dbd44800cba40eee5bf0/3.%20Root%20locus%20without%20washout%20filter.png)
<small>*Figure 1: Root locus without including washout filter*</small>  

## 4. Full Yaw-Damper Root-Locus Analysis & Gain Selection

The design root locus includes the aircraft plant, rudder servo, and washout filter evaluated together in the feedback loop:

$$ L(s)=G_s(s)P(s)H_w(s), \qquad T(s)=\frac{G_s(s)P(s)}{1+K_1G_s(s)P(s)H_w(s)} $$

The washout filter makes the feedback frequency-dependent, allowing the system to reject constant yaw-rate components at steady state. A root-locus plot and closed-loop pole sweep were used to evaluate how the yaw-damper gain ($K_1$) affects closed-loop pole locations, system stability, and Dutch-roll damping.

```matlab
Hw = s/(s + 0.5);
L_full = Gs*P*Hw;

figure;
rlocus(L_full);
grid on;
title('Full Yaw-Damper Root Locus');
```



The following closed-loop poles, damping ratios ($\zeta$), and natural frequencies were obtained across a sweep of $K_1$. The table reports the complex-conjugate pair associated with the Dutch-roll response at each gain. 

| Gain ($K_1$) | Dutch-roll poles | Dutch-roll damping ratio ($\zeta$) | Stable? | Meets ($\zeta \geq 0.4$)? |
| :--- | :--- | :--- | :--- | :--- |
| 0.050 | -0.6721 $\pm$ j1.9189 | 0.331 | Yes | No |
| 0.100 | -0.8010 $\pm$ j1.8479 | 0.398 | Yes | No |
| 0.150 | -0.9377 $\pm$ j1.7560 | 0.471 | Yes | Yes |
| 0.200 | -1.0836 $\pm$ j1.6340 | 0.553 | Yes | Yes |
| 0.250 | -1.2398 $\pm$ j1.4647 | 0.646 | Yes | Yes |
| 0.300 | -1.4048 $\pm$ j1.2066 | 0.759 | Yes | Yes |
| 0.400 | -0.9143 $\pm$ j0.4727 | 0.888 | Yes | Yes |
| 0.500 | -0.6647 $\pm$ j0.5344 | 0.779 | Yes | Yes |

![image alt](https://github.com/grim-reaper-original/Flight-Dynamics-7th-Sem-EL-Project/blob/3826e59b1e7ca15f96c4dbd44800cba40eee5bf0/4.%20Full%20yaw%20damper%20root%20locus%20(with%20filter).png)
<small>*Figure 2: Full Yaw-Damper root locus*</small>

## 5. Interpretation of the Root-Locus Results

At low gains, the Dutch-roll damping ratio remains below the assignment's minimum requirement of $0.4$. At $K_1=0.10$, the damping ratio is $0.398$, which is close to, but still below, the required value.

Increasing the gain to $K_1=0.15$ raises the damping ratio to $0.471$. Further increases improve the damping ratio over the tested range up to $K_1=0.40$, where it reaches $0.888$. At $K_1=0.50$, the dominant complex pair changes, and its damping ratio decreases to $0.779$.

All the tested gain values in this full-loop analysis have closed-loop poles with negative real parts. Therefore, the closed-loop system is stable at each tested gain.

## 6. Candidate Yaw-Damper Gain

The gain $K_1=0.25$ was selected as an initial candidate for further validation.

Its Dutch-roll poles are:

$$ s_{1,2}=-1.2398 \pm j1.4647 $$

with damping ratio:

$$ \zeta=0.646 $$

This exceeds the assignment's minimum damping requirement of $0.4$. The other two closed-loop poles are located at approximately $-0.6753$ and $-8.4451$, so all four poles lie in the left half-plane.

## 7. Closed-Loop Yaw-Rate Step Response

A unit-step response was evaluated in MATLAB for the full yaw-damper configuration using the selected gain ($K_1=0.25$). The closed-loop transfer function was defined as:

$$ T_{\mathrm{yaw}}(s) = \frac{G_s(s)P(s)}{1+K_1G_s(s)H_w(s)P(s)} $$

| Pole | Damping ratio | Natural frequency (rad/s) | Time constant (s) |
| :--- | :--- | :--- | :--- |
| -0.6753 | 1.000 | 0.675 | 1.48 |
| -1.2398 + j1.4647 | 0.646 | 1.92 | 0.807 |
| -1.2398 - j1.4647 | 0.646 | 1.92 | 0.807 |
| -8.4451 | 1.000 | 8.445 | 0.118 |

All four poles have negative real parts, confirming that the closed-loop system is stable for the selected gain. The Dutch-roll pole pair has a damping ratio of $0.646$, exceeding the assignment requirement of $\zeta \geq 0.4$.

## 8. Step-Response Characteristics

| Parameter | MATLAB result |
| :--- | :--- |
| Rise time | 0.1774 s |
| Transient time | 4.3505 s |
| Settling time | 7.5360 s |
| Settling minimum | -0.3761 |
| Settling maximum | -0.1651 |
| Overshoot | 110.5954% |
| Undershoot | 632.2747% |
| Peak value | 1.1291 |
| Peak time | 0.6544 s |

![image alt](https://github.com/grim-reaper-original/Flight-Dynamics-7th-Sem-EL-Project/blob/3826e59b1e7ca15f96c4dbd44800cba40eee5bf0/8.%20Closed%20loop%20yaw%20rate%20response.png)
<small>*Figure 3: Closed loop yaw rate response*</small>

## 9. Interpretation and Design Status

The selected gain $K_1=0.25$ satisfies the Dutch-roll damping requirement in the linear closed-loop pole analysis. The step-response analysis also confirms stability and provides quantitative measures of the transient behaviour.

## 10. Simulink Validation of the Closed-Loop Yaw-Rate Response

The closed-loop yaw-damper configuration was simulated in Simulink using the selected gain ($K_1=0.25$).

| Parameter | MATLAB | Simulink |
| :--- | :--- | :--- |
| Final yaw rate | -0.178572 | -0.178572 |
| Maximum yaw rate | 1.1291 | 1.112467 |
| Time of maximum yaw rate (s) | 0.6544 | 0.7330 |
| Minimum yaw rate | -0.3761 | -0.374355 |
| Time of minimum yaw rate (s) | Approx 2.65--2.75 | 2.7290 |
| Settling time (s) | 7.5360 | Not calculated |

![image alt](https://github.com/grim-reaper-original/Flight-Dynamics-7th-Sem-EL-Project/blob/3826e59b1e7ca15f96c4dbd44800cba40eee5bf0/10.0%20Simulink%20Set%20up.png)
<small>*Figure 4: Simulink Set Up for conducting verification tests*</small>

![image alt](https://github.com/grim-reaper-original/Flight-Dynamics-7th-Sem-EL-Project/blob/3826e59b1e7ca15f96c4dbd44800cba40eee5bf0/10.%20Closed%20loop%20yaw%20rate%20response%20(Simulink)%20(Basically%20Damper%20on).png)
<small>*Figure 5: Closed loop yaw rate response from Simulink scope (Damper ON)*</small>


## 11. MATLAB–Simulink Comparison and Conclusion

The comparison demonstrates close agreement between the MATLAB transfer-function model and the Simulink implementation for the selected gain and unit-step input.

## 12. Simulink Settling-Time Verification

The settling time of the closed-loop yaw-rate response was calculated from the logged Simulink output using MATLAB. A 2% settling band was defined around the final simulated yaw-rate value.

$$ y_{\mathrm{final}} \pm 0.02 |y_{\mathrm{final}}| $$

giving the limits:

$$ -0.182144 \leq y(t) \leq -0.175001 $$

| Parameter | MATLAB | Simulink |
| :--- | :--- | :--- |
| Final yaw rate | -0.178572 | -0.178572 |
| 2% settling time | 7.5360 s | 7.8979 s |

## 13. Closed-Loop Validation Summary

For the selected yaw-damper gain ($K_1=0.25$), the full closed-loop pole analysis gives a Dutch-roll damping ratio of $0.646$.

## 14. Effect of Washout Time Constant on Dutch-Roll Damping

A parametric sweep was performed in MATLAB with the yaw-damper gain held constant at $K_1=0.25$, while the washout time constant was varied from $1$ to $4$ seconds.

$$ H_w(s)=\frac{s}{s+1/\tau} $$

#### 14.1 Results of the Time-Constant Sweep

| Washout time constant, $\tau$ (s) | Dutch-roll damping ratio, $\zeta$ | Natural frequency (rad/s) | Steady-state yaw rate |
| :--- | :--- | :--- | :--- |
| 1 | 0.55622 | 1.5741 | -0.17857 |
| 2 | 0.64608 | 1.9189 | -0.17857 |
| 3 | 0.62826 | 2.0271 | -0.17857 |
| 4 | 0.61806 | 2.0695 | -0.17857 |

#### 14.2 Effect on Dutch-Roll Damping
The results show that the damping ratio does not increase monotonically with the washout time constant. 

#### 14.3 Effect on Steady-State Yaw Rate
The steady-state yaw rate remains approximately $-0.17857$ for every tested time constant, consistent with the washout filter's zero DC gain ($H_w(0)=0$).

## 15. Closed-Loop Yaw-Damper Performance: Damper OFF vs ON

The selected yaw-damper parameters were $K_1=0.25, \qquad \tau=2 \text{s}$.

#### 15.1 Simulation Results

| Performance parameter | Damper OFF | Damper ON |
| :--- | :--- | :--- |
| Dutch-roll damping ratio | 0.2684 | 0.6461 |
| Dutch-roll natural freq (rad/s) | 2.0494 | 1.9189 |
| Maximum yaw rate | 1.387418 | 1.129073 |
| Time of max yaw rate (s) | 0.723 | 0.652 |
| Minimum yaw rate | -0.831079 | -0.376070 |
| Time of min yaw rate (s) | 2.315 | 2.644 |
| Theoretical steady-state | -0.178571 | -0.178571 |
| Simulated final yaw rate | -0.178531 | -0.178572 |
| 2% settling time (s) | 10.887 | 7.537 |

![image alt](https://github.com/grim-reaper-original/Flight-Dynamics-7th-Sem-EL-Project/blob/3826e59b1e7ca15f96c4dbd44800cba40eee5bf0/15.%20Unit%20step%20Damper%20response%20on%20vs%20off.png)
<small>*Figure 6: Matlab Yaw rate response for Unit Step Input with Damper ON and OFF*</small>

#### 15.2 Closed-Loop Pole Analysis

**Damper OFF:**
$s_{1,2} = -0.55 \pm j1.9742$,  $s_3 = -10$

**Damper ON:**
$s_{1,2} = -1.2398 \pm j1.4647$
$s_3 = -0.6753$
$s_4 = -8.4451$

## 16. Pulse-Response Comparison: Yaw Damper OFF vs ON

#### 16.1 Simulation Results

| Performance parameter | Damper OFF | Damper ON |
| :--- | :--- | :--- |
| Maximum yaw rate | 2.774836 | 2.258147 |
| Time of max yaw rate (s) | 0.723 | 0.652 |
| Minimum yaw rate | -3.734635 | -2.307466 |
| Time of min yaw rate (s) | 1.930 | 1.835 |
| Final yaw rate | 0.000124 | 0.000001 |
| 2% settling time (s) | 8.745 | 5.255 |

![image alt](https://github.com/grim-reaper-original/Flight-Dynamics-7th-Sem-EL-Project/blob/3826e59b1e7ca15f96c4dbd44800cba40eee5bf0/16.%20Pulse%20Damper%20response%20on%20vs%20off.png)
<small>*Figure 7: Matlab Yaw rate response for Pulse Input with Damper ON and OFF*</small>

## 17. Steady-Turn and Washout-Filter Verification

#### 17.1 Simulation Results

| Parameter | Result |
| :--- | :--- |
| Washout time constant, $\tau$ | $2 \text{s}$ |
| Constant input yaw rate | 1 |
| Initial filter output | 1.000000 |
| Maximum filter output | 1.000000 |
| Filter output at $t=10 \text{s}$ | 0.00673795 |
| Filter output at $t=20 \text{s}$ | 0.00004540 |
| Theoretical DC gain | 0 |
| Theoretical steady-state output | 0 |

![image alt](https://github.com/grim-reaper-original/Flight-Dynamics-7th-Sem-EL-Project/blob/3826e59b1e7ca15f96c4dbd44800cba40eee5bf0/17.%20Washout%20filter%20response.png)
<small>*Figure 8: Simulink Washout Filter Response*</small>

## 18. Simulink Verification: Damper OFF vs ON

### 18.1 Damper-OFF Simulation Results

| Performance parameter | Damper OFF — Simulink |
| :--- | :--- |
| Maximum yaw rate | 1.387267 |
| Time of max yaw rate (s) | 0.7304 |
| Minimum yaw rate | -0.830770 |
| Time of min yaw rate (s) | 2.3294 |
| Final yaw rate | -0.178184 |
| 2% settling time (s) | 11.0127 |

### 18.2 Simulink Comparison: Damper OFF vs ON

| Performance parameter | Damper OFF | Damper ON |
| :--- | :--- | :--- |
| Maximum yaw rate | 1.387267 | 1.112467 |
| Time of max yaw rate (s) | 0.7304 | 0.7330 |
| Minimum yaw rate | -0.830770 | -0.374355 |
| Time of min yaw rate (s) | 2.3294 | 2.7290 |
| Final yaw rate | -0.178184 | -0.178572 |
| 2% settling time (s) | 11.0127 | 7.8979 |

![image alt](https://github.com/grim-reaper-original/Flight-Dynamics-7th-Sem-EL-Project/blob/3826e59b1e7ca15f96c4dbd44800cba40eee5bf0/18.%20Simulink%20Damper%20off%20(K%3D0)%20unit%20step%20response.png)
<small>*Figure 9: Simulink yaw rate response with Damper OFF*</small>

## 19. MIL-F-8785C Level I Dutch-Roll Assessment

### 19.1 Assumed Aircraft Class and Flight Category

For this design study, the aircraft is assumed to be **Class IV, Category A**. MIL-F-8785C Table VI specifies:
- Minimum Dutch-roll damping ratio: $\zeta_{dr} \geq 0.4$
- Minimum undamped natural frequency: $\omega_{nd} \geq 1.0 \mathrm{rad/s}$

### 19.2 Open-Loop Assessment

| Parameter | Open-loop result | Level I requirement | Result |
| :--- | :--- | :--- | :--- |
| Damping ratio ($\zeta_{dr}$) | 0.2684 | $\geq 0.4$ | Fail |
| Undamped natural freq ($\omega_{nd}$) | 2.0494 rad/s | $\geq 1.0$ rad/s | Pass |

### 19.3 Closed-Loop Assessment with the Selected Yaw Damper

| Parameter | Closed-loop result | Level I requirement | Result |
| :--- | :--- | :--- | :--- |
| Damping ratio ($\zeta_{dr}$) | 0.6461 | $\geq 0.4$ | Pass |
| Undamped natural freq ($\omega_{nd}$) | 1.9189 rad/s | $\geq 1.0$ rad/s | Pass |

![image alt](https://github.com/grim-reaper-original/Flight-Dynamics-7th-Sem-EL-Project/blob/3826e59b1e7ca15f96c4dbd44800cba40eee5bf0/19.%20Relevant%20MIL-F-8785C%20Section%20(Table%20VI).png)
<small>*Figure 10: Table VI from MIL-F-8785C document*</small>
