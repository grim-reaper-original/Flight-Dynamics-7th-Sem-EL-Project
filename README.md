# Flight-Dynamics-7th-Sem-EL-Project

# Yaw Damper Design for Level I Dutch-Roll Flying Qualities

## 1. Objective

The objective of this study is to investigate the open-loop yaw-rate dynamics of an aircraft and establish a baseline for the design of a yaw damper to improve Dutch-roll damping. MATLAB is used to model the aircraft dynamics, calculate the system poles and zeros, evaluate its damping characteristics, and analyse its response to a unit-step rudder input.

The subsequent design stages will incorporate a rudder servo and a washout filter, followed by feedback gain selection using root-locus analysis and verification of the closed-loop response in MATLAB and Simulink.

## 2. Aircraft Yaw-Rate Transfer Function

The aircraft yaw-rate dynamics are represented by the transfer function supplied in the problem statement:

$$
G(s)=\\frac{r(s)}{\\delta_r(s)}
=\\frac{-4.5s+0.75}{s^2+1.1s+4.2}
$$

where:

- $G(s)$ is the aircraft yaw-rate transfer function.
- $r(s)$ is the Laplace transform of yaw rate.
- $\\delta_r(s)$ is the Laplace transform of rudder deflection.
- $s$ is the Laplace-domain complex frequency variable.

The denominator describes the system's natural modes, while the numerator determines its zeros and influences the transient response.

The model is implemented in MATLAB using the Control System Toolbox.

```matlab
clear;
clc;
close all;

s = tf('s');

P = (-4.5*s + 0.75)/(s^2 + 1.1*s + 4.2);
```

The transfer function is continuous-time and linear. The present analysis considers the aircraft plant independently, without the rudder servo, washout filter, feedback controller, or actuator saturation.

## 3. Open-Loop Step Response

The open-loop response is evaluated by applying a unit-step input to the rudder channel.

A unit-step input is defined as

$$
\\delta_r(t)=1,\\qquad t\\geq 0
$$

with Laplace transform

$$
\\Delta_r(s)=\\frac{1}{s}.
$$

The output in the Laplace domain is therefore

$$
R(s)=G(s)\\Delta_r(s)
$$

and hence

$$
R(s)=
\\frac{-4.5s+0.75}
{s(s^2+1.1s+4.2)}.
$$

The MATLAB simulation is performed using:

```matlab
figure;
step(P,20);

grid on;
title('Open-loop yaw-rate response');
xlabel('Time (seconds)');
ylabel('Yaw rate per unit rudder input');
```

The simulated response initially moves in the negative direction, subsequently crosses zero, and exhibits oscillations of decreasing amplitude. At longer times, the response approaches a positive steady-state value.

The initial inverse response is associated with the positive real zero of the transfer function. The progressively decreasing oscillation amplitude is associated with the stable, damped complex poles of the system.

The steady-state value for a unit-step input can be calculated independently using the final value theorem.

## 4. Pole-Zero Analysis

### 4.1 System poles

The poles are obtained by setting the denominator equal to zero:

$$
s^2+1.1s+4.2=0.
$$

Using the quadratic formula,

$$
s=\\frac{-1.1\\pm\\sqrt{1.1^2-4(4.2)}}{2}.
$$

Since the discriminant is negative,

$$
1.1^2-4(4.2)=-15.59,
$$

the poles form a complex-conjugate pair:

$$
\\boxed{s_{1,2}=-0.55\\pm j1.9742}
$$

Both poles have negative real parts, indicating that the open-loop system is asymptotically stable. Their nonzero imaginary components indicate an oscillatory mode.

The real part determines the exponential decay rate, while the imaginary part determines the oscillation frequency.

### 4.2 System zero

The zero is obtained by setting the numerator equal to zero:

$$
-4.5s+0.75=0.
$$

Therefore,

$$
\\boxed{s_z=+0.16667}
$$

The zero lies in the right half of the complex plane. Such a zero is associated with non-minimum-phase behaviour and can produce an inverse response to an input step.

In the simulated response, the yaw rate initially moves opposite to its eventual steady-state direction before reversing and settling at a positive value.

## 5. Natural Frequency and Damping Ratio

The denominator of a standard second-order system is

$$
s^2+2\\zeta\\omega_n s+\\omega_n^2,
$$

where $\\omega_n$ is the undamped natural frequency and $\\zeta$ is the damping ratio.

Comparing it with the aircraft denominator,

$$
s^2+1.1s+4.2,
$$

gives

$$
\\omega_n^2=4.2
$$

and

$$
2\\zeta\\omega_n=1.1.
$$

Thus, the natural frequency is

$$
\\boxed{\\omega_n=\\sqrt{4.2}
=2.0494\\ \\text{rad/s}}
$$

and the damping ratio is

$$
\\boxed{\\zeta=\\frac{1.1}{2\\sqrt{4.2}}
=0.2684}.
$$

The damped natural frequency is

$$
\\omega_d=\\omega_n\\sqrt{1-\\zeta^2}
$$

which gives

$$
\\boxed{\\omega_d\\approx1.9742\\ \\text{rad/s}}.
$$

The corresponding oscillation period is

$$
T_d=\\frac{2\\pi}{\\omega_d}
$$

or approximately

$$
\\boxed{T_d\\approx3.18\\ \\text{s}}.
$$

The exponential decay rate is determined by the real part of the poles:

$$
\\sigma=0.55\\ \\text{s}^{-1}.
$$

The corresponding decay time constant is

$$
\\boxed{\\tau_{\\mathrm{decay}}=\\frac{1}{0.55}
\\approx1.82\\ \\text{s}}.
$$

This decay time constant is a characteristic of the aircraft's open-loop oscillatory mode and is distinct from the washout-filter time constant introduced in the controller design stage.

## 6. Steady-State Analysis

For a stable system subjected to a unit-step input, the final value theorem gives

$$
r(\\infty)=\\lim_{s\\to0}sR(s).
$$

Substituting the output expression,

$$
r(\\infty)=
\\lim_{s\\to0}
\\frac{-4.5s+0.75}{s^2+1.1s+4.2}.
$$

Evaluating the limit yields

$$
\\boxed{r(\\infty)=\\frac{0.75}{4.2}
=0.17857}.
$$

The same result follows from the plant's DC gain:

$$
G(0)=\\frac{0.75}{4.2}=0.17857.
$$

Therefore, the response does not decay to zero because the unit-step input remains applied. Instead, the transient oscillations decay and the output approaches the nonzero steady-state value.

The final value theorem is applicable because the poles of the step-response output are in the left half-plane, with the pole at the origin introduced by the step input cancelling in the final-value expression.

## 7. Analytical Step-Response Verification

The unit-step output is

$$
R(s)=
\\frac{-4.5s+0.75}
{s(s^2+1.1s+4.2)}.
$$

Using partial-fraction decomposition and inverse Laplace transformation, the time-domain response is approximately

$$
\\begin{aligned}
r(t)={}&0.178571\\\\
&+e^{-0.55t}\\big[
-0.178571\\cos(1.9742t)\\\\
&\\qquad-2.3217\\sin(1.9742t)
\\big],\\quad t\\geq0.
\\end{aligned}
$$

The response comprises a constant steady-state term and a decaying oscillatory transient. The exponential factor governs the decay, while the sine and cosine terms govern the oscillatory behaviour.

The analytical expression can be compared with MATLAB's `step` response to verify the transfer-function implementation and the calculations.

## 8. MATLAB Verification

The pole-zero locations, damping characteristics, and steady-state gain are obtained using the following commands:

```matlab
p = pole(P);
z = zero(P);

disp('Poles:');
disp(p);

disp('Zeros:');
disp(z);

disp('Damping ratio and natural frequency:');
damp(P);

disp('Steady-state gain:');
dcgain(P);
```

The `pole` and `zero` functions determine the roots of the denominator and numerator, respectively. The `damp` function provides modal information, including natural frequencies and damping ratios. The `dcgain` function evaluates the system's steady-state gain.

These numerical results provide an independent check of the analytical calculations.

## 9. Preliminary Results

| Parameter | Calculated result | Interpretation |
|---|---:|---|
| Open-loop poles | $-0.55\\pm j1.9742$ | Stable oscillatory mode |
| Open-loop zero | $+0.16667$ | Right-half-plane zero |
| Natural frequency | $2.0494\\ \\text{rad/s}$ | Undamped modal frequency |
| Damping ratio | $0.2684$ | Open-loop damping characteristic |
| Damped frequency | $1.9742\\ \\text{rad/s}$ | Oscillation frequency |
| Oscillation period | $3.18\\ \\text{s}$ | Time per oscillation |
| Decay time constant | $1.82\\ \\text{s}$ | Exponential transient decay |
| Unit-step steady-state output | $0.17857$ | DC gain for a unit-step input |

The calculated open-loop damping ratio is below the assignment's specified target of $\\zeta_{dr}\\geq0.4$. This establishes a quantitative motivation for investigating yaw-rate feedback and washout-filter compensation.

The pole and damping calculations describe the supplied second-order denominator. The identification of the mode as the aircraft's Dutch-roll mode is consistent with the assignment's intended model, but a full modal identification would require the underlying aircraft dynamics or additional model context.

## 10. Work Completed and Next Steps

The following work has been completed:

1. The supplied aircraft yaw-rate transfer function has been implemented in MATLAB.
2. The open-loop unit-step response has been simulated.
3. The system poles and zero have been determined analytically and numerically.
4. The natural frequency, damping ratio, damped frequency, oscillation period, and decay time constant have been calculated.
5. The final value of the unit-step response has been obtained using the final value theorem.
6. The observed oscillatory and inverse-response characteristics have been related to the pole-zero structure.

The next stage will introduce the rudder servo,

$$
G_s(s)=\\frac{-10}{s+10},
$$

and the washout filter,

$$
H_w(s)=\\frac{s}{s+1/\\tau},
\\qquad 1\\leq\\tau\\leq4\\ \\text{s}.
$$

## Update 2 — Work done on 20 October 2026

### 11. Compensated Open-Loop Root-Locus Analysis

The aircraft plant was combined with the specified rudder servo and washout filter to form the compensated open-loop transfer function. The initial washout time constant was set to $\tau=2\ \text{s}$.

The component transfer functions are

$$
P(s)=\frac{-4.5s+0.75}{s^2+1.1s+4.2},
\qquad
G_s(s)=\frac{-10}{s+10},
\qquad
H_w(s)=\frac{s}{s+0.5}.
$$

The compensated open-loop transfer function, excluding the adjustable gain $K$, is

$$
L_0(s)=P(s)G_s(s)H_w(s)
=\frac{(-4.5s+0.75)(-10)s}
{(s^2+1.1s+4.2)(s+10)(s+0.5)}.
$$

### 11.1 Poles and zeros

The open-loop poles are

$$
s=-0.55\pm j1.9742,\qquad s=-10,\qquad s=-0.5.
$$

The open-loop zeros are

$$
s=0,\qquad s=+0.16667.
$$

The complex-conjugate pair comes from the aircraft plant. The pole at $-10$ represents the servo dynamics, and the pole at $-0.5$ comes from the washout filter for $\tau=2\ \text{s}$. The washout filter introduces a zero at the origin. The aircraft plant contributes a right-half-plane zero at $+0.16667$.

### 11.2 Interpreting the root-locus plot

The root-locus plot displays the possible closed-loop pole locations as the scalar gain $K$ varies under MATLAB's standard negative-feedback convention. At zero gain, the branches begin at the four open-loop poles. As the gain changes, the branches move and terminate at the finite open-loop zeros or approach infinity.

The horizontal axis is the real part of the pole and the vertical axis is its imaginary part. Poles in the left half-plane correspond to decaying modes; poles in the right half-plane correspond to growing modes. Complex poles indicate oscillatory behaviour. The grey contours indicate constant damping ratio and can be used to estimate damping at candidate pole locations.

There are four open-loop poles and two finite zeros, so two branches terminate at the finite zeros and two branches approach infinity. The asymptote centroid is

$$
\sigma_a=\frac{\sum p_i-\sum z_i}{n-m}
=\frac{-11.6-0.16667}{2}
\approx -5.8833,
$$

with asymptote angles $+90^\circ$ and $-90^\circ$.

The right-half-plane zero is an important limitation: one root-locus branch approaches this zero as gain becomes large. Therefore, increasing gain indefinitely cannot be assumed to preserve closed-loop stability.

### 11.3 Design implications

The assignment specifies a Dutch-roll damping target of $\zeta_{dr}\geq0.4$. A candidate gain must be selected by checking the actual closed-loop poles and damping ratios, not by visual inspection of the root-locus plot alone. The feedback sign must also be justified from the chosen signal convention and the signs of the plant and servo transfer functions.

The root-locus plot is a linear analysis. Rudder saturation is not represented by the root locus and must be checked separately in time-domain simulation.

### 11.4 Status of work

Completed in this update:

1. Combined the aircraft plant, servo, and washout filter for an initial washout time constant of $2\ \text{s}$.
2. Generated the compensated open-loop root-locus plot.
3. Identified the four open-loop poles and two finite zeros.
4. Interpreted the real and imaginary axes, damping-ratio contours, and asymptotes.

Not yet completed: selecting the feedback sign and gain, verifying closed-loop damping and stability, and running closed-loop time-domain simulations with rudder limits.

## Update 3 — Work done on 20 October 2026: Closed-Loop Gain Sweep

### 11. Purpose

The compensated open-loop model was evaluated using MATLAB's standard negative-feedback convention to investigate how the scalar feedback gain \(K\) affects closed-loop pole locations and damping. The washout-filter time constant was fixed at \(\tau=2\ \text{s}\) for this initial comparison.

The compensated open-loop transfer function, excluding \(K\), is

$$
L_0(s)=P(s)G_s(s)H_w(s)
$$

where

$$
P(s)=\frac{-4.5s+0.75}{s^2+1.1s+4.2},
\qquad
G_s(s)=\frac{-10}{s+10},
\qquad
H_w(s)=\frac{s}{s+0.5}.
$$

The closed-loop system for this comparison is

$$
T(s)=\frac{K L_0(s)}{1+K L_0(s)}.
$$

This equation describes the standard negative-feedback connection used by `feedback(K*L0,1)` in MATLAB. The physical feedback sign still needs to be justified from the intended signal definitions and controller architecture.

### 12. Compensated Open-Loop Poles and Zeros

MATLAB returned the following compensated open-loop poles:

$$
s=-10,\quad -0.55\pm j1.9742,\quad -0.5.
$$

The zeros are

$$
s=0,\qquad s=+0.1667.
$$

The poles comprise the servo pole, the aircraft's complex-conjugate pair, and the washout-filter pole. The zero at the origin comes from the washout filter. The positive real zero is inherited from the aircraft plant and is a right-half-plane zero.

### 13. Gain Sweep Results

The following table summarizes the closed-loop pole pair with the smallest magnitude of negative real part for each tested gain. This is the slowest mode in each reported case and is a useful initial indicator of the system's transient behaviour.

| Gain \(K\) | Slowest pole pair | Damping ratio | Status |
|---:|---|---:|---|
| 0.10 | \(-0.8010\pm j1.8479\) | 0.398 | Stable; slightly below the target |
| 0.25 | \(-1.2398\pm j1.4647\) | 0.646 | Stable; meets the numerical damping target |
| 0.50 | \(-0.6647\pm j0.5344\) | 0.779 | Stable; meets the numerical damping target |
| 1.00 | \(-0.3277\pm j0.5033\) | 0.546 | Stable; meets the numerical damping target |
| 2.00 | \(-0.1511\pm j0.4161\) | 0.341 | Stable; below the target |
| 5.00 | \(-0.0227\pm j0.2918\) | 0.077 | Stable; below the target |

All four poles in each tested case had negative real parts, so every tested closed-loop linear model was stable under the assumed feedback convention.

The results show that the damping ratio does not increase monotonically with gain. From \(K=0.1\) to \(K=0.25\), the slowest complex pair moves farther into the left half-plane and its damping ratio increases. At higher gains, the slowest pair moves closer to the imaginary axis and its damping ratio decreases. For example, at \(K=5\), the pair at approximately \(-0.0227\pm j0.2918\) is very lightly damped despite remaining mathematically stable.

At \(K=0.5\), the four poles form two complex-conjugate pairs. The slower pair has a damping ratio of approximately 0.779, while the faster pair at approximately \(-5.1353\pm j1.5803\) has a damping ratio of approximately 0.956. These modes must be considered together when interpreting the complete response.

### 14. Preliminary Interpretation

The numerical sweep identifies \(K=0.25\) as a promising candidate for further investigation: its slowest pole pair has a damping ratio of approximately 0.646 and a real part of approximately \(-1.24\), indicating faster decay than the slowest pair at \(K=0.5\). This is not yet a final gain selection.

A final choice cannot be made from pole damping alone. The next checks must include the time-domain yaw-rate response, rudder command and saturation, the response to the specified step and pulse inputs, and verification that the feedback sign matches the intended physical yaw-damper action. The relevant Dutch-roll mode must also be identified consistently as the gain changes.

### 15. MATLAB Commands Used

```matlab
K_values = [0.1 0.25 0.5 1 2 5];

for K = K_values
    T = feedback(K*L0, 1);

    fprintf('\\nK = %.2f\\n', K);
    disp('Closed-loop poles:');
    disp(pole(T));

    disp('Damping ratios and natural frequencies:');
    damp(T);
end
```

### 16. Work Status and Next Steps

Completed in this update:

1. Calculated the compensated open-loop poles and zeros.
2. Evaluated the closed-loop poles for six positive gain values using standard negative feedback.
3. Compared damping ratios and stability across the gain sweep.
4. Identified \(K=0.25\) as a candidate for further time-domain testing, not as the final design.

The compensated open-loop system will then be analysed using root-locus methods to select a suitable feedback gain. Closed-loop poles, damping ratio, time-domain responses, and rudder saturation will subsequently be evaluated in MATLAB and Simulink against the assignment requirements.

At this stage, no closed-loop controller has been designed or validated. The results presented here constitute the open-loop baseline for the subsequent yaw-damper design.
