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

The compensated open-loop system will then be analysed using root-locus methods to select a suitable feedback gain. Closed-loop poles, damping ratio, time-domain responses, and rudder saturation will subsequently be evaluated in MATLAB and Simulink against the assignment requirements.

At this stage, no closed-loop controller has been designed or validated. The results presented here constitute the open-loop baseline for the subsequent yaw-damper design.
