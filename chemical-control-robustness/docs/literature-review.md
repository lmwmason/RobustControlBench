# Focused Literature Review

This is a focused verification, not a comprehensive novelty search.

| Reference | Verified metadata | Relevance |
|---|---|---|
| Åström and Hägglund, *Advanced PID Control* | ISA, 2006, ISBN 978-1-55617-942-6, DOI 10.1002/9781394442102 | PID tuning, noise, robustness, and anti-windup. |
| Åström and Rundqwist, “Integrator Windup and How to Avoid It” | American Control Conference, 1989, pp. 1693–1698, DOI 10.23919/ACC.1989.4790464 | Why saturation must be visible to the integrator. |
| Rundqwist, “Anti-reset Windup for PID Controllers” | IFAC Proceedings Volumes 23(8), 1990, pp. 453–458, DOI 10.1016/S1474-6670(17)51865-0 | Conditional and tracking anti-windup. |
| Anderson and Moore, *Optimal Control: Linear Quadratic Methods* | Prentice Hall, 1990, ISBN 978-0-13-638560-8 | LQR cost, Riccati equation, and stabilizability. |
| Zhou, Doyle, and Glover, *Robust and Optimal Control* | Prentice Hall, 1996, ISBN 978-0-13-456567-5 | Nominal versus robust stability and performance. |
| Teppa-Garran, Bohórquez, and Garcia, “Optimal tuning of PID-type controllers” | *Journal of Applied Research and Technology* 23(2), 2025, pp. 145–154, DOI 10.22201/icat.24486736e.2025.23.2.2470 | Explicit tuning objectives in PID/LQR comparisons and heat-flow simulation. |
| MathWorks, “Linear-Quadratic-Integral Control” | https://www.mathworks.com/help/control/ref/ss.lqi.html | Discrete forward-Euler integrator convention. |
| MathWorks, “Continuous-Discrete Conversion Methods” | https://www.mathworks.com/help/ident/ug/continuous-discrete-conversion-methods.html | Exact time-domain ZOH interpretation. |

The benchmark does not propose a novel controller. Its supported contribution is an auditable computational case study of integral action, mismatch, noise, and saturation in a first-order thermal process.

