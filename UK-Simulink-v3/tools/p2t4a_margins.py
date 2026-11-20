# p2t4a_margins.py -- Plan 2 Task 4a. Recomputes, in Python (double), the 3-step hand sequence of
# tests/test_base_plant.m check (i) from the ROME_Mega_Code.ino equations, prints every intermediate, and
# reports how close each floor() argument (PWM, L249; encoder edges, L267/L276-281) comes to an integer, so
# that round-off cannot flip a PWM count or an encoder count between this script, the test and base_plant_step.
# Source of the literal anchors A1_*, A2_*, A3_* in the test. Run: python -I p2t4a_margins.py
import math
Kp, Ki, IM, dt, MR = 0.190, 0.125, 500.0, 0.050, 120.0   # ino L33, L34, L35, L9, L8
tau, gain = 0.10, 0.9                                     # test values for motor_tau (s), motor_gain (-)
q = 60.0 / (dt * 12 * 50)                                 # rpm per edge, L6, L7, L276-281
cmds = [[40, 60, 150, -30], [0, -60, 150, -30], [0, -60, 150, -30]]   # desiredRPMs per step (rpm)
e_int = [0.0]*4; w_mot = [0.0]*4; absavg = [0.0]*4; frac = [0.0]*4; dirv = [1]*4
a = math.exp(-dt/tau)
m_pwm = []; m_edge = []                                   # distances of floor() arguments to an integer
print(f"q = {q!r} rpm per edge")
for k, c in enumerate(cmds, 1):
    print(f"--- step {k}")
    for i in range(4):
        er = frac[i] + absavg[i]/q
        cnt = math.floor(er)
        frac[i] = er - cnt
        if er != 0.0:
            m_edge.append(min(er - math.floor(er), math.ceil(er) - er))
        meas = dirv[i]*cnt*q
        e = c[i] - meas
        e_int[i] = min(max(e_int[i] + e*dt, -IM), IM)
        corr = c[i] + Kp*e + Ki*e_int[i]
        d = 1 if corr >= 0 else -1
        pr = abs(corr)/MR*255
        m_pwm.append(min(pr - math.floor(pr), math.ceil(pr) - pr))
        pwm = min(max(math.floor(pr), 0), 255)
        T = gain*pwm/255*MR*d
        w0 = w_mot[i]
        w1 = T + (w0-T)*a
        iw = T*dt + (w0-T)*tau*(1-a)
        ia = abs(iw); cross = None
        if w0*T < 0:
            tc = tau*math.log((T-w0)/T)
            cross = tc
            if tc < dt:
                i1 = T*tc + (w0-T)*tau*(1-math.exp(-tc/tau))
                ia = abs(i1) + abs(iw-i1)
        print(f"w{i+1}: edges_real={er:.6f} cnt={cnt} meas={meas!r} e={e!r} eint={e_int[i]!r} corr={corr!r} "
              f"pwm_real={pr:.6f} pwm={pwm} dir={d} T={T:.6f} w1={w1:.6f} absavg={ia/dt:.6f} tc={cross}")
        w_mot[i] = w1; absavg[i] = ia/dt; dirv[i] = d
print(f"min distance to an integer: PWM argument {min(m_pwm):.4f}, nonzero encoder-edge argument {min(m_edge):.4f}")
