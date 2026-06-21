# Online DMPC

**State:** position + velocity  |  **one timestep:** $h$  |  **acceleration:** $u$

## Prediction Model over Horizon

$$
\text{new\_pos} = \text{old\_pos} + \text{velocity}\cdot h + \tfrac{1}{2} u h^2
\qquad\big(s = ut + \tfrac{1}{2}at^2\big)
$$

$$
\text{new\_vel} = \text{old\_vel} + u h
\qquad\big(v = u + at\big)
$$

$$
x_k = \begin{bmatrix} p_k \\ v_k \end{bmatrix}
\qquad
x_{k+1} = A x_k + B u_k
\qquad
\therefore\; A = \begin{bmatrix} 1 & h \\ 0 & 1 \end{bmatrix},
\quad
B = \begin{bmatrix} h^2/2 \\ h \end{bmatrix}
$$

$$
\begin{aligned}
x_1 &= A x_0 + B u_0 \\
x_2 &= A x_1 + B u_1 = A\big(A x_0 + B u_0\big) + B u_1 = A^2 x_0 + AB u_0 + B u_1 \\
x_3 &= A^3 x_0 + A^2 B u_0 + AB u_1 + B u_2
\end{aligned}
$$

$$
\therefore\;
X = \begin{bmatrix} x_1 \\ x_2 \\ x_3 \\ \vdots \end{bmatrix}
\qquad
U = \begin{bmatrix} u_0 \\ u_1 \\ u_2 \\ \vdots \end{bmatrix}
$$

$$
X =
\begin{bmatrix} A \\ A^2 \\ A^3 \\ \vdots \end{bmatrix} x_0
+
\begin{bmatrix}
B & 0 & 0 & \cdots \\
AB & B & 0 & \cdots \\
A^2 B & AB & B & \cdots \\
\vdots & & & \ddots
\end{bmatrix} U
$$

$$
X = \underbrace{A_0\, x_0}_{\text{constant} \,\Rightarrow\, \text{current state}} + \Lambda\, U
$$

$\therefore\; X$ (or future position) is an **affine function** of $U$.

---

# Bézier Curves

A **'d' degree Bézier curve** (control points are literal points in physical space):

$$
p(t) = \sum_{n=0}^{d} P_n\, b_{n,d}(t)
$$

Basically a weighted average of $P_n$.  $b_{n,d}(t)$ = control point $\times$ fixed weighting function (**Bernstein basis polynomial**):

$$
b_{n,d}(t) = \binom{d}{n}\left(\frac{t}{T}\right)^{n}\left(1 - \frac{t}{T}\right)^{d-n}
$$

- always between $0$ and $1$
- always sum to $1$
- at $t = 0$, first point gets weight $1$, others $0$
- at $t = T$, last point gets weight $1$, others $0$

$$
\therefore\; \text{Order: } 6 \text{ Control Points } P_n
\;\to\; \text{Position curve} \;\to\; \text{velocity curve} \;\to\; \text{acceleration curve}
$$

(5 degree segment) — differences of position curve: $\dfrac{d}{T}(P_{n+1} - P_n)$

### Why Bézier curves?

- Variable to optimize $\to U$; solver can pick jerky accelerations.
- Bézier curve of degree $d$: uses $d+1$ control points & defines a segment.
- Polynomial whose shape is controlled by control points.

**Property:**

- Derivatives are linear & stay linear in control points.
- velocity control points = (scaled) differences between control points.
- end & start remain the same.

### So we pick control points 'P', we need state X

$$
X = \phi P
$$

$\phi \to$ Bernstein weights & differences at each timestep (**computed once**).

> **Note:** *everything* is a function of $P$ $\to$ Goal error, jerk, speed limits, collision dist.

---

# Quadratic Program (QP)

**Why?** Bcuz guarantees $\to$ one minimum & always finds it.

$$
\underbrace{\tfrac{1}{2} P^\top H P}_{\text{Quadratic cost}}
+ \;f^\top P
\qquad\qquad
\underbrace{A_\text{ineq} \le b_\text{ineq}}_{\text{Linear Constraints}}
$$

- $H$ (PSD) $\to$ shape of the convex bowl — cost curvature matrix.
- $f^\top P \to$ Tilt of the bowl / Direction of minima.

We use this to minimise whatever we want.

**eg 1: Minimize Jerk** $\to$ Jerk is linear in $P$ (differences of differences of differences of $P$):

$$
\text{jerk} = D P
$$

Penalize squared jerk:

$$
\|\text{jerk}\|^2 = (DP)^\top (DP) = P^\top \underbrace{D^\top D}_{\text{one part of } H}\, P
$$

**eg 2: Minimize Goal Error**

$$
\text{error} = \phi P - g
$$

Penalize squared error:

$$
\|\phi P - g\|^2 = (\phi P - g)^\top(\phi P - g)
= \underbrace{P^\top \phi^\top \phi P}_{\text{part of } H}
\;\underbrace{-\, 2 g^\top \phi P}_{\text{part of } f}
\;+\; \underbrace{g^\top g}_{\substack{\text{constant, ignored} \\ \text{(does not affect minima)}}}
$$

(BASIC REARRANGING)

### Linear Constraints: $A_\text{ineq} \le b_\text{ineq}$

**eg 1: Speed limit** (same with acc limit) — velocity is linear in $P$ (differences of control points):

$$
v = D_v P
\qquad
\text{limit: } v \le v_\text{max} \;\Rightarrow\; \underbrace{D_v}_{\text{part of } A_\text{ineq}} P \le \underbrace{v_\text{max}}_{\text{part of } b_\text{ineq}}
$$

**eg. Collision Avoidance** $\to$ we have predicted pos for '$i$' $= \bar p_i$

- '$j$' treated as fixed $\to \bar p_j$
- unit direction from $i$ to $j$:

$$
\eta = \frac{\bar p_i - \bar p_j}{\|\bar p_i - \bar p_j\|}
$$

Constraint (with $\phi P$):

$$
\eta^\top (p_i - \bar p_j) \ge r_\text{min}
$$

$$
\therefore\; -\eta^\top \phi P \le -\big(r_\text{min} - \eta^\top \bar p_j\big)
\qquad [\text{part of } A_\text{ineq},\; \text{part of } b_\text{ineq}]
$$

### "on-demand" part

Robot '$i$' checks pos of all bots and only solves collision if

$$
\|\bar p_i^k - \bar p_j^k\| < r_\text{min} + \text{margin}
$$

---

# Distributed Coordination (Linear Scaling)

- each robot solves own QP in parallel (Receding Horizon fashion)
  - Trick is to use 'slightly stale' $\to$ 1 timestep old data for position of '$j$'th Robot
- Broadcasts own freshly computed traj
- Repeat

### Event Triggered Replanning

- Resolve only when:

$$
\|x_\text{actual} - x_\text{pred}\| > \varepsilon
$$

- otherwise, keep using prev plan

**BASE DMPC COMPLETE**

---

# Task Reallocation $\to$ Hungarian Algorithm

**Idea:** "subtracting a constant from each row or column does not change which assignment is optimal."

**Steps:**

1. **Row Reduction** $\Rightarrow$ subtract each row's smallest entry from row
2. **Column Reduction** $\Rightarrow$ subtract each column's smallest entry from column
3. **Cover all zero's** with minimum possible number of horizontal/vertical lines
4. If num of minimum lines $= N \Rightarrow$ Assignment Done; otherwise step 5
5. **Create more zeroes:**
   - Find smallest uncovered entry $\delta$
   - a) subtract $\delta$ from all uncovered ones
   - b) add $\delta$ to all line intersections
   - c) leave others alone

---

# BVC Method

### Voronoi Diagram

Robot '$i$' voronoi-cell is every point in room closer to '$i$' than anyone:

$$
V_i = \{\, p : \|p - p_i\| \le \|p - p_j\| \quad \forall\, j \neq i \,\}
$$

Each boundary is a Half Space $\to$ Convex polytope:

$$
\frac{(p_j - p_i)^\top}{\|p_j - p_i\|}\left( p - \frac{p_i + p_j}{2} \right) \le -r_s
\qquad
\left.\begin{array}{l} \text{Stay on own} \\ \text{side with} \\ \text{safety radius} \end{array}\right\}
\to \text{2 robots always } 2r_s \text{ apart}
$$

**GUARANTEED COLLISION AVOIDANCE BY GEOMETRY**

### Optimization (ineq part)

traj $p = \phi P$:

$$
\therefore\;
\underbrace{\frac{(p_j - p_i)^\top}{\|p_j - p_i\|}\, \phi P}_{A_\text{ineq}}
\;\le\;
\underbrace{\frac{(p_j - p_i)^\top}{\|p_j - p_i\|}\, \frac{p_i + p_j}{2} - r_s}_{b_\text{ineq}}
$$

| BVC | on-demand |
| --- | --- |
| Guaranteed safe | occasional fail |
| **BUT** Conservative | **BUT** Efficient |

---

> ### Note — change from code vs. theory (BVC)
>
> The BVC math above is the **textbook** form (perpendicular bisector through the
> midpoint $\frac{p_i+p_j}{2}$, buffered by $r_s$), which is what yields the geometric
> "guaranteed collision avoidance."
>
> The actual implementation in `cpp/src/bvc_avoidance.cpp` does **not** use this
> bisector/midpoint form. It reuses the **same linearized ellipsoidal keep-out
> constraint as on-demand** (gradient $\propto (p_i - p_j)$, tangent to the keep-out
> ball around $p_j$ — not the midpoint), applied **proactively** to all neighbours
> within $3\,r_\text{min}$ across the whole horizon, as soft (slack-penalized)
> constraints.
>
> So in the code, **"BVC" $=$ proactive on-demand** (the only real difference from
> on-demand is reactive $\to$ proactive scheduling and the trigger radius
> $2r_\text{min} \to 3r_\text{min}$). It therefore produces the *conservative,
> wider-path behaviour* associated with BVC, but does **not** provide the hard,
> geometry-based collision-free guarantee written above. Any claim of a guaranteed-safe
> property holds only for the textbook BVC, not for this implementation.
